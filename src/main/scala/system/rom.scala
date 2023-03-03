package system

import chisel3._
import chisel3.util._
import chisel3.util.random._
import midgard.util._

import chipsalliance.rocketchip.config._

import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

class RomBlackBoxIO extends Bundle {
  val clk = Input(Clock())
  val en = Input(Bool())
  val we = Input(Bool())
  val addr = Input(UInt(27.W))
  val di  = Input(UInt(64.W))
  val dout = Output(UInt(64.W))
}

// https://docs.xilinx.com/r/en-US/ug901-vivado-synthesis/ROM-HDL-Coding-Techniques
class RomBlackBox extends BlackBox{
  val io = IO(new RomBlackBoxIO)
}

class Rom(implicit p: Parameters) extends LazyModule {

  val node = TLManagerNode(
    Seq(
      TLSlavePortParameters.v1(
        Seq(
          TLSlaveParameters.v1(
            address = Seq(AddressSet(0x10000000L, 0xfffffff)),
            regionType = RegionType.UNCACHED,
            supportsGet = TransferSizes(1, 8),
            supportsPutFull = TransferSizes(1, 8),
            supportsPutPartial = TransferSizes(1, 8),
            fifoId = Some(0)
          )
        ),
        beatBytes = 8
      )
    )
  )


  lazy val module = new LazyModuleImp(this) {
    val (i, ie) = node.in.head
    val rom = Module(new RomBlackBox()).io
    val rom_dbg = IO(Output(Bool()))
    rom_dbg  := i.a.fire

    val idle :: read :: read_resp :: write_full :: write_partial :: write_resp :: Nil = Enum(6)
    val state = RegInit(idle)
    val context = Reg(i.a.bits.cloneType) // TLbundleA
    val data = Reg(UInt(64.W))

    rom.clk := clock
    rom.en := false.B
    rom.we := false.B
    rom.di := 0.U
    rom.addr := 0.U

    i.a.ready := state === idle
    i.d.valid := state === read_resp || state === write_resp

    switch(state) {
      is(idle) {
        when(i.a.fire) {
          val wr_full = (i.a.bits.opcode === TLMessages.PutFullData)
          val wr_part = (i.a.bits.opcode === TLMessages.PutPartialData)
          rom.en := true.B
          rom.addr := i.a.bits.address(26,3)
          rom.we := wr_full
          rom.di := i.a.bits.data

          context := i.a.bits
          when (wr_full) {
            state := write_full
          }.elsewhen (wr_part) {
            state := write_partial
          }.otherwise{
            state := read
          }
        }
      }
      is(read) {
        data := rom.dout
        state := read_resp
      }
      is(write_full) {
        state := write_resp
      }
      is(write_partial) {
        rom.en := true.B
        rom.addr := context.address(26,3)
        rom.we := true.B
        val data = Wire(Vec(8, UInt(8.W)))
        for (i <- 0 until 8) {
          val sel = (context.mask >> i.U)
          when (sel(0)) {
            data(i) := (context.data >> (i*8).U) & (255.U)
          }.otherwise {
            data(i) := (rom.dout >> (i*8).U) & (255.U)
          }
        }
        rom.di := data.asTypeOf(UInt(64.W))
        state := write_resp
      }
      is(read_resp) {
        i.d.bits := ie.AccessAck(context, data)
        when(i.d.fire) { state := idle }
      }
      is(write_resp) {
        i.d.bits := ie.AccessAck(context)
        when(i.d.fire) { state := idle }
      }
    }

  }
}
