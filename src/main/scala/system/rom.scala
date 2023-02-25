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
  val clk = Input(Bool())
  val en = Input(Bool())
  val addr = Input(UInt(16.W))
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

    val idle :: read :: resp :: Nil = Enum(3)
    val state = RegInit(idle)
    val context = Reg(i.a.bits.cloneType) // TLbundleA
    val data = Reg(UInt(64.W))

    rom.en := false.B
    rom.addr := 0.U
    i.a.ready := state === idle
    i.d.valid := state === resp

    switch(state) {
      is(idle) {
        when(i.a.fire) {
          rom.en := true.B
          rom.addr := i.a.bits.address
          context := i.a.bits
          state := read
        }
      }
      is(read) {
        data := rom.dout
        state := resp
      }
      is(resp) {
        i.d.bits := ie.AccessAck(context, data)
        when(i.d.fire) { state := idle }
      }

    }

  }
}
