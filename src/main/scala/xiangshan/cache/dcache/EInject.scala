package xiangshan.cache.dcache

import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.HasXSParameter

class EInject( PAddrBits : Int) extends Module {
  val io = IO(new Bundle() {
    val i_addr = Flipped(ValidIO(UInt(PAddrBits.W)))
    val o_err  = Output(Bool())
  })

  io.o_err := false.B

  when (io.i_addr.valid) {
    val match_addr = VecInit(Seq(0x100000000L,
                                 0x100000000L + (0x1L << 12),
                                 0x100000000L + (0x2L << 12))
                              .map(_.U))
    val fire = RegInit(VecInit(true.B, true.B, true.B))
    val mask_page = Cat(Fill(52,1.U), Fill(12,0.U))

    val addr = io.i_addr.bits & mask_page
    for (i <- 0 until 3) {
      when (addr === match_addr(i) && fire(i)) {
        io.o_err := true.B
        fire(i) := false.B
      }
    }
  }

}
