package xiangshan.cache.mmu

import  chisel3._
import  chisel3.util._
import  midgard._

import  xiangshan._
import  utils._

import  chipsalliance.rocketchip.config._
import  freechips.rocketchip.diplomacy._
import  freechips.rocketchip.tilelink._


class FSPTWWrapper(N: Int, P: Param)(implicit p: Parameters) extends LazyModule
  with HasXSParameter {

  val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
      "ptw",
      sourceId = IdRange(0, 1)
    ))
  )))

  lazy val module = new LazyModuleImp(this) {

    // --------------------------
    // io

    val satp_i = IO(               Input(UInt(P.mcnBits.W)))

    val vlb_i  = IO(Vec(N, Flipped(      new FSPTWIO(P))))


    // --------------------------
    // logic

    //
    // inst

    val u_ptw = Module(new frontside.PTW(P, N))

    u_ptw.satp_i <> satp_i

    for (i <- 0 until N) {
      u_ptw.vlb_req_i(i) <> vlb_i(i).ptw_req_o
      u_ptw.vlb_res_o(i) <> vlb_i(i).ptw_res_i
    }


    //
    // connect

    val (mem, edge) = node.out.head

    // a channel
    mem.a.valid               := u_ptw.mem_req_o.valid
    mem.a.bits                := edge.Get(0.U,
                                          u_ptw.mem_req_o.bits.mcn ## 0.U(6.W),
                                          6.U)._2

    u_ptw.mem_req_o.ready     := mem.a.ready

    // d channel
    u_ptw.mem_res_i.valid     := mem.d.valid
    u_ptw.mem_res_i.bits.data := mem.d.bits.data

    mem.d.ready               := u_ptw.mem_res_i.ready
  }
}