package xiangshan.cache.mmu

import  chisel3._
import  chisel3.util._
import  midgard._
import  midgard.util._

import  xiangshan._
import  huancun._
import  utils._

import  chipsalliance.rocketchip.config._
import  freechips.rocketchip.diplomacy._
import  freechips.rocketchip.tilelink._


class FSTTWWrapper(P: Param)(implicit p: Parameters) extends LazyModule
  with HasXSParameter {

  val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
      "ttw",
      sourceId = IdRange(0, mgFSParam.ttwNum)
    )),
    requestFields = Seq(VTDIField())
  )))

  lazy val module = new LazyModuleImp(this) {

    // --------------------------
    // io

    val satp_i = IO(                          Input(UInt(64.W)))
    val uatp_i = IO(                          Input(UInt(64.W)))
    val uatc_i = IO(                          Input(new frontside.VSCCfg()))
    val asid_i = IO(                          Input(UInt(P.asidBits.W)))
    val sdid_i = IO(                          Input(UInt(P.sdidBits.W)))

    val vlb_i  = IO(Vec(P.ttwNum, Flipped(          new FSTWIO(P))))
    val vtd_i  = IO(              Flipped(Decoupled(new frontside.VTDReq(P))))


    // --------------------------
    // logic

    //
    // inst

    val u_ttw = Module(new frontside.VSC(P))

    u_ttw.satp_i    := satp_i
    u_ttw.uatp_i    := uatp_i
    u_ttw.uatc_i    := uatc_i
    u_ttw.asid_i    := asid_i
    u_ttw.sdid_i    := sdid_i

    u_ttw.vtd_req_i <> vtd_i

    for (i <- 0 until P.ttwNum) {
      vlb_i(i).ttw_req_o <> u_ttw.vlb_req_i(i)
      vlb_i(i).ttw_res_i <> u_ttw.vlb_res_o(i)
      vlb_i(i).ttw_ext_i := u_ttw.vlb_ext_o(i)
      vlb_i(i).vtd_req_i := u_ttw.vtd_req_o
    }


    //
    // connect

    val (mem, edge) = node.out.head

    // a channel
    mem.a.valid               := u_ttw.mem_req_o.valid
    mem.a.bits                := edge.Get(u_ttw.mem_req_o.bits.idx,
                                          u_ttw.mem_req_o.bits.mcn ## 0.U(6.W),
                                          6.U)._2

    // inform the vlb directory
    mem.a.bits.user.lift(VTDIKey).foreach(_ := true.B)

    u_ttw.mem_req_o.ready     := mem.a.ready

    // d channel
    u_ttw.mem_res_i.valid     := mem.d.valid
    u_ttw.mem_res_i.bits.idx  := mem.d.bits.source(P.ttwIdx.W)
    u_ttw.mem_res_i.bits.data := mem.d.bits.data

    mem.d.ready               := u_ttw.mem_res_i.ready
  }
}