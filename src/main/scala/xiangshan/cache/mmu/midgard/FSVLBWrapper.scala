package xiangshan.cache.mmu

import  chisel3._
import  chisel3.util._
import  midgard._
import  midgard.util._

import  xiangshan._
import  xiangshan.backend.fu._
import  xiangshan.backend.fu.util._

import  chipsalliance.rocketchip.config._


class FSPTWIO(P: Param) extends Bundle {
  val ptw_req_o  =         Decoupled(new frontside.VLBReq(P))
  val ptw_resp_i = Flipped(    Valid(new frontside.VMA(P)))
}


class FSVLBWrapper(N: Int, B: Boolean, F: Int, P: Param)(implicit val p: Parameters) extends Module
  with HasXSParameter
  with HasCSRConst {

  // --------------------------
  // io

  val sfence_i   = IO(                   Input(new SfenceBundle))
  val csr_i      = IO(                   Input(new TlbCsrBundle))
  val flush_i    = IO(                   Input(Bool()))

  val tlb_i      = IO(Vec(N, Flipped(          new TlbRequestIO)))
  val tlb_o      = IO(Vec(N,                   new TlbRequestIO))

  val pmp_i      = IO(Vec(N, Flipped(          new PMPRespBundle)))
  val pmp_o      = IO(Vec(N,                   new PMPRespBundle))

  val ptw_req_o  = IO(               Decoupled(new frontside.VLBReq(P)))
  val ptw_resp_i = IO(       Flipped(    Valid(new frontside.VMA(P))))


  // --------------------------
  // logic

  val sfence     = sfence_i.valid
  val sfence_all = sfence && !sfence_i.bits.rs2
  val sfence_one = sfence &&  sfence_i.bits.rs2

  val priv       = csr_i.priv
  val satp       = csr_i.satp
  val mode       = P.tlbEn.B ?? priv.imode :: priv.dmode

  // expand flush
  val flush      = Any(Exp(flush_i, F))
  val flush_q    = RegNext(flush_i)

  //
  // inst

  // now everyone takes two cycles for the tlb
  require(!P.tlbEn)

  val u_vlb = Module(new frontside.VLB(P, N))

  u_vlb.asid_i      := satp.asid

  u_vlb.kill_i      := sfence_all ##
                       sfence_one ##
                      (sfence || satp.changed || flush)
  u_vlb.kill_asid_i := sfence_i.bits.asid

  ptw_req_o         <> u_vlb.ptw_req_o
  ptw_resp_i        <> u_vlb.ptw_resp_i


  //
  // switch

  val sel_mg    = (satp.mode === 0xf.U) && (mode =/= ModeM) && P.en.B
  val sel_mg_q  =  RegNext(sel_mg)

  // mode can toggle
  val sel_to_xs = !sel_mg &&  sel_mg_q
  val sel_to_mg =  sel_mg && !sel_mg_q

  // default connection
  for (i <- 0 until N) {
    tlb_o(i) <> tlb_i(i)
    pmp_o(i) <> pmp_i(i)
  }

  // midgard connection
  for (i <- 0 until N) {
    val vlb_req  = u_vlb.vlb_req_i (i)
    val vlb_resp = u_vlb.vlb_resp_o(i)
    val vlb_fill = u_vlb.vlb_fill_o

    val tlb_req  = tlb_i(i).req
    val tlb_resp = tlb_i(i).resp
    val tlb_kill = tlb_i(i).req_kill || flush_q || sel_to_xs

    val tlb_vpn  = tlb_req.bits.vaddr(VAddrBits := 12)
    val tlb_offs = tlb_req.bits.vaddr(12.W)

    val pmp_resp = pmp_o(i)

    val req_vld  = dontTouch(Wire(Bool()))
    val req_vpn  = dontTouch(Wire(UInt((VAddrBits - 12).W)))
    val req_cmd  = dontTouch(Wire(UInt(3.W)))
    val req_kill = dontTouch(Wire(UInt(2.W)))

    vlb_req.valid := req_vld
    vlb_req.bits  := frontside.VLBReq(P,
                                      0.U,
                                      req_vpn,
                                      req_kill)

    if (B) {
      val vld_old = dontTouch(Wire(Bool()))
      val vld_new = dontTouch(Wire(Bool()))

      val vld_raw = sel_mg && (vld_old || vld_new)
      val vld     = sel_mg && (vld_old || vld_new && !tlb_req.bits.kill) && !flush_q
      val vld_q   = sel_mg &&  RegNext(vld)

      assert(tlb_req.valid -> !flush_q)

      // upstream issues random reqs. always stick to the oldest one
      vld_new  := tlb_req.fire
      vld_old  := vld_q && !tlb_resp.fire

      req_vld  := vld_raw
      req_vpn  := vld_old  ??  RegEnable(tlb_vpn,          vld_new) :: tlb_vpn
      req_cmd  := vld_old  ??  RegEnable(tlb_req.bits.cmd, vld_new) :: tlb_req.bits.cmd
      req_kill := tlb_kill ## (flush ||  vld_new && tlb_req.bits.kill)

      when (sel_mg) {
        tlb_req.ready := tlb_resp.fire || !vld_q

        // also fake a response for flush
        tlb_resp.valid          := vlb_resp.valid && vlb_resp.bits.vld || vld_q && flush_q
        tlb_resp.bits.paddr     := vlb_resp.bits.mpn ## RegEnable(tlb_offs, vld_new)
        tlb_resp.bits.miss      := false.B
        tlb_resp.bits.fast_miss := false.B
      }

    } else {
      req_vld  := sel_mg && tlb_req.valid
      req_vpn  := tlb_vpn
      req_cmd  := RegNext(tlb_req.bits.cmd)
      req_kill := tlb_kill ## (flush || tlb_req.bits.kill)

      when (sel_mg) {
        tlb_req.ready := true.B

        tlb_resp.valid          :=     vlb_resp.valid
        tlb_resp.bits.paddr     :=     vlb_resp.bits.mpn ## RegNext(tlb_offs)
        tlb_resp.bits.miss      := Non(vlb_resp.bits.vld)
        tlb_resp.bits.fast_miss := Non(vlb_resp.bits.vld)
      }
    }

    when (sel_mg) {
      // break the default connection
      tlb_o(i).req .valid := false.B
      tlb_o(i).resp.ready :=  true.B

      val cmd_ld = TlbCmd.isRead (req_cmd) && !TlbCmd.isAmo(req_cmd)
      val cmd_st = TlbCmd.isWrite(req_cmd) ||  TlbCmd.isAmo(req_cmd)
      val cmd_if = TlbCmd.isExec (req_cmd)

      val perm_r = vlb_resp.bits.attr(1)
      val perm_w = vlb_resp.bits.attr(2)
      val perm_x = vlb_resp.bits.attr(3)
      val perm_u = vlb_resp.bits.attr(4)
      val perm_g = vlb_resp.bits.attr(5)
      val perm_a = vlb_resp.bits.attr(6)
      val perm_d = vlb_resp.bits.attr(7)

      val pf     = vlb_resp.bits.err ||
                  (mode === ModeS) &&  perm_u && (!priv.sum || P.tlbEn.B) ||
                  (mode === ModeU) && !perm_u

      tlb_resp.bits.excp.pf.ld      := pf || cmd_ld && !(perm_a &&          (perm_r || priv.mxr && perm_x))
      tlb_resp.bits.excp.pf.st      := pf || cmd_st && !(perm_a && perm_d && perm_w)
      tlb_resp.bits.excp.pf.instr   := pf || cmd_if && !(perm_a &&           perm_x)
      tlb_resp.bits.excp.af.ld      := false.B
      tlb_resp.bits.excp.af.st      := false.B
      tlb_resp.bits.excp.af.instr   := false.B

      tlb_resp.bits.static_pm.valid := tlb_resp.valid
      tlb_resp.bits.static_pm.bits  := vlb_resp.bits.attr(0)
      tlb_resp.bits.ptwBack         := vlb_fill.fire

      pmp_resp.ld                   := false.B
      pmp_resp.st                   := false.B
      pmp_resp.instr                := false.B
      pmp_resp.mmio                 := RegNext(vlb_resp.bits.attr(0))
    }
  }
}