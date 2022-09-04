package xiangshan.cache.mmu

import  chisel3._
import  chisel3.util._
import  midgard._
import  midgard.util._

import  xiangshan._
import  xiangshan.backend.fu._
import  xiangshan.backend.fu.util._

import  chipsalliance.rocketchip.config._


class MidgardFSPTWIO(P: Param) extends Bundle {
  val ptw_req_o  =         Decoupled(new frontside.VLBReq(P))
  val ptw_resp_i = Flipped(    Valid(new frontside.VMA(P)))
}


class MidgardFSVLBWrapper(N: Int, P: Param)(implicit val p: Parameters) extends Module
  with HasXSParameter
  with HasCSRConst {

  // --------------------------
  // io

  val sfence_i   = IO(                   Input(new SfenceBundle))
  val csr_i      = IO(                   Input(new TlbCsrBundle))
  val flush_i    = IO(                   Input(UInt(N.W)))

  val tlb_i      = IO(Vec(N, Flipped(          new TlbRequestIO)))
  val tlb_o      = IO(Vec(N,                   new TlbRequestIO))

  val pmp_i      = IO(Vec(N,                   new PMPRespBundle))
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


  //
  // inst

  val u_vlb = Module(new frontside.VLB(P, N))

  u_vlb.asid_i      := satp.asid

  u_vlb.kill_i      := sfence_all ##
                       sfence_one ##
                      (sfence || satp.changed || Any(flush_i))
  u_vlb.kill_asid_i := sfence_i.bits.asid

  ptw_req_o         <> u_vlb.ptw_req_o
  ptw_resp_i        <> u_vlb.ptw_resp_i


  //
  // switch

  val sel_mg = (satp.mode === 0xf.U) && (mode =/= ModeM)

  // default connection
  for (i <- 0 until N) {
    tlb_o(i) <> tlb_i(i)
    pmp_o(i) <> pmp_i(i)
  }

  // midgard connection
  for (i <- 0 until N) {
    val flush    = flush_i(i)

    val vlb_req  = u_vlb.vlb_req_i (i)
    val vlb_resp = u_vlb.vlb_resp_o(i)

    val tlb_req  = tlb_i(i).req
    val tlb_resp = tlb_i(i).resp
    val tlb_kill = tlb_i(i).req_kill

    val tlb_vpn  = tlb_req.bits.vaddr(VAddrBits := 12)
    val tlb_offs = tlb_req.bits.vaddr(12.W)

    val pmp_resp = pmp_o(i)

    val req_vld  = dontTouch(Wire(Bool()))
    val req_vpn  = dontTouch(Wire(UInt((VAddrBits - 12).W)))
    val req_cmd  = dontTouch(Wire(UInt(3.W)))
    val req_kill = dontTouch(Wire(UInt(3.W)))

    vlb_req.valid := req_vld && sel_mg
    vlb_req.bits  := frontside.VLBReq(P,
                                      0.U,
                                      req_vpn,
                                      req_kill)

    if (P.tlbEn) {
      val vld_raw = dontTouch(Wire(Bool()))
      val vld_q   = dontTouch(Wire(Bool()))
      val vld     = vld_raw || vld_q

      // upstream may not reissue the tlb req after sfence/satp.
      // just retry until succeed
      vld_raw  := tlb_req.valid && !vld_q
      vld_q    := RegEnable(vld && !tlb_resp.fire() && !flush,
                            false.B,
                            vld)

      req_vld  := vld
      req_vpn  := vld_q ?? RegEnable(tlb_vpn,          vld_raw) :: tlb_vpn
      req_cmd  := vld_q ?? RegEnable(tlb_req.bits.cmd, vld_raw) :: tlb_req.bits.cmd
      req_kill := tlb_kill ## flush

      when (sel_mg) {
        tlb_resp.valid          := vlb_resp.valid && vlb_resp.bits.vld
        tlb_resp.bits.paddr     := vlb_resp.bits.mpn ## (vld_q ?? RegEnable(tlb_offs, vld_raw) :: tlb_offs)
        tlb_resp.bits.miss      := false.B
        tlb_resp.bits.fast_miss := false.B
      }

    } else {
      req_vld  := tlb_req.valid
      req_vpn  := tlb_vpn
      req_cmd  := RegNext(tlb_req.bits.cmd)
      req_kill := tlb_kill ## flush

      when (sel_mg) {
        tlb_resp.valid          :=     vlb_resp.valid
        tlb_resp.bits.paddr     :=     vlb_resp.bits.mpn ## RegNext(tlb_offs)
        tlb_resp.bits.miss      := Non(vlb_resp.bits.vld)
        tlb_resp.bits.fast_miss := Non(vlb_resp.bits.vld)
      }
    }

    when (sel_mg) {
      // break the default connection
      tlb_i(i).req .ready :=  true.B

      tlb_o(i).req .valid := false.B
      tlb_o(i).resp.ready :=  true.B

      // sanity check
      assert(tlb_resp.valid -> tlb_resp.ready)

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

      tlb_resp.bits.static_pm.valid := RegNext(tlb_resp.valid)
      tlb_resp.bits.static_pm.bits  := RegNext(vlb_resp.bits.attr(0))
      tlb_resp.bits.ptwBack         := false.B

      pmp_resp.ld                   := false.B
      pmp_resp.st                   := false.B
      pmp_resp.instr                := false.B
      pmp_resp.mmio                 := tlb_resp.bits.static_pm.bits
    }
  }
}