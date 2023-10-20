package xiangshan.cache.mmu

import  chisel3._
import  chisel3.util._
import  midgard._
import  midgard.util._

import  xiangshan._
import  xiangshan.backend.fu._
import  xiangshan.backend.fu.util._
import  huancun._

import  chipsalliance.rocketchip.config._


class FSTWIO(P: Param)(implicit p: Parameters) extends Bundle {
  val ttw_req_o =         Valid(new frontside.VLBReq(P))
  val ttw_res_i = Flipped(Valid(new frontside.VMA   (P)))
  val ttw_ext_i =         Input(new frontside.TTWExt(P))

  val vtd_req_i =         Input(new frontside.VTDReq(P))
}


class FSVLBWrapper(N: Int, B: Boolean, P: Param)(implicit val p: Parameters) extends Module
  with HasXSParameter
  with HasCSRConst {

  // --------------------------
  // io

  val csr_i     = IO(               Input(new TlbCsrBundle))
  val sfence_i  = IO(               Input(new SfenceBundle))
  val flush_i   = IO(               Input(Bool()))

  val tlb_i     = IO(Vec(N, Flipped(      new TlbRequestIO)))
  val tlb_o     = IO(Vec(N,               new TlbRequestIO))

  val pmp_i     = IO(Vec(N, Flipped(      new PMPRespBundle)))
  val pmp_o     = IO(Vec(N,               new PMPRespBundle))

  val ttw_o     = IO(                     new FSTWIO(P))


  // --------------------------
  // logic

  val sfence     = sfence_i.valid
  val sfence_all = sfence && !sfence_i.bits.rs2
  val sfence_one = sfence &&  sfence_i.bits.rs2

  val priv       = csr_i.priv

  val mode       = B.B ?? priv.imode :: priv.dmode
  val mode_u     = mode === ModeU
  val mode_s     = mode === ModeS

  //
  // inst

  val u_vlb = Module(new frontside.VLB(P, N))

  u_vlb.uatc_i      := csr_i.uatc
  u_vlb.asid_i      := csr_i.satp.asid
  u_vlb.sdid_i      := csr_i.sdid.sdid

  u_vlb.kill_i      := sfence_all ##
                       sfence_one ##
                      (sfence ||
                       csr_i.satp_changed ||
                       csr_i.uatp_changed ||
                       csr_i.uatc_changed ||
                       csr_i.sdid_changed ||
                       flush_i)

  u_vlb.kill_asid_i := sfence_i.bits.asid
  u_vlb.kill_sdid_i := csr_i.sdid.sdid

  u_vlb.vtd_req_i   := ttw_o.vtd_req_i
  u_vlb.ttw_ext_i   := ttw_o.ttw_ext_i

  ttw_o.ttw_req_o   <> u_vlb.ttw_req_o
  ttw_o.ttw_res_i   <> u_vlb.ttw_res_i


  //
  // connect

  for (i <- 0 until N) {
    val tlb_req      = tlb_o(i).req
    val tlb_res      = tlb_o(i).resp
    val tlb_kill     = tlb_o(i).req_kill

    val vlb_req      = u_vlb.vlb_req_i(i)
    val vlb_res      = u_vlb.vlb_res_o(i)
    val vlb_ttw      = u_vlb.vlb_ttw_o

    val src_req      = tlb_i(i).req
    val src_res      = tlb_i(i).resp
    val src_kill     = tlb_i(i).req_kill || src_req.bits.kill
    val src_pmp      = pmp_i(i)

    val src_req_fire = src_req.fire
    val src_req_cmd  = src_req.bits.cmd
    val src_req_vpn  = src_req.bits.vaddr(VAddrBits := 12)

    val src_res_fire = src_res.fire

    val dst_pmp      = pmp_o(i)

    // stage 0

    val s0_sel       = mode_u && csr_i.uatp.en && (src_req_vpn(24 :+ 2) === 1.U)
    val s1_sel_q     = RegEnable(s0_sel,   false.B, src_req_fire)
    val s2_sel_q     = RegEnable(s1_sel_q, false.B, src_res_fire)

    val s1_req_pld_q = RegEnable(src_req.bits, src_req_fire)
    val s1_req_cmd   = s1_req_pld_q.cmd
    val s1_req_vpn   = s1_req_pld_q.vaddr(VAddrBits := 12)
    val s1_req_offs  = s1_req_pld_q.vaddr(12.W)

    val s1_res_fake  = Pin(Bool())

    // see: handle_block
    tlb_req.valid      := src_req.valid && !s0_sel
    tlb_req.bits.vaddr := src_req.bits.vaddr
    tlb_req.bits.cmd   := src_req.bits.cmd
    tlb_req.bits.size  := src_req.bits.size
    tlb_req.bits.debug := src_req.bits.debug
    tlb_req.bits.kill  := src_kill
    tlb_kill           := src_kill

    val flush_q = RegNext(flush_i || src_kill, false.B)

    vlb_req.bits.idx   := 0.U
    vlb_req.bits.kill  := flush_q ## src_kill

    if (B) {
      // see: req_out_v
      val s1_vld_q = RegEnable(src_req_fire && !src_kill && !flush_i,
                               false.B,
                               src_req_fire ||
                               src_res_fire ||
                               flush_i)

      val s1_vld   = s1_vld_q && !src_res_fire

      // see: handle_block
      vlb_req.valid    := s1_vld && s1_sel_q   || src_req.valid && s0_sel
      vlb_req.bits.vpn := s1_vld ?? s1_req_vpn :: src_req_vpn

      src_req.ready    := s1_vld_q &&
                             (vlb_res.valid &&  s1_sel_q  ||
                              tlb_res.valid && !s1_sel_q) ||
                         !s1_vld_q

      tlb_res.ready    := src_res.ready && !s1_sel_q
      vlb_res.ready    := src_res.ready &&  s1_sel_q

      // icache doesn't care about flush_pipe
      s1_res_fake      := s1_vld_q && s1_sel_q && flush_i

    } else {
      // see: handle_nonblock
      vlb_req.valid    := src_req.valid &&  s0_sel
      vlb_req.bits.vpn := src_req_vpn

      src_req.ready    := src_res.ready &&
                             (vlb_req.ready &&  s0_sel ||
                              tlb_req.ready && !s0_sel)

      tlb_res.ready    := src_res.ready && !s0_sel
      vlb_res.ready    := src_res.ready &&  s0_sel

      s1_res_fake      := false.B
    }

    // stage 1

    val s1_ld      = TlbCmd.isRead (s1_req_cmd) && !TlbCmd.isAmo(s1_req_cmd)
    val s1_st      = TlbCmd.isWrite(s1_req_cmd) ||  TlbCmd.isAmo(s1_req_cmd)
    val s1_if      = TlbCmd.isExec (s1_req_cmd)

    val s1_vlb_r   = vlb_res.bits.r
    val s1_vlb_w   = vlb_res.bits.w
    val s1_vlb_x   = vlb_res.bits.x
    val s1_vlb_u   = vlb_res.bits.u
    val s1_vlb_g   = vlb_res.bits.g
    val s1_vlb_z   = vlb_res.bits.z
    val s1_vlb_e   = vlb_res.bits.e

    val s1_vlb_xpf = vlb_res.bits.err ||
                     mode_s &&  s1_vlb_u && !priv.sum ||
                     mode_u && !s1_vlb_u

    val s1_vlb_lpf = s1_vlb_xpf || s1_ld && !(s1_vlb_r || priv.mxr && s1_vlb_x)
    val s1_vlb_spf = s1_vlb_xpf || s1_st &&  !s1_vlb_w
    val s1_vlb_ipf = s1_vlb_xpf || s1_if &&  !s1_vlb_x

    val s1_vlb_vld = vlb_res.valid && !(B.B && !vlb_res.bits.vld) || s1_res_fake
    val s1_vlb_ma  = vlb_res.bits.mpn ## s1_req_offs
    val s1_vlb_hit = vlb_res.bits.vld ||
                     vlb_res.bits.err

    src_res.valid                := s1_sel_q ??  s1_vlb_vld    :: tlb_res.valid
    src_res.bits.paddr           := s1_sel_q ??  s1_vlb_ma     :: tlb_res.bits.paddr
    src_res.bits.miss            := s1_sel_q ?? !s1_vlb_hit    :: tlb_res.bits.miss
    src_res.bits.fast_miss       := s1_sel_q ?? !s1_vlb_hit    :: tlb_res.bits.fast_miss
    src_res.bits.excp.pf.ld      := s1_sel_q ??  s1_vlb_lpf    :: tlb_res.bits.excp.pf.ld
    src_res.bits.excp.pf.st      := s1_sel_q ??  s1_vlb_spf    :: tlb_res.bits.excp.pf.st
    src_res.bits.excp.pf.instr   := s1_sel_q ??  s1_vlb_ipf    :: tlb_res.bits.excp.pf.instr
    src_res.bits.excp.af.ld      := s1_sel_q ??  false.B       :: tlb_res.bits.excp.af.ld
    src_res.bits.excp.af.st      := s1_sel_q ??  false.B       :: tlb_res.bits.excp.af.st
    src_res.bits.excp.af.instr   := s1_sel_q ??  false.B       :: tlb_res.bits.excp.af.instr
    src_res.bits.static_pm.valid := s1_sel_q ??  s1_vlb_vld    :: tlb_res.bits.static_pm.valid
    src_res.bits.static_pm.bits  := s1_sel_q ??  s1_vlb_e      :: tlb_res.bits.static_pm.bits
    src_res.bits.ptwBack         :=              vlb_ttw.valid || tlb_res.bits.ptwBack

    // stage 2
    val s2_vlb_e_q = RegEnable(s1_vlb_e, src_res_fire)

    dst_pmp.ld                   := s2_sel_q ??  false.B       :: src_pmp.ld
    dst_pmp.st                   := s2_sel_q ??  false.B       :: src_pmp.st
    dst_pmp.instr                := s2_sel_q ??  false.B       :: src_pmp.instr
    dst_pmp.mmio                 := s2_sel_q ??  s2_vlb_e_q    :: src_pmp.mmio
  }
}