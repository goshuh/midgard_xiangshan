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
    val tlb_req_pld  = tlb_o(i).req.bits
    val tlb_res      = tlb_o(i).resp
    val tlb_res_pld  = tlb_o(i).resp.bits
    val tlb_kill     = tlb_o(i).req_kill

    val vlb_req      = u_vlb.vlb_req_i(i)
    val vlb_req_pld  = u_vlb.vlb_req_i(i).bits
    val vlb_res      = u_vlb.vlb_res_o(i)
    val vlb_res_pld  = u_vlb.vlb_res_o(i).bits
    val vlb_ttw      = u_vlb.vlb_ttw_o

    val src_req      = tlb_i(i).req
    val src_req_pld  = tlb_i(i).req.bits
    val src_res      = tlb_i(i).resp
    val src_res_pld  = tlb_i(i).resp.bits
    val src_kill     = tlb_i(i).req_kill
    val src_pmp      = pmp_i(i)

    val src_req_fire = src_req.fire
    val src_req_cmd  = src_req_pld.cmd
    val src_req_vpn  = src_req_pld.vaddr(VAddrBits := 12)

    val src_res_fire = src_res.fire

    val dst_pmp      = pmp_o(i)

    // stage 0
    val s0_sel       = mode_u && csr_i.uatp.en && (src_req_vpn(24 :+ 2) === 1.U)
    val s1_sel_q     = RegEnable(s0_sel,   false.B, src_req_fire)
    val s2_sel_q     = RegEnable(s1_sel_q, false.B, src_res_fire)

    val s1_res_fake  = Pin(Bool())

    // see: handle_block
    tlb_req.valid     := src_req.valid && !s0_sel
    tlb_req_pld.vaddr := src_req_pld.vaddr
    tlb_req_pld.cmd   := src_req_pld.cmd
    tlb_req_pld.size  := src_req_pld.size
    tlb_req_pld.debug := src_req_pld.debug
    tlb_req_pld.kill  := src_req_pld.kill
    tlb_kill          := src_kill

    val s0_kill = Pin(Bool())
    val s1_kill = src_kill || RegNext(s0_kill, false.B)

    vlb_req_pld.idx   := DontCare
    vlb_req_pld.kill  := s1_kill ## s0_kill

    // stage 1
    val s1_req_pld_q = RegEnable(src_req_pld, src_req_fire)

    val s1_req_cmd   = s1_req_pld_q.cmd
    val s1_req_vpn   = s1_req_pld_q.vaddr(VAddrBits := 12)
    val s1_req_offs  = s1_req_pld_q.vaddr(12.W)

    val s1_ld        = TlbCmd.isRead (s1_req_cmd) && !TlbCmd.isAmo(s1_req_cmd)
    val s1_st        = TlbCmd.isWrite(s1_req_cmd) ||  TlbCmd.isAmo(s1_req_cmd)
    val s1_if        = TlbCmd.isExec (s1_req_cmd)

    val s1_vlb_r     = vlb_res_pld.r
    val s1_vlb_w     = vlb_res_pld.w
    val s1_vlb_x     = vlb_res_pld.x
    val s1_vlb_u     = vlb_res_pld.u
    val s1_vlb_g     = vlb_res_pld.g
    val s1_vlb_z     = vlb_res_pld.z
    val s1_vlb_e     = vlb_res_pld.e

    val s1_vlb_xpf   = vlb_res_pld.err ||
                       mode_s &&  s1_vlb_u && !priv.sum ||
                       mode_u && !s1_vlb_u

    val s1_vlb_lpf   = s1_vlb_xpf || s1_ld && !(s1_vlb_r || priv.mxr && s1_vlb_x)
    val s1_vlb_spf   = s1_vlb_xpf || s1_st &&  !s1_vlb_w
    val s1_vlb_ipf   = s1_vlb_xpf || s1_if &&  !s1_vlb_x

    val s1_vlb_hit   = vlb_res_pld.vld ||
                       vlb_res_pld.err
    val s1_vlb_ma    = vlb_res_pld.mpn ## s1_req_offs

    val s1_vlb_vld   = vlb_res.valid && !(B.B && !s1_vlb_hit) || s1_res_fake
    val s1_tlb_vld   = tlb_res.valid

    src_res.valid               := s1_sel_q ??  s1_vlb_vld    :: tlb_res.valid
    src_res_pld.paddr           := s1_sel_q ??  s1_vlb_ma     :: tlb_res_pld.paddr
    src_res_pld.miss            := s1_sel_q ?? !s1_vlb_hit    :: tlb_res_pld.miss
    src_res_pld.fast_miss       := s1_sel_q ?? !s1_vlb_hit    :: tlb_res_pld.fast_miss
    src_res_pld.excp.pf.ld      := s1_sel_q ??  s1_vlb_lpf    :: tlb_res_pld.excp.pf.ld
    src_res_pld.excp.pf.st      := s1_sel_q ??  s1_vlb_spf    :: tlb_res_pld.excp.pf.st
    src_res_pld.excp.pf.instr   := s1_sel_q ??  s1_vlb_ipf    :: tlb_res_pld.excp.pf.instr
    src_res_pld.excp.af.ld      := s1_sel_q ??  false.B       :: tlb_res_pld.excp.af.ld
    src_res_pld.excp.af.st      := s1_sel_q ??  false.B       :: tlb_res_pld.excp.af.st
    src_res_pld.excp.af.instr   := s1_sel_q ??  false.B       :: tlb_res_pld.excp.af.instr
    src_res_pld.static_pm.valid := s1_sel_q ??  s1_vlb_hit    :: tlb_res_pld.static_pm.valid
    src_res_pld.static_pm.bits  := s1_sel_q ??  s1_vlb_e      :: tlb_res_pld.static_pm.bits
    src_res_pld.ptwBack         :=              vlb_ttw.valid || tlb_res_pld.ptwBack

    if (B) {
      // see: req_out_v
      val s1_vld_q = RegEnable(src_req_fire && !s0_kill && !flush_i,
                               false.B,
                               src_req_fire ||
                               src_res_fire ||
                               flush_i)

      val s1_vld   = s1_vld_q && !src_res_fire

      // see: handle_block
      vlb_req.valid   := s1_vld && s1_sel_q   || src_req.valid && s0_sel
      vlb_req_pld.vpn := s1_vld ?? s1_req_vpn :: src_req_vpn

      src_req.ready   := s1_vld_q &&
                            (s1_vlb_vld &&  s1_sel_q  ||
                             s1_tlb_vld && !s1_sel_q) ||
                        !s1_vld_q

      tlb_res.ready   := src_res.ready && !s1_sel_q
      vlb_res.ready   := src_res.ready &&  s1_sel_q

      // icache only cares about s0_kill for new reqs
      s0_kill         := src_req_pld.kill && !s1_vld

      // icache doesn't care about flush_pipe
      s1_res_fake     := s1_vld_q && s1_sel_q && flush_i

    } else {
      // see: handle_nonblock
      vlb_req.valid   := src_req.valid &&  s0_sel
      vlb_req_pld.vpn := src_req_vpn

      src_req.ready   := src_res.ready &&
                            (vlb_req.ready &&  s0_sel ||
                             tlb_req.ready && !s0_sel)

      tlb_res.ready   := src_res.ready && !s0_sel
      vlb_res.ready   := src_res.ready &&  s0_sel

      s0_kill         := src_req_pld.kill
      s1_res_fake     := false.B
    }

    // stage 2
    val s2_vlb_e_q = RegEnable(s1_vlb_e, src_res_fire)

    dst_pmp.ld    := s2_sel_q ?? false.B    :: src_pmp.ld
    dst_pmp.st    := s2_sel_q ?? false.B    :: src_pmp.st
    dst_pmp.instr := s2_sel_q ?? false.B    :: src_pmp.instr
    dst_pmp.mmio  := s2_sel_q ?? s2_vlb_e_q :: src_pmp.mmio
  }
}