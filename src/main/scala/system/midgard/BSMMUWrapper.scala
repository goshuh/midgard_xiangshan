package system

import  chisel3._
import  chisel3.util._
import  midgard._
import  midgard.util._

import  huancun._

import  chipsalliance.rocketchip.config._

import  freechips.rocketchip.config._
import  freechips.rocketchip.diplomacy._
import  freechips.rocketchip.tilelink._


class BSMMUWrapper(implicit p: Parameters) extends LazyModule{
  val Q = p(MidgardKey)

  val ctl_node = TLManagerNode(
                   Seq(TLSlavePortParameters.v1(
                     Seq(TLSlaveParameters.v1(
                       address            = Seq(AddressSet(Q.ctlBase, Q.ctlSize)),
                       regionType         = RegionType.UNCACHED,
                       supportsGet        = TransferSizes(1, 8),
                       supportsPutFull    = TransferSizes(1, 8),
                       supportsPutPartial = TransferSizes(1, 8),
                       fifoId             = Some(0),
                       resources          = new SimpleDevice("bmmu", Seq("midgard.bmmu")).reg)),
                     beatBytes = 8)))

  // only support llc as the single master
  val llc_end_idx = p(SoCParamsKey).L3CacheParamsOpt.get.mshrs + 2

  val adp_node = TLAdapterNode(
    clientFn  = { cp =>
      cp.v1copy(
        clients = cp.clients.map { c =>
          require(c.sourceId.end == llc_end_idx)

          c.v1copy(
            supportsProbe = TransferSizes.none,
            sourceId      = IdRange(0, Q.mrqWays))
        })
    },
    managerFn = { mp =>
      mp.v1copy(
        managers = mp.managers.map { m =>
          require(m.regionType == RegionType.UNCACHED)

          m.v1copy(
            address          = AddressSet(0L, (1L << Q.maBits) - 1L).subtract(AddressSet(0x0L, 0x7fffffffL)),
            supportsAcquireB = m.supportsGet,
            supportsAcquireT = m.supportsPutFull,
            alwaysGrantsT    = true,
            mayDenyGet       = true,
            mayDenyPut       = true)
        },
        endSinkId  = Q.mrqWays,
        beatBytes  = 64,
        minLatency = 0)
    })

  lazy val module = new LazyModuleImp(this) {
    val (c, ce) = ctl_node.in .head
    val (i, ie) = adp_node.in .head
    val (o, oe) = adp_node.out.head

    val
      (fsm_idle ::
       fsm_req  ::
       fsm_pend ::
       fsm_res  ::
       fsm_null) = Enum(4)


    //
    // inst

    val N = ie.bundle.sourceBits
    val M = ie.client.clients.head.sourceId.end

    val P = Q.copy(llcIdx  = N,
                   mrqWays = Q.mrqWays)

    val u_mmu = Module(new backside.MMU(P))

    val llc_req   = u_mmu.llc_req_i
    val llc_res   = u_mmu.llc_res_o
    val prb_req   = u_mmu.llc_req_o
    val prb_res   = u_mmu.llc_res_i
    val mem_req   = u_mmu.mem_req_o
    val mem_res   = u_mmu.mem_res_i


    //
    // ctl

    val ctl_req   = c.a.fire
    val ctl_res   = c.d.fire

    val ctl_sel   = Dec(c.a.bits.address(6, 3))
    val ctl_wnr   = c.a.bits.opcode === TLMessages.PutFullData ||
                    c.a.bits.opcode === TLMessages.PutPartialData

    val ctl_wnr_q = RegEnable(ctl_wnr,         ctl_req)
    val ctl_src_q = RegEnable(c.a.bits.source, ctl_req)

    // body
    val ctl_q     = Pin(Vec(8, UInt(64.W)))

    val ctl_wen   = ctl_req && ctl_wnr
    val ctl_wdata = c.a.bits.data
    val ctl_rdata = OrM(ctl_sel, ctl_q)

    for (i <- 0 until 8) {
      val wen = ctl_wen && ctl_sel(i)

      i match {
        case 0 =>
          ctl_q(i) := RegEnable(ctl_wdata, P.ctlInit.U, wen)
        case 7 =>
          ctl_q(i) := 0.U
        case _ =>
          ctl_q(i) := RegEnable(ctl_wdata, wen)
      }
    }

    val inv_wen = ctl_wen && ctl_sel(7)

    val ctl_reg_q   = RegEnable(ctl_req && !inv_wen,
                                false.B,
                                ctl_req ||  ctl_res)

    val ctl_reg_res = ctl_wnr_q ??
                          ce.AccessAck(ctl_src_q,
                                       3.U) ::
                          ce.AccessAck(ctl_src_q,
                                       3.U,
                                       RegEnable(ctl_rdata, ctl_req))

    u_mmu.ctl_i := ctl_q
    u_mmu.rst_i := RegNext(ctl_wen && Any(ctl_sel(6.W)))

    // inv
    val inv_fsm_en  = Pin(Bool())
    val inv_fsm_nxt = Pin(UInt(2.W))
    val inv_fsm_q   = Pin(UInt(2.W))
    val inv_mcn_q   = Pin(UInt(P.mcnBits.W))

    val chb_inv_gnt = Pin(Bool())
    val chc_inv     = Pin(Bool())

    // default
    inv_fsm_en  := false.B
    inv_fsm_nxt := inv_fsm_q

    switch (inv_fsm_q) {
      is (fsm_idle) {
        inv_fsm_en  := inv_wen
        inv_fsm_nxt := fsm_req
      }
      is (fsm_req) {
        inv_fsm_en  := chb_inv_gnt
        inv_fsm_nxt := fsm_pend
      }
      is (fsm_pend) {
        inv_fsm_en  := chc_inv
        inv_fsm_nxt := fsm_res
      }
      is (fsm_res) {
        inv_fsm_en  := c.d.ready && !ctl_reg_q
        inv_fsm_nxt := fsm_idle
      }
    }

    val inv_fsm_is_idle = inv_fsm_q === fsm_idle
    val inv_fsm_is_req  = inv_fsm_q === fsm_req
    val inv_fsm_is_pend = inv_fsm_q === fsm_pend
    val inv_fsm_is_res  = inv_fsm_q === fsm_res

    inv_fsm_q := RegEnable(inv_fsm_nxt, fsm_idle,          inv_fsm_en)
    inv_mcn_q := RegEnable(ctl_wdata(P.maBits := P.clWid), inv_wen)

    val inv_res_vld = inv_fsm_is_res
    val inv_res     = ce.AccessAck(ctl_src_q,
                                   3.U)

    // force serialization
    c.a.ready   := inv_fsm_is_idle

    c.d.valid   := ctl_reg_q || inv_res_vld
    c.d.bits    := ctl_reg_q ?? ctl_reg_res :: inv_res


    //
    // mem

    o.a.valid   := mem_req.valid
    o.a.bits    := mem_req.bits.wnr ??
                       oe.Put(mem_req.bits.idx,
                              mem_req.bits.pcn ## 0.U(P.clWid.W),
                              P.clWid.U,
                              mem_req.bits.data)._2 ::
                       oe.Get(mem_req.bits.idx,
                              mem_req.bits.pcn ## 0.U(P.clWid.W),
                              P.clWid.U)._2

    mem_req.ready := o.a.ready

    mem_res.valid := o.d.valid
    mem_res.bits  := backside.MemRes(P,
                                     o.d.bits.source,
                                     o.d.bits.corrupt || o.d.bits.denied,
                                     o.d.bits.opcode === TLMessages.AccessAck,
                                     o.d.bits.data)

    o.d.ready   := mem_res.ready


    //
    // llc

    val cha_acq_req_raw = i.a.bits.opcode === TLMessages.AcquireBlock
    val cha_pfd_req_raw = i.a.bits.opcode === TLMessages.PutFullData
    val cha_ppd_req_raw = i.a.bits.opcode === TLMessages.PutPartialData
    val chc_mis_req_raw = i.c.bits.opcode === TLMessages.ProbeAck
    val chc_hit_req_raw = i.c.bits.opcode === TLMessages.ProbeAckData
    val chc_cev_req_raw = i.c.bits.opcode === TLMessages.Release
    val chc_dev_req_raw = i.c.bits.opcode === TLMessages.ReleaseData

    // encoding doesn't work
    val chc_sel_inv = Pin(Bool())

    val cha_acq_req = i.a.valid && cha_acq_req_raw
    val cha_pfd_req = i.a.valid && cha_pfd_req_raw
    val cha_ppd_req = i.a.valid && cha_ppd_req_raw
    val chc_mis_req = i.c.valid && chc_mis_req_raw && !chc_sel_inv
    val chc_hit_req = i.c.valid && chc_hit_req_raw && !chc_sel_inv
    val chc_inv_req = i.c.valid && chc_mis_req_raw &&  chc_sel_inv
    val chc_wrb_req = i.c.valid && chc_hit_req_raw &&  chc_sel_inv
    val chc_cev_req = i.c.valid && chc_cev_req_raw
    val chc_dev_req = i.c.valid && chc_dev_req_raw
    val chd_acq_req = Pin(Bool())

    val chc_prb = i.c.fire && (chc_mis_req_raw || chc_hit_req_raw)

    // no other possibilities
    Chk(i.a.valid -> (cha_acq_req_raw ||
                      cha_pfd_req_raw ||
                      cha_ppd_req_raw))
    Chk(i.c.valid -> (chc_mis_req_raw ||
                      chc_hit_req_raw ||
                      chc_cev_req_raw ||
                      chc_dev_req_raw))

    val cha_acq_gnt = Pin(Bool())
    val chb_prb_gnt = Pin(Bool())
    val chc_wrb_gnt = Pin(Bool())
    val chc_cev_gnt = Pin(Bool())
    val chc_dev_gnt = Pin(Bool())
    val chd_acq_gnt = Pin(Bool())

    // a channel
    i.a.ready   := cha_acq_gnt

    // b channel
    val chb_inv_req = inv_fsm_is_req
    val chb_prb_req = prb_req.valid

    // priority: inv > prb
    chb_inv_gnt := i.b.ready
    chb_prb_gnt := chb_inv_gnt && !chb_inv_req

    chc_sel_inv := Src(i.b.fire, chc_prb, chb_inv_req)

    i.b.valid   := chb_inv_req || chb_prb_req
    i.b.bits    := chb_inv_req ??
                       ie.Probe(inv_mcn_q ## 0.U(P.clWid.W),
                                0.U,
                                P.clWid.U,
                                TLPermissions.toN)._2 ::
                       ie.Probe(prb_req.bits.mcn ## 0.U(P.clWid.W),
                                0.U,
                                P.clWid.U,
                                TLPermissions.toT)._2

    // c channel
    i.c.ready   := chc_mis_req_raw && (chc_sel_inv ?? true.B      :: prb_res.ready) ||
                   chc_hit_req_raw && (chc_sel_inv ?? chc_wrb_gnt :: prb_res.ready) ||
                   chc_cev_req_raw &&  chc_cev_gnt ||
                   chc_dev_req_raw &&  chc_dev_gnt

    chc_inv     := chc_prb && chc_sel_inv

    // d channel
    chc_cev_gnt := i.d.ready

    // priority: cev/dev > acq
    val llc_res_sel_chd = Pin(Bool())
    val llc_res_sel_pxd = Pin(Bool())

    val chd_sel_dev = chd_acq_req && llc_res.bits.wnr && !llc_res_sel_pxd
    val chd_sel_pxd = chd_acq_req && llc_res.bits.wnr &&  llc_res_sel_pxd

    chd_acq_req := llc_res.valid &&  llc_res_sel_chd
    chd_acq_gnt := chc_cev_gnt   && !chc_cev_req

    i.d.valid   := chc_cev_req || chd_acq_req
    i.d.bits    := chc_cev_req ??
                       ie.ReleaseAck(i.c.bits.source,
                                     i.c.bits.size,
                                     false.B) ::
                   chd_sel_dev ??
                       ie.ReleaseAck(llc_res.bits.idx,
                                     P.clWid.U,
                                     llc_res.bits.err) ::
                   chd_sel_pxd ??
                       ie.AccessAck (llc_res.bits.idx,
                                     P.clWid.U,
                                     llc_res.bits.err) ::
                       ie.Grant(llc_res.bits.idx,
                                llc_res.bits.idx,
                                P.clWid.U,
                                TLPermissions.toT,
                                llc_res.bits.data,
                                llc_res.bits.err,
                                llc_res.bits.err)

    // e channel
    i.e.ready   := true.B


    //
    // mmu

    // llc req
    // priority: wrb > dev > acq
    chc_wrb_gnt := llc_req.ready
    chc_dev_gnt := chc_wrb_gnt && !chc_wrb_req
    cha_acq_gnt := chc_dev_gnt && !chc_dev_req

    val llc_sel_chc = chc_dev_req ||
                      chc_wrb_req

    llc_req.valid := llc_sel_chc ||
                     cha_acq_req ||
                     cha_pfd_req ||
                     cha_ppd_req

    llc_req.bits  := llc_sel_chc ??
                         backside.MemReq(P,
                                         i.c.bits.source,
                                         true.B,
                                         i.c.bits.address(P.maBits := P.clWid),
                                         i.c.bits.address(P.maBits := P.clWid),
                                         i.c.bits.data,
                                         P.llcIdx) ::
                         backside.MemReq(P,
                                         i.a.bits.source,
                                         cha_pfd_req_raw ||
                                         cha_ppd_req_raw,
                                         i.a.bits.address(P.maBits := P.clWid),
                                         i.a.bits.address(P.maBits := P.clWid),
                                         i.a.bits.data,
                                         P.llcIdx)

    val llc_req_sel_pxd = cha_acq_gnt && (cha_pfd_req ||
                                          cha_ppd_req)

    val llc_req_sel_chd = chc_dev_gnt &&  chc_dev_req ||
                          cha_acq_gnt &&  cha_acq_req ||
                          llc_req_sel_pxd

    val llc_dec_req  = Dec(llc_req.bits.idx)(M.W)
    val llc_dec_res  = Dec(llc_res.bits.idx)(M.W)

    val llc_sel_req  = EnQ(llc_req.fire, llc_dec_req)
    val llc_sel_res  = EnQ(llc_res.fire, llc_dec_res)

    llc_res_sel_chd := OrM(llc_dec_res,
                           Seq.tabulate(M) { i =>
                             Src(llc_sel_req(i), llc_sel_res(i), llc_req_sel_chd)
                           })

    llc_res_sel_pxd := OrM(llc_dec_res,
                           Seq.tabulate(M) { i =>
                             Src(llc_sel_req(i), llc_sel_res(i), llc_req_sel_pxd)
                           })

    // llc resp
    llc_res.ready := llc_res_sel_chd && chd_acq_gnt ||
                    !llc_res_sel_chd

    // prb req
    prb_req.ready := chb_prb_gnt

    // prb resp
    prb_res.valid := chc_mis_req || chc_hit_req
    prb_res.bits  := backside.LLCRes(P,
                                     chc_hit_req,
                                     i.c.bits.data)
  }
}
