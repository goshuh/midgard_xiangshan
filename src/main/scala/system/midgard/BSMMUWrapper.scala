package system

import  chisel3._
import  chisel3.util._
import  midgard._
import  midgard.util._

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
                       resources          = new SimpleDevice("midgard", Seq("midgard.bmmu")).reg)),
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
            sourceId      = IdRange(0, if (Q.bsSkip) llc_end_idx else Q.mrqWays))
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
        endSinkId  = if (Q.bsSkip) llc_end_idx else Q.mrqWays,
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
       fsm_resp ::
       fsm_null) = Enum(4)


    //
    // inst

    val N = ie.bundle.sourceBits
    val M = ie.client.clients.head.sourceId.end

    val P = Q.copy(llcIdx  = N,
                   mrqWays = if (Q.bsSkip) 1 << N else Q.mrqWays)

    val u_mmu = Module(new backside.MMU(P))

    val llc_req   = u_mmu.llc_req_i
    val llc_resp  = u_mmu.llc_resp_o
    val prb_req   = u_mmu.llc_req_o
    val prb_resp  = u_mmu.llc_resp_i
    val mem_req   = u_mmu.mem_req_o
    val mem_resp  = u_mmu.mem_resp_i


    //
    // ctl

    val ctl_req   = c.a.fire
    val ctl_resp  = c.d.fire

    val ctl_sel   = Dec(c.a.bits.address(6, 3))
    val ctl_rnw   = c.a.bits.opcode === TLMessages.Get

    val ctl_rnw_q = RegEnable(ctl_rnw,         ctl_req)
    val ctl_src_q = RegEnable(c.a.bits.source, ctl_req)

    // body
    val ctl_q     = dontTouch(Wire(Vec(8, UInt(64.W))))

    val ctl_wen   = ctl_req && !ctl_rnw
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

    val ctl_norm_q    = RegEnable(ctl_req && !inv_wen,
                                  false.B,
                                  ctl_req ||  ctl_resp)

    val ctl_norm_resp = ctl_rnw_q ??
                            ce.AccessAck(ctl_src_q,
                                         3.U,
                                         RegEnable(ctl_rdata, ctl_req)) ::
                            ce.AccessAck(ctl_src_q,
                                         3.U)

    u_mmu.ctl_i := ctl_q
    u_mmu.rst_i := RegNext(ctl_wen && Any(ctl_sel(6.W)))

    // inv
    val inv_fsm_en  = dontTouch(Wire(Bool()))
    val inv_fsm_nxt = dontTouch(Wire(UInt(2.W)))
    val inv_fsm_q   = dontTouch(Wire(UInt(2.W)))
    val inv_mcn_q   = dontTouch(Wire(UInt(P.mcnBits.W)))

    val chb_inv_gnt = dontTouch(Wire(Bool()))
    val chc_inv     = dontTouch(Wire(Bool()))

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
        inv_fsm_nxt := fsm_resp
      }
      is (fsm_resp) {
        inv_fsm_en  := c.d.ready && !ctl_norm_q
        inv_fsm_nxt := fsm_idle
      }
    }

    val inv_fsm_is_idle = inv_fsm_q === fsm_idle
    val inv_fsm_is_req  = inv_fsm_q === fsm_req
    val inv_fsm_is_pend = inv_fsm_q === fsm_pend
    val inv_fsm_is_resp = inv_fsm_q === fsm_resp

    inv_fsm_q := RegEnable(inv_fsm_nxt, fsm_idle,          inv_fsm_en)
    inv_mcn_q := RegEnable(ctl_wdata(P.maBits := P.clWid), inv_wen)

    val inv_resp_vld = inv_fsm_is_resp
    val inv_resp     = ce.AccessAck(ctl_src_q,
                                    3.U)

    // force serialization
    c.a.ready   := inv_fsm_is_idle

    c.d.valid   := ctl_norm_q || inv_resp_vld
    c.d.bits    := ctl_norm_q ?? ctl_norm_resp :: inv_resp


    //
    // mem

    val mem_req_qual = dontTouch(Wire(Bool()))

    if (P.bsSkip) {
      val mem_scb_q  = dontTouch(Wire(Vec(M, Bool())))

      val mem_dec_req  = Dec(o.a.bits.source)
      val mem_dec_resp = Dec(o.d.bits.source)

      for (i <- 0 until M) {
        val set = o.a.fire && mem_dec_req (i)
        val clr = o.d.fire && mem_dec_resp(i)

        mem_scb_q(i) := RegEnable(set && !clr,
                                  false.B,
                                  set ||  clr)
      }

      mem_req_qual := Non(mem_scb_q.U & mem_dec_req)

    } else {
      mem_req_qual := false.B
    }

    o.a.valid   := mem_req.valid && mem_req_qual
    o.a.bits    := mem_req.bits.rnw ??
                       oe.Get(mem_req.bits.idx,
                              mem_req.bits.pcn ## 0.U(P.clWid.W),
                              mem_req.bits.siz)._2 ::
                       oe.Put(mem_req.bits.idx,
                              mem_req.bits.pcn ## 0.U(P.clWid.W),
                              mem_req.bits.siz,
                              mem_req.bits.data)._2

    mem_req.ready  := o.a.ready  && mem_req_qual

    mem_resp.valid := o.d.valid
    mem_resp.bits  := backside.MemResp(P,
                                       o.d.bits.source,
                                       o.d.bits.corrupt || o.d.bits.denied,
                                       o.d.bits.opcode === TLMessages.AccessAckData,
                                       o.d.bits.size,
                                       o.d.bits.data)

    o.d.ready   := mem_resp.ready


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
    val chc_sel_inv = dontTouch(Wire(Bool()))

    val cha_acq_req = i.a.valid && cha_acq_req_raw
    val cha_pfd_req = i.a.valid && cha_pfd_req_raw
    val cha_ppd_req = i.a.valid && cha_ppd_req_raw
    val chc_mis_req = i.c.valid && chc_mis_req_raw && !chc_sel_inv
    val chc_hit_req = i.c.valid && chc_hit_req_raw && !chc_sel_inv
    val chc_inv_req = i.c.valid && chc_mis_req_raw &&  chc_sel_inv
    val chc_wrb_req = i.c.valid && chc_hit_req_raw &&  chc_sel_inv
    val chc_cev_req = i.c.valid && chc_cev_req_raw
    val chc_dev_req = i.c.valid && chc_dev_req_raw
    val chd_acq_req = dontTouch(Wire(Bool()))

    val chc_prb = i.c.fire && (chc_mis_req_raw || chc_hit_req_raw)

    // no other possibilities
    assert(i.a.valid -> (cha_acq_req_raw ||
                         cha_pfd_req_raw ||
                         cha_ppd_req_raw))
    assert(i.c.valid -> (chc_mis_req_raw ||
                         chc_hit_req_raw ||
                         chc_cev_req_raw ||
                         chc_dev_req_raw))

    val cha_acq_gnt = dontTouch(Wire(Bool()))
    val chb_prb_gnt = dontTouch(Wire(Bool()))
    val chc_wrb_gnt = dontTouch(Wire(Bool()))
    val chc_cev_gnt = dontTouch(Wire(Bool()))
    val chc_dev_gnt = dontTouch(Wire(Bool()))
    val chd_acq_gnt = dontTouch(Wire(Bool()))

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
    i.c.ready   := chc_mis_req_raw && (chc_sel_inv ?? true.B      :: prb_resp.ready) ||
                   chc_hit_req_raw && (chc_sel_inv ?? chc_wrb_gnt :: prb_resp.ready) ||
                   chc_cev_req_raw &&  chc_cev_gnt ||
                   chc_dev_req_raw &&  chc_dev_gnt

    chc_inv     := chc_prb && chc_sel_inv

    // d channel
    chc_cev_gnt := i.d.ready

    // priority: cev/dev > acq
    val llc_resp_sel_chd = dontTouch(Wire(Bool()))
    val llc_resp_sel_pxd = dontTouch(Wire(Bool()))

    val chd_sel_dev = chd_acq_req && !llc_resp.bits.rnw && !llc_resp_sel_pxd
    val chd_sel_pxd = chd_acq_req && !llc_resp.bits.rnw &&  llc_resp_sel_pxd

    chd_acq_req := llc_resp.valid &&  llc_resp_sel_chd
    chd_acq_gnt := chc_cev_gnt    && !chc_cev_req

    i.d.valid   := chc_cev_req || chd_acq_req
    i.d.bits    := chc_cev_req ??
                       ie.ReleaseAck(i.c.bits.source,
                                     i.c.bits.size,
                                     false.B) ::
                   chd_sel_dev ??
                       ie.ReleaseAck(llc_resp.bits.idx,
                                     llc_resp.bits.siz,
                                     llc_resp.bits.err) ::
                   chd_sel_pxd ??
                       ie.AccessAck (llc_resp.bits.idx,
                                     llc_resp.bits.siz,
                                     llc_resp.bits.err) ::
                       ie.Grant(llc_resp.bits.idx,
                                llc_resp.bits.idx,
                                llc_resp.bits.siz,
                                TLPermissions.toT,
                                llc_resp.bits.data,
                                llc_resp.bits.err,
                                llc_resp.bits.err)

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

    llc_req.valid  := llc_sel_chc ||
                      cha_acq_req ||
                      cha_pfd_req ||
                      cha_ppd_req

    llc_req.bits   := llc_sel_chc ??
                          backside.MemReq(P,
                                          i.c.bits.source,
                                          false.B,
                                          i.c.bits.size,
                                          i.c.bits.address(P.maBits := P.clWid),
                                          0.U,
                                          i.c.bits.data,
                                         ~0.U(P.clBytes.W),
                                          P.llcIdx) ::
                          backside.MemReq(P,
                                          i.a.bits.source,
                                          cha_acq_req_raw,
                                          i.a.bits.size,
                                          i.a.bits.address(P.maBits := P.clWid),
                                          0.U,
                                          i.a.bits.data,
                                          i.a.bits.mask,
                                          P.llcIdx)

    val llc_req_sel_pxd = cha_acq_gnt && (cha_pfd_req ||
                                          cha_ppd_req)

    val llc_req_sel_chd = chc_dev_gnt &&  chc_dev_req ||
                          cha_acq_gnt &&  cha_acq_req ||
                          llc_req_sel_pxd

    val llc_dec_req   = Dec(llc_req.bits.idx )(M.W)
    val llc_dec_resp  = Dec(llc_resp.bits.idx)(M.W)

    val llc_sel_req   = EnQ(llc_req.fire,  llc_dec_req)
    val llc_sel_resp  = EnQ(llc_resp.fire, llc_dec_resp)

    llc_resp_sel_chd := OrM(llc_dec_resp,
                            Seq.tabulate(M) { i =>
                              Src(llc_sel_req(i), llc_sel_resp(i), llc_req_sel_chd)
                            })

    llc_resp_sel_pxd := OrM(llc_dec_resp,
                            Seq.tabulate(M) { i =>
                              Src(llc_sel_req(i), llc_sel_resp(i), llc_req_sel_pxd)
                            })

    // llc resp
    llc_resp.ready := llc_resp_sel_chd && chd_acq_gnt ||
                     !llc_resp_sel_chd

    // prb req
    prb_req.ready  := chb_prb_gnt

    // prb resp
    prb_resp.valid := chc_mis_req || chc_hit_req
    prb_resp.bits  := backside.LLCResp(P,
                                       chc_hit_req,
                                       i.c.bits.data)
  }
}
