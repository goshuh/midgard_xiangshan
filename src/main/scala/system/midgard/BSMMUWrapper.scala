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
                       supportsGet        = TransferSizes(1,  8),
                       supportsPutFull    = TransferSizes(1, 64),
                       supportsPutPartial = TransferSizes(1, 16),
                       fifoId             = Some(0),
                       resources          = new SimpleDevice("midgard", Seq("midgard.bmmu")).reg)),
                     beatBytes = 64)))

  val adp_node = TLAdapterNode(
    clientFn  = { cp =>
      cp.v1copy(
        clients = cp.clients.map { c =>
          c.v1copy(
            supportsProbe = TransferSizes.none,
            sourceId      = IdRange(0, if (Q.bsSkip) c.sourceId.end else Q.mrqWays))
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
        endSinkId = if (Q.bsSkip) mp.endSinkId.max(1) else Q.mrqWays,
        beatBytes = 64)
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

    val P = Q.copy(llcIdx = N)

    val u_mmu = Module(new backside.MMU(P))

    val llc_req   = u_mmu.llc_req_i
    val llc_resp  = u_mmu.llc_resp_o
    val prb_req   = u_mmu.llc_req_o
    val prb_resp  = u_mmu.llc_resp_i
    val stb_req   = u_mmu.stb_req_i
    val stb_resp  = u_mmu.stb_resp_o
    val mem_req   = u_mmu.mem_req_o
    val mem_resp  = u_mmu.mem_resp_i


    //
    // ctl

    val ctl_req   = c.a.fire
    val ctl_resp  = c.d.fire

    val ctl_cfg   = Non(c.a.bits.address(11, 7))
    val ctl_sel   = Dec(c.a.bits.address( 6, 3))
    val ctl_rnw   = c.a.bits.opcode === TLMessages.Get

    val ctl_rnw_q = RegEnable(ctl_rnw,          ctl_req)
    val ctl_src_q = RegEnable(c.a.bits.source,  ctl_req)

    // body
    val ctl_q     = dontTouch(Wire(Vec(16, UInt(64.W))))

    val cfg_wen   = ctl_req && ctl_cfg && !ctl_rnw
    val cfg_wdata = OrM(ctl_sel(0 :+ 8) |
                        ctl_sel(8 :+ 8),
                        Div(c.a.bits.data, 64))

    val cfg_rdata = ctl_sel(10) ?? u_mmu.deq_head_o :: OrM(ctl_sel, ctl_q)

    for (i <- 0 until 16) {
      val wen = cfg_wen && ctl_sel(i)

      i match {
        case 0 =>
          ctl_q(i) := RegEnable(cfg_wdata, P.ctlInit.U, wen)
        case 7 | 10 =>
          ctl_q(i) := 0.U
        case x if x > 11 =>
          ctl_q(i) := 0.U
        case _ =>
          ctl_q(i) := RegEnable(cfg_wdata,              wen)
      }
    }

    val inv_wen  = dontTouch(Wire(Bool()))
    val deq_wen  = dontTouch(Wire(Bool()))
    val cfg_norm = ctl_req && !inv_wen && !deq_wen

    val cfg_resp_vld = RegEnable(cfg_norm, false.B, ctl_req || ctl_resp)
    val cfg_resp     = ctl_rnw_q ??
                           ce.AccessAck(ctl_src_q,
                                        3.U,
                                        Rep(RegNext(cfg_rdata), 8)) ::
                           ce.AccessAck(ctl_src_q,
                                        3.U)

    u_mmu.ctl_i := ctl_q
    u_mmu.rst_i := RegNext(cfg_wen && Any(ctl_sel(9, 8))) ##
                   RegNext(cfg_wen && Any(ctl_sel(6, 0)))

    // inv
    inv_wen := cfg_wen && ctl_sel(7)

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
        inv_fsm_en  := c.d.ready
        inv_fsm_nxt := fsm_idle
      }
    }

    val inv_fsm_is_idle = inv_fsm_q === fsm_idle
    val inv_fsm_is_req  = inv_fsm_q === fsm_req
    val inv_fsm_is_pend = inv_fsm_q === fsm_pend
    val inv_fsm_is_resp = inv_fsm_q === fsm_resp

    inv_fsm_q := RegEnable(inv_fsm_nxt, fsm_idle,          inv_fsm_en)
    inv_mcn_q := RegEnable(cfg_wdata(P.maBits := P.clWid), inv_wen)

    val inv_resp_vld = inv_fsm_is_resp
    val inv_resp     = ce.AccessAck(ctl_src_q,
                                    3.U)

    // deq
    val deq_sel = Seq.tabulate(P.deqWays) { i =>
      c.a.bits.address(11, 6) === (i + 2).U
    }.U

    deq_wen := ctl_req && !ctl_rnw && Any(deq_sel)

    assert(deq_wen -> ((c.a.bits.size === 4.U) ||
                       (c.a.bits.size === 6.U)))

    // deq fsm is, e.g., writing mem
    val deq_busy = Any(deq_sel & u_mmu.deq_busy_o) ||
                   RegEnable(stb_req.fire && !stb_resp.fire,
                             false.B,
                             stb_req.fire ||  stb_resp.fire)

    stb_req.valid  := deq_wen
    stb_req.bits   := backside.MemReq(P,
                                      Enc(deq_sel),
                                      false.B,
                                      0.U,
                                      0.U,
                                      c.a.bits.data,
                                      P.deqIdx)

    stb_resp.ready := c.d.ready

    val deq_resp_vld = stb_resp.valid
    val deq_resp     = ce.AccessAck(ctl_src_q,
                                    stb_resp.bits.rnw ?? 4.U :: P.clWid.U)

    // force serialization
    c.a.ready   := inv_fsm_is_idle && !deq_busy

    c.d.valid   := cfg_resp_vld || inv_resp_vld || deq_resp_vld
    c.d.bits    := cfg_resp_vld ?? cfg_resp ::
                   inv_resp_vld ?? inv_resp ::
                                   deq_resp


    //
    // mem

    o.a.valid   := mem_req.valid
    o.a.bits    := mem_req.bits.rnw ??
                       oe.Get(mem_req.bits.idx,
                              mem_req.bits.pcn ## 0.U(P.clWid.W),
                              P.clWid.U)._2 ::
                       oe.Put(mem_req.bits.idx,
                              mem_req.bits.pcn ## 0.U(P.clWid.W),
                              P.clWid.U,
                              mem_req.bits.data)._2

    mem_req.ready  := o.a.ready

    mem_resp.valid := o.d.valid
    mem_resp.bits  := backside.MemResp(P,
                                       o.d.bits.source,
                                       o.d.bits.corrupt || o.d.bits.denied,
                                       o.d.bits.opcode === TLMessages.AccessAckData,
                                       o.d.bits.data)

    o.d.ready   := mem_resp.ready


    //
    // llc

    val cha_acq_req_raw = i.a.bits.opcode === TLMessages.AcquireBlock
    val chc_mis_req_raw = i.c.bits.opcode === TLMessages.ProbeAck
    val chc_hit_req_raw = i.c.bits.opcode === TLMessages.ProbeAckData
    val chc_cev_req_raw = i.c.bits.opcode === TLMessages.Release
    val chc_dev_req_raw = i.c.bits.opcode === TLMessages.ReleaseData

    // encoding doesn't work
    val chc_sel_inv = dontTouch(Wire(Bool()))

    val cha_acq_req = i.a.valid && cha_acq_req_raw
    val chc_mis_req = i.c.valid && chc_mis_req_raw && !chc_sel_inv
    val chc_hit_req = i.c.valid && chc_hit_req_raw && !chc_sel_inv
    val chc_inv_req = i.c.valid && chc_mis_req_raw &&  chc_sel_inv
    val chc_wrb_req = i.c.valid && chc_hit_req_raw &&  chc_sel_inv
    val chc_cev_req = i.c.valid && chc_cev_req_raw
    val chc_dev_req = i.c.valid && chc_dev_req_raw

    // no other possibilities
    assert(i.a.valid ->  cha_acq_req_raw)
    assert(i.c.valid -> (chc_mis_req_raw ||
                         chc_hit_req_raw ||
                         chc_cev_req_raw ||
                         chc_dev_req_raw))

    val cha_acq_gnt = dontTouch(Wire(Bool()))
    val chb_prb_gnt = dontTouch(Wire(Bool()))
    val chc_wrb_gnt = dontTouch(Wire(Bool()))
    val chc_cev_gnt = dontTouch(Wire(Bool()))
    val chc_dev_gnt = dontTouch(Wire(Bool()))

    // a channel
    i.a.ready   := cha_acq_gnt

    // b channel
    val chb_inv_req = inv_fsm_is_req
    val chb_prb_req = prb_req.valid

    // priority: inv > prb
    chb_inv_gnt := i.b.ready
    chb_prb_gnt := chb_inv_gnt && !chb_inv_req

    chc_sel_inv := Src(i.b.fire, i.c.fire, chb_inv_req)

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

    chc_inv     := i.c.fire && chc_sel_inv && (chc_mis_req_raw || chc_hit_req_raw)

    // d channel
    chc_cev_gnt := i.d.ready

    // priority: cev/dev > acq
    val llc_resp_sel_chd = dontTouch(Wire(Bool()))

    val chd_ack_req = llc_resp.valid && llc_resp_sel_chd
    val chd_ack_gnt = chc_cev_gnt && !chc_cev_req

    val chd_sel_dev = chd_ack_req && !llc_resp.bits.rnw

    i.d.valid   := chc_cev_req || chd_ack_req
    i.d.bits    := chc_cev_req ??
                       ie.ReleaseAck(i.c.bits.source,
                                     P.clWid.U,
                                     false.B) ::
                   chd_sel_dev ??
                       ie.ReleaseAck(llc_resp.bits.idx,
                                     P.clWid.U,
                                     llc_resp.bits.err) ::
                       ie.Grant(if (P.bsSkip) 0.U else llc_resp.bits.idx,
                                llc_resp.bits.idx,
                                P.clWid.U,
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

    val llc_sel_chc = chc_dev_req || chc_wrb_req

    llc_req.valid  := llc_sel_chc || cha_acq_req
    llc_req.bits   := llc_sel_chc ??
                          backside.MemReq(P,
                                          i.c.bits.source,
                                          false.B,
                                          i.c.bits.address(P.maBits := P.clWid),
                                          0.U,
                                          i.c.bits.data,
                                          P.llcIdx) ::
                          backside.MemReq(P,
                                          i.a.bits.source,
                                          true.B,
                                          i.a.bits.address(P.maBits := P.clWid),
                                          0.U,
                                          0.U,
                                          P.llcIdx)

    val llc_req_sel_chd = chc_dev_req && chc_dev_gnt ||
                          cha_acq_req && cha_acq_gnt

    llc_resp_sel_chd := OrM(Dec(llc_resp.bits.idx)(M.W),
                            Seq.tabulate(M) { i =>
                              Src(llc_req.fire  && (llc_req.bits.idx  === i.U),
                                  llc_resp.fire && (llc_resp.bits.idx === i.U),
                                  llc_req_sel_chd)
                            })

    // llc resp
    llc_resp.ready := llc_resp_sel_chd && chd_ack_gnt ||
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
