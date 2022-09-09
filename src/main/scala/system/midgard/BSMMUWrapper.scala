package system

import  chisel3._
import  chisel3.util._
import  midgard._
import  midgard.util._

import  chipsalliance.rocketchip.config._

import  freechips.rocketchip.config._
import  freechips.rocketchip.diplomacy._
import  freechips.rocketchip.tilelink._


class MidgardBSMMUWrapper(implicit p: Parameters) extends LazyModule{
  val Q = p(MidgardKey)

  val ctl_node = TLManagerNode(
                   Seq(TLSlavePortParameters.v1(
                     Seq(TLSlaveParameters.v1(
                       address         = Seq(AddressSet(Q.ctlBase, Q.ctlSize)),
                       regionType      = RegionType.UNCACHED,
                       supportsGet     = TransferSizes(1, 8),
                       supportsPutFull = TransferSizes(1, 8),
                       fifoId          = Some(0))),
                     beatBytes = 8)))

  val adp_node = TLAdapterNode(
    clientFn  = { cp =>
      cp.v1copy(
        clients = cp.clients.map { c =>
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
            supportsAcquireB = m.supportsGet,
            supportsAcquireT = m.supportsPutFull,
            alwaysGrantsT    = true,
            mayDenyGet       = true,
            mayDenyPut       = true)
        },
        endSinkId = Q.mrqWays)
    })

  lazy val module = new LazyModuleImp(this) {
    val (c, ce) = ctl_node.in .head
    val (i, ie) = adp_node.in .head
    val (o, oe) = adp_node.out.head


    //
    // inst

    val N = ie.bundle.sourceBits
    val P = p(MidgardKey).copy(llcIdx  = N + 1)

    val u_mmu = Module(new backside.MMU(P))

    // for simplicity
    val ctl_req  = u_mmu.ctl_req_i
    val ctl_resp = u_mmu.ctl_resp_o
    val mem_req  = u_mmu.mem_req_o
    val mem_resp = u_mmu.mem_resp_i
    val llc_req  = u_mmu.llc_req_i
    val llc_resp = u_mmu.llc_resp_o
    val prb_req  = u_mmu.llc_req_o
    val prb_resp = u_mmu.llc_resp_i


    //
    // ctl

    val
      (fsm_idle     ::
       fsm_inv_req  ::
       fsm_inv_resp ::
       fsm_resp     ::
       fsm_null) = Enum(4)

    val inv_fsm_en  = dontTouch(Wire(Bool()))
    val inv_fsm_nxt = dontTouch(Wire(UInt(2.W)))
    val inv_fsm_q   = dontTouch(Wire(UInt(2.W)))
    val inv_mcn_q   = dontTouch(Wire(UInt(P.mcnBits.W)))

    val inv_iss     = dontTouch(Wire(Bool()))
    val inv_req     = dontTouch(Wire(Bool()))
    val inv_resp    = dontTouch(Wire(Bool()))

    // default
    inv_fsm_en  := false.B
    inv_fsm_nxt := inv_fsm_q

    switch (inv_fsm_q) {
      is (fsm_idle) {
        inv_fsm_en  := inv_iss
        inv_fsm_nxt := fsm_inv_req
      }
      is (fsm_inv_req) {
        inv_fsm_en  := inv_req
        inv_fsm_nxt := fsm_inv_resp
      }
      is (fsm_inv_resp) {
        inv_fsm_en  := inv_resp
        inv_fsm_nxt := fsm_resp
      }
      is (fsm_resp) {
        inv_fsm_en  := c.d.ready
        inv_fsm_nxt := fsm_idle
      }
    }

    val inv_fsm_is_idle     = inv_fsm_q === fsm_idle
    val inv_fsm_is_inv_req  = inv_fsm_q === fsm_inv_req
    val inv_fsm_is_inv_resp = inv_fsm_q === fsm_inv_resp
    val inv_fsm_is_resp     = inv_fsm_q === fsm_resp

    inv_fsm_q := RegEnable(inv_fsm_nxt, fsm_idle,     inv_fsm_en)
    inv_mcn_q := RegEnable(c.a.bits.data >> P.clWid, inv_iss)

    // flush is always a write to mmio register 8-15
    val ctl_sel_rnw = ctl_resp.valid && ctl_resp.bits.rnw

    inv_iss        := c.a.fire()  &&  c.a.bits.address(6)
    ctl_req.valid  := c.a.valid   && !c.a.bits.address(6)
    ctl_req.bits   := backside.CtlReq(c.a.bits.opcode === TLMessages.Get,
                                      c.a.bits.address(6, 3),
                                      c.a.bits.data)

    // at most a single req
    c.a.ready   := inv_fsm_is_idle && ctl_req.ready

    c.d.valid   := inv_fsm_is_resp || ctl_resp.valid
    c.d.bits    := ctl_sel_rnw ??
                       ce.AccessAck(0.U,
                                    3.U,
                                    ctl_resp.bits.data) ::
                       ce.AccessAck(0.U,
                                    3.U)

    ctl_resp.ready := c.d.ready


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

    // encoding: 0: prb, 1: inv
    val cha_acq_req = i.a.valid && cha_acq_req_raw
    val chc_mis_req = i.c.valid && chc_mis_req_raw && !i.c.bits.source(0)
    val chc_hit_req = i.c.valid && chc_hit_req_raw && !i.c.bits.source(0)
    val chc_inv_req = i.c.valid && chc_mis_req_raw &&  i.c.bits.source(0)
    val chc_wrb_req = i.c.valid && chc_hit_req_raw &&  i.c.bits.source(0)
    val chc_cev_req = i.c.valid && chc_cev_req_raw
    val chc_dev_req = i.c.valid && chc_dev_req_raw

    // no other possibilities
    assert(i.a.valid ->  cha_acq_req_raw)
    assert(i.c.valid -> (chc_mis_req_raw ||
                         chc_hit_req_raw ||
                         chc_cev_req_raw ||
                         chc_dev_req_raw))

    val cha_acq_gnt = dontTouch(Wire(Bool()))
    val chc_prb_gnt = dontTouch(Wire(Bool()))
    val chc_wrb_gnt = dontTouch(Wire(Bool()))
    val chc_cev_gnt = dontTouch(Wire(Bool()))
    val chc_dev_gnt = dontTouch(Wire(Bool()))

    // a channel
    i.a.ready   := cha_acq_gnt

    // b channel
    val chb_inv_req = inv_fsm_is_inv_req
    val chb_prb_req = prb_req.valid

    // priority: inv > prb
    val chb_inv_gnt = i.b.ready
    val chb_prb_gnt = chb_inv_gnt && !chb_inv_req

    i.b.valid   := chb_inv_req || chb_prb_req
    i.b.bits    := chb_inv_req ??
                       ie.Probe(inv_mcn_q ## 0.U(P.clWid.W),
                                1.U,
                                P.clWid.U,
                                TLPermissions.toN)._2 ::
                       ie.Probe(prb_req.bits.mcn ## 0.U(P.clWid.W),
                                0.U,
                                P.clWid.U,
                                TLPermissions.toB)._2

    inv_req     := chb_inv_req && chb_inv_gnt

    // c channel
    i.c.ready   := chc_mis_req_raw && (i.c.bits.source(0) ?? true.B      :: chc_prb_gnt) ||
                   chc_hit_req_raw && (i.c.bits.source(0) ?? chc_wrb_gnt :: chc_prb_gnt) ||
                   chc_cev_req_raw &&  chc_cev_gnt ||
                   chc_dev_req_raw &&  chc_dev_gnt

    inv_resp    := i.c.fire() && (chc_mis_req_raw || chc_hit_req_raw) && i.c.bits.source(0)

    // d channel
    chc_cev_gnt := i.d.ready

    // priority: cev/dev > acq
    val chd_ack_req = llc_resp.valid && llc_resp.bits.idx(N)
    val chd_ack_gnt = chc_cev_gnt && !chc_cev_req

    val chd_sel_dev = chd_ack_req && !llc_resp.bits.rnw

    i.d.valid   := chc_cev_req || chd_ack_req
    i.d.bits    := chc_cev_req ??
                       ie.ReleaseAck(i.c.bits.source,
                                     P.clWid.U,
                                     false.B) ::
                   chd_sel_dev ??
                       ie.ReleaseAck(llc_resp.bits.idx(N.W),
                                     P.clWid.U,
                                     llc_resp.bits.err) ::
                       ie.Grant(llc_resp.bits.idx(N.W),
                                llc_resp.bits.idx(N.W),
                                P.clWid.U,
                                TLPermissions.toT,
                                llc_resp.bits.data,
                                false.B,
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

    val llc_sel_wnr =  chc_wrb_req ||  chc_dev_req
    val llc_sel_ack = !chc_wrb_req && (chc_dev_req || cha_acq_req)

    llc_req.valid  := llc_sel_wnr || cha_acq_req
    llc_req.bits   := llc_sel_wnr ??
                          backside.MemReq(P,
                                          llc_sel_ack ## i.c.bits.source,
                                          false.B,
                                          i.c.bits.address >> P.clWid,
                                          0.U,
                                          i.c.bits.data,
                                          P.llcIdx) ::
                          backside.MemReq(P,
                                          llc_sel_ack ## i.a.bits.source,
                                          true.B,
                                          i.a.bits.address >> P.clWid,
                                          0.U,
                                          0.U,
                                          P.llcIdx)

    // llc resp
    llc_resp.ready := llc_resp.bits.idx(N) && chd_ack_gnt ||
                     !llc_resp.bits.idx(N)

    // prb req
    prb_req.ready  := chb_prb_gnt

    // prb resp
    chc_prb_gnt := prb_resp.ready

    prb_resp.valid := chc_mis_req || chc_hit_req
    prb_resp.bits  := backside.LLCResp(P,
                                       chc_hit_req,
                                       i.c.bits.data)
  }
}