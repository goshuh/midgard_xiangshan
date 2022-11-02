package system

import  chisel3._
import  chisel3.util._
import  midgard.util._

import  chipsalliance.rocketchip.config._

import  freechips.rocketchip.config._
import  freechips.rocketchip.diplomacy._
import  freechips.rocketchip.tilelink._
import  freechips.rocketchip.util._


case class ESerialParam(
  en:      Boolean = true,  

  paBits:  Int     = 48,
  clBits:  Int     = 512,

  srcBits: Int     = 2,

  ctlBase: BigInt,
  ctlSize: BigInt,

  harts:   Int) {

  val clBytes = clBits / 8
  val clWid   = log2Ceil(clBytes)

  val pcnBits = paBits - clWid

  val dsqIdx  = log2Ceil(harts).max(1)
}

case object ESerialKey extends Field[ESerialParam]


class ESerial(implicit p: Parameters) extends LazyModule {
  val Q = p(ESerialKey)

  val ctl_node = TLManagerNode(
                   Seq(TLSlavePortParameters.v1(
                     Seq(TLSlaveParameters.v1(
                       address            =  Seq(AddressSet(Q.ctlBase, Q.ctlSize)),
                       regionType         =  RegionType.UNCACHED,
                       supportsGet        =  TransferSizes(1,  8),
                       supportsPutFull    =  TransferSizes(1, 64),
                       supportsPutPartial =  TransferSizes(1, 16),
                       fifoId             =  Some(0),
                       resources          =  new SimpleDevice("eserial", Seq("eserial")).reg)),
                     beatBytes = 64)))

  val mst_node = TLClientNode(
                   Seq(TLMasterPortParameters.v1(
                     Seq(TLMasterParameters.v1(
                       name     = "eserial",
                       sourceId =  IdRange(0, Q.harts),
                     )))))

  lazy val module = new LazyModuleImp(this) {
    val (c, ce) = ctl_node.in.head
    val (m, me) = mst_node.out.head


    //
    // dsq

    val N = c.params.sourceBits
    val P = Q.copy(srcBits = N)

    val u_dsq     = Module(new DSQ(P))

    val des_req   = u_dsq.des_req_i
    val des_resp  = u_dsq.des_resp_o
    val ser_req   = u_dsq.ser_req_o
    val ser_resp  = u_dsq.ser_resp_i
    val prb_req   = m.b.fire
    val prb_resp  = m.c.fire

    val ser_addr  = ser_req.bits.pcn ## 0.U(P.clWid.W)

    ser_req.ready  := m.a.ready

    ser_resp.valid := m.d.valid
    ser_resp.bits  := MemResp(P,
                              m.d.bits.source,
                              false.B)

    m.a.valid      := ser_req.valid
    m.a.bits       := ser_req.bits.sel ??
                          me.Put(ser_req.bits.idx,
                                 ser_addr,
                                 4.U,
                                 ser_req.bits.data,
                                 MaskGen(ser_addr, 4.U, 64))._2 ::
                          me.Put(ser_req.bits.idx,
                                 ser_addr,
                                 P.clWid.U,
                                 ser_req.bits.data)._2

    m.d.ready      := ser_resp.ready


    //
    // ctl

    val ctl_req   = c.a.fire
    val ctl_resp  = c.d.fire

    val ctl_norm  = Non(c.a.bits.address(11, 7))
    val ctl_sel   = Dec(c.a.bits.address( 5, 3))
    val ctl_rnw   = c.a.bits.opcode === TLMessages.Get

    val ctl_rnw_q = RegEnable(ctl_rnw,         ctl_req)
    val ctl_src_q = RegEnable(c.a.bits.source, ctl_req)

    // body
    val ctl_q     = dontTouch(Wire(Vec(4, UInt(64.W))))

    val ctl_wen   = ctl_req && ctl_norm && !ctl_rnw
    val ctl_wdata = OrM(ctl_sel, Div(c.a.bits.data(256.W), 64))
    val ctl_rdata = OrM(ctl_sel, ctl_q)

    for (i <- 0 until 4) {
      val wen = ctl_wen && ctl_sel(i)

      i match {
        case 2 =>
          ctl_q(i) := u_dsq.dsq_head_o
        case _ =>
          ctl_q(i) := RegEnable(ctl_wdata, wen)
      }
    }

    val dsq_sel  = Seq.tabulate(P.harts) { i =>
      c.a.bits.address(11, 6) === (i + 2).U
    }.U
    val dsq_wen  = ctl_req && !ctl_norm && !ctl_rnw

    assert(dsq_wen -> (Any(dsq_sel) &&
                         ((c.a.bits.size === 4.U) ||
                          (c.a.bits.size === 6.U))))

    val ctl_norm_q    = RegEnable(ctl_req && ctl_norm,
                                  false.B,
                                  ctl_req || ctl_resp)

    val ctl_norm_resp = ctl_rnw_q ??
                            ce.AccessAck(ctl_src_q,
                                         3.U,
                                         RegEnable(ctl_rdata, ctl_req)) ::
                            ce.AccessAck(ctl_src_q,
                                         3.U)

    // dsq fsm is, e.g., writing mem
    val dsq_busy = Any(dsq_sel & u_dsq.dsq_busy_o) ||
                   RegEnable(des_req.fire && !des_resp.fire,
                             false.B,
                             des_req.fire ||  des_resp.fire)

    des_req.valid  := dsq_wen && !dsq_busy
    des_req.bits   := MemReq(P,
                             c.a.bits.source,
                             false.B,
                             dsq_sel,
                             c.a.bits.data,
                             P.srcBits)

    des_resp.ready := c.d.ready && !ctl_norm_q

    c.a.ready      := ctl_norm  ||
                     !ctl_norm  && !dsq_busy && des_req.ready

    c.d.valid      := ctl_norm_q || des_resp.valid
    c.d.bits       := ctl_norm_q ??
                          ctl_norm_resp ::
                          ce.AccessAck(des_resp.bits.idx,
                                       des_resp.bits.sel ?? 4.U :: P.clWid.U)

    u_dsq.ctl_i    := ctl_q
    u_dsq.rst_i    := ctl_wen && Any(ctl_sel(2.W))
  }
}


class MemReq  (val P: ESerialParam, val w: Int = 0) extends Bundle {
  val wid  = if (w <= 0) P.dsqIdx else w
  val idx  = UInt(wid.W)
  val sel  = Bool()
  val pcn  = UInt(P.pcnBits.W)
  val data = UInt(P.clBits.W)
}

class MemResp (val P: ESerialParam, val w: Int = 0) extends Bundle {
  val wid  = if (w <= 0) P.dsqIdx else w
  val idx  = UInt(wid.W)
  val sel  = Bool()
}

object MemReq {
  def apply(P: ESerialParam, i: UInt, s: Bool, p: UInt, d: UInt, w: Int = 0): MemReq = {
    val ret = Wire(new MemReq(P, w))

    ret.idx  := i
    ret.sel  := s
    ret.pcn  := p
    ret.data := d
    ret
  }
}

object MemResp {
  def apply(P: ESerialParam, i: UInt, s: Bool, w: Int = 0): MemResp = {
    val ret = Wire(new MemResp(P, w))

    ret.idx  := i
    ret.sel  := s
    ret
  }
}


class DSQEntry(val P: ESerialParam) extends Bundle {
  val wid  = 31 - P.clWid
  val fsm  = UInt(2.W)
  val sel  = Bool()
  val ptr  = UInt(wid.W)
  val idx  = UInt(P.srcBits.W)
  val pcn  = UInt(P.pcnBits.W)
  val data = UInt(P.clBits.W)
  val mask = UInt(P.clBytes.W)
}


class DSQ(val P: ESerialParam) extends Module {

  // ---------------------------
  // io

  val des_req_i  = IO(Flipped(Decoupled(new MemReq (P, P.srcBits))))
  val des_resp_o = IO(        Decoupled(new MemResp(P, P.srcBits)))

  val ser_req_o  = IO(        Decoupled(new MemReq (P)))
  val ser_resp_i = IO(Flipped(Decoupled(new MemResp(P))))

  val dsq_busy_o = IO(           Output(UInt(P.harts.W)))
  val dsq_head_o = IO(           Output(UInt(32.W)))

  val ctl_i      = IO(            Input(Vec (4, UInt(64.W))))
  val rst_i      = IO(            Input(Bool()))


  // ---------------------------
  // logic

  val
     (fsm_idle ::
      fsm_req  ::
      fsm_pend ::
      fsm_resp ::
      fsm_null) = Enum(4)

  val des_req      = des_req_i.fire
  val des_resp     = des_resp_o.fire
  val ser_req      = ser_req_o.fire
  val ser_resp     = ser_resp_i.fire

  // re-purpose
  val des_req_sel  = des_req_i.bits.pcn(P.harts.W)
  val ser_resp_sel = Dec(ser_resp_i.bits.idx)


  //
  // slots

  val dsq_vld     = dontTouch(Wire(Vec (P.harts, Bool())))
  val dsq_busy    = dontTouch(Wire(Vec (P.harts, Bool())))
  val dsq_ser_req = dontTouch(Wire(Vec (P.harts, Bool())))
  val dsq_des_req = dontTouch(Wire(Vec (P.harts, Bool())))

  val dsq_ser_sel = dontTouch(Wire(UInt(P.harts.W)))
  val dsq_des_sel = dontTouch(Wire(UInt(P.harts.W)))

  val dsq_set     = dontTouch(Wire(Vec (P.harts, Bool())))
  val dsq_clr     = dontTouch(Wire(Vec (P.harts, Bool())))

  val int_head_q  = dontTouch(Wire(UInt(32.W)))
  val ext_head_q  = dontTouch(Wire(UInt(32.W)))

  // body
  val dsq_q = dontTouch(Wire(Vec(P.harts, new DSQEntry(P))))
  val age_q = dontTouch(Wire(Vec(P.harts, Vec(P.harts, Bool()))))

  for (i <- 0 until P.harts) {
    // age matrix
    for (j <- 0 until P.harts) {
      if (i == j)
        age_q(i)(j) := false.B
      else if (i > j)
        age_q(i)(j) := Non(age_q(j)(i))
      else
        age_q(i)(j) := RegEnable(dsq_set(i),
                                 false.B,
                                 dsq_set(i) || dsq_set(j))
    }

    val set = des_req  && des_req_sel(i)
    val clr = des_resp && dsq_des_sel(i)

    val dsq_fsm_en  = dontTouch(Wire(Bool()))
    val dsq_fsm_nxt = dontTouch(Wire(UInt(2.W)))

    dsq_fsm_en  := false.B
    dsq_fsm_nxt := dsq_q(i).fsm

    switch (dsq_q(i).fsm) {
      is (fsm_idle) {
        dsq_fsm_en  := set
        dsq_fsm_nxt := fsm_req
      }
      is (fsm_req) {
        dsq_fsm_en  := ser_req_o.ready  && dsq_ser_sel (i)
        dsq_fsm_nxt := fsm_pend
      }
      is (fsm_pend) {
        dsq_fsm_en  := ser_resp_i.valid && ser_resp_sel(i)
        dsq_fsm_nxt := fsm_resp
      }
      is (fsm_resp) {
        dsq_fsm_en  := clr
        dsq_fsm_nxt := fsm_idle
      }
    }

    val dsq_fsm_is_busy = dsq_q(i).fsm =/= fsm_idle
    val dsq_fsm_is_req  = dsq_q(i).fsm === fsm_req
    val dsq_fsm_is_pend = dsq_q(i).fsm === fsm_pend
    val dsq_fsm_is_resp = dsq_q(i).fsm === fsm_resp

    // lock-step
    assert(set -> !dsq_fsm_is_busy)

    // make life easier
    dsq_q(i).fsm   := RegEnable(dsq_fsm_nxt, fsm_idle, dsq_fsm_en)

    val dsq_fst = !dsq_q(i).sel
    val dsq_snd =  dsq_q(i).sel

    // variable fields
    val dsq_tog      = clr && dsq_fsm_is_resp
    val dsq_set_data = set && dsq_fst
    val dsq_set_misc = set && dsq_snd
    val dsq_clr_all  = clr && dsq_snd

    dsq_q(i).sel   := RegEnable(dsq_fst, false.B,    dsq_tog)
    dsq_q(i).ptr   := RegEnable(int_head_q,          dsq_set_data)
    dsq_q(i).idx   := RegEnable(des_req_i.bits.idx,  set)
    dsq_q(i).data  := RegEnable(des_req_i.bits.data, dsq_set_data)

    dsq_q(i).pcn   := RegEnable(des_req_i.bits.data(P.paBits := P.clWid),   dsq_set_misc)
    dsq_q(i).mask  := RegEnable(des_req_i.bits.data(64       :+ P.clBytes), dsq_set_misc)

    // output
    dsq_vld    (i) := dsq_fsm_is_busy ||  dsq_snd
    dsq_busy   (i) := dsq_fsm_is_busy
    dsq_ser_req(i) := dsq_fsm_is_req

    // bring back global order again
    dsq_des_req(i) := dsq_fsm_is_resp && (dsq_fst || Non(dsq_vld.U & age_q(i).U))

    dsq_set    (i) := dsq_set_data
    dsq_clr    (i) := dsq_clr_all
  }

  int_head_q := RegEnable(NeQ(rst_i, (int_head_q + 1.U) & ctl_i(1)(32.W)),
                          0.U,
                          rst_i || Any(dsq_set))

  // global order
  ext_head_q := RegEnable(NeQ(rst_i, (ext_head_q + 1.U) & ctl_i(1)(32.W)),
                          0.U,
                          rst_i || Any(dsq_clr))

  // arb
  val ser_req_vld   = Any(dsq_ser_req)
  val des_req_vld   = Any(dsq_des_req)
  val arb_ser_req   = RRA(dsq_ser_req.U, ser_req)
  val arb_des_req   = RRA(dsq_des_req.U, des_resp)

  // stability
  val ser_req_vld_q = RegEnable(ser_req_vld && !ser_req_o.ready,  false.B, ser_req_vld)
  val des_req_vld_q = RegEnable(des_req_vld && !des_resp_o.ready, false.B, des_req_vld)
  val arb_ser_req_q = RegEnable(arb_ser_req, ser_req_vld && !ser_req_vld_q)
  val arb_des_req_q = RegEnable(arb_des_req, des_req_vld && !ser_req_vld_q)

  dsq_ser_sel := ser_req_vld_q ?? arb_ser_req_q :: arb_ser_req
  dsq_des_sel := des_req_vld_q ?? arb_des_req_q :: arb_des_req

  // big partial mux
  val dsq_ser_mux = OrM(dsq_ser_sel, dsq_q)
  val dsq_des_mux = OrM(dsq_des_sel, dsq_q)

  // output
  des_req_i.ready  := true.B

  des_resp_o.valid := des_req_vld
  des_resp_o.bits  := MemResp(P,
                              dsq_des_mux.idx,
                              dsq_des_mux.sel,
                              P.srcBits)

  // os should guarantee that the region is properly aligned
  val ser_req_pcn   = ctl_i(0)(P.paBits := P.clWid) | (dsq_ser_mux.ptr ## dsq_ser_mux.sel)
  val ser_req_data  = dsq_ser_mux.sel ?? (dsq_ser_mux.mask ## Ext(dsq_ser_mux.pcn ## 0.U(P.clWid.W), 64)) ::
                                          dsq_ser_mux.data

  ser_req_o.valid  := ser_req_vld
  ser_req_o.bits   := MemReq (P,
                              Enc(dsq_ser_sel),
                              dsq_ser_mux.sel,
                              ser_req_pcn,
                              ser_req_data)

  ser_resp_i.ready := true.B

  dsq_busy_o       := dsq_busy.U
  dsq_head_o       := ext_head_q
}