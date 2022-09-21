package system

import  chisel3._
import  chisel3.util._
import  chisel3.util.random._
import  midgard.util._

import  chipsalliance.rocketchip.config._

import  freechips.rocketchip.config._
import  freechips.rocketchip.diplomacy._
import  freechips.rocketchip.tilelink._


case class EInjectParam(
  en:      Boolean = true,

  ctlBase: BigInt,
  ctlSize: BigInt,
  memBase: BigInt,
  memSize: BigInt,

  queue:   Int
)

case object EInjectKey extends Field[EInjectParam]


class EInject(implicit p: Parameters) extends LazyModule {
  val P = p(EInjectKey)

  val dev = new SimpleDevice("berr", Seq("einject")) {
    override def describe(resources: ResourceBindings): Description = {
      val Description(name, mapping) = super.describe(resources)

      Description(name, mapping ++ Map(
        "mem" -> Seq(ResourceAddress(Seq(AddressSet(P.memBase, P.memSize)),
                                     ResourcePermissions(true, true, true, true, false)))
      ))
    }
  }

  val ctl_node = TLManagerNode(
                   Seq(TLSlavePortParameters.v1(
                     Seq(TLSlaveParameters.v1(
                       address         = Seq(AddressSet(P.ctlBase, P.ctlSize)),
                       regionType      = RegionType.UNCACHED,
                       supportsGet     = TransferSizes(1, 8),
                       supportsPutFull = TransferSizes(1, 8),
                       fifoId          = Some(0),
                       resources       = dev.reg)),
                     beatBytes = 8)))

  val adp_node = TLAdapterNode(
    managerFn = { mp =>
      mp.v1copy(
        managers  = mp.managers.map { m =>
          require(m.regionType == RegionType.UNCACHED)

          m.v1copy(
            mayDenyGet = true,
            mayDenyPut = true)
        },
        beatBytes = 64)
    })

  lazy val module = new LazyModuleImp(this) {
    // runtime
    val dly_i = IO(Input(UInt(32.W)))

    val (c, ce) = ctl_node.in.head
    val (i, ie) = adp_node.in.head
    val (o, oe) = adp_node.out.head


    //
    // mem & queue

    val N = P.memSize >> 18
    val W = log2Ceil(N)

    val bitmap = SyncReadMem(N, UInt(64.W))

    val ren    = dontTouch(Wire(Bool()))
    val raddr  = dontTouch(Wire(UInt(W.W)))
    val roffs  = dontTouch(Wire(UInt(6.W)))
    val rdata  = dontTouch(Wire(UInt(64.W)))
    val err    = BSR(rdata, roffs)(0)

    val wen    = dontTouch(Wire(Bool()))
    val waddr  = dontTouch(Wire(UInt(W.W)))
    val wdata  = dontTouch(Wire(UInt(64.W)))

    // single port mem model
    rdata := DontCare

    when (wen || ren) {
      val port = bitmap(wen ?? waddr :: raddr)

      when (wen) {
        port  := wdata
      } .otherwise {
        rdata := port
      }
    }

    val que = Module(new Queue(new Bundle {
      val rnw    = Bool()
      val size   = UInt(i.params.sizeBits.W)
      val source = UInt(i.params.sourceBits.W)
    }, P.queue))

    val enq = que.io.enq
    val deq = que.io.deq

    // random delay
    val dly_vld = LFSR(32, deq.valid && !o.d.valid) <= dly_i


    //
    // ctl

    val ctl_req   = c.a.fire
    val ctl_resp  = c.d.fire

    val ctl_set   = Non(c.a.bits.address(3))
    val ctl_rnw   = c.a.bits.opcode === TLMessages.Get

    val ctl_vld_q = RegEnable(ctl_req, false.B, ctl_req || ctl_resp)
    val ctl_rnw_q = RegEnable(ctl_rnw,          ctl_req)
    val ctl_src_q = RegEnable(c.a.bits.source,  ctl_req)

    val ctl_ren   = ctl_req
    val ctl_raddr = c.a.bits.data(18 :+ W)
    val ctl_roffs = c.a.bits.data(18 :- 6)

    // workaround
    val ctl_chk_q = RegNext(ctl_req)
    val ctl_wen   = RegNext(ctl_chk_q)
    val ctl_wmask = RegEnable(Dec(ctl_roffs), ctl_ren)
    val ctl_wdata = ctl_wmask & Rep(RegNext(ctl_set), 64) |
                   ~ctl_wmask & rdata

    c.a.ready := Non(ctl_chk_q ## ctl_wen) && rst_done

    c.d.valid := ctl_vld_q
    c.d.bits  := ctl_rnw_q ??
                     ce.AccessAck(ctl_src_q,
                                  3.U,
                                  0.U) ::
                     ce.AccessAck(ctl_src_q,
                                  3.U)


    //
    // err

    val
      (fsm_idle  ::
       fsm_chk   ::
       fsm_req   ::
       fsm_null) = Enum(3)

    // fsm
    val cha_fsm_en  = dontTouch(Wire(Bool()))
    val cha_fsm_nxt = dontTouch(Wire(UInt(2.W)))
    val cha_fsm_q   = dontTouch(Wire(UInt(2.W)))

    val cha_gnt     = dontTouch(Wire(Bool()))
    val cha_err     = dontTouch(Wire(Bool()))

    // default
    cha_fsm_en  := false.B
    cha_fsm_nxt := cha_fsm_q

    switch (cha_fsm_q) {
      is (fsm_idle) {
        cha_fsm_en  := i.a.valid
        cha_fsm_nxt := cha_gnt ?? fsm_chk  :: fsm_idle
      }
      is (fsm_chk) {
        cha_fsm_en  := true.B
        cha_fsm_nxt := cha_err ?? fsm_idle :: fsm_req
      }
      is (fsm_req) {
        cha_fsm_en  := o.a.fire
        cha_fsm_nxt := fsm_idle
      }
    }

    val cha_fsm_is_idle = cha_fsm_q === fsm_idle
    val cha_fsm_is_chk  = cha_fsm_q === fsm_chk
    val cha_fsm_is_req  = cha_fsm_q === fsm_req

    cha_fsm_q := RegEnable(cha_fsm_nxt, fsm_idle, cha_fsm_en)

    val cha_chk = (i.a.bits.address >=  P.memBase.U) &&
                  (i.a.bits.address <= (P.memBase + P.memSize).U)

    cha_gnt := ren && !wen && !ctl_ren || !cha_chk
    cha_err := err &&                      cha_chk

    val deq_get =  deq.valid &&  deq.bits.rnw
    val deq_put =  deq.valid && !deq.bits.rnw

    // reset
    val rst_q    = dontTouch(Wire(UInt((W + 1).W)))
    val rst_done = rst_q(W)

    rst_q := RegEnable(rst_q + 1.U,
                       0.U,
                      !rst_done)

    // output
    ren             := ctl_ren  || rst_done  && i.a.valid && cha_chk && enq.ready
    raddr           := ctl_ren  ?? ctl_raddr :: i.a.bits.address(18 :+ W)
    roffs           := ctl_ren  ?? ctl_roffs :: i.a.bits.address(18 :- 6)

    wen             := ctl_wen  || Non(rst_done)
    waddr           := rst_done ?? RegEnable(ctl_raddr, ctl_ren)   :: rst_q(W.W)
    wdata           := rst_done ?? RegEnable(ctl_wdata, ctl_chk_q) :: 0.U

    enq.valid       := cha_fsm_is_chk && cha_err
    enq.bits.rnw    := i.a.bits.opcode === TLMessages.Get
    enq.bits.size   := i.a.bits.size
    enq.bits.source := i.a.bits.source

    deq.ready       := i.d.fire && deq.valid

    i.a.ready       := cha_fsm_is_chk

    o.a.valid       := cha_fsm_is_req
    o.a.bits        := RegEnable(i.a.bits, cha_fsm_is_chk && !cha_err)

    i.d.valid       := deq.valid && dly_vld || o.d.valid
    i.d.bits        := deq_get ??
                           ie.AccessAck(deq.bits.source,
                                        deq.bits.size,
                                        0.U,
                                        true.B,
                                        true.B) ::
                       deq_put ??
                           ie.AccessAck(deq.bits.source,
                                        deq.bits.size,
                                        true.B) ::
                           o.d.bits

    o.d.ready       := i.d.ready &&
                          (deq.valid && dly_vld ||
                          !deq.valid)
  }
}