/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.frontend
import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import utils._
import xiangshan._
import xiangshan.backend.fu.{PFEvent, PMP, PMPChecker,PMPReqBundle}
import xiangshan.cache.mmu._
import xiangshan.frontend.icache._
import system._


class Frontend()(implicit p: Parameters) extends LazyModule with HasXSParameter{

  val instrUncache  = LazyModule(new InstrUncache())
  val icache        = LazyModule(new ICache())

  lazy val module = new FrontendImp(this)
}


class FrontendImp (outer: Frontend) extends LazyModuleImp(outer)
  with HasXSParameter
  with HasPerfEvents
{
  val io = IO(new Bundle() {
    val hartId = Input(UInt(8.W))
    val reset_vector = Input(UInt(PAddrBits.W))
    val fencei = Input(Bool())
    val ptw = new TlbPtwIO(6)
    val ttw = new FSTWIO(mgFSParam)
    val backend = new FrontendToCtrlIO
    val sfence = Input(new SfenceBundle)
    val tlbCsr = Input(new TlbCsrBundle)
    val csrCtrl = Input(new CustomCSRCtrlIO)
    val csrUpdate = new DistributedCSRUpdateReq
    val error  = new L1CacheErrorInfo
    val frontendInfo = new Bundle {
      val ibufFull  = Output(Bool())
      val bpuInfo = new Bundle {
        val bpRight = Output(UInt(XLEN.W))
        val bpWrong = Output(UInt(XLEN.W))
      }
    }
  })

  //decouped-frontend modules
  val instrUncache = outer.instrUncache.module
  val icache       = outer.icache.module
  val bpu     = Module(new Predictor)
  val ifu     = Module(new NewIFU)
  val ibuffer =  Module(new Ibuffer)
  val ftq = Module(new Ftq)

  val tlbCsr = DelayN(io.tlbCsr, 2)
  val csrCtrl = DelayN(io.csrCtrl, 2)
  val sfence = RegNext(RegNext(io.sfence))

  // bpu ctrl
  bpu.io.ctrl := csrCtrl.bp_ctrl

// pmp
  val pmp = Module(new PMP())
  val pmp_check = VecInit(Seq.fill(4)(Module(new PMPChecker(3, sameCycle = true)).io))
  pmp.io.distribute_csr := csrCtrl.distribute_csr
  val pmp_req_vec     = Wire(Vec(4, Valid(new PMPReqBundle())))
  pmp_req_vec(0) <> icache.io.pmp(0).req
  pmp_req_vec(1) <> icache.io.pmp(1).req
  pmp_req_vec(2) <> icache.io.pmp(2).req
  pmp_req_vec(3) <> ifu.io.pmp.req

  for (i <- pmp_check.indices) {
    pmp_check(i).apply(tlbCsr.priv.imode, pmp.io.pmp, pmp.io.pma, pmp_req_vec(i))
  }

  icache.io.prefetch <> ftq.io.toPrefetch

  val needFlush = RegNext(io.backend.toFtq.redirect.valid)

  // midgard
  val ivlb = Module(new FSVLBWrapper(6, true, mgFSParam))

  ivlb.csr_i    := tlbCsr
  ivlb.fence_i  := io.sfence
  ivlb.flush_i  := needFlush

  ivlb.pmp_i(0) := pmp_check(0).resp
  ivlb.pmp_i(1) := DontCare
  ivlb.pmp_i(2) := pmp_check(1).resp
  ivlb.pmp_i(3) := DontCare
  ivlb.pmp_i(4) := pmp_check(2).resp
  ivlb.pmp_i(5) := pmp_check(3).resp

  val itlb_req = Seq.fill(6)(Wire(new BlockTlbRequestIO))

  def block_dst(d: BlockTlbRequestIO, s: TlbRequestIO): Unit = {
    d.req      <> s.req
    d.resp     <> s.resp
  }

  def block_src(d: TlbRequestIO, s: BlockTlbRequestIO): Unit = {
    d.req      <> s.req
    d.resp     <> s.resp
    d.req_kill := false.B
  }

  block_dst(itlb_req(0), ivlb.tlb_o(0))
  block_dst(itlb_req(1), ivlb.tlb_o(1))
  block_dst(itlb_req(2), ivlb.tlb_o(2))
  block_dst(itlb_req(3), ivlb.tlb_o(3))
  block_dst(itlb_req(4), ivlb.tlb_o(4))
  block_dst(itlb_req(5), ivlb.tlb_o(5))

  io.ptw <> TLB(
    in = itlb_req,
    sfence = DelayN(io.sfence, 1),
    csr = DelayN(io.tlbCsr, 1),
    width = 6,
    nRespDups = 1,
    shouldBlock = true,
    itlbParams
  )

  block_src(ivlb.tlb_i(0), icache.io.itlb(0))
  block_src(ivlb.tlb_i(1), icache.io.itlb(1))
  block_src(ivlb.tlb_i(2), icache.io.itlb(2))
  block_src(ivlb.tlb_i(3), icache.io.itlb(3))
  block_src(ivlb.tlb_i(4), icache.io.itlb(4))
  block_src(ivlb.tlb_i(5), ifu.io.iTLBInter)

  io.ttw                <> ivlb.ttw_o

  icache.io.pmp(0).resp <> ivlb.pmp_o(0)
  icache.io.pmp(1).resp <> ivlb.pmp_o(2)
  icache.io.pmp(2).resp <> ivlb.pmp_o(4)
  ifu.io.pmp.resp       <> ivlb.pmp_o(5)

  //IFU-Ftq
  ifu.io.ftqInter.fromFtq <> ftq.io.toIfu
  ftq.io.toIfu.req.ready :=  ifu.io.ftqInter.fromFtq.req.ready && icache.io.fetch.req.ready

  ftq.io.fromIfu          <> ifu.io.ftqInter.toFtq
  bpu.io.ftq_to_bpu       <> ftq.io.toBpu
  ftq.io.fromBpu          <> bpu.io.bpu_to_ftq

  ftq.io.mmioCommitRead   <> ifu.io.mmioCommitRead
  //IFU-ICache

  icache.io.fetch.req <> ftq.io.toICache.req
  ftq.io.toICache.req.ready :=  ifu.io.ftqInter.fromFtq.req.ready && icache.io.fetch.req.ready

  ifu.io.icacheInter.resp <>    icache.io.fetch.resp
  ifu.io.icacheInter.icacheReady :=  icache.io.toIFU
  icache.io.stop := ifu.io.icacheStop

  ifu.io.icachePerfInfo := icache.io.perfInfo

  icache.io.csr.distribute_csr <> csrCtrl.distribute_csr
  io.csrUpdate := RegNext(icache.io.csr.update)

  icache.io.csr_pf_enable     := RegNext(csrCtrl.l1I_pf_enable)
  icache.io.csr_parity_enable := RegNext(csrCtrl.icache_parity_enable)

  //IFU-Ibuffer
  ifu.io.toIbuffer    <> ibuffer.io.in

  ftq.io.fromBackend <> io.backend.toFtq
  io.backend.fromFtq <> ftq.io.toBackend
  io.frontendInfo.bpuInfo <> ftq.io.bpuInfo

  ifu.io.rob_commits <> io.backend.toFtq.rob_commits

  ibuffer.io.flush := needFlush
  io.backend.cfVec <> ibuffer.io.out

  instrUncache.io.req   <> ifu.io.uncacheInter.toUncache
  ifu.io.uncacheInter.fromUncache <> instrUncache.io.resp
  instrUncache.io.flush := false.B
  io.error <> RegNext(RegNext(icache.io.error))

  icache.io.hartId := io.hartId

  val frontendBubble = PopCount((0 until DecodeWidth).map(i => io.backend.cfVec(i).ready && !ibuffer.io.out(i).valid))
  XSPerfAccumulate("FrontendBubble", frontendBubble)
  io.frontendInfo.ibufFull := RegNext(ibuffer.io.full)

  // PFEvent
  val pfevent = Module(new PFEvent)
  pfevent.io.distribute_csr := io.csrCtrl.distribute_csr
  val csrevents = pfevent.io.hpmevent.take(8)

  val perfFromUnits = Seq(ifu, ibuffer, icache, ftq, bpu).flatMap(_.getPerfEvents)
  val perfFromIO    = Seq()
  val perfBlock     = Seq()
  // let index = 0 be no event
  val allPerfEvents = Seq(("noEvent", 0.U)) ++ perfFromUnits ++ perfFromIO ++ perfBlock

  if (printEventCoding) {
    for (((name, inc), i) <- allPerfEvents.zipWithIndex) {
      println("Frontend perfEvents Set", name, inc, i)
    }
  }

  val allPerfInc = allPerfEvents.map(_._2.asTypeOf(new PerfEvent))
  override val perfEvents = HPerfMonitor(csrevents, allPerfInc).getPerfEvents
  generatePerfEvent()
}
