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

package xiangshan.backend

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.{BundleBridgeSource, LazyModule, LazyModuleImp}
import freechips.rocketchip.tile.HasFPUParameters
import huancun.PrefetchRecv
import huancun.utils.{RegNextN, ValidIODelay}
import utils._
import xiangshan._
import xiangshan.backend.exu.StdExeUnit
import xiangshan.backend.fu._
import xiangshan.backend.rob.RobLsqIO
import xiangshan.cache._
import xiangshan.cache.mmu.{BTlbPtwIO, TLB, TlbReplace, FSTWIO, FSVLBWrapper}
import xiangshan.mem._
import xiangshan.mem.prefetch.{BasePrefecher, SMSParams, SMSPrefetcher}
import system._

import midgard._

class Std(implicit p: Parameters) extends FunctionUnit {
  io.in.ready := true.B
  io.out.valid := io.in.valid
  io.out.bits.uop := io.in.bits.uop
  io.out.bits.data := io.in.bits.src(0)
}

class MemBlock()(implicit p: Parameters) extends LazyModule
  with HasXSParameter with HasWritebackSource {

  val dcache = LazyModule(new DCacheWrapper())
  val uncache = LazyModule(new Uncache())
  val pf_sender_opt = coreParams.prefetcher.map(_ =>
    BundleBridgeSource(() => new PrefetchRecv)
  )

  lazy val module = new MemBlockImp(this)

  override val writebackSourceParams: Seq[WritebackSourceParams] = {
    val params = new WritebackSourceParams
    params.exuConfigs = (loadExuConfigs ++ storeExuConfigs).map(cfg => Seq(cfg))
    Seq(params)
  }
  override lazy val writebackSourceImp: HasWritebackSourceImp = module
}

class MemBlockImp(outer: MemBlock) extends LazyModuleImp(outer)
  with HasXSParameter
  with HasFPUParameters
  with HasWritebackSourceImp
  with HasPerfEvents
{

  val io = IO(new Bundle {
    val hartId = Input(UInt(8.W))
    val redirect = Flipped(ValidIO(new Redirect))
    // in
    val issue = Vec(exuParameters.LsExuCnt + exuParameters.StuCnt, Flipped(DecoupledIO(new ExuInput)))
    val loadFastMatch = Vec(exuParameters.LduCnt, Input(UInt(exuParameters.LduCnt.W)))
    val loadFastImm = Vec(exuParameters.LduCnt, Input(UInt(12.W)))
    val loadPc = Vec(exuParameters.LduCnt, Input(UInt(VAddrBits.W)))
    val rsfeedback = Vec(exuParameters.LsExuCnt, new MemRSFeedbackIO)
    val stIssuePtr = Output(new SqPtr())
    // out
    val writeback = Vec(exuParameters.LsExuCnt + exuParameters.StuCnt, DecoupledIO(new ExuOutput))
    val s3_delayed_load_error = Vec(exuParameters.LduCnt, Output(Bool()))
    val otherFastWakeup = Vec(exuParameters.LduCnt + 2 * exuParameters.StuCnt, ValidIO(new MicroOp))
    val ise = new ISEIO()
    val fsb = new FSBIO()
    val uat  = Decoupled(new frontside.UATReq(mgFSParam))
    // misc
    val stIn = Vec(exuParameters.StuCnt, ValidIO(new ExuInput))
    val memoryViolation = ValidIO(new Redirect)
    val ptw = new BTlbPtwIO(ld_tlb_ports + exuParameters.StuCnt)
    val ttw = new FSTWIO(mgFSParam)
    val sfence = Input(new SfenceBundle)
    val tlbCsr = Input(new TlbCsrBundle)
    val fenceToSbuffer = Flipped(new FenceToSbuffer)
    val enqLsq = new LsqEnqIO
    // val memPredUpdate = Vec(exuParameters.StuCnt, Input(new MemPredUpdateReq))
    val lsqio = new Bundle {
      val exceptionAddr = new ExceptionAddrIO // to csr
      val rob = Flipped(new RobLsqIO) // rob to lsq
    }
    val csrCtrl = Flipped(new CustomCSRCtrlIO)
    val csrUpdate = new DistributedCSRUpdateReq
    val error = new L1CacheErrorInfo
    val memInfo = new Bundle {
      val sqFull = Output(Bool())
      val lqFull = Output(Bool())
      val dcacheMSHRFull = Output(Bool())
    }
    val perfEventsPTW = Input(Vec(19, new PerfEvent))
    val perfEventsTTW = Input(Vec(5,  new PerfEvent))
    val lqCancelCnt = Output(UInt(log2Up(LoadQueueSize + 1).W))
    val sqCancelCnt = Output(UInt(log2Up(StoreQueueSize + 1).W))
    val sqDeq = Output(UInt(2.W))
  })

  override def writebackSource1: Option[Seq[Seq[DecoupledIO[ExuOutput]]]] = Some(Seq(io.writeback))

  val redirect = RegNextWithEnable(io.redirect)

  val dcache = outer.dcache.module
  val uncache = outer.uncache.module

  val csrCtrl = DelayN(io.csrCtrl, 2)
  dcache.io.csr.distribute_csr <> csrCtrl.distribute_csr
  dcache.io.l2_pf_store_only := RegNext(io.csrCtrl.l2_pf_store_only, false.B)
  io.csrUpdate := RegNext(dcache.io.csr.update)
  io.error <> RegNext(RegNext(dcache.io.error))
  when(!csrCtrl.cache_error_enable){
    io.error.report_to_beu := false.B
    io.error.valid := false.B
  }

  val loadUnits = Seq.fill(exuParameters.LduCnt)(Module(new LoadUnit))
  val storeUnits = Seq.fill(exuParameters.StuCnt)(Module(new StoreUnit))
  val stdExeUnits = Seq.fill(exuParameters.StuCnt)(Module(new StdExeUnit))
  val stData = stdExeUnits.map(_.io.out)
  val exeUnits = loadUnits ++ storeUnits
  val prefetcherOpt: Option[BasePrefecher] = coreParams.prefetcher.map {
    case _: SMSParams =>
      val sms = Module(new SMSPrefetcher())
      sms.io_agt_en := RegNextN(io.csrCtrl.l1D_pf_enable_agt, 2, Some(false.B))
      sms.io_pht_en := RegNextN(io.csrCtrl.l1D_pf_enable_pht, 2, Some(false.B))
      sms.io_act_threshold := RegNextN(io.csrCtrl.l1D_pf_active_threshold, 2, Some(12.U))
      sms.io_act_stride := RegNextN(io.csrCtrl.l1D_pf_active_stride, 2, Some(30.U))
      sms.io_stride_en := RegNextN(io.csrCtrl.l1D_pf_enable_stride, 2, Some(true.B))
      sms
  }
  prefetcherOpt.foreach(pf => {
    val pf_to_l2 = ValidIODelay(pf.io.pf_addr, 2)
    outer.pf_sender_opt.get.out.head._1.addr_valid := pf_to_l2.valid
    outer.pf_sender_opt.get.out.head._1.addr := pf_to_l2.bits
    outer.pf_sender_opt.get.out.head._1.l2_pf_en := RegNextN(io.csrCtrl.l2_pf_enable, 2, Some(true.B))
    pf.io.enable := RegNextN(io.csrCtrl.l1D_pf_enable, 2, Some(false.B))
  })
  val pf_train_on_hit = RegNextN(io.csrCtrl.l1D_pf_train_on_hit, 2, Some(true.B))

  loadUnits.zipWithIndex.map(x => x._1.suggestName("LoadUnit_"+x._2))
  storeUnits.zipWithIndex.map(x => x._1.suggestName("StoreUnit_"+x._2))

  val atomicsUnit = Module(new AtomicsUnit)

  // Atom inst comes from sta / std, then its result
  // will be writebacked using load writeback port
  //
  // However, atom exception will be writebacked to rob
  // using store writeback port

  val loadWritebackOverride  = Mux(atomicsUnit.io.out.valid, atomicsUnit.io.out.bits, loadUnits.head.io.ldout.bits)
  val ldOut0 = Wire(Decoupled(new ExuOutput))
  ldOut0.valid := atomicsUnit.io.out.valid || loadUnits.head.io.ldout.valid
  ldOut0.bits  := loadWritebackOverride
  atomicsUnit.io.out.ready := ldOut0.ready
  loadUnits.head.io.ldout.ready := ldOut0.ready
  when(atomicsUnit.io.out.valid){
    ldOut0.bits.uop.cf.exceptionVec := 0.U(ldOut0.bits.uop.cf.exceptionVec.size.W).asBools // exception will be writebacked via store wb port
  }

  val ldExeWbReqs = ldOut0 +: loadUnits.tail.map(_.io.ldout)
  io.writeback <> ldExeWbReqs ++ VecInit(storeUnits.map(_.io.stout)) ++ VecInit(stdExeUnits.map(_.io.out))
  io.otherFastWakeup := DontCare
  io.otherFastWakeup.take(2).zip(loadUnits.map(_.io.fastUop)).foreach{case(a,b)=> a := b}
  val stOut = io.writeback.drop(exuParameters.LduCnt).dropRight(exuParameters.StuCnt)

  // TODO: fast load wakeup
  val lsq     = Module(new LsqWrappper)
  val sbuffer = Module(new Sbuffer)
  // if you wants to stress test dcache store, use FakeSbuffer
  // val sbuffer = Module(new FakeSbuffer)
  io.stIssuePtr := lsq.io.issuePtrExt

  dcache.io.hartId := io.hartId
  lsq.io.hartId := io.hartId
  sbuffer.io.hartId := io.hartId
  atomicsUnit.io.hartId := io.hartId

  io.ise.expt  := dcache.io.ise
  io.ise.empty := RegNext(sbuffer.io.ise.empty, true.B)
  io.fsb       <> dcache.io.fsb
  io.uat       <> dcache.io.uat

  sbuffer.io.ise.drain := RegNext(io.ise.expt || io.ise.drain, false.B)
  sbuffer.io.ise.valid := DontCare

  // dtlb
  val total_tlb_ports = ld_tlb_ports + exuParameters.StuCnt
  val NUMSfenceDup = 3
  val NUMTlbCsrDup = 8
  val sfence_dup = Seq.fill(NUMSfenceDup)(RegNext(io.sfence))
  val tlbcsr_dup = Seq.fill(NUMTlbCsrDup)(RegNext(io.tlbCsr))

  val dtlb_ld = VecInit(Seq.fill(1){
    val tlb_ld = Module(new TLB(ld_tlb_ports, 1, ldtlbParams))
    tlb_ld.io // let the module have name in waveform
  })
  val dtlb_st = VecInit(Seq.fill(1){
    val tlb_st = Module(new TLB(exuParameters.StuCnt, 1, sttlbParams))
    tlb_st.io // let the module have name in waveform
  })
  val dtlb = dtlb_ld ++ dtlb_st
  val dtlb_reqs = dtlb.flatMap(_.requestor)
  val dtlb_pmps = dtlb.flatMap(_.pmp)
  dtlb.zip(sfence_dup.take(2)).foreach{ case (d,s) => d.sfence := s }
  dtlb.zip(tlbcsr_dup.take(2)).foreach{ case (d,c) => d.csr := c }
  if (refillBothTlb) {
    require(ldtlbParams.outReplace == sttlbParams.outReplace)
    require(ldtlbParams.outReplace)

    val replace = Module(new TlbReplace(total_tlb_ports, ldtlbParams))
    replace.io.apply_sep(dtlb_ld.map(_.replace) ++ dtlb_st.map(_.replace), io.ptw.resp.bits.data.entry.tag)
  } else {
    if (ldtlbParams.outReplace) {
      val replace_ld = Module(new TlbReplace(ld_tlb_ports, ldtlbParams))
      replace_ld.io.apply_sep(dtlb_ld.map(_.replace), io.ptw.resp.bits.data.entry.tag)
    }
    if (sttlbParams.outReplace) {
      val replace_st = Module(new TlbReplace(exuParameters.StuCnt, sttlbParams))
      replace_st.io.apply_sep(dtlb_st.map(_.replace), io.ptw.resp.bits.data.entry.tag)
    }
  }

  dcache.io.tlb := tlbcsr_dup.last

  val ptw_resp_next = RegEnable(io.ptw.resp.bits, io.ptw.resp.valid)
  val ptw_resp_v = RegNext(
    io.ptw.resp.valid && !(RegNext(sfence_dup.last.valid && tlbcsr_dup.last.satp_changed)),
    init = false.B
  )
  io.ptw.resp.ready := true.B

  dtlb.flatMap(a => a.ptw.req)
    .zipWithIndex
    .foreach{ case (tlb, i) =>
    tlb <> io.ptw.req(i)
    val vector_hit = if (refillBothTlb) Cat(ptw_resp_next.vector).orR
      else if (i < ld_tlb_ports) Cat(ptw_resp_next.vector.take(ld_tlb_ports)).orR
      else Cat(ptw_resp_next.vector.drop(ld_tlb_ports)).orR
    io.ptw.req(i).valid := tlb.valid && !(ptw_resp_v && vector_hit &&
      ptw_resp_next.data.entry.hit(tlb.bits.vpn, RegNext(tlbcsr_dup(i).satp.asid), allType = true, ignoreAsid = true))
  }
  dtlb.foreach(_.ptw.resp.bits := ptw_resp_next.data)
  if (refillBothTlb) {
    dtlb.foreach(_.ptw.resp.valid := ptw_resp_v && Cat(ptw_resp_next.vector).orR)
  } else {
    dtlb_ld.foreach(_.ptw.resp.valid := ptw_resp_v && Cat(ptw_resp_next.vector.take(ld_tlb_ports)).orR)
    dtlb_st.foreach(_.ptw.resp.valid := ptw_resp_v && Cat(ptw_resp_next.vector.drop(ld_tlb_ports)).orR)
  }


  // pmp
  val pmp = Module(new PMP())
  pmp.io.distribute_csr <> csrCtrl.distribute_csr

  val pmp_check = VecInit(Seq.fill(total_tlb_ports)(
    Module(new PMPChecker(3, leaveHitMux = true)).io
  ))
  val tlbcsr_pmp = tlbcsr_dup.drop(2).map(RegNext(_))
  for (((p,d),i) <- (pmp_check zip dtlb_pmps).zipWithIndex) {
    p.apply(tlbcsr_pmp(i).priv.dmode, pmp.io.pmp, pmp.io.pma, d)
    require(p.req.bits.size.getWidth == d.bits.size.getWidth)
  }
  val pmp_check_ptw = Module(new PMPCheckerv2(lgMaxSize = 3, sameCycle = false, leaveHitMux = true))
  pmp_check_ptw.io.apply(
    tlbcsr_pmp.last.priv.dmode,
    pmp.io.pmp, pmp.io.pma, io.ptw.resp.valid,
    Cat(io.ptw.resp.bits.data.entry.ppn, 0.U(12.W)).asUInt
  )
  dtlb.foreach(_.ptw_replenish := pmp_check_ptw.io.resp)

  // midgard
  val dvlb = Module(new FSVLBWrapper(total_tlb_ports, false, mgFSParam))

  dvlb.csr_i   := tlbcsr_dup.last
  dvlb.fence_i := sfence_dup.last
  dvlb.flush_i := false.B

  for (i <- 0 until (total_tlb_ports)) {
    dvlb.tlb_o(i) <> dtlb_reqs(i)
    dvlb.pmp_i(i) <> pmp_check(i).resp
  }

  io.ttw <> dvlb.ttw_o

  // LoadUnit
  for (i <- 0 until exuParameters.LduCnt) {
    loadUnits(i).io.redirect <> redirect
    loadUnits(i).io.feedbackSlow <> io.rsfeedback(i).feedbackSlow
    loadUnits(i).io.feedbackFast <> io.rsfeedback(i).feedbackFast
    loadUnits(i).io.rsIdx := io.rsfeedback(i).rsIdx
    loadUnits(i).io.isFirstIssue := io.rsfeedback(i).isFirstIssue // NOTE: just for dtlb's perf cnt
    // get input form dispatch
    loadUnits(i).io.ldin <> io.issue(i)
    // dcache access
    loadUnits(i).io.dcache <> dcache.io.lsu.load(i)
    // forward
    loadUnits(i).io.lsq.forward <> lsq.io.forward(i)
    loadUnits(i).io.sbuffer <> sbuffer.io.forward(i)
    // ld-ld violation check
    loadUnits(i).io.lsq.loadViolationQuery <> lsq.io.loadViolationQuery(i)
    loadUnits(i).io.csrCtrl       <> csrCtrl
    // dtlb
    loadUnits(i).io.tlb <> dvlb.tlb_i(i)
    // pmp
    loadUnits(i).io.pmp <> dvlb.pmp_o(i)
    // prefetch
    prefetcherOpt.foreach(pf => {
      pf.io.ld_in(i).valid := Mux(pf_train_on_hit,
        loadUnits(i).io.prefetch_train.valid,
        loadUnits(i).io.prefetch_train.valid && loadUnits(i).io.prefetch_train.bits.miss
      )
      pf.io.ld_in(i).bits := loadUnits(i).io.prefetch_train.bits
      pf.io.ld_in(i).bits.uop.cf.pc := Mux(loadUnits(i).io.s2IsPointerChasing, io.loadPc(i), RegNext(io.loadPc(i)))
    })

    // load to load fast forward: load(i) prefers data(i)
    val fastPriority = (i until exuParameters.LduCnt) ++ (0 until i)
    val fastValidVec = fastPriority.map(j => loadUnits(j).io.fastpathOut.valid)
    val fastDataVec = fastPriority.map(j => loadUnits(j).io.fastpathOut.data)
    val fastMatchVec = fastPriority.map(j => io.loadFastMatch(i)(j))
    loadUnits(i).io.fastpathIn.valid := VecInit(fastValidVec).asUInt.orR
    loadUnits(i).io.fastpathIn.data := ParallelPriorityMux(fastValidVec, fastDataVec)
    val fastMatch = ParallelPriorityMux(fastValidVec, fastMatchVec)
    loadUnits(i).io.loadFastMatch := fastMatch
    loadUnits(i).io.loadFastImm := io.loadFastImm(i)

    // Lsq to load unit's rs

    // passdown to lsq (load s1)
    lsq.io.loadPaddrIn(i) <> loadUnits(i).io.lsq.loadPaddrIn

    // passdown to lsq (load s2)
    lsq.io.loadIn(i) <> loadUnits(i).io.lsq.loadIn
    lsq.io.ldout(i) <> loadUnits(i).io.lsq.ldout
    lsq.io.ldRawDataOut(i) <> loadUnits(i).io.lsq.ldRawData
    lsq.io.s2_load_data_forwarded(i) <> loadUnits(i).io.lsq.s2_load_data_forwarded

    // passdown to lsq (load s3)
    lsq.io.s2_dcache_require_replay(i) <> loadUnits(i).io.lsq.s2_dcache_require_replay
    lsq.io.s3_replay_from_fetch(i) <> loadUnits(i).io.lsq.s3_replay_from_fetch
    lsq.io.s3_delayed_load_error(i) <> loadUnits(i).io.s3_delayed_load_error

    // alter writeback exception info
    io.s3_delayed_load_error(i) := loadUnits(i).io.lsq.s3_delayed_load_error

    // update mem dependency predictor
    // io.memPredUpdate(i) := DontCare
  }

  // Prefetcher
  prefetcherOpt.foreach(pf => {
    pf.io.tlb_req               <> dvlb.tlb_i(ld_tlb_ports - 1)
    dtlb_reqs(ld_tlb_ports - 1) <> dvlb.tlb_o(ld_tlb_ports - 1)
  })

  // StoreUnit
  for (i <- 0 until exuParameters.StuCnt) {
    val stu = storeUnits(i)

    stdExeUnits(i).io.redirect <> redirect
    stdExeUnits(i).io.fromInt <> io.issue(i + exuParameters.LduCnt + exuParameters.StuCnt)
    stdExeUnits(i).io.fromFp := DontCare
    stdExeUnits(i).io.out := DontCare

    stu.io.redirect     <> redirect
    stu.io.feedbackSlow <> io.rsfeedback(exuParameters.LduCnt + i).feedbackSlow
    stu.io.rsIdx        <> io.rsfeedback(exuParameters.LduCnt + i).rsIdx
    // NOTE: just for dtlb's perf cnt
    stu.io.isFirstIssue <> io.rsfeedback(exuParameters.LduCnt + i).isFirstIssue
    stu.io.stin         <> io.issue(exuParameters.LduCnt + i)
    stu.io.lsq          <> lsq.io.storeIn(i)
    stu.io.lsq_replenish <> lsq.io.storeInRe(i)
    // dtlb
    stu.io.tlb          <> dvlb.tlb_i(exuParameters.LduCnt + i)
    stu.io.pmp          <> dvlb.pmp_o(exuParameters.LduCnt + i)

    // store unit does not need fast feedback
    io.rsfeedback(exuParameters.LduCnt + i).feedbackFast := DontCare

    // Lsq to sta unit
    lsq.io.storeMaskIn(i) <> stu.io.storeMaskOut

    // Lsq to std unit's rs
    lsq.io.storeDataIn(i) := stData(i)


    // 1. sync issue info to store set LFST
    // 2. when store issue, broadcast issued sqPtr to wake up the following insts
    io.stIn(i).valid := io.issue(exuParameters.LduCnt + i).valid
    io.stIn(i).bits := io.issue(exuParameters.LduCnt + i).bits

    stu.io.stout.ready := true.B

    // store data
//    when(lsq.io.storeDataIn(i).fire()){
//
//      val hit = Wire(Vec(3, Bool()))
//      for (j <- 0 until 3) {
//        when(tdata(sTriggerMapping(j)).select) {
//          hit(j) := TriggerCmp(lsq.io.storeDataIn(i).bits.data, tdata(sTriggerMapping(j)).tdata2, tdata(sTriggerMapping(j)).matchType, tEnable(sTriggerMapping(j)))
//          lsq.io.storeDataIn(i).bits.uop.cf.trigger.backendHit(sTriggerMapping(j)) := hit(j)
//          lsq.io.storeDataIn(i).bits.uop.cf.trigger.backendTiming(sTriggerMapping(j)) := tdata(sTriggerMapping(j)).timing
////          if (sChainMapping.contains(j)) lsq.io.storeDataIn(i).bits.uop.cf.trigger.triggerChainVec(sChainMapping(j)) := hit && tdata(j + 3).chain
//        }
//      }
//
//      when(tdata(0).chain) {
//        lsq.io.storeDataIn(i).bits.uop.cf.trigger.backendHit(0) := hit(0) && hit(1)
//        lsq.io.storeDataIn(i).bits.uop.cf.trigger.backendHit(1) := hit(0) && hit(1)
//      }
//      when(lsq.io.storeDataIn(i).bits.uop.cf.trigger.backendEn(1)) {
//        lsq.io.storeDataIn(i).bits.uop.cf.trigger.backendHit(4) := Mux(io.writeback(i).bits.uop.cf.trigger.backendConsiderTiming(1),
//          tdata(4).timing === lsq.io.storeDataIn(i).bits.uop.cf.trigger.backendChainTiming(1), true.B) && hit(2)
//      } .otherwise {
//        lsq.io.storeDataIn(i).bits.uop.cf.trigger.backendHit(4) := false.B
//      }
//    }
  }

  // mmio store writeback will use store writeback port 0
  lsq.io.mmioStout.ready := false.B
  when (lsq.io.mmioStout.valid && !storeUnits(0).io.stout.valid) {
    stOut(0).valid := true.B
    stOut(0).bits  := lsq.io.mmioStout.bits
    lsq.io.mmioStout.ready := true.B
  }

  // atomic exception / trigger writeback
  when (atomicsUnit.io.out.valid) {
    // atom inst will use store writeback port 0 to writeback exception info
    stOut(0).valid := true.B
    stOut(0).bits  := atomicsUnit.io.out.bits
    assert(!lsq.io.mmioStout.valid && !storeUnits(0).io.stout.valid)
  }

  // Lsq
  lsq.io.rob            <> io.lsqio.rob
  lsq.io.enq            <> io.enqLsq
  lsq.io.brqRedirect    <> redirect
  io.memoryViolation    <> lsq.io.rollback
  // lsq.io.uncache        <> uncache.io.lsq
  AddPipelineReg(lsq.io.uncache.req, uncache.io.lsq.req, false.B)
  AddPipelineReg(uncache.io.lsq.resp, lsq.io.uncache.resp, false.B)
  // delay dcache refill for 1 cycle for better timing
  // TODO: remove RegNext after fixing refill paddr timing
  // lsq.io.dcache         <> dcache.io.lsu.lsq
  lsq.io.dcache         := RegNext(dcache.io.lsu.lsq)
  lsq.io.release        := dcache.io.lsu.release
  lsq.io.lqCancelCnt <> io.lqCancelCnt
  lsq.io.sqCancelCnt <> io.sqCancelCnt
  lsq.io.sqDeq <> io.sqDeq

  // LSQ to store buffer
  lsq.io.sbuffer        <> sbuffer.io.in
  lsq.io.sqempty        <> sbuffer.io.sqempty

  // Sbuffer
  sbuffer.io.csrCtrl    <> csrCtrl
  sbuffer.io.dcache     <> dcache.io.lsu.store
  // TODO: if dcache sbuffer resp needs to ne delayed
  // sbuffer.io.dcache.pipe_resp.valid := RegNext(dcache.io.lsu.store.pipe_resp.valid)
  // sbuffer.io.dcache.pipe_resp.bits := RegNext(dcache.io.lsu.store.pipe_resp.bits)

  // flush sbuffer
  val fenceFlush   = io.fenceToSbuffer.flushSb
  val atomicsFlush = atomicsUnit.io.flush_sbuffer.valid

  // both can now have exceptions
  val sb_flush_q   = Wire(Bool())
  val sb_flush_set = atomicsFlush || fenceFlush
  val sb_flush_clr = io.fenceToSbuffer.sbIsEmpty
  val sb_flush_vld = sb_flush_set || sb_flush_q

  val sb_drain_q   = Wire(Bool())
  val sb_drain_set = sb_flush_vld && io.ise.valid
  val sb_drain_vld = sb_drain_set || sb_drain_q

  sb_flush_q := RegEnable(sb_flush_set && !sb_flush_clr, false.B, sb_flush_set || sb_flush_clr)
  sb_drain_q := RegEnable(sb_drain_set && !sb_flush_clr, false.B, sb_drain_set || sb_flush_clr)

  io.fenceToSbuffer.sbIsEmpty := RegNext(sbuffer.io.flush.empty)
  io.fenceToSbuffer.ise       := sb_drain_vld
  // if both of them tries to flush sbuffer at the same time
  // something must have gone wrong
  assert(!(fenceFlush && atomicsFlush))
  sbuffer.io.flush.valid := RegNext(fenceFlush || atomicsFlush)

  sbuffer.io.tlbcsr := tlbcsr_dup.last

  // AtomicsUnit: AtomicsUnit will override other control signials,
  // as atomics insts (LR/SC/AMO) will block the pipeline
  val s_normal :: s_atomics_0 :: s_atomics_1 :: Nil = Enum(3)
  val state = RegInit(s_normal)

  val atomic_rs0  = exuParameters.LduCnt + 0
  val atomic_rs1  = exuParameters.LduCnt + 1
  val st0_atomics = io.issue(atomic_rs0).valid && FuType.storeIsAMO(io.issue(atomic_rs0).bits.uop.ctrl.fuType)
  val st1_atomics = io.issue(atomic_rs1).valid && FuType.storeIsAMO(io.issue(atomic_rs1).bits.uop.ctrl.fuType)

  val st0_data_atomics = stData(0).valid && FuType.storeIsAMO(stData(0).bits.uop.ctrl.fuType)
  val st1_data_atomics = stData(1).valid && FuType.storeIsAMO(stData(1).bits.uop.ctrl.fuType)

  when (st0_atomics) {
    io.issue(atomic_rs0).ready := atomicsUnit.io.in.ready
    storeUnits(0).io.stin.valid := false.B

    state := s_atomics_0
    assert(!st1_atomics)
  }
  when (st1_atomics) {
    io.issue(atomic_rs1).ready := atomicsUnit.io.in.ready
    storeUnits(1).io.stin.valid := false.B

    state := s_atomics_1
    assert(!st0_atomics)
  }
  when (atomicsUnit.io.out.valid) {
    assert(state === s_atomics_0 || state === s_atomics_1)
    state := s_normal
  }

  atomicsUnit.io.in.valid := st0_atomics || st1_atomics
  atomicsUnit.io.in.bits  := Mux(st0_atomics, io.issue(atomic_rs0).bits, io.issue(atomic_rs1).bits)
  atomicsUnit.io.storeDataIn.valid := st0_data_atomics || st1_data_atomics
  atomicsUnit.io.storeDataIn.bits  := Mux(st0_data_atomics, stData(0).bits, stData(1).bits)
  atomicsUnit.io.rsIdx    := Mux(st0_atomics, io.rsfeedback(atomic_rs0).rsIdx, io.rsfeedback(atomic_rs1).rsIdx)
  atomicsUnit.io.redirect <> redirect

  // TODO: complete amo's pmp support
  val amoTlb = dvlb.tlb_i(0)
  atomicsUnit.io.dtlb.resp.valid := false.B
  atomicsUnit.io.dtlb.resp.bits  := DontCare
  atomicsUnit.io.dtlb.req.ready  := amoTlb.req.ready
  atomicsUnit.io.pmpResp <> dvlb.pmp_o(0)

  atomicsUnit.io.dcache <> dcache.io.lsu.atomics
  atomicsUnit.io.flush_sbuffer.empty := io.fenceToSbuffer.sbIsEmpty
  atomicsUnit.io.flush_sbuffer.ise   := sb_drain_vld

  atomicsUnit.io.csrCtrl := csrCtrl

  // for atomicsUnit, it uses loadUnit(0)'s TLB port

  when (state === s_atomics_0 || state === s_atomics_1) {
    loadUnits(0).io.ldout.ready := false.B
    atomicsUnit.io.dtlb <> amoTlb

    // make sure there's no in-flight uops in load unit
    assert(!loadUnits(0).io.ldout.valid)
  }

  when (state === s_atomics_0) {
    atomicsUnit.io.feedbackSlow <> io.rsfeedback(atomic_rs0).feedbackSlow

    assert(!storeUnits(0).io.feedbackSlow.valid)
  }
  when (state === s_atomics_1) {
    atomicsUnit.io.feedbackSlow <> io.rsfeedback(atomic_rs1).feedbackSlow

    assert(!storeUnits(1).io.feedbackSlow.valid)
  }

  lsq.io.exceptionAddr.isStore := io.lsqio.exceptionAddr.isStore
  // Exception address is used several cycles after flush.
  // We delay it by 10 cycles to ensure its flush safety.
  val atomicsException = RegInit(false.B)
  when (DelayN(redirect.valid, 10) && atomicsException) {
    atomicsException := false.B
  }.elsewhen (atomicsUnit.io.exceptionAddr.valid) {
    atomicsException := true.B
  }
  val atomicsExceptionAddress = RegEnable(atomicsUnit.io.exceptionAddr.bits, atomicsUnit.io.exceptionAddr.valid)
  io.lsqio.exceptionAddr.vaddr := RegNext(Mux(atomicsException, atomicsExceptionAddress, lsq.io.exceptionAddr.vaddr))
  XSError(atomicsException && atomicsUnit.io.in.valid, "new instruction before exception triggers\n")

  io.memInfo.sqFull := RegNext(lsq.io.sqFull)
  io.memInfo.lqFull := RegNext(lsq.io.lqFull)
  io.memInfo.dcacheMSHRFull := RegNext(dcache.io.mshrFull)

  val ldDeqCount = PopCount(io.issue.take(2).map(_.valid))
  val stDeqCount = PopCount(io.issue.drop(2).map(_.valid))
  val rsDeqCount = ldDeqCount + stDeqCount
  XSPerfAccumulate("load_rs_deq_count", ldDeqCount)
  XSPerfHistogram("load_rs_deq_count", ldDeqCount, true.B, 1, 2, 1)
  XSPerfAccumulate("store_rs_deq_count", stDeqCount)
  XSPerfHistogram("store_rs_deq_count", stDeqCount, true.B, 1, 2, 1)
  XSPerfAccumulate("ls_rs_deq_count", rsDeqCount)

  val pfevent = Module(new PFEvent)
  pfevent.io.distribute_csr := csrCtrl.distribute_csr
  val csrevents = pfevent.io.hpmevent.slice(16,24)

  val memBlockPerfEvents = Seq(
    ("ldDeqCount", ldDeqCount),
    ("stDeqCount", stDeqCount),
  )

  val perfFromUnits = (loadUnits ++ Seq(sbuffer, lsq, dcache, dvlb)).flatMap(_.getPerfEvents)
  val perfFromPTW   = io.perfEventsPTW.map(x => ("perfEventsPTW", x.value))
  val perfFromTTW   = io.perfEventsTTW.map(x => ("perfEventsTTW", x.value))
  val perfBlock     = Seq(("ldDeqCount", ldDeqCount),
                          ("stDeqCount", stDeqCount))
  // let index = 0 be no event
  val allPerfEvents = Seq(("noEvent", 0.U)) ++ perfFromUnits ++ perfFromPTW ++ perfFromTTW ++ perfBlock

  if (printEventCoding) {
    for (((name, inc), i) <- allPerfEvents.zipWithIndex) {
      println("MemBlock perfEvents Set", name, inc, i)
    }
  }

  val allPerfInc = allPerfEvents.map(_._2.asTypeOf(new PerfEvent))
  val perfEvents = HPerfMonitor(csrevents, allPerfInc).getPerfEvents
  generatePerfEvent()
}
