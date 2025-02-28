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

package xiangshan.backend.fu

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import difftest._
import freechips.rocketchip.util._
import utils.MaskedRegMap.WritableMask
import utils._
import xiangshan.ExceptionNO._
import xiangshan._
import xiangshan.backend.fu.util._
import xiangshan.cache._

// Trigger Tdata1 bundles
trait HasTriggerConst {
  def I_Trigger = 0.U
  def S_Trigger = 1.U
  def L_Trigger = 2.U
  def GenESL(triggerType: UInt) = Cat((triggerType === I_Trigger), (triggerType === S_Trigger), (triggerType === L_Trigger))
}

class FpuCsrIO extends Bundle {
  val fflags = Output(Valid(UInt(5.W)))
  val isIllegal = Output(Bool())
  val dirty_fs = Output(Bool())
  val frm = Input(UInt(3.W))
}

// uat support
class UATM extends Bundle {
  val h  = UInt(5.W)
  val c  = UInt(64.W)
  val cu = UInt(64.W)
  val i  = UInt(64.W)
  val iu = UInt(64.W)
}


class PerfCounterIO(implicit p: Parameters) extends XSBundle {
  val perfEventsFrontend  = Vec(numCSRPCntFrontend, new PerfEvent)
  val perfEventsCtrl      = Vec(numCSRPCntCtrl, new PerfEvent)
  val perfEventsLsu       = Vec(numCSRPCntLsu, new PerfEvent)
  val perfEventsHc        = Vec(numPCntHc * coreParams.L2NBanks, new PerfEvent)
  val retiredInstr = UInt(3.W)
  val frontendInfo = new Bundle {
    val ibufFull  = Bool()
    val bpuInfo = new Bundle {
      val bpRight = UInt(XLEN.W)
      val bpWrong = UInt(XLEN.W)
    }
  }
  val ctrlInfo = new Bundle {
    val robFull   = Bool()
    val intdqFull = Bool()
    val fpdqFull  = Bool()
    val lsdqFull  = Bool()
  }
  val memInfo = new Bundle {
    val sqFull = Bool()
    val lqFull = Bool()
    val dcacheMSHRFull = Bool()
  }

  val cacheInfo = new Bundle {
    val l2MSHRFull = Bool()
    val l3MSHRFull = Bool()
    val l2nAcquire = UInt(XLEN.W)
    val l2nAcquireMiss = UInt(XLEN.W)
    val l3nAcquire = UInt(XLEN.W)
    val l3nAcquireMiss = UInt(XLEN.W)
  }
}

class CSRFileIO(implicit p: Parameters) extends XSBundle {
  val hartId = Input(UInt(8.W))
  // output (for func === CSROpType.jmp)
  val perf = Input(new PerfCounterIO)
  val isPerfCnt = Output(Bool())
  // to FPU
  val fpu = Flipped(new FpuCsrIO)
  // from rob
  val exception = Flipped(ValidIO(new ExceptionInfo))
  // to ROB
  val isXRet = Output(Bool())
  val trapTarget = Output(UInt(VAddrBits.W))
  val interrupt = Output(Bool())
  val wfi_event = Output(Bool())
  // from LSQ
  val memExceptionVAddr = Input(UInt(VAddrBits.W))
  // from outside cpu,externalInterrupt
  val externalInterrupt = new ExternalInterruptIO
  val ise = Flipped(new ISEIO())
  val fsb = Flipped(new FSBIO())
  // TLB
  val tlb = Output(new TlbCsrBundle)
  // Debug Mode
  // val singleStep = Output(Bool())
  val debugMode = Output(Bool())
  // to Fence to disable sfence
  val disableSfence = Output(Bool())
  // Custom microarchiture ctrl signal
  val customCtrl = Output(new CustomCSRCtrlIO)
  // distributed csr write
  val distributedUpdate = Vec(2, Flipped(new DistributedCSRUpdateReq))
}

class CSR(implicit p: Parameters) extends FunctionUnit with HasCSRConst with PMPMethod with PMAMethod with HasTriggerConst
  with DebugCSR
{
  val csrio = IO(new CSRFileIO)

  val cfIn = io.in.bits.uop.cf
  val cfOut = Wire(new CtrlFlow)
  cfOut := cfIn
  val flushPipe = Wire(Bool())

  val (valid, src1, src2, func) = (
    io.in.valid,
    io.in.bits.src(0),
    io.in.bits.uop.ctrl.imm,
    io.in.bits.uop.ctrl.fuOpType
  )

  // CSR define

  class Priv extends Bundle {
    val m = Output(Bool())
    val h = Output(Bool())
    val s = Output(Bool())
    val u = Output(Bool())
  }

  val csrNotImplemented = RegInit(UInt(XLEN.W), 0.U)

  class MstatusStruct extends Bundle {
    val sd = Output(UInt(1.W))

    val pad1 = if (XLEN == 64) Output(UInt(25.W)) else null
    val mbe  = if (XLEN == 64) Output(UInt(1.W)) else null
    val sbe  = if (XLEN == 64) Output(UInt(1.W)) else null
    val sxl  = if (XLEN == 64) Output(UInt(2.W))  else null
    val uxl  = if (XLEN == 64) Output(UInt(2.W))  else null
    val pad0 = if (XLEN == 64) Output(UInt(9.W))  else Output(UInt(8.W))

    val tsr = Output(UInt(1.W))
    val tw = Output(UInt(1.W))
    val tvm = Output(UInt(1.W))
    val mxr = Output(UInt(1.W))
    val sum = Output(UInt(1.W))
    val mprv = Output(UInt(1.W))
    val xs = Output(UInt(2.W))
    val fs = Output(UInt(2.W))
    val mpp = Output(UInt(2.W))
    val hpp = Output(UInt(2.W))
    val spp = Output(UInt(1.W))
    val pie = new Priv
    val ie = new Priv
    assert(this.getWidth == XLEN)

    def ube = pie.h // a little ugly
    def ube_(r: UInt): Unit = {
      pie.h := r(0)
    }
  }

  class Interrupt extends Bundle {
//  val d = Output(Bool())    // Debug
    val e = new Priv
    val t = new Priv
    val s = new Priv
  }

  // Debug CSRs
  val dcsr = RegInit(UInt(32.W), DcsrStruct.init)
  val dpc = Reg(UInt(64.W))
  val dscratch0 = Reg(UInt(64.W))
  val dscratch1 = Reg(UInt(64.W))
  val debugMode = RegInit(false.B)
  val debugIntrEnable = RegInit(true.B) // debug interrupt will be handle only when debugIntrEnable
  csrio.debugMode := debugMode

  val dpcPrev = RegNext(dpc)
  XSDebug(dpcPrev =/= dpc, "Debug Mode: dpc is altered! Current is %x, previous is %x\n", dpc, dpcPrev)

  val dcsrData = dcsr.asTypeOf(new DcsrStruct)
  val dcsrMask = ZeroExt(GenMask(15) | GenMask(13, 11) | GenMask(4) | GenMask(2, 0), XLEN)// Dcsr write mask
  def dcsrUpdateSideEffect(dcsr: UInt): UInt = {
    val dcsrOld = WireInit(dcsr.asTypeOf(new DcsrStruct))
    val dcsrNew = dcsr | (dcsrOld.prv(0) | dcsrOld.prv(1)).asUInt // turn 10 priv into 11
    dcsrNew
  }
  // csrio.singleStep := dcsrData.step
  csrio.customCtrl.singlestep := dcsrData.step && !debugMode

  // Machine-Level CSRs
  // mtvec: {BASE (WARL), MODE (WARL)} where mode is 0 or 1
  val mtvecMask = ~(0x2.U(XLEN.W))
  val mtvec = RegInit(UInt(XLEN.W), 0.U)
  val mcounteren = RegInit(UInt(XLEN.W), 0.U)
  val mcause = RegInit(UInt(XLEN.W), 0.U)
  val mtval = RegInit(UInt(XLEN.W), 0.U)
  val mepc = Reg(UInt(XLEN.W))
  // Page 36 in riscv-priv: The low bit of mepc (mepc[0]) is always zero.
  val mepcMask = ~(0x1.U(XLEN.W))

  val mie = RegInit(0.U(XLEN.W))
  val mipWire = WireInit(0.U.asTypeOf(new Interrupt))
  val mipReg  = RegInit(0.U(XLEN.W))
  val mipFixMask = ZeroExt(GenMask(9) | GenMask(5) | GenMask(1), XLEN)
  val mip = (mipWire.asUInt | mipReg).asTypeOf(new Interrupt)

  def getMisaMxl(mxl: BigInt): BigInt = mxl << (XLEN - 2)
  def getMisaExt(ext: Char): Long = 1 << (ext.toInt - 'a'.toInt)
  var extList = List('a', 's', 'i', 'u')
  if (HasMExtension) { extList = extList :+ 'm' }
  if (HasCExtension) { extList = extList :+ 'c' }
  if (HasFPU) { extList = extList ++ List('f', 'd') }
  val misaInitVal = getMisaMxl(2) | extList.foldLeft(0L)((sum, i) => sum | getMisaExt(i)) //"h8000000000141105".U
  val misa = RegInit(UInt(XLEN.W), misaInitVal.U)

  // MXL = 2          | 0 | EXT = b 00 0000 0100 0001 0001 0000 0101
  // (XLEN-1, XLEN-2) |   |(25, 0)  ZY XWVU TSRQ PONM LKJI HGFE DCBA

  val mvendorid = RegInit(UInt(XLEN.W), 0.U) // this is a non-commercial implementation
  val marchid = RegInit(UInt(XLEN.W), 25.U) // architecture id for XiangShan is 25; see https://github.com/riscv/riscv-isa-manual/blob/master/marchid.md
  val mimpid = RegInit(UInt(XLEN.W), 0.U) // provides a unique encoding of the version of the processor implementation
  val mhartid = Reg(UInt(XLEN.W)) // the hardware thread running the code
  when (RegNext(RegNext(reset.asBool) && !reset.asBool)) {
    mhartid := csrio.hartId
  }
  val mconfigptr = RegInit(UInt(XLEN.W), 0.U) // the read-only pointer pointing to the platform config structure, 0 for not supported.
  val mstatus = RegInit("ha00002000".U(XLEN.W))

  // mstatus Value Table
  // | sd   |
  // | pad1 |
  // | sxl  | hardlinked to 10, use 00 to pass xv6 test
  // | uxl  | hardlinked to 10
  // | pad0 |
  // | tsr  |
  // | tw   |
  // | tvm  |
  // | mxr  |
  // | sum  |
  // | mprv |
  // | xs   | 00 |
  // | fs   | 01 |
  // | mpp  | 00 |
  // | hpp  | 00 |
  // | spp  | 0 |
  // | pie  | 0000 | pie.h is used as UBE
  // | ie   | 0000 | uie hardlinked to 0, as N ext is not implemented

  val mstatusStruct = mstatus.asTypeOf(new MstatusStruct)
  def mstatusUpdateSideEffect(mstatus: UInt): UInt = {
    val mstatusOld = WireInit(mstatus.asTypeOf(new MstatusStruct))
    val mstatusNew = Cat(mstatusOld.xs === "b11".U || mstatusOld.fs === "b11".U, mstatus(XLEN-2, 0))
    mstatusNew
  }

  val mstatusWMask = (~ZeroExt((
    GenMask(XLEN - 2, 36) | // WPRI
    GenMask(35, 32)       | // SXL and UXL cannot be changed
    GenMask(31, 23)       | // WPRI
    GenMask(16, 15)       | // XS is read-only
    GenMask(10, 9)        | // WPRI
    GenMask(6)            | // WPRI
    GenMask(2)              // WPRI
  ), 64)).asUInt
  val mstatusMask = (~ZeroExt((
    GenMask(XLEN - 2, 36) | // WPRI
    GenMask(31, 23)       | // WPRI
    GenMask(10, 9)        | // WPRI
    GenMask(6)            | // WPRI
    GenMask(2)              // WPRI
  ), 64)).asUInt

  val medeleg = RegInit(UInt(XLEN.W), 0.U)
  val mideleg = RegInit(UInt(XLEN.W), 0.U)
  val mscratch = RegInit(UInt(XLEN.W), 0.U)

  // PMP Mapping
  val pmp = Wire(Vec(NumPMP, new PMPEntry())) // just used for method parameter
  val pma = Wire(Vec(NumPMA, new PMPEntry())) // just used for method parameter
  val pmpMapping = pmp_gen_mapping(pmp_init, NumPMP, PmpcfgBase, PmpaddrBase, pmp)
  val pmaMapping = pmp_gen_mapping(pma_init, NumPMA, PmacfgBase, PmaaddrBase, pma)

  // uat support
  val uatp = dontTouch(RegInit(0.U(64.W)))
  val uatc = dontTouch(RegInit(0.U(64.W)))
  val ucid = dontTouch(RegInit(0.U(64.W)))
  val ucsp = dontTouch(RegInit(0.U(64.W)))

  // Superviser-Level CSRs

  // val sstatus = RegInit(UInt(XLEN.W), "h00000000".U)
  val sstatusWmask = "hc6122".U(XLEN.W)
  // Sstatus Write Mask
  // -------------------------------------------------------
  //    19           9   5     2
  // 0  1100 0000 0001 0010 0010
  // 0  c    0    1    2    2
  // -------------------------------------------------------
  val sstatusRmask = sstatusWmask | "h8000000300018000".U
  // Sstatus Read Mask = (SSTATUS_WMASK | (0xf << 13) | (1ull << 63) | (3ull << 32))
  // stvec: {BASE (WARL), MODE (WARL)} where mode is 0 or 1
  val stvecMask = ~(0x2.U(XLEN.W))
  val stvec = RegInit(UInt(XLEN.W), 0.U)
  // val sie = RegInit(0.U(XLEN.W))
  val sieMask = "h222".U & mideleg
  val sipMask = "h222".U & mideleg
  val sipWMask = "h2".U(XLEN.W) // ssip is writeable in smode
  val satp = if(EnbaleTlbDebug) RegInit(UInt(XLEN.W), "h8000000000087fbe".U) else RegInit(0.U(XLEN.W))
  // val satp = RegInit(UInt(XLEN.W), "h8000000000087fbe".U) // only use for tlb naive debug
  // val satpMask = "h80000fffffffffff".U(XLEN.W) // disable asid, mode can only be 8 / 0
  // TODO: use config to control the length of asid
  // val satpMask = "h8fffffffffffffff".U(XLEN.W) // enable asid, mode can only be 8 / 0
  val satpMask = Cat("hf".U(Satp_Mode_len.W), satp_part_wmask(Satp_Asid_len, AsidLength), satp_part_wmask(Satp_Addr_len, PAddrBits-12))
  val sepc = RegInit(UInt(XLEN.W), 0.U)
  // Page 60 in riscv-priv: The low bit of sepc (sepc[0]) is always zero.
  val sepcMask = ~(0x1.U(XLEN.W))
  val scause = RegInit(UInt(XLEN.W), 0.U)
  val stval = Reg(UInt(XLEN.W))
  val sscratch = RegInit(UInt(XLEN.W), 0.U)
  val scounteren = RegInit(UInt(XLEN.W), 0.U)

  // sbpctl
  // Bits 0-7: {LOOP, RAS, SC, TAGE, BIM, BTB, uBTB}
  val sbpctl = RegInit(UInt(XLEN.W), "h7f".U)
  csrio.customCtrl.bp_ctrl.ubtb_enable := sbpctl(0)
  csrio.customCtrl.bp_ctrl.btb_enable  := sbpctl(1)
  csrio.customCtrl.bp_ctrl.bim_enable  := sbpctl(2)
  csrio.customCtrl.bp_ctrl.tage_enable := sbpctl(3)
  csrio.customCtrl.bp_ctrl.sc_enable   := sbpctl(4)
  csrio.customCtrl.bp_ctrl.ras_enable  := sbpctl(5)
  csrio.customCtrl.bp_ctrl.loop_enable := sbpctl(6)

  // spfctl Bit 0: L1I Cache Prefetcher Enable
  // spfctl Bit 1: L2Cache Prefetcher Enable
  // spfctl Bit 2: L1D Cache Prefetcher Enable
  // spfctl Bit 3: L1D train prefetch on hit
  // spfctl Bit 4: L1D prefetch enable agt
  // spfctl Bit 5: L1D prefetch enable pht
  // spfctl Bit [9:6]: L1D prefetch active page threshold
  // spfctl Bit [15:10]: L1D prefetch active page stride
  // turn off L2 BOP, turn on L1 SMS by default
  val spfctl = RegInit(UInt(XLEN.W), Seq(
    0 << 17,    // L2 pf store only [17] init: false
    1 << 16,    // L1D pf enable stride [16] init: true
    30 << 10,   // L1D active page stride [15:10] init: 30
    12 << 6,    // L1D active page threshold [9:6] init: 12
    1  << 5,    // L1D enable pht [5] init: true
    1  << 4,    // L1D enable agt [4] init: true
    0  << 3,    // L1D train on hit [3] init: false
    1  << 2,    // L1D pf enable [2] init: true
    1  << 1,    // L2 pf enable [1] init: true
    1  << 0,    // L1I pf enable [0] init: true
  ).reduce(_|_).U(XLEN.W))
  csrio.customCtrl.l1I_pf_enable := spfctl(0)
  csrio.customCtrl.l2_pf_enable := spfctl(1)
  csrio.customCtrl.l1D_pf_enable := spfctl(2)
  csrio.customCtrl.l1D_pf_train_on_hit := spfctl(3)
  csrio.customCtrl.l1D_pf_enable_agt := spfctl(4)
  csrio.customCtrl.l1D_pf_enable_pht := spfctl(5)
  csrio.customCtrl.l1D_pf_active_threshold := spfctl(9, 6)
  csrio.customCtrl.l1D_pf_active_stride := spfctl(15, 10)
  csrio.customCtrl.l1D_pf_enable_stride := spfctl(16)
  csrio.customCtrl.l2_pf_store_only := spfctl(17)

  // sfetchctl Bit 0: L1I Cache Parity check enable
  val sfetchctl = RegInit(UInt(XLEN.W), "b0".U)
  csrio.customCtrl.icache_parity_enable := sfetchctl(0)

  // sdsid: Differentiated Services ID
  val sdsid = RegInit(UInt(XLEN.W), 0.U)
  csrio.customCtrl.dsid := sdsid

  // slvpredctl: load violation predict settings
  // Default reset period: 2^16
  // Why this number: reset more frequently while keeping the overhead low
  // Overhead: extra two redirections in every 64K cycles => ~0.1% overhead
  val slvpredctl = RegInit(UInt(XLEN.W), "h60".U)
  csrio.customCtrl.lvpred_disable := slvpredctl(0)
  csrio.customCtrl.no_spec_load := slvpredctl(1)
  csrio.customCtrl.storeset_wait_store := slvpredctl(2)
  csrio.customCtrl.storeset_no_fast_wakeup := slvpredctl(3)
  csrio.customCtrl.lvpred_timeout := slvpredctl(8, 4)

  // smblockctl: memory block configurations
  // bits 0-3: store buffer flush threshold (default: 8 entries)
  val smblockctl_init_val =
    (0xf & StoreBufferThreshold) |
    (EnableLdVioCheckAfterReset.toInt << 4) |
    (EnableSoftPrefetchAfterReset.toInt << 5) |
    (EnableCacheErrorAfterReset.toInt << 6) |
    (EnablePTWPreferCache.toInt << 7)
  val smblockctl = RegInit(UInt(XLEN.W), smblockctl_init_val.U)
  csrio.customCtrl.sbuffer_threshold := smblockctl(3, 0)
  // bits 4: enable load load violation check
  csrio.customCtrl.ldld_vio_check_enable := smblockctl(4)
  csrio.customCtrl.soft_prefetch_enable := smblockctl(5)
  csrio.customCtrl.cache_error_enable := smblockctl(6)
  csrio.customCtrl.ptw_prefercache_enable := smblockctl(7)

  println("CSR smblockctl init value:")
  println("  Store buffer replace threshold: " + StoreBufferThreshold)
  println("  Enable ld-ld vio check after reset: " + EnableLdVioCheckAfterReset)
  println("  Enable soft prefetch after reset: " + EnableSoftPrefetchAfterReset)
  println("  Enable cache error after reset: " + EnableCacheErrorAfterReset)

  val srnctl = RegInit(UInt(XLEN.W), "hf".U)
  csrio.customCtrl.fusion_enable := srnctl(0)
  csrio.customCtrl.svinval_enable := srnctl(1)
  csrio.customCtrl.wfi_enable := srnctl(2)
  csrio.customCtrl.move_elim_enable := srnctl(3)

  val tlbBundle = Wire(new TlbCsrBundle)
  csrio.tlb := tlbBundle

  // User-Level CSRs
  val uepc = Reg(UInt(XLEN.W))

  // fcsr
  class FcsrStruct extends Bundle {
    val reserved = UInt((XLEN-3-5).W)
    val frm = UInt(3.W)
    val fflags = UInt(5.W)
    assert(this.getWidth == XLEN)
  }
  val fcsr = RegInit(0.U(XLEN.W))
  // set mstatus->sd and mstatus->fs when true
  val csrw_dirty_fp_state = WireInit(false.B)

  def frm_wfn(wdata: UInt): UInt = {
    val fcsrOld = WireInit(fcsr.asTypeOf(new FcsrStruct))
    csrw_dirty_fp_state := true.B
    fcsrOld.frm := wdata(2,0)
    fcsrOld.asUInt
  }
  def frm_rfn(rdata: UInt): UInt = rdata(7,5)

  def fflags_wfn(update: Boolean)(wdata: UInt): UInt = {
    val fcsrOld = fcsr.asTypeOf(new FcsrStruct)
    val fcsrNew = WireInit(fcsrOld)
    csrw_dirty_fp_state := true.B
    if (update) {
      fcsrNew.fflags := wdata(4,0) | fcsrOld.fflags
    } else {
      fcsrNew.fflags := wdata(4,0)
    }
    fcsrNew.asUInt
  }
  def fflags_rfn(rdata:UInt): UInt = rdata(4,0)

  def fcsr_wfn(wdata: UInt): UInt = {
    val fcsrOld = WireInit(fcsr.asTypeOf(new FcsrStruct))
    csrw_dirty_fp_state := true.B
    Cat(fcsrOld.reserved, wdata.asTypeOf(fcsrOld).frm, wdata.asTypeOf(fcsrOld).fflags)
  }

  val fcsrMapping = Map(
    MaskedRegMap(Fflags, fcsr, wfn = fflags_wfn(update = false), rfn = fflags_rfn),
    MaskedRegMap(Frm, fcsr, wfn = frm_wfn, rfn = frm_rfn),
    MaskedRegMap(Fcsr, fcsr, wfn = fcsr_wfn)
  )

  // Hart Priviledge Mode
  val priviledgeMode = RegInit(UInt(2.W), ModeM)

  //val perfEventscounten = List.fill(nrPerfCnts)(RegInit(false(Bool())))
  // Perf Counter
  val nrPerfCnts = 29  // 3...31
  val priviledgeModeOH = UIntToOH(priviledgeMode)
  val perfEventscounten = RegInit(0.U.asTypeOf(Vec(nrPerfCnts, Bool())))
  val perfCnts   = List.fill(nrPerfCnts)(RegInit(0.U(XLEN.W)))
  val perfEvents = List.fill(8)(RegInit("h0000000000".U(XLEN.W))) ++
                   List.fill(8)(RegInit("h4010040100".U(XLEN.W))) ++
                   List.fill(8)(RegInit("h8020080200".U(XLEN.W))) ++
                   List.fill(5)(RegInit("hc0300c0300".U(XLEN.W)))
  for (i <-0 until nrPerfCnts) {
    perfEventscounten(i) := (perfEvents(i)(63,60) & priviledgeModeOH).orR
  }

  val hpmEvents = Wire(Vec(numPCntHc * coreParams.L2NBanks, new PerfEvent))
  for (i <- 0 until numPCntHc * coreParams.L2NBanks) {
    hpmEvents(i) := csrio.perf.perfEventsHc(i)
  }

  // print perfEvents
  val allPerfEvents = hpmEvents.map(x => (s"Hc", x.value))
  if (printEventCoding) {
    for (((name, inc), i) <- allPerfEvents.zipWithIndex) {
      println("CSR perfEvents Set", name, inc, i)
    }
  }

  val csrevents = perfEvents.slice(24, 29)
  val hpm_hc = HPerfMonitor(csrevents, hpmEvents)
  val mcountinhibit = RegInit(0.U(XLEN.W))
  val mcycle = RegInit(0.U(XLEN.W))
  mcycle := mcycle + 1.U
  val minstret = RegInit(0.U(XLEN.W))
  val perf_events = csrio.perf.perfEventsFrontend ++
                    csrio.perf.perfEventsCtrl ++
                    csrio.perf.perfEventsLsu ++
                    hpm_hc.getPerf
  minstret := minstret + RegNext(csrio.perf.retiredInstr)
  for(i <- 0 until 29){
    perfCnts(i) := Mux(mcountinhibit(i+3) | !perfEventscounten(i), perfCnts(i), perfCnts(i) + perf_events(i).value)
  }

  if (coreParams.L2CacheParamsOpt.isEmpty) {
    val u = RegNext(priviledgeMode === ModeU)
    val s = RegNext(priviledgeMode === ModeS)
    val r = RegNext(csrio.perf.retiredInstr)

    perfCnts(25) := perfCnts(25) + Mux(u, 1.U, 0.U)
    perfCnts(26) := perfCnts(26) + Mux(u, r,   0.U)
    perfCnts(27) := perfCnts(27) + Mux(s, 1.U, 0.U)
    perfCnts(28) := perfCnts(28) + Mux(s, r,   0.U)
  }

  val sfsbbase = dontTouch(Reg(UInt(64.W)))
  val sfsbmask = dontTouch(Reg(UInt(64.W)))
  val sfsbhead = dontTouch(Reg(UInt(64.W)))

  csrio.fsb.base := sfsbbase
  csrio.fsb.mask := sfsbmask
  csrio.fsb.head := sfsbhead

  // CSR reg map
  val basicPrivMapping = Map(

    // Unprivileged Floating-Point CSRs
    // Has been mapped above

    // Unprivileged Counter/Timers
    MaskedRegMap(Cycle,   mcycle),
    // We don't support read time CSR.
    // MaskedRegMap(Time, mtime),
    MaskedRegMap(Instret, minstret),

    //--- Supervisor Trap Setup ---
    MaskedRegMap(Sstatus, mstatus, sstatusWmask, mstatusUpdateSideEffect, sstatusRmask),
    // MaskedRegMap(Sedeleg, Sedeleg),
    // MaskedRegMap(Sideleg, Sideleg),
    MaskedRegMap(Sie, mie, sieMask, MaskedRegMap.NoSideEffect, sieMask),
    MaskedRegMap(Stvec, stvec, stvecMask, MaskedRegMap.NoSideEffect, stvecMask),
    MaskedRegMap(Scounteren, scounteren),

    //--- Supervisor Trap Handling ---
    MaskedRegMap(Sscratch, sscratch),
    MaskedRegMap(Sepc, sepc, sepcMask, MaskedRegMap.NoSideEffect, sepcMask),
    MaskedRegMap(Scause, scause),
    MaskedRegMap(Stval, stval),
    MaskedRegMap(Sip, mip.asUInt, sipWMask, MaskedRegMap.Unwritable, sipMask),

    //--- Supervisor Protection and Translation ---
    MaskedRegMap(Satp, satp, satpMask, MaskedRegMap.NoSideEffect, satpMask),

    //--- Supervisor Custom Read/Write Registers
    MaskedRegMap(Sbpctl, sbpctl),
    MaskedRegMap(Spfctl, spfctl),
    MaskedRegMap(Sfetchctl, sfetchctl),
    MaskedRegMap(Sdsid, sdsid),
    MaskedRegMap(Slvpredctl, slvpredctl),
    MaskedRegMap(Smblockctl, smblockctl),
    MaskedRegMap(Srnctl, srnctl),

    // uat support
    MaskedRegMap(Uatp, uatp),
    MaskedRegMap(Uatc, uatc),
    MaskedRegMap(Ucid, ucid),
    MaskedRegMap(Ucsp, ucsp),

    MaskedRegMap(Sfsbbase, sfsbbase),
    MaskedRegMap(Sfsbmask, sfsbmask),
    MaskedRegMap(Sfsbhead, sfsbhead),
    MaskedRegMap(Sfsbtail, 0.U, wfn = null, rfn = _ => csrio.fsb.tail),

    //--- Machine Information Registers ---
    MaskedRegMap(Mvendorid, mvendorid, 0.U(XLEN.W), MaskedRegMap.Unwritable),
    MaskedRegMap(Marchid, marchid, 0.U(XLEN.W), MaskedRegMap.Unwritable),
    MaskedRegMap(Mimpid, mimpid, 0.U(XLEN.W), MaskedRegMap.Unwritable),
    MaskedRegMap(Mhartid, mhartid, 0.U(XLEN.W), MaskedRegMap.Unwritable),
    MaskedRegMap(Mconfigptr, mconfigptr, 0.U(XLEN.W), MaskedRegMap.Unwritable),

    //--- Machine Trap Setup ---
    MaskedRegMap(Mstatus, mstatus, mstatusWMask, mstatusUpdateSideEffect, mstatusMask),
    MaskedRegMap(Misa, misa, 0.U, MaskedRegMap.Unwritable), // now whole misa is unchangeable
    MaskedRegMap(Medeleg, medeleg, "hf00b3ff".U(XLEN.W)),
    MaskedRegMap(Mideleg, mideleg, "h222".U(XLEN.W)),
    MaskedRegMap(Mie, mie),
    MaskedRegMap(Mtvec, mtvec, mtvecMask, MaskedRegMap.NoSideEffect, mtvecMask),
    MaskedRegMap(Mcounteren, mcounteren),

    //--- Machine Trap Handling ---
    MaskedRegMap(Mscratch, mscratch),
    MaskedRegMap(Mepc, mepc, mepcMask, MaskedRegMap.NoSideEffect, mepcMask),
    MaskedRegMap(Mcause, mcause),
    MaskedRegMap(Mtval, mtval),
    MaskedRegMap(Mip, mip.asUInt, 0.U(XLEN.W), MaskedRegMap.Unwritable),

    //--- Debug Mode ---
    MaskedRegMap(Dcsr, dcsr, dcsrMask, dcsrUpdateSideEffect),
    MaskedRegMap(Dpc, dpc),
    MaskedRegMap(Dscratch0, dscratch0),
    MaskedRegMap(Dscratch1, dscratch1),
    MaskedRegMap(Mcountinhibit, mcountinhibit),
    MaskedRegMap(Mcycle, mcycle),
    MaskedRegMap(Minstret, minstret),
  )

  val perfCntMapping = (0 until 29).map(i => {Map(
    MaskedRegMap(addr = Mhpmevent3 +i,
                 reg  = perfEvents(i),
                 wmask = "hf87fff3fcff3fcff".U(XLEN.W)),
    MaskedRegMap(addr = Mhpmcounter3 +i,
                 reg  = perfCnts(i)),
    MaskedRegMap(addr = Hpmcounter3 + i,
                 reg  = perfCnts(i))
  )}).fold(Map())((a,b) => a ++ b)
  // TODO: mechanism should be implemented later
  // val MhpmcounterStart = Mhpmcounter3
  // val MhpmeventStart   = Mhpmevent3
  // for (i <- 0 until nrPerfCnts) {
  //   perfCntMapping += MaskedRegMap(MhpmcounterStart + i, perfCnts(i))
  //   perfCntMapping += MaskedRegMap(MhpmeventStart + i, perfEvents(i))
  // }

  val cacheopRegs = CacheInstrucion.CacheInsRegisterList.map{case (name, attribute) => {
    name -> RegInit(0.U(attribute("width").toInt.W))
  }}
  val cacheopMapping = CacheInstrucion.CacheInsRegisterList.map{case (name, attribute) => {
    MaskedRegMap(
      Scachebase + attribute("offset").toInt,
      cacheopRegs(name)
    )
  }}

  val mapping = basicPrivMapping ++
                perfCntMapping ++
                pmpMapping ++
                pmaMapping ++
                (if (HasFPU) fcsrMapping else Nil) ++
                (if (HasCustomCSRCacheOp) cacheopMapping else Nil)

  println("XiangShan CSR Lists")

  for (addr <- mapping.keys.toSeq.sorted) {
    println(f"$addr%#03x ${mapping(addr)._1}")
  }

  val addr = src2(11, 0)
  val csri = ZeroExt(src2(16, 12), XLEN)
  val rdata = Wire(UInt(XLEN.W))
  val wdata = LookupTree(func, List(
    CSROpType.wrt  -> src1,
    CSROpType.set  -> (rdata | src1),
    CSROpType.clr  -> (rdata & (~src1).asUInt),
    CSROpType.wrti -> csri,
    CSROpType.seti -> (rdata | csri),
    CSROpType.clri -> (rdata & (~csri).asUInt)
  ))

  val addrInPerfCnt = (addr >= Mcycle.U) && (addr <= Mhpmcounter31.U) ||
    (addr >= Mcountinhibit.U) && (addr <= Mhpmevent31.U) ||
    (addr >= Cycle.U) && (addr <= Hpmcounter31.U) ||
    addr === Mip.U
  csrio.isPerfCnt := addrInPerfCnt && valid && func =/= CSROpType.jmp

  // satp wen check
  val satpLegalMode = (wdata.asTypeOf(new TlbSatpBundle).mode===0.U) || (wdata.asTypeOf(new TlbSatpBundle).mode===8.U) || (wdata.asTypeOf(new TlbSatpBundle).mode===15.U)

  // csr access check, special case
  val tvmNotPermit = (priviledgeMode === ModeS && mstatusStruct.tvm.asBool)
  val accessPermitted = !(addr === Satp.U && tvmNotPermit)
  csrio.disableSfence := tvmNotPermit

  // general CSR wen check
  val wen = valid && func =/= CSROpType.jmp && (addr=/=Satp.U || satpLegalMode)
  val dcsrPermitted = dcsrPermissionCheck(addr, false.B, debugMode)
  val triggerPermitted = triggerPermissionCheck(addr, true.B, debugMode) // todo dmode
  val modePermitted = csrAccessPermissionCheck(addr, false.B, priviledgeMode) && dcsrPermitted && triggerPermitted
  val perfcntPermitted = perfcntPermissionCheck(addr, priviledgeMode, mcounteren, scounteren)
  val permitted = Mux(addrInPerfCnt, perfcntPermitted, modePermitted) && accessPermitted

  MaskedRegMap.generate(mapping, addr, rdata, wen && permitted, wdata)
  io.out.bits.data := rdata
  io.out.bits.uop := io.in.bits.uop
  io.out.bits.uop.cf := cfOut
  io.out.bits.uop.ctrl.flushPipe := flushPipe

  // send distribute csr a w signal
  csrio.customCtrl.distribute_csr.w.valid := wen && permitted
  csrio.customCtrl.distribute_csr.w.bits.data := wdata
  csrio.customCtrl.distribute_csr.w.bits.addr := addr

  // Fix Mip/Sip write
  val fixMapping = Map(
    MaskedRegMap(Mip, mipReg.asUInt, mipFixMask),
    MaskedRegMap(Sip, mipReg.asUInt, sipWMask, MaskedRegMap.NoSideEffect, sipMask)
  )
  val rdataFix = Wire(UInt(XLEN.W))
  val wdataFix = LookupTree(func, List(
    CSROpType.wrt  -> src1,
    CSROpType.set  -> (rdataFix | src1),
    CSROpType.clr  -> (rdataFix & (~src1).asUInt),
    CSROpType.wrti -> csri,
    CSROpType.seti -> (rdataFix | csri),
    CSROpType.clri -> (rdataFix & (~csri).asUInt)
  ))
  MaskedRegMap.generate(fixMapping, addr, rdataFix, wen && permitted, wdataFix)

  when (RegNext(csrio.fpu.fflags.valid)) {
    fcsr := fflags_wfn(update = true)(RegNext(csrio.fpu.fflags.bits))
  }
  // set fs and sd in mstatus
  when (csrw_dirty_fp_state || RegNext(csrio.fpu.dirty_fs)) {
    val mstatusNew = WireInit(mstatus.asTypeOf(new MstatusStruct))
    mstatusNew.fs := "b11".U
    mstatusNew.sd := true.B
    mstatus := mstatusNew.asUInt
  }
  csrio.fpu.frm := fcsr.asTypeOf(new FcsrStruct).frm

  // CSR inst decode
  val isEbreak = addr === privEbreak && func === CSROpType.jmp
  val isEcall  = addr === privEcall  && func === CSROpType.jmp
  val isMret   = addr === privMret   && func === CSROpType.jmp
  val isSret   = addr === privSret   && func === CSROpType.jmp
  val isUret   = addr === privUret   && func === CSROpType.jmp
  val isDret   = addr === privDret   && func === CSROpType.jmp
  val isWFI    = func === CSROpType.wfi

  XSDebug(wen, "csr write: pc %x addr %x rdata %x wdata %x func %x\n", cfIn.pc, addr, rdata, wdata, func)
  XSDebug(wen, "pc %x mstatus %x mideleg %x medeleg %x mode %x\n", cfIn.pc, mstatus, mideleg , medeleg, priviledgeMode)

  // Illegal priviledged operation list
  val illegalMret = valid && isMret && priviledgeMode < ModeM
  val illegalSret = valid && isSret && priviledgeMode < ModeS
  val illegalSModeSret = valid && isSret && priviledgeMode === ModeS && mstatusStruct.tsr.asBool
  // When TW=1, then if WFI is executed in any less-privileged mode,
  // and it does not complete within an implementation-specific, bounded time limit,
  // the WFI instruction causes an illegal instruction exception.
  // The time limit may always be 0, in which case WFI always causes
  // an illegal instruction exception in less-privileged modes when TW=1.
  val illegalWFI = valid && isWFI && priviledgeMode < ModeM && mstatusStruct.tw === 1.U

  // Illegal priviledged instruction check
  val isIllegalAddr = valid && CSROpType.needAccess(func) && MaskedRegMap.isIllegalAddr(mapping, addr)
  val isIllegalAccess = wen && !permitted
  val isIllegalPrivOp = illegalMret || illegalSret || illegalSModeSret || illegalWFI

  // expose several csr bits for tlb
  tlbBundle.priv.mxr   := mstatusStruct.mxr.asBool
  tlbBundle.priv.sum   := mstatusStruct.sum.asBool
  tlbBundle.priv.imode := priviledgeMode
  tlbBundle.priv.dmode := Mux(debugMode && dcsr.asTypeOf(new DcsrStruct).mprven, ModeM, Mux(mstatusStruct.mprv.asBool, mstatusStruct.mpp, priviledgeMode))

  // Branch control
  val retTarget = WireInit(0.U)
  val resetSatp = addr === Satp.U && wen // write to satp will cause the pipeline be flushed
  val w_fcsr_change_rm = wen && addr === Fcsr.U && wdata(7, 5) =/= fcsr(7, 5)
  val w_frm_change_rm = wen && addr === Frm.U && wdata(2, 0) =/= fcsr(7, 5)
  val frm_change = w_fcsr_change_rm || w_frm_change_rm
  val isXRet = valid && func === CSROpType.jmp && !isEcall && !isEbreak

  val resetUatp = addr === Uatp.U && wen
  val resetUatc = addr === Uatc.U && wen
  val resetUcid = addr === Ucid.U && wen

  flushPipe := resetSatp || resetUatp || resetUatc || resetUcid || frm_change || isXRet

  tlbBundle.satp.apply(satp)
  tlbBundle.uatp.apply(uatp)
  tlbBundle.uatc.apply(uatc, wdata, resetUatc)
  tlbBundle.ucid.apply(ucid)

  tlbBundle.satp_changed := RegNext(resetSatp, false.B)
  tlbBundle.uatp_changed := RegNext(resetUatp, false.B)
  tlbBundle.uatc_changed := RegNext(resetUatc, false.B)
  tlbBundle.ucid_changed := RegNext(resetUcid, false.B)

  tlbBundle.uatm.h  := mhartid(5, 0)
  tlbBundle.uatm.c  := mcycle
  tlbBundle.uatm.cu := perfCnts(25)
  tlbBundle.uatm.i  := minstret
  tlbBundle.uatm.iu := perfCnts(26)

  private val illegalRetTarget = WireInit(false.B)

  // Mux tree for wires
  when (valid) {
    when (isDret) {
      retTarget := dpc(VAddrBits-1, 0)
    }.elsewhen (isMret && !illegalMret) {
      retTarget := mepc(VAddrBits-1, 0)
    }.elsewhen (isSret && !illegalSret && !illegalSModeSret) {
      retTarget := sepc(VAddrBits-1, 0)
    }.elsewhen (isUret) {
      retTarget := uepc(VAddrBits-1, 0)
    }.otherwise {
      illegalRetTarget := true.B
    }
  }.otherwise {
    illegalRetTarget := true.B // when illegalRetTarget setted, retTarget should never be used
  }

  // Mux tree for regs
  when (valid) {
    when (isDret) {
      val mstatusNew = WireInit(mstatus.asTypeOf(new MstatusStruct))
      val debugModeNew = WireInit(debugMode)
      when (dcsr.asTypeOf(new DcsrStruct).prv =/= ModeM) {mstatusNew.mprv := 0.U} //If the new privilege mode is less privileged than M-mode, MPRV in mstatus is cleared.
      mstatus := mstatusNew.asUInt
      priviledgeMode := dcsr.asTypeOf(new DcsrStruct).prv
      debugModeNew := false.B
      debugIntrEnable := true.B
      debugMode := debugModeNew
      XSDebug("Debug Mode: Dret executed, returning to %x.", retTarget)
    }.elsewhen(isMret && !illegalMret) {
      val mstatusOld = WireInit(mstatus.asTypeOf(new MstatusStruct))
      val mstatusNew = WireInit(mstatus.asTypeOf(new MstatusStruct))
      mstatusNew.ie.m := mstatusOld.pie.m
      priviledgeMode := mstatusOld.mpp
      mstatusNew.pie.m := true.B
      mstatusNew.mpp := ModeU
      when (mstatusOld.mpp =/= ModeM) { mstatusNew.mprv := 0.U }
      mstatus := mstatusNew.asUInt
    }.elsewhen(isSret && !illegalSret && !illegalSModeSret) {
      val mstatusOld = WireInit(mstatus.asTypeOf(new MstatusStruct))
      val mstatusNew = WireInit(mstatus.asTypeOf(new MstatusStruct))
      mstatusNew.ie.s := mstatusOld.pie.s
      priviledgeMode := Cat(0.U(1.W), mstatusOld.spp)
      mstatusNew.pie.s := true.B
      mstatusNew.spp := ModeU
      mstatus := mstatusNew.asUInt
      when (mstatusOld.spp =/= ModeM) { mstatusNew.mprv := 0.U }
    }.elsewhen(isUret) {
      val mstatusOld = WireInit(mstatus.asTypeOf(new MstatusStruct))
      val mstatusNew = WireInit(mstatus.asTypeOf(new MstatusStruct))
      // mstatusNew.mpp.m := ModeU //TODO: add mode U
      mstatusNew.ie.u := mstatusOld.pie.u
      priviledgeMode := ModeU
      mstatusNew.pie.u := true.B
      mstatus := mstatusNew.asUInt
    }
  }

  io.in.ready := true.B
  io.out.valid := valid

  // In this situation, hart will enter debug mode instead of handling a breakpoint exception simply.
  // Ebreak block instructions backwards, so it's ok to not keep extra info to distinguish between breakpoint
  // exception and enter-debug-mode exception.
  val ebreakEnterDebugMode =
    (priviledgeMode === ModeM && dcsrData.ebreakm) ||
    (priviledgeMode === ModeS && dcsrData.ebreaks) ||
    (priviledgeMode === ModeU && dcsrData.ebreaku)

  // raise a debug exception waiting to enter debug mode, instead of a breakpoint exception
  val raiseDebugException = !debugMode && isEbreak && ebreakEnterDebugMode

  val csrExceptionVec = WireInit(cfIn.exceptionVec)
  csrExceptionVec(breakPoint) := io.in.valid && isEbreak
  csrExceptionVec(ecallM) := priviledgeMode === ModeM && io.in.valid && isEcall
  csrExceptionVec(ecallS) := priviledgeMode === ModeS && io.in.valid && isEcall
  csrExceptionVec(ecallU) := priviledgeMode === ModeU && io.in.valid && isEcall
  // Trigger an illegal instr exception when:
  // * unimplemented csr is being read/written
  // * csr access is illegal
  csrExceptionVec(illegalInstr) := isIllegalAddr || isIllegalAccess || isIllegalPrivOp
  cfOut.exceptionVec := csrExceptionVec

  XSDebug(io.in.valid, s"Debug Mode: an Ebreak is executed, ebreak cause enter-debug-mode exception ? ${raiseDebugException}\n")

  /**
    * Exception and Intr
    */
  val ideleg =  (mideleg & mip.asUInt)
  def priviledgedEnableDetect(x: Bool): Bool = Mux(x, ((priviledgeMode === ModeS) && mstatusStruct.ie.s) || (priviledgeMode < ModeS),
    ((priviledgeMode === ModeM) && mstatusStruct.ie.m) || (priviledgeMode < ModeM))

  val debugIntr = csrio.externalInterrupt.debug & debugIntrEnable
  XSDebug(debugIntr, "Debug Mode: debug interrupt is asserted and valid!")
  // send interrupt information to ROB
  val intrVecEnable = Wire(Vec(12, Bool()))
  val disableInterrupt = debugMode || (dcsrData.step && !dcsrData.stepie)
  intrVecEnable.zip(ideleg.asBools).map{case(x,y) => x := priviledgedEnableDetect(y) && !disableInterrupt}
  val intrVec = Cat(debugIntr && !debugMode, (mie(11, 0) & mip.asUInt & intrVecEnable.asUInt))
  val intrBitSet = intrVec.orR
  csrio.interrupt := intrBitSet || csrio.ise.valid
  // Page 45 in RISC-V Privileged Specification
  // The WFI instruction can also be executed when interrupts are disabled. The operation of WFI
  // must be unaffected by the global interrupt bits in mstatus (MIE and SIE) and the delegation
  // register mideleg, but should honor the individual interrupt enables (e.g, MTIE).
  csrio.wfi_event := debugIntr || (mie(11, 0) & mip.asUInt).orR
  mipWire.t.m := csrio.externalInterrupt.mtip
  mipWire.s.m := csrio.externalInterrupt.msip
  mipWire.e.m := csrio.externalInterrupt.meip
  mipWire.e.s := csrio.externalInterrupt.seip

  // interrupts
  val intrNO = IntPriority.foldRight(0.U)((i: Int, sum: UInt) => Mux(intrVec(i), i.U, sum))
  // Def: intrVec -> intrBitSet -> csrio.interrupt
  // T1: CSR.csrio.interrupt -> JumpCSRExeUnit -> FUBlock -> ExuBlock -> CtrlBlock
  //     -> ROB.io_csr_intrBitSet -> ROB.intrBitSetReg
  // T2: ROB.intrBitSetReg -> intrEnable/ROB.exceptionHappen -> io_exception_bits_isInterrupt_r
  // T3: ROB.io_exception_bits_isInterrupt_r -> ROB.io_exception_bits_isInterrupt
  //     -> CtrlBlock -> ExuBlock -> FuBlock -> JumpCSRExeUnit
  //     -> DelayN(2).io_in_bits_isInterrupt -> DelayN(2).REG_bits_isInterrupt
  // T4: DelayN(2).REG_bits_isInterrupt -> DelayN(2).io_out_bits_isInterrupt
  // Use: DelayN(2).io_out_bits_isInterrupt -> CSR.csrio_exception_bits_isInterrupt -> mcause
  val intrNOReg = DelayN(intrNO, 4)
  val hasIntr = csrio.exception.valid && csrio.exception.bits.isInterrupt && !csrio.ise.valid
  val ivmEnable = tlbBundle.priv.imode < ModeM && satp.asTypeOf(new TlbSatpBundle).mode === 8.U
  val iexceptionPC = Mux(ivmEnable, SignExt(csrio.exception.bits.uop.cf.pc, XLEN), csrio.exception.bits.uop.cf.pc)
  val dvmEnable = tlbBundle.priv.dmode < ModeM && satp.asTypeOf(new TlbSatpBundle).mode === 8.U
  val dexceptionPC = Mux(dvmEnable, SignExt(csrio.exception.bits.uop.cf.pc, XLEN), csrio.exception.bits.uop.cf.pc)
  XSDebug(hasIntr, "interrupt: pc=0x%x, %d\n", dexceptionPC, intrNOReg)
  val hasDebugIntr = intrNOReg === IRQ_DEBUG.U && hasIntr

  // exceptions
  val ise_expt = csrio.exception.valid && csrio.exception.bits.isInterrupt && csrio.ise.valid

  // exceptions from rob need to handle
  val exceptionVecFromRob   = csrio.exception.bits.uop.cf.exceptionVec
  val hasException          = csrio.exception.valid && !csrio.exception.bits.isInterrupt || ise_expt
  val hasInstrPageFault     = hasException && exceptionVecFromRob(instrPageFault)
  val hasLoadPageFault      = hasException && exceptionVecFromRob(loadPageFault)
  val hasStorePageFault     = hasException && exceptionVecFromRob(storePageFault)
  val hasStoreAddrMisalign  = hasException && exceptionVecFromRob(storeAddrMisaligned)
  val hasLoadAddrMisalign   = hasException && exceptionVecFromRob(loadAddrMisaligned)
  val hasInstrAccessFault   = hasException && exceptionVecFromRob(instrAccessFault)
  val hasLoadAccessFault    = hasException && exceptionVecFromRob(loadAccessFault)
  val hasStoreAccessFault   = hasException && exceptionVecFromRob(storeAccessFault)
  val hasDelayedInstrFault  = hasException && exceptionVecFromRob(delayedInstrFault)
  val hasDelayedLoadFault   = hasException && exceptionVecFromRob(delayedLoadFault)
  val hasDelayedStoreFault  = hasException && exceptionVecFromRob(delayedStoreFault) || ise_expt
  val hasBreakPoint         = hasException && exceptionVecFromRob(breakPoint)
  val hasSingleStep         = hasException && csrio.exception.bits.uop.ctrl.singleStep

  XSDebug(hasSingleStep, "Debug Mode: single step exception\n")

  val hasExceptionVec = Mux(ise_expt, VecInit(GenMask(delayedStoreFault).asBools), csrio.exception.bits.uop.cf.exceptionVec)
  val regularExceptionNO = ExceptionNO.priorities.foldRight(0.U)((i: Int, sum: UInt) => Mux(hasExceptionVec(i), i.U, sum))
  val exceptionNO = Mux(hasSingleStep, 3.U, regularExceptionNO)
  val causeNO = (hasIntr << (XLEN-1)).asUInt | Mux(hasIntr, intrNOReg, exceptionNO)

  val hasExceptionIntr = csrio.exception.valid

  val hasDebugEbreakException = hasBreakPoint && ebreakEnterDebugMode
  val hasDebugException = hasDebugEbreakException || hasSingleStep
  val hasDebugTrap = hasDebugException || hasDebugIntr
  val ebreakEnterParkLoop = debugMode && hasExceptionIntr

  XSDebug(hasExceptionIntr, "int/exc: pc %x int (%d):%x exc: (%d):%x\n",
    dexceptionPC, intrNOReg, intrVec, exceptionNO, hasExceptionVec.asUInt
  )
  XSDebug(hasExceptionIntr,
    "pc %x mstatus %x mideleg %x medeleg %x mode %x\n",
    dexceptionPC,
    mstatus,
    mideleg,
    medeleg,
    priviledgeMode
  )

  val
     (ise_fsm_idle  ::
      ise_fsm_coll  ::
      ise_fsm_drain ::
      ise_fsm_pend  ::
      ise_fsm_req   ::
      ise_fsm_null) = Enum(5)

  val ise_fsm_q   = dontTouch(Wire(UInt(3.W)))
  val ise_fsm_en  = dontTouch(Wire(Bool()))
  val ise_fsm_nxt = dontTouch(Wire(UInt(3.W)))

  ise_fsm_en  := false.B
  ise_fsm_nxt := ise_fsm_q

  val ise_taken = hasException     && (exceptionNO    === delayedStoreFault.U)
  val ise_other = hasExceptionIntr && (priviledgeMode === ModeU) && !csrio.ise.empty && !ise_taken

  switch (ise_fsm_q) {
    is (ise_fsm_idle) {
      ise_fsm_en  := ise_other || csrio.ise.expt
      ise_fsm_nxt := Mux(ise_other && csrio.ise.expt, ise_fsm_coll,
                     Mux(ise_other,                   ise_fsm_drain,
                                                      ise_fsm_pend))
    }
    is (ise_fsm_coll) {
      ise_fsm_en  := csrio.ise.empty
      ise_fsm_nxt := ise_fsm_req
    }
    is (ise_fsm_drain) {
      ise_fsm_en  := csrio.ise.expt || csrio.ise.empty
      ise_fsm_nxt := Mux(csrio.ise.expt,  ise_fsm_coll,
                                          ise_fsm_idle)
    }
    is (ise_fsm_pend) {
      ise_fsm_en  := ise_other || csrio.ise.empty
      ise_fsm_nxt := Mux(csrio.ise.empty, ise_fsm_req,
                                          ise_fsm_coll)
    }
    is (ise_fsm_req) {
      ise_fsm_en  := ise_taken
      ise_fsm_nxt := ise_fsm_idle
    }
  }

  ise_fsm_q := RegEnable(ise_fsm_nxt,
                         ise_fsm_idle,
                         ise_fsm_en)

  val ise_fsm_is_busy = ise_fsm_q(1, 0).orR
  val ise_fsm_is_req  = ise_fsm_q(2)

  csrio.ise.valid := ise_fsm_is_req &&
                       ((priviledgeMode === ModeU) ||
                        (priviledgeMode === ModeS) && mstatusStruct.ie.s)

  csrio.ise.drain := ise_fsm_is_busy

  // mtval write logic
  // Due to timing reasons of memExceptionVAddr, we delay the write of mtval and stval
  val memExceptionAddr = SignExt(csrio.memExceptionVAddr, XLEN)
  val updateTval = VecInit(Seq(
    hasInstrPageFault,
    hasLoadPageFault,
    hasStorePageFault,
    hasInstrAccessFault,
    hasLoadAccessFault,
    hasStoreAccessFault,
    hasLoadAddrMisalign,
    hasStoreAddrMisalign,
    hasDelayedInstrFault,
    hasDelayedLoadFault,
    hasDelayedStoreFault
  )).asUInt.orR
  when (RegNext(RegNext(updateTval))) {
      val tval = Mux(
        RegNext(RegNext(hasInstrPageFault || hasInstrAccessFault)),
        RegNext(RegNext(Mux(
          csrio.exception.bits.uop.cf.crossPageIPFFix,
          SignExt(csrio.exception.bits.uop.cf.pc + 2.U, XLEN),
          iexceptionPC
        ))),
        memExceptionAddr
    )
    when (RegNext(priviledgeMode === ModeM)) {
      mtval := tval
    }.otherwise {
      stval := tval
    }
  }

  val debugTrapTarget = Mux(!isEbreak && debugMode, 0x38020808.U, 0x38020800.U) // 0x808 is when an exception occurs in debug mode prog buf exec
  val deleg = Mux(hasIntr, mideleg , medeleg)
  // val delegS = ((deleg & (1 << (causeNO & 0xf))) != 0) && (priviledgeMode < ModeM);
  val delegS = deleg(causeNO(4, 0)) && (priviledgeMode < ModeM)
  val clearTval = !updateTval || hasIntr

  // ctrl block will use theses later for flush
  val isXRetFlag = RegInit(false.B)
  when (DelayN(io.redirectIn.valid, 5)) {
    isXRetFlag := false.B
  }.elsewhen (isXRet) {
    isXRetFlag := true.B
  }
  csrio.isXRet := isXRetFlag
  private val retTargetReg = RegEnable(retTarget, isXRet && !illegalRetTarget)
  private val illegalXret = RegEnable(illegalMret || illegalSret || illegalSModeSret, isXRet)

  private val xtvec = Mux(delegS, stvec, mtvec)
  private val xtvecBase = xtvec(VAddrBits - 1, 2)
  // When MODE=Vectored, all synchronous exceptions into M/S mode
  // cause the pc to be set to the address in the BASE field, whereas
  // interrupts cause the pc to be set to the address in the BASE field
  // plus four times the interrupt cause number.
  private val pcFromXtvec = Cat(xtvecBase + Mux(xtvec(0) && hasIntr, causeNO(4, 0), 0.U), 0.U(2.W))

  // XRet sends redirect instead of Flush and isXRetFlag is true.B before redirect.valid.
  // ROB sends exception at T0 while CSR receives at T2.
  // We add a RegNext here and trapTarget is valid at T3.
  csrio.trapTarget := RegEnable(
    MuxCase(pcFromXtvec, Seq(
      (isXRetFlag && !illegalXret) -> retTargetReg,
      ((hasDebugTrap && !debugMode) || ebreakEnterParkLoop) -> debugTrapTarget
    )),
    isXRetFlag || csrio.exception.valid)

  when (hasExceptionIntr) {
    val mstatusOld = WireInit(mstatus.asTypeOf(new MstatusStruct))
    val mstatusNew = WireInit(mstatus.asTypeOf(new MstatusStruct))
    val dcsrNew = WireInit(dcsr.asTypeOf(new DcsrStruct))
    val debugModeNew = WireInit(debugMode)
    when (hasDebugTrap && !debugMode) {
      import DcsrStruct._
      debugModeNew := true.B
      dcsrNew.prv := priviledgeMode
      priviledgeMode := ModeM
      when (hasDebugIntr) {
        dpc := iexceptionPC
        dcsrNew.cause := CAUSE_HALTREQ
        XSDebug(hasDebugIntr, "Debug Mode: Trap to %x at pc %x\n", debugTrapTarget, dpc)
      }.otherwise { // hasDebugException
        dpc := iexceptionPC // TODO: check it when hasSingleStep
        dcsrNew.cause := MuxCase(0.U, Seq(
          hasBreakPoint -> CAUSE_HALTREQ,
          hasSingleStep -> CAUSE_STEP
        ))
      }
      dcsr := dcsrNew.asUInt
      debugIntrEnable := false.B
    }.elsewhen (debugMode) {
      //do nothing
    }.elsewhen (delegS) {
      scause := causeNO
      sepc := Mux(hasInstrPageFault || hasInstrAccessFault, iexceptionPC, dexceptionPC)
      mstatusNew.spp := priviledgeMode
      mstatusNew.pie.s := mstatusOld.ie.s
      mstatusNew.ie.s := false.B
      priviledgeMode := ModeS
      when (clearTval) { stval := 0.U }
    }.otherwise {
      mcause := causeNO
      mepc := Mux(hasInstrPageFault || hasInstrAccessFault, iexceptionPC, dexceptionPC)
      mstatusNew.mpp := priviledgeMode
      mstatusNew.pie.m := mstatusOld.ie.m
      mstatusNew.ie.m := false.B
      priviledgeMode := ModeM
      when (clearTval) { mtval := 0.U }
    }
    mstatus := mstatusNew.asUInt
    debugMode := debugModeNew
  }

  XSDebug(hasExceptionIntr && delegS, "sepc is writen!!! pc:%x\n", cfIn.pc)

  // Distributed CSR update req
  //
  // For now we use it to implement customized cache op
  // It can be delayed if necessary

  val delayedUpdate0 = DelayN(csrio.distributedUpdate(0), 2)
  val delayedUpdate1 = DelayN(csrio.distributedUpdate(1), 2)
  val distributedUpdateValid = delayedUpdate0.w.valid || delayedUpdate1.w.valid
  val distributedUpdateAddr = Mux(delayedUpdate0.w.valid,
    delayedUpdate0.w.bits.addr,
    delayedUpdate1.w.bits.addr
  )
  val distributedUpdateData = Mux(delayedUpdate0.w.valid,
    delayedUpdate0.w.bits.data,
    delayedUpdate1.w.bits.data
  )

  assert(!(delayedUpdate0.w.valid && delayedUpdate1.w.valid))

  when(distributedUpdateValid){
    // cacheopRegs can be distributed updated
    CacheInstrucion.CacheInsRegisterList.map{case (name, attribute) => {
      when((Scachebase + attribute("offset").toInt).U === distributedUpdateAddr){
        cacheopRegs(name) := distributedUpdateData
      }
    }}
  }

  // Cache error debug support
  if(HasCustomCSRCacheOp){
    val cache_error_decoder = Module(new CSRCacheErrorDecoder)
    cache_error_decoder.io.encoded_cache_error := cacheopRegs("CACHE_ERROR")
  }

  // Implicit add reset values for mepc[0] and sepc[0]
  // TODO: rewrite mepc and sepc using a struct-like style with the LSB always being 0
  when (RegNext(RegNext(reset.asBool) && !reset.asBool)) {
    mepc := Cat(mepc(XLEN - 1, 1), 0.U(1.W))
    sepc := Cat(sepc(XLEN - 1, 1), 0.U(1.W))
  }

  def readWithScala(addr: Int): UInt = mapping(addr)._1

  val difftestIntrNO = Mux(hasIntr, causeNO, 0.U)

  // Always instantiate basic difftest modules.
  if (env.AlwaysBasicDiff || env.EnableDifftest) {
    val difftest = Module(new DifftestArchEvent)
    difftest.io.clock := clock
    difftest.io.reset := reset
    difftest.io.coreid := csrio.hartId
    difftest.io.intrNO := RegNext(RegNext(RegNext(difftestIntrNO)))
    difftest.io.cause  := RegNext(RegNext(RegNext(Mux(csrio.exception.valid, causeNO, 0.U))))
    difftest.io.exceptionPC := RegNext(RegNext(RegNext(dexceptionPC)))
    if (env.EnableDifftest) {
      difftest.io.exceptionInst := RegNext(RegNext(RegNext(csrio.exception.bits.uop.cf.instr)))
    }
  }

  // Always instantiate basic difftest modules.
  if (env.AlwaysBasicDiff || env.EnableDifftest) {
    val difftest = Module(new DifftestCSRState)
    difftest.io.clock := clock
    difftest.io.reset := reset
    difftest.io.coreid := csrio.hartId
    difftest.io.priviledgeMode := priviledgeMode
    difftest.io.mstatus := mstatus
    difftest.io.sstatus := mstatus & sstatusRmask
    difftest.io.mepc := mepc
    difftest.io.sepc := sepc
    difftest.io.mtval:= mtval
    difftest.io.stval:= stval
    difftest.io.mtvec := mtvec
    difftest.io.stvec := stvec
    difftest.io.mcause := mcause
    difftest.io.scause := scause
    difftest.io.satp := satp
    difftest.io.mip := mipReg
    difftest.io.mie := mie
    difftest.io.mscratch := mscratch
    difftest.io.sscratch := sscratch
    difftest.io.mideleg := mideleg
    difftest.io.medeleg := medeleg
  }

  if(env.AlwaysBasicDiff || env.EnableDifftest) {
    val difftest = Module(new DifftestDebugMode)
    difftest.io.clock := clock
    difftest.io.reset := reset
    difftest.io.coreid := csrio.hartId
    difftest.io.debugMode := debugMode
    difftest.io.dcsr := dcsr
    difftest.io.dpc := dpc
    difftest.io.dscratch0 := dscratch0
    difftest.io.dscratch1 := dscratch1
  }
}

class PFEvent(implicit p: Parameters) extends XSModule with HasCSRConst  {
  val io = IO(new Bundle {
    val distribute_csr = Flipped(new DistributedCSRIO())
    val hpmevent = Output(Vec(29, UInt(XLEN.W)))
  })

  val w = io.distribute_csr.w

  val perfEvents = List.fill(8)(RegInit("h0000000000".U(XLEN.W))) ++
                   List.fill(8)(RegInit("h4010040100".U(XLEN.W))) ++
                   List.fill(8)(RegInit("h8020080200".U(XLEN.W))) ++
                   List.fill(5)(RegInit("hc0300c0300".U(XLEN.W)))

  val perfEventMapping = (0 until 29).map(i => {Map(
    MaskedRegMap(addr = Mhpmevent3 +i,
                 reg  = perfEvents(i),
                 wmask = "hf87fff3fcff3fcff".U(XLEN.W))
  )}).fold(Map())((a,b) => a ++ b)

  val rdata = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(perfEventMapping, w.bits.addr, rdata, w.valid, w.bits.data)
  for(i <- 0 until 29){
    io.hpmevent(i) := perfEvents(i)
  }
}
