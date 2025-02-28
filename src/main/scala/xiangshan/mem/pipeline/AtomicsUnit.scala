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

package xiangshan.mem

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xiangshan._
import xiangshan.cache.{AtomicWordIO, MemoryOpConstants}
import xiangshan.cache.mmu.{TlbCmd, TlbRequestIO}
import difftest._
import xiangshan.ExceptionNO._
import xiangshan.backend.fu.PMPRespBundle

class AtomicsUnit(implicit p: Parameters) extends XSModule with MemoryOpConstants {
  val io = IO(new Bundle() {
    val hartId = Input(UInt(8.W))
    val in            = Flipped(Decoupled(new ExuInput))
    val storeDataIn   = Flipped(Valid(new ExuOutput)) // src2 from rs
    val out           = Decoupled(new ExuOutput)
    val dcache        = new AtomicWordIO
    val dtlb          = new TlbRequestIO()
    val pmpResp       = Flipped(new PMPRespBundle())
    val rsIdx         = Input(UInt(log2Up(IssQueSize).W))
    val flush_sbuffer = new SbufferFlushBundle
    val feedbackSlow  = ValidIO(new RSFeedback)
    val redirect      = Flipped(ValidIO(new Redirect))
    val exceptionAddr = ValidIO(UInt(VAddrBits.W))
    val csrCtrl       = Flipped(new CustomCSRCtrlIO)
  })

  //-------------------------------------------------------
  // Atomics Memory Accsess FSM
  //-------------------------------------------------------
  val s_invalid :: s_tlb :: s_pm :: s_flush_sbuffer_req :: s_flush_sbuffer_resp :: s_cache_req :: s_cache_resp :: s_finish :: Nil = Enum(8)
  val state = RegInit(s_invalid)
  val out_valid = RegInit(false.B)
  val data_valid = RegInit(false.B)
  val in = Reg(new ExuInput())
  val exceptionVec = RegInit(0.U.asTypeOf(ExceptionVec()))
  val atom_override_xtval = RegInit(false.B)
  val isLr = in.uop.ctrl.fuOpType === LSUOpType.lr_w || in.uop.ctrl.fuOpType === LSUOpType.lr_d
  // paddr after translation
  val paddr = Reg(UInt())
  val vaddr = in.src(0)
  val is_mmio = Reg(Bool())
  // pmp check
  val static_pm = Reg(Valid(Bool())) // valid for static, bits for mmio
  // dcache response data
  val resp_data = Reg(UInt())
  val resp_data_wire = WireInit(0.U)
  val is_lrsc_valid = Reg(Bool())


  // Difftest signals
  val paddr_reg = Reg(UInt(64.W))
  val data_reg = Reg(UInt(64.W))
  val mask_reg = Reg(UInt(8.W))
  val fuop_reg = Reg(UInt(8.W))

  io.exceptionAddr.valid := atom_override_xtval
  io.exceptionAddr.bits  := in.src(0)

  // assign default value to output signals
  io.in.ready          := false.B

  io.dcache.req.valid  := false.B
  io.dcache.req.bits   := DontCare
  io.dcache.resp.ready := false.B

  io.dtlb.req.valid    := false.B
  io.dtlb.req.bits     := DontCare
  io.dtlb.req_kill     := false.B
  io.dtlb.resp.ready   := true.B

  io.flush_sbuffer.valid := false.B

  XSDebug("state: %d\n", state)

  when (state === s_invalid) {
    io.in.ready := true.B
    when (io.in.fire) {
      in := io.in.bits
      in.src(1) := in.src(1) // leave src2 unchanged
      state := s_tlb
    }
  }

  when (io.storeDataIn.fire) {
    in.src(1) := io.storeDataIn.bits.data
    data_valid := true.B
  }

  assert(!(io.storeDataIn.fire && data_valid), "atomic unit re-receive data")

  // Send TLB feedback to store issue queue
  // we send feedback right after we receives request
  // also, we always treat amo as tlb hit
  // since we will continue polling tlb all by ourself
  io.feedbackSlow.valid       := RegNext(RegNext(io.in.valid))
  io.feedbackSlow.bits.hit    := true.B
  io.feedbackSlow.bits.rsIdx  := RegEnable(io.rsIdx, io.in.valid)
  io.feedbackSlow.bits.flushState := DontCare
  io.feedbackSlow.bits.sourceType := DontCare
  io.feedbackSlow.bits.dataInvalidSqIdx := DontCare

  // tlb translation, manipulating signals && deal with exception
  when (state === s_tlb) {
    // send req to dtlb
    // keep firing until tlb hit
    io.dtlb.req.valid       := true.B
    io.dtlb.req.bits.vaddr  := in.src(0)
    io.dtlb.req.bits.robIdx := in.uop.robIdx
    io.dtlb.resp.ready      := true.B
    io.dtlb.req.bits.cmd    := Mux(isLr, TlbCmd.atom_read, TlbCmd.atom_write)
    io.dtlb.req.bits.debug.pc := in.uop.cf.pc
    io.dtlb.req.bits.debug.isFirstIssue := false.B

    when(io.dtlb.resp.fire){
      paddr := io.dtlb.resp.bits.paddr(0)
      // exception handling
      val addrAligned = LookupTree(in.uop.ctrl.fuOpType(1,0), List(
        "b00".U   -> true.B,              //b
        "b01".U   -> (in.src(0)(0) === 0.U),   //h
        "b10".U   -> (in.src(0)(1,0) === 0.U), //w
        "b11".U   -> (in.src(0)(2,0) === 0.U)  //d
      ))

      val priv_chk = io.dtlb.resp.bits.priv && !in.uop.cf.priv

      exceptionVec(loadAddrMisaligned)  := !addrAligned && isLr
      exceptionVec(storeAddrMisaligned) := !addrAligned && !isLr
      exceptionVec(storePageFault)      := io.dtlb.resp.bits.excp(0).pf.st
      exceptionVec(loadPageFault)       := io.dtlb.resp.bits.excp(0).pf.ld
      exceptionVec(storeAccessFault)    := io.dtlb.resp.bits.excp(0).af.st || priv_chk && !isLr
      exceptionVec(loadAccessFault)     := io.dtlb.resp.bits.excp(0).af.ld || priv_chk &&  isLr
      exceptionVec(delayedLoadFault)    := false.B
      exceptionVec(delayedStoreFault)   := false.B
      static_pm := io.dtlb.resp.bits.static_pm

      when (!io.dtlb.resp.bits.miss) {
        when (!addrAligned) {
          // NOTE: when addrAligned, do not need to wait tlb actually
          // check for miss aligned exceptions, tlb exception are checked next cycle for timing
          // if there are exceptions, no need to execute it
          state := s_finish
          out_valid := true.B
          atom_override_xtval := true.B
        } .otherwise {
          state := s_pm
        }
      }
    }
  }

  when (state === s_pm) {
    val pmp = WireInit(io.pmpResp)
    when (static_pm.valid) {
      pmp.ld := false.B
      pmp.st := false.B
      pmp.instr := false.B
      pmp.mmio := static_pm.bits
    }
    is_mmio := pmp.mmio
    // NOTE: only handle load/store exception here, if other exception happens, don't send here
    val exception_va = exceptionVec(storePageFault) || exceptionVec(loadPageFault) ||
      exceptionVec(storeAccessFault) || exceptionVec(loadAccessFault)
    val exception_pa = pmp.st || pmp.ld
    when (exception_va || exception_pa) {
      state := s_finish
      out_valid := true.B
      atom_override_xtval := true.B
    }.otherwise {
      state := s_flush_sbuffer_req
    }
    // update storeAccessFault bit
    exceptionVec(loadAccessFault) := exceptionVec(loadAccessFault) || pmp.ld && isLr
    exceptionVec(storeAccessFault) := exceptionVec(storeAccessFault) || pmp.st || pmp.ld && !isLr
  }

  when (state === s_flush_sbuffer_req) {
    io.flush_sbuffer.valid := true.B
    state := s_flush_sbuffer_resp
  }

  when (state === s_flush_sbuffer_resp) {
    when (io.flush_sbuffer.empty) {
      out_valid                       := io.flush_sbuffer.ise
      atom_override_xtval             := io.flush_sbuffer.ise
      exceptionVec(delayedStoreFault) := io.flush_sbuffer.ise

      state := Mux(io.flush_sbuffer.ise, s_finish, s_cache_req)
    }
  }

  when (state === s_cache_req) {
    io.dcache.req.valid := true.B
    io.dcache.req.bits.cmd := LookupTree(in.uop.ctrl.fuOpType, List(
      LSUOpType.lr_w      -> M_XLR,
      LSUOpType.sc_w      -> M_XSC,
      LSUOpType.amoswap_w -> M_XA_SWAP,
      LSUOpType.amoadd_w  -> M_XA_ADD,
      LSUOpType.amoxor_w  -> M_XA_XOR,
      LSUOpType.amoand_w  -> M_XA_AND,
      LSUOpType.amoor_w   -> M_XA_OR,
      LSUOpType.amomin_w  -> M_XA_MIN,
      LSUOpType.amomax_w  -> M_XA_MAX,
      LSUOpType.amominu_w -> M_XA_MINU,
      LSUOpType.amomaxu_w -> M_XA_MAXU,

      LSUOpType.lr_d      -> M_XLR,
      LSUOpType.sc_d      -> M_XSC,
      LSUOpType.amoswap_d -> M_XA_SWAP,
      LSUOpType.amoadd_d  -> M_XA_ADD,
      LSUOpType.amoxor_d  -> M_XA_XOR,
      LSUOpType.amoand_d  -> M_XA_AND,
      LSUOpType.amoor_d   -> M_XA_OR,
      LSUOpType.amomin_d  -> M_XA_MIN,
      LSUOpType.amomax_d  -> M_XA_MAX,
      LSUOpType.amominu_d -> M_XA_MINU,
      LSUOpType.amomaxu_d -> M_XA_MAXU
    ))

    io.dcache.req.bits.addr := paddr
    io.dcache.req.bits.vaddr := in.src(0) // vaddr
    io.dcache.req.bits.data := genWdata(in.src(1), in.uop.ctrl.fuOpType(1,0))
    // TODO: atomics do need mask: fix mask
    io.dcache.req.bits.mask := genWmask(paddr, in.uop.ctrl.fuOpType(1,0))
    io.dcache.req.bits.id   := DontCare

    when(io.dcache.req.fire){
      state := s_cache_resp
      paddr_reg := io.dcache.req.bits.addr
      data_reg := io.dcache.req.bits.data
      mask_reg := io.dcache.req.bits.mask
      fuop_reg := in.uop.ctrl.fuOpType
    }
  }

  when (state === s_cache_resp) {
    io.dcache.resp.ready := data_valid
    when(io.dcache.resp.fire) {
      is_lrsc_valid := io.dcache.resp.bits.id
      val rdata = io.dcache.resp.bits.data
      val rdataSel = LookupTree(paddr(2, 0), List(
        "b000".U -> rdata(63, 0),
        "b001".U -> rdata(63, 8),
        "b010".U -> rdata(63, 16),
        "b011".U -> rdata(63, 24),
        "b100".U -> rdata(63, 32),
        "b101".U -> rdata(63, 40),
        "b110".U -> rdata(63, 48),
        "b111".U -> rdata(63, 56)
      ))

      resp_data_wire := LookupTree(in.uop.ctrl.fuOpType, List(
        LSUOpType.lr_w      -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.sc_w      -> rdata,
        LSUOpType.amoswap_w -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.amoadd_w  -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.amoxor_w  -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.amoand_w  -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.amoor_w   -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.amomin_w  -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.amomax_w  -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.amominu_w -> SignExt(rdataSel(31, 0), XLEN),
        LSUOpType.amomaxu_w -> SignExt(rdataSel(31, 0), XLEN),

        LSUOpType.lr_d      -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.sc_d      -> rdata,
        LSUOpType.amoswap_d -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.amoadd_d  -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.amoxor_d  -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.amoand_d  -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.amoor_d   -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.amomin_d  -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.amomax_d  -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.amominu_d -> SignExt(rdataSel(63, 0), XLEN),
        LSUOpType.amomaxu_d -> SignExt(rdataSel(63, 0), XLEN)
      ))

      // when (io.dcache.resp.bits.error && io.csrCtrl.cache_error_enable) {
      //   exceptionVec(loadAccessFault)  := isLr
      //   exceptionVec(storeAccessFault) := !isLr
      //   assert(!exceptionVec(loadAccessFault))
      //   assert(!exceptionVec(storeAccessFault))
      // }

      when (io.dcache.resp.bits.l2_err) {
        exceptionVec(delayedLoadFault ) :=  isLr
        exceptionVec(delayedStoreFault) := !isLr
        atom_override_xtval   := true.B
        io.exceptionAddr.bits := paddr
      }

      resp_data := resp_data_wire
      state := s_finish
      out_valid := true.B
    }
  }

  io.out.valid := out_valid
  XSError((state === s_finish) =/= out_valid, "out_valid reg error\n")
  io.out.bits := DontCare
  io.out.bits.uop := in.uop
  io.out.bits.uop.cf.exceptionVec := exceptionVec
  io.out.bits.data := resp_data
  io.out.bits.redirectValid := false.B
  io.out.bits.debug.isMMIO := is_mmio
  io.out.bits.debug.paddr := paddr
  when (io.out.fire) {
    XSDebug("atomics writeback: pc %x data %x\n", io.out.bits.uop.cf.pc, io.dcache.resp.bits.data)
    state := s_invalid
    out_valid := false.B
  }

  when (state === s_finish) {
    data_valid := false.B
  }

  when (io.redirect.valid) {
    atom_override_xtval := false.B
  }

  if (env.EnableDifftest) {
    val difftest = Module(new DifftestAtomicEvent)
    difftest.io.clock      := clock
    difftest.io.reset      := reset
    difftest.io.coreid     := io.hartId
    difftest.io.atomicResp := io.dcache.resp.fire
    difftest.io.atomicErr  := io.dcache.resp.bits.l2_err
    difftest.io.atomicAddr := paddr_reg
    difftest.io.atomicData := data_reg
    difftest.io.atomicMask := mask_reg
    difftest.io.atomicFuop := fuop_reg
    difftest.io.atomicOut  := resp_data_wire
  }

  if (env.EnableDifftest || env.AlwaysBasicDiff) {
    val uop = io.out.bits.uop
    val difftest = Module(new DifftestLrScEvent)
    difftest.io.clock := clock
    difftest.io.reset := reset
    difftest.io.coreid := io.hartId
    difftest.io.valid := io.out.fire &&
      (uop.ctrl.fuOpType === LSUOpType.sc_d || uop.ctrl.fuOpType === LSUOpType.sc_w)
    difftest.io.success := is_lrsc_valid
  }
}
