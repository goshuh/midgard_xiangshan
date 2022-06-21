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

package xiangshan.cache.mmu

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.internal.naming.chiselName
import chisel3.util._
import freechips.rocketchip.util.SRAMAnnotation
import xiangshan._
import utils._
import xiangshan.backend.fu.{PMPChecker, PMPReqBundle}
import xiangshan.backend.rob.RobPtr
import xiangshan.backend.fu.util.HasCSRConst


@chiselName
class TLB(Width: Int, q: TLBParameters)(implicit p: Parameters) extends TlbModule with HasCSRConst with HasPerfEvents {
  /*
    This Module:
      1. Instantiates Super and Normal TLB Storage modules: There are multiple ports in the TLB, specified by WIDTH.
      2. Connects the Request Channel TLBs to the outside module using IO
      3. Performs TLB Reads using Requests coming from the channel using TLBNormalRead():
          a. This function Reads Both Super and Normal TLB.
          b. It checks various permissions and reports if there was any hit or miss
          c. In case sameCycle is enabled, this information is sent in the same clock cycle
              as the request arrived.
          d. In case of a hit, the Physical Address is sent to the requester as well.
      4. In case of a miss, the PTW is sent a request to get the PPN using VPN.
      5. PTW gives a response on a request and asserts the "refill" signal
      6. The Super/Normal TLB write the response as soon as the PTW response is valid
   */
  val io = IO(new TlbIO(Width, q))  //Width is the number of ports in the TLB

  require(q.superAssociative == "fa")
  if (q.sameCycle || q.missSameCycle) {
    require(q.normalAssociative == "fa")
  }

  //All req and resp are for the TLB Requests from the processor!
  //req will have the VPN, Command:RWX, Size, ROB_ID
  //resp will contain the PPN, Hit/Fast-Hit or Miss/Fast-Miss information and Exception information
  val req = io.requestor.map(_.req)
  val resp = io.requestor.map(_.resp)
  val ptw = io.ptw
  val pmp = io.pmp

  val sfence = io.sfence
  val csr = io.csr
  val satp = csr.satp
  val priv = csr.priv //CSR include in VLB
  val ifecth = if (q.fetchi) true.B else false.B
  val mode = if (q.useDmode) priv.dmode else priv.imode
  // val vmEnable = satp.mode === 8.U // && (mode < ModeM) // !FIXME: fix me when boot xv6/linux...
  val vmEnable = if (EnbaleTlbDebug) (satp.mode === 8.U)
  else (satp.mode === 8.U && (mode < ModeM))
  //Midgard has mode 15
  //If vmEnale is false, VA = PA and no miss

  val reqAddr = req.map(_.bits.vaddr.asTypeOf(new VaBundle))
  val vpn = reqAddr.map(_.vpn)
  val cmd = req.map(_.bits.cmd)
  val valid = req.map(_.valid)

  def widthMapSeq[T <: Seq[Data]](f: Int => T) = (0 until Width).map(f)

  def widthMap[T <: Data](f: Int => T) = (0 until Width).map(f)

  //Instantiates Super and Normal TLB Storage modules: There are multiple ports in the TLB, specified by WIDTH.
  // Normal page && Super page
  val normalPage = TlbStorage(
    name = "normal",
    associative = q.normalAssociative,
    sameCycle = q.sameCycle,
    ports = Width,
    nSets = q.normalNSets,
    nWays = q.normalNWays,
    sramSinglePort = sramSinglePort,
    saveLevel = q.saveLevel,
    normalPage = true,
    superPage = false
  )
  val superPage = TlbStorage(
    name = "super",
    associative = q.superAssociative,
    sameCycle = q.sameCycle,
    ports = Width,
    nSets = q.superNSets,
    nWays = q.superNWays,
    sramSinglePort = sramSinglePort,
    saveLevel = q.saveLevel,
    normalPage = q.normalAsVictim,
    superPage = true,
  )

  /*
    Normal and Super TLBs are accessed in parallel to get the Physical Tag for both Normal and Super Page.
    This is required because the caches are VIPT. 
    At the same time, the Cache Set is indexed by the Page Offset bits
   */
  //Connects the Request Channel TLBs to the outside module using IO
  for (i <- 0 until Width) {
    normalPage.r_req_apply(
      valid = io.requestor(i).req.valid,
      vpn = vpn(i),
      asid = csr.satp.asid,
      i = i
    )
    superPage.r_req_apply(
      valid = io.requestor(i).req.valid,
      vpn = vpn(i),
      asid = csr.satp.asid,
      i = i
    )
  }

  normalPage.victim.in <> superPage.victim.out
  normalPage.victim.out <> superPage.victim.in
  normalPage.sfence <> io.sfence
  superPage.sfence <> io.sfence
  normalPage.csr <> io.csr
  superPage.csr <> io.csr

  //Performs TLB Reads using Requests coming from the channel using TLBNormalRead()
  //*_hit_sameCycle: is the hit/miss response coming at the same clock cycle as the request.
  def TLBNormalRead(i: Int) = {
    val (n_hit_sameCycle, normal_hit, normal_ppn, normal_perm) = normalPage.r_resp_apply(i)
    val (s_hit_sameCycle, super_hit, super_ppn, super_perm) = superPage.r_resp_apply(i)
    // assert(!(normal_hit && super_hit && vmEnable && RegNext(req(i).valid, init = false.B)))

    //Checks various permissions and reports if there was any hit or miss
    val hit = normal_hit || super_hit   //Hit on either Normal TLB or Super TLB
    val hit_sameCycle = n_hit_sameCycle || s_hit_sameCycle  //Responded on same cycle as the request
    val ppn = Mux(super_hit, super_ppn, normal_ppn)   //Select either Super PPN or Norma PPN based on where the request Hit.
    val perm = Mux(super_hit, super_perm, normal_perm)  // Permissions.

    val pf = perm.pf  //Page Fault Exception Coming from the TLB Entry
    val af = perm.af  //Access Fault Exception Coming from the TLB Entry
    val cmdReg = if (!q.sameCycle) RegNext(cmd(i)) else cmd(i)  //Command: Request for Read, Write, Execute, Atomic Read, Atomic Write
    val validReg = if (!q.sameCycle) RegNext(valid(i)) else valid(i)  //Valid Request or not
    val offReg = if (!q.sameCycle) RegNext(reqAddr(i).off) else reqAddr(i).off  //Offset
    val sizeReg = if (!q.sameCycle) RegNext(req(i).bits.size) else req(i).bits.size 

    /** *************** next cycle when two cycle is false******************* */
    val miss = !hit && vmEnable
    val fast_miss = !super_hit && vmEnable
    val miss_sameCycle = !hit_sameCycle && vmEnable
    hit.suggestName(s"hit_${i}")
    miss.suggestName(s"miss_${i}")

    XSDebug(validReg, p"(${i.U}) hit:${hit} miss:${miss} ppn:${Hexadecimal(ppn)} perm:${perm}\n")

    // In case of a hit, the Physical Address is sent to the requester as well.
    val paddr = Cat(ppn, offReg)  //PA: PPN + OFFSET
    val vaddr = SignExt(req(i).bits.vaddr, PAddrBits)//?Sign extend Virtual Address to make it as atleast as big as physical Address. **Why? Isn't VA supposed to be larger than PA?**
    val refill_reg = RegNext(io.ptw.resp.valid) //?We haven't sent PTW any request, how to expect response?
    req(i).ready := resp(i).ready   //?If the Requester is ready to accept the response, only then accept the request?
    resp(i).valid := validReg
    resp(i).bits.paddr := Mux(vmEnable, paddr, if (!q.sameCycle) RegNext(vaddr) else vaddr) //We need to kepp vmEnable 1
    resp(i).bits.miss := { if (q.missSameCycle) miss_sameCycle else (miss || refill_reg) }
    //  If the refill is asserted and we receive the TLB enty from Page Table, then a Miss is reported any way.
    //  *This will trigger a Miss and a re-request Hopefully
    resp(i).bits.fast_miss := fast_miss || refill_reg
    resp(i).bits.ptwBack := io.ptw.resp.fire()

    //pmp: Physical Memory Protection
    // for timing optimization, pmp check is divided into dynamic and static
    // dynamic: superpage (or full-connected reg entries) -> check pmp when translation done
    // static: 4K pages (or sram entries) -> check pmp with pre-checked results
    val pmp_paddr = Mux(vmEnable, Cat(super_ppn, offReg), if (!q.sameCycle) RegNext(vaddr) else vaddr)
    pmp(i).valid := resp(i).valid
    pmp(i).bits.addr := pmp_paddr
    pmp(i).bits.size := sizeReg
    pmp(i).bits.cmd := cmdReg

    //Permission Checks
    val ldUpdate = !perm.a && TlbCmd.isRead(cmdReg) && !TlbCmd.isAmo(cmdReg) //update A/D through exception
    val stUpdate = (!perm.a || !perm.d) && (TlbCmd.isWrite(cmdReg) || TlbCmd.isAmo(cmdReg)) // update A/D through exception
    val instrUpdate = !perm.a && TlbCmd.isExec(cmdReg) // update A/D through exception
    val modeCheck = !(mode === ModeU && !perm.u || mode === ModeS && perm.u && (!priv.sum || ifecth)) //Fail
    val ldPermFail = !(modeCheck && (perm.r || priv.mxr && perm.x))
    val stPermFail = !(modeCheck && perm.w)
    val instrPermFail = !(modeCheck && perm.x)  //Instruction Permission Pass if Mode is correct and has execute permission
    val ldPf = (ldPermFail || pf) && (TlbCmd.isRead(cmdReg) && !TlbCmd.isAmo(cmdReg))
    val stPf = (stPermFail || pf) && (TlbCmd.isWrite(cmdReg) || TlbCmd.isAmo(cmdReg))
    val instrPf = (instrPermFail || pf) && TlbCmd.isExec(cmdReg)
    val fault_valid = vmEnable
    resp(i).bits.excp.pf.ld := (ldPf || ldUpdate) && fault_valid && !af //This will be created only for Data TLB
    resp(i).bits.excp.pf.st := (stPf || stUpdate) && fault_valid && !af //This will be created only for Data TLB
    resp(i).bits.excp.pf.instr := (instrPf || instrUpdate) && fault_valid && !af  //This will be created only for Instruction TLB
    // NOTE: pf need && with !af, page fault has higher priority than access fault
    // but ptw may also have access fault, then af happens, the translation is wrong.
    // In this case, pf has lower priority than af

    val spm = normal_perm.pm // static physical memory protection or attribute
    val spm_v = !super_hit && vmEnable && q.partialStaticPMP.B // static pm valid; do not use normal_hit, it's too long.
    // for tlb without sram, tlb will miss, pm should be ignored outsize
    resp(i).bits.excp.af.ld    := (af || (spm_v && !spm.r)) && TlbCmd.isRead(cmdReg) && fault_valid
    resp(i).bits.excp.af.st    := (af || (spm_v && !spm.w)) && TlbCmd.isWrite(cmdReg) && fault_valid
    resp(i).bits.excp.af.instr := (af || (spm_v && !spm.x)) && TlbCmd.isExec(cmdReg) && fault_valid
    resp(i).bits.static_pm.valid := spm_v && fault_valid // ls/st unit should use this mmio, not the result from pmp
    resp(i).bits.static_pm.bits := !spm.c //Non Cacheable

    (hit, miss, validReg)
  }

  val readResult = (0 until Width).map(TLBNormalRead(_))
  val hitVec = readResult.map(_._1)
  val missVec = readResult.map(_._2)
  val validRegVec = readResult.map(_._3)

  // replacement
  def get_access(one_hot: UInt, valid: Bool): Valid[UInt] = {
    val res = Wire(Valid(UInt(log2Up(one_hot.getWidth).W)))
    res.valid := Cat(one_hot).orR && valid  //orR: Or Reduction operator
    res.bits := OHToUInt(one_hot)
    res
  }

  //?This is used to identify the TLB entry to be replaced?
  val normal_refill_idx = if (q.outReplace) {
    io.replace.normalPage.access <> normalPage.access
    io.replace.normalPage.chosen_set := get_set_idx(io.ptw.resp.bits.entry.tag, q.normalNSets)
    io.replace.normalPage.refillIdx
  } else if (q.normalAssociative == "fa") {
    val re = ReplacementPolicy.fromString(q.normalReplacer, q.normalNWays)
    re.access(normalPage.access.map(_.touch_ways)) // normalhitVecVec.zipWithIndex.map{ case (hv, i) => get_access(hv, validRegVec(i))})
    re.way
  } else { // set-acco && plru
    val re = ReplacementPolicy.fromString(q.normalReplacer, q.normalNSets, q.normalNWays)
    re.access(normalPage.access.map(_.sets), normalPage.access.map(_.touch_ways))
    re.way(get_set_idx(io.ptw.resp.bits.entry.tag, q.normalNSets))
  }

  val super_refill_idx = if (q.outReplace) {
    io.replace.superPage.access <> superPage.access
    io.replace.superPage.chosen_set := DontCare //Probably because it is Fully Associative
    io.replace.superPage.refillIdx
  } else {
    val re = ReplacementPolicy.fromString(q.superReplacer, q.superNWays)
    re.access(superPage.access.map(_.touch_ways))
    re.way
  }

  //PTW gives a response on a request and asserts the "refill" signal
  val refill = ptw.resp.fire() && !sfence.valid && !satp.changed  //Fire is true when Valid is Asserted

  //The Super/Normal TLB write the response as soon as the PTW response is valid
  normalPage.w_apply(
    valid = { if (q.normalAsVictim) false.B
    else refill && ptw.resp.bits.entry.level.get === 2.U },
    wayIdx = normal_refill_idx,
    data = ptw.resp.bits,
    data_replenish = io.ptw_replenish
  )
  superPage.w_apply(
    valid = { if (q.normalAsVictim) refill
    else refill && ptw.resp.bits.entry.level.get =/= 2.U },
    wayIdx = super_refill_idx,
    data = ptw.resp.bits,
    data_replenish = io.ptw_replenish
  )

  //In case of a miss, the PTW is sent a request to get the PPN using VPN.

  // if sameCycle, just req.valid
  // if !sameCycle, add one more RegNext based on !sameCycle's RegNext 
  // because sram is too slow and dtlb is too distant from dtlbRepeater
  for (i <- 0 until Width) {  //Requests to PTW are two cycles delayed
    io.ptw.req(i).valid :=  need_RegNextInit(!q.sameCycle, validRegVec(i) && missVec(i), false.B) &&  //Send to PTW only when TLB misses on a Valid Request 
      !RegNext(refill, init = false.B) && //!Don't send to PTW if the previous request is not served yet
      param_choose(!q.sameCycle, !RegNext(RegNext(refill, init = false.B), init = false.B), true.B)
    io.ptw.req(i).bits.vpn := need_RegNext(!q.sameCycle, need_RegNext(!q.sameCycle, reqAddr(i).vpn))
  }
  io.ptw.resp.ready := true.B

  def need_RegNext[T <: Data](need: Boolean, data: T): T = {
    if (need) RegNext(data)
    else data
  }
  def need_RegNextInit[T <: Data](need: Boolean, data: T, init_value: T): T = {
    if (need) RegNext(data, init = init_value)
    else data
  }

  def param_choose[T <: Data](need: Boolean, truedata: T, falsedata: T): T = {
    if (need) truedata
    else falsedata
  }

  if (!q.shouldBlock) {
    for (i <- 0 until Width) {
      XSPerfAccumulate("first_access" + Integer.toString(i, 10), validRegVec(i) && vmEnable && RegNext(req(i).bits.debug.isFirstIssue))
      XSPerfAccumulate("access" + Integer.toString(i, 10), validRegVec(i) && vmEnable)
    }
    for (i <- 0 until Width) {
      XSPerfAccumulate("first_miss" + Integer.toString(i, 10), validRegVec(i) && vmEnable && missVec(i) && RegNext(req(i).bits.debug.isFirstIssue))
      XSPerfAccumulate("miss" + Integer.toString(i, 10), validRegVec(i) && vmEnable && missVec(i))
    }
  } else {
    // NOTE: ITLB is blocked, so every resp will be valid only when hit
    // every req will be ready only when hit
    for (i <- 0 until Width) {
      XSPerfAccumulate(s"access${i}", io.requestor(i).req.fire() && vmEnable)
      XSPerfAccumulate(s"miss${i}", ptw.req(i).fire())
    }

  }
  //val reqCycleCnt = Reg(UInt(16.W))
  //reqCycleCnt := reqCycleCnt + BoolStopWatch(ptw.req(0).fire(), ptw.resp.fire || sfence.valid)
  //XSPerfAccumulate("ptw_req_count", ptw.req.fire())
  //XSPerfAccumulate("ptw_req_cycle", Mux(ptw.resp.fire(), reqCycleCnt, 0.U))
  XSPerfAccumulate("ptw_resp_count", ptw.resp.fire())
  XSPerfAccumulate("ptw_resp_pf_count", ptw.resp.fire() && ptw.resp.bits.pf)

  // Log
  for(i <- 0 until Width) {
    XSDebug(req(i).valid, p"req(${i.U}): (${req(i).valid} ${req(i).ready}) ${req(i).bits}\n")
    XSDebug(resp(i).valid, p"resp(${i.U}): (${resp(i).valid} ${resp(i).ready}) ${resp(i).bits}\n")
  }

  XSDebug(sfence.valid, p"Sfence: ${sfence}\n")
  XSDebug(ParallelOR(valid)|| ptw.resp.valid, p"CSR: ${csr}\n")
  XSDebug(ParallelOR(valid) || ptw.resp.valid, p"vmEnable:${vmEnable} hit:${Binary(VecInit(hitVec).asUInt)} miss:${Binary(VecInit(missVec).asUInt)}\n")
  for (i <- ptw.req.indices) {
    XSDebug(ptw.req(i).fire(), p"L2TLB req:${ptw.req(i).bits}\n")
  }
  XSDebug(ptw.resp.valid, p"L2TLB resp:${ptw.resp.bits} (v:${ptw.resp.valid}r:${ptw.resp.ready}) \n")

  println(s"${q.name}: normal page: ${q.normalNWays} ${q.normalAssociative} ${q.normalReplacer.get} super page: ${q.superNWays} ${q.superAssociative} ${q.superReplacer.get}")

//   // NOTE: just for simple tlb debug, comment it after tlb's debug
  // assert(!io.ptw.resp.valid || io.ptw.resp.bits.entry.tag === io.ptw.resp.bits.entry.ppn, "Simple tlb debug requires vpn === ppn")

  val perfEvents = if(!q.shouldBlock) {
    Seq(
      ("access", PopCount((0 until Width).map(i => vmEnable && validRegVec(i)))              ),
      ("miss  ", PopCount((0 until Width).map(i => vmEnable && validRegVec(i) && missVec(i)))),
    )
  } else {
    Seq(
      ("access", PopCount((0 until Width).map(i => io.requestor(i).req.fire()))),
      ("miss  ", PopCount((0 until Width).map(i => ptw.req(i).fire()))         ),
    )
  }
  generatePerfEvent()
}

class TlbReplace(Width: Int, q: TLBParameters)(implicit p: Parameters) extends TlbModule {
  val io = IO(new TlbReplaceIO(Width, q))

  if (q.normalAssociative == "fa") {
    val re = ReplacementPolicy.fromString(q.normalReplacer, q.normalNWays)
    re.access(io.normalPage.access.map(_.touch_ways))
    io.normalPage.refillIdx := re.way
  } else { // set-acco && plru
    val re = ReplacementPolicy.fromString(q.normalReplacer, q.normalNSets, q.normalNWays)
    re.access(io.normalPage.access.map(_.sets), io.normalPage.access.map(_.touch_ways))
    io.normalPage.refillIdx := { if (q.normalNWays == 1) 0.U else re.way(io.normalPage.chosen_set) }
  }

  if (q.superAssociative == "fa") {
    val re = ReplacementPolicy.fromString(q.superReplacer, q.superNWays)
    re.access(io.superPage.access.map(_.touch_ways))
    io.superPage.refillIdx := re.way
  } else { // set-acco && plru
    val re = ReplacementPolicy.fromString(q.superReplacer, q.superNSets, q.superNWays)
    re.access(io.superPage.access.map(_.sets), io.superPage.access.map(_.touch_ways))
    io.superPage.refillIdx := { if (q.superNWays == 1) 0.U else re.way(io.superPage.chosen_set) }
  }
}

object TLB {
  def apply
  (
    in: Seq[BlockTlbRequestIO],
    sfence: SfenceBundle,
    csr: TlbCsrBundle,
    width: Int,
    shouldBlock: Boolean,
    q: TLBParameters
  )(implicit p: Parameters) = {
    require(in.length == width)

    val tlb = Module(new TLB(width, q))

    tlb.io.sfence <> sfence
    tlb.io.csr <> csr
    tlb.suggestName(s"tlb_${q.name}")

    if (!shouldBlock) { // dtlb
      for (i <- 0 until width) {
        tlb.io.requestor(i) <> in(i)
        // tlb.io.requestor(i).req.valid := in(i).req.valid
        // tlb.io.requestor(i).req.bits := in(i).req.bits
        // in(i).req.ready := tlb.io.requestor(i).req.ready

        // in(i).resp.valid := tlb.io.requestor(i).resp.valid
        // in(i).resp.bits := tlb.io.requestor(i).resp.bits
        // tlb.io.requestor(i).resp.ready := in(i).resp.ready
      }
    } else { // itlb
      //require(width == 1)
      (0 until width).map{ i =>
        tlb.io.requestor(i).req.valid := in(i).req.valid
        tlb.io.requestor(i).req.bits := in(i).req.bits
        in(i).req.ready := !tlb.io.requestor(i).resp.bits.miss && in(i).resp.ready && tlb.io.requestor(i).req.ready

        require(q.missSameCycle || q.sameCycle)
        // NOTE: the resp.valid seems to be useless, it must be true when need
        //       But don't know what happens when true but not need, so keep it correct value, not just true.B
        if (q.missSameCycle && !q.sameCycle) {
          in(i).resp.valid := tlb.io.requestor(i).resp.valid && !RegNext(tlb.io.requestor(i).resp.bits.miss)
        } else {
          in(i).resp.valid := tlb.io.requestor(i).resp.valid && !tlb.io.requestor(i).resp.bits.miss
        }
        in(i).resp.bits := tlb.io.requestor(i).resp.bits
        tlb.io.requestor(i).resp.ready := in(i).resp.ready
      }
      tlb.io.ptw_replenish <> DontCare // itlb only use reg, so no static pmp/pma
    }
    tlb.io.ptw
  }
}
