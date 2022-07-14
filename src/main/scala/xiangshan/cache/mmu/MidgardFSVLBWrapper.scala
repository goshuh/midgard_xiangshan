package xiangshan.cache.mmu

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.internal.naming.chiselName
import chisel3.util._
import freechips.rocketchip.util.SRAMAnnotation
import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp}
import freechips.rocketchip.tilelink._
import xiangshan._
import utils._
import xiangshan.backend.fu.{PMPChecker, PMPReqBundle}
import xiangshan.backend.rob.RobPtr
import xiangshan.backend.fu.util.HasCSRConst

import midgard._
import midgard.frontside._

class MidgardFSVLBWrapper(Width: Int, q: TLBParameters, P: Param)(implicit p: Parameters) extends TlbModule with HasCSRConst  {
  /*
    IO:
        Requestor:
            Req: vaddr(used), size, cmd(used), debug, robidx
            Resp: paddr(Done), miss(Done), fast_miss(Drive it with miss), excp, ptw_back, static_pm
        Replace: It is used for communicating between two dtlbs so it might not be necessary for VLB
        Replenish: Input, so it can be ignored

  */
  val io = IO(new VlbIO(Width, q, P))  //Width is the number of ports in the TLB

  require(!(q.sameCycle && !q.missSameCycle))

  // 1.  missSameCycle &&  sameCycle ->  tlbEn, hit/miss -> outer, addr/perm -> outer
  // 2.  missSameCycle && !sameCycle ->  tlbEn, hit/miss -> outer, addr/perm -> flop -> outer
  // 3. !missSameCycle &&  sameCycle ->  nonsense
  // 4. !missSameCycle && !sameCycle -> !tlbEn, hit/miss -> flop -> outer, addr/perm -> flop -> outer
  //     1) req -> flop -> vlb
  //     2) vlb -> resp -> flop (y)

  val MidgardFSVLB = Module(new VLB(N = Width, P = P.copy(tlbEn = q.missSameCycle)))
//   val MidgardFSVLB = Module(new VLB(N = Width, P = P))

  //VLB Interface
  val VLBReq = MidgardFSVLB.vlb_req_i
  val VLBResp = MidgardFSVLB.vlb_resp_o
  val VLBFill = MidgardFSVLB.vlb_fill_o
  //PTW Interface
  val PTWReq = MidgardFSVLB.ptw_req_o
  val PTWResp = MidgardFSVLB.ptw_resp_i

  //CSR and Privilige Mode
  val csr = io.csr
  val priv = csr.priv //CSR include in VLB
  val mode = if (q.useDmode) priv.dmode else priv.imode

  //ASID
  MidgardFSVLB.asid_i := csr.satp.asid

  MidgardFSVLB.kill_i := Fill(2, io.sfence.valid)
  MidgardFSVLB.kill_asid_i := io.sfence.bits.rs2    //Flush only the VLB entries corresponding to the ASID
                                                    //That may be problem so we flush all entries

  val kill = Fill(3, io.sfence.valid || io.csr.satp.changed)

  val ifecth = if (q.fetchi) true.B else false.B

  val vmEnable = if (EnbaleTlbDebug) (csr.satp.mode === 15.U)
  else (csr.satp.mode === 15.U && (mode < ModeM))

  //Creating buffer for unmapped requests:
  val unmappedReq = Reg(Valid(new VLBReq (P)))  //Vector To store unmapped entries
  when(VLBFill.valid && !VLBFill.bits.vld){
    unmappedReq.valid := true.B
    unmappedReq.bits  := PTWReq.bits
  }

  //Connecting Wrapper IO to the VLB
  (0 until Width).map{i => 
    val addressOffset = io.requestor(i).req.bits.vaddr(offLen - 1, 0)
    val cmd = io.requestor(i).req.bits.cmd  //Command that tries to access the TLB

    //Request Channel
    VLBReq(i).bits.vpn := io.requestor(i).req.bits.vaddr(VAddrBits - 1, offLen)
    VLBReq(i).bits.idx := DontCare
    VLBReq(i).bits.kill := kill
    VLBReq(i).valid := io.requestor(i).req.valid & vmEnable
    io.requestor(i).req.ready := true.B

    val excp = io.requestor(i).resp.bits.excp

    val hitUnmappedReq = unmappedReq.valid && unmappedReq.bits.vpn === VLBReq(i).bits.vpn

    //Response Channel
    val valid = io.requestor(i).req.valid
    val vaddr = io.requestor(i).req.bits.vaddr
    //unmapped scheduling may be wrong
    io.requestor(i).resp.bits.paddr := Mux(vmEnable, Mux(hitUnmappedReq, unmappedReq.bits.vpn ## addressOffset, VLBResp(i).bits.mpn ## (if(!q.missSameCycle) RegNext(addressOffset) else addressOffset)), if(!q.missSameCycle) RegNext(vaddr) else vaddr)
    io.requestor(i).resp.bits.miss := (!hitUnmappedReq && !VLBResp(i).bits.vld) & vmEnable
    io.requestor(i).resp.bits.fast_miss := (!hitUnmappedReq && !VLBResp(i).bits.vld) & vmEnable  //Sent to Ld unit
    io.requestor(i).resp.bits.excp := Mux(vmEnable, excp, RegNext(excp))  //TODO: AF
    io.requestor(i).resp.bits.ptwBack := false.B
    io.requestor(i).resp.valid := Mux(vmEnable, Mux(hitUnmappedReq, true.B, VLBResp(i).valid), if(!q.missSameCycle) RegNext(valid) else valid)  //Bypass VLB Request to Response in case of Machine Mode

    //Generating Exception Signals

    val VLBAttribute = VLBResp(i).bits.attr //Encaps`ulates Permissions
    val perm = Wire(new TlbPermBundle())  //Permissions of the VLB Entry
    val pf = perm.pf
    val af = perm.af
    
    //Need to include Cachable bit 
    perm.pf := VLBAttribute(0)
    perm.af := VLBAttribute(1)
    perm.d := VLBAttribute(2)
    perm.a := VLBAttribute(3)
    perm.g := VLBAttribute(4)
    perm.u := VLBAttribute(5)
    perm.x := VLBAttribute(6)
    perm.w := VLBAttribute(7)
    perm.r := VLBAttribute(8)
    perm.pm := DontCare

    //Address Range < 0x80000000 goes directly to the MMIO and not to the I/D Cache
    //Without this, the addresses < 0x80000000, if requested by the d-cache are requested to be cached if VM is not enabled.
    //The DCache is not supposed to work on these addresses and an assertion in the DCache fails.
    //Hence we do the checking for cacheablility here based on the address range and hardcode the static_pm bits.
    val isNonCachableAddress = io.requestor(i).resp.bits.paddr < 0x80000000L.U
    io.requestor(i).resp.bits.static_pm.valid := Mux(isNonCachableAddress,io.requestor(i).resp.valid, VLBResp(i).valid && VLBResp(i).bits.vld & vmEnable)
    io.requestor(i).resp.bits.static_pm.bits  := Mux(isNonCachableAddress, true.B, !VLBAttribute(9))    //MMIO/Non-Cachable

    //Permission Checks
    val ldUpdate = !perm.a && TlbCmd.isRead(cmd) && !TlbCmd.isAmo(cmd) //update A/D through exception
    val stUpdate = (!perm.a || !perm.d) && (TlbCmd.isWrite(cmd) || TlbCmd.isAmo(cmd)) // update A/D through exception
    val instrUpdate = !perm.a && TlbCmd.isExec(cmd) // update A/D through exception
    val modeCheck = !(mode === ModeU && !perm.u || mode === ModeS && perm.u && (!priv.sum || ifecth)) //Fail
    val ldPermFail = !(modeCheck && (perm.r || priv.mxr && perm.x))
    val stPermFail = !(modeCheck && perm.w)
    val instrPermFail = !(modeCheck && perm.x)  //Instruction Permission Pass if Mode is correct and has execute permission
    val ldPf = (ldPermFail || pf) && (TlbCmd.isRead(cmd) && !TlbCmd.isAmo(cmd))
    val stPf = (stPermFail || pf) && (TlbCmd.isWrite(cmd) || TlbCmd.isAmo(cmd))
    val instrPf = (instrPermFail || pf) && TlbCmd.isExec(cmd)

    //Generating Page Fault
    excp.pf.ld := (hitUnmappedReq || (ldPf || ldUpdate) && !af) && vmEnable //This will be created only for Data TLB
    excp.pf.st := (hitUnmappedReq || (stPf || stUpdate) && !af) && vmEnable //This will be created only for Data TLB
    excp.pf.instr := (hitUnmappedReq || (instrPf || instrUpdate) && !af) && vmEnable  //This will be created only for Instruction TLB

    //TODO: Generating Access Fault
    excp.af.instr := false.B
    excp.af.ld := false.B
    excp.af.st := false.B
  }

  // when(wrong) {
  //   unmappedReq.valid := false.B
  // }

  // val ptw_resp = Wire(Decoupled(new VMA(P)))

  // ptw_resp.valid := MidgardFSVLB.ptw_resp_i.valid
  // ptw_resp.bits  := MidgardFSVLB.ptw_resp_i.bits

  //PTW Interface
  io.ptw.req <> PTWReq
  io.ptw.resp <> PTWResp
}

object MidgardFSVLBWrapper {
  def apply
  (
    in: Seq[BlockTlbRequestIO],
    sfence: SfenceBundle,
    csr: TlbCsrBundle,
    width: Int,
    shouldBlock: Boolean,
    q: TLBParameters,
    P: Param
  )(implicit p: Parameters) = {
    require(in.length == width)

    val tlb = Module(new MidgardFSVLBWrapper(width, q, P))

    tlb.io.sfence <> sfence
    tlb.io.csr <> csr
    tlb.suggestName(s"Vlb_${q.name}")

    for (i <- 0 until width) {
      tlb.io.requestor(i) <> in(i)

      if (q.missSameCycle && !q.sameCycle) {  //ITLB
        in(i).resp.bits           := RegNext(tlb.io.requestor(i).resp.bits)
        in(i).resp.valid          := RegNext(tlb.io.requestor(i).resp.valid)
        in(i).resp.bits.miss      := tlb.io.requestor(i).resp.bits.miss
        in(i).resp.bits.fast_miss := tlb.io.requestor(i).resp.bits.fast_miss
      }

      if (shouldBlock) {  //ITLB
        in(i).req.ready := !tlb.io.requestor(i).resp.bits.miss && in(i).resp.ready && tlb.io.requestor(i).req.ready
      }
    }
    tlb.io.ptw
  }
}
