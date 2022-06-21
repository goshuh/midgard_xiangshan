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

//   require(q.sameCycle == q.missSameCycle)

  val MidgardFSVLB = Module(new VLB(N = Width, P = P.copy(tlbEn = q.missSameCycle)))
//   val MidgardFSVLB = Module(new VLB(N = Width, P = P))

  //VLB Interface
  val VLBReq = MidgardFSVLB.vlb_req_i
  val VLBResp = MidgardFSVLB.vlb_resp_o
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

  val vmEnable = if (EnbaleTlbDebug) (csr.satp.mode === 8.U)
  else (csr.satp.mode === 8.U && (mode < ModeM))

  //Connecting Wrapper IO to the VLB
  (0 until Width).map{i => 
    val addressOffset = io.requestor(i).req.bits.vaddr(offLen, 0)
    //Request Channel
    VLBReq(i).bits.vpn := io.requestor(i).req.bits.vaddr(VAddrBits - 1, offLen)
    VLBReq(i).bits.idx := DontCare
    VLBReq(i).bits.kill := kill
    VLBReq(i).valid := io.requestor(i).req.valid
    io.requestor(i).req.ready := true.B

    val excp = io.requestor(i).resp.bits.excp

    //Response Channel
    io.requestor(i).resp.bits.paddr := Mux(vmEnable, VLBResp(i).bits.mpn ## addressOffset, io.requestor(i).req.bits.vaddr)
    io.requestor(i).resp.bits.miss := !VLBResp(i).bits.vld & vmEnable
    io.requestor(i).resp.bits.fast_miss := !VLBResp(i).bits.vld & vmEnable  //Sent to Ld unit
    io.requestor(i).resp.bits.excp := excp  //TODO: AF
    io.requestor(i).resp.bits.ptwBack := DontCare
    io.requestor(i).resp.valid := VLBResp(i).valid

    //Generating Exception Signals

    val VLBAttribute = VLBResp(i).bits.attr //Encapsulates Permissions
    val cmd = io.requestor(i).req.bits.cmd  //Command that tries to access the TLB
    val perm = Wire(new TlbPermBundle())  //Permissions of the VLB Entry
    val pf = perm.pf
    val af = perm.af
    val cmdReg = if (!q.sameCycle) RegNext(cmd) else cmd  //Command: Request for Read, Write, Execute, Atomic Read, Atomic Write

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

    io.requestor(i).resp.bits.static_pm.valid := VLBResp(i).valid && VLBResp(i).bits.vld & vmEnable
    io.requestor(i).resp.bits.static_pm.bits  := VLBAttribute(9)    //MMIO/Non-Cachable

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

    //Generating Page Fault
    excp.pf.ld := (ldPf || ldUpdate) && !af & vmEnable //This will be created only for Data TLB
    excp.pf.st := (stPf || stUpdate) && !af & vmEnable //This will be created only for Data TLB
    excp.pf.instr := (instrPf || instrUpdate) && !af & vmEnable  //This will be created only for Instruction TLB

    //TODO: Generating Access Fault
    excp.af.instr := false.B
    excp.af.ld := false.B
    excp.af.st := false.B
  }

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
    }
    tlb.io.ptw
  }
}
