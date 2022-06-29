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


class MidgardFSPTWWrapper(Width: Int, P: Param)(implicit p: Parameters) extends LazyModule with HasXSParameter {

    val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
      "ptw",
      sourceId = IdRange(0, 2)  //!0-2 is hardcoded for now
    ))
  )))

  def blockBytes_align(addr: UInt) = {
    Cat(addr(PAddrBits - 1, log2Up(l2tlbParams.blockBytes)), 0.U(log2Up(l2tlbParams.blockBytes).W))
  }

  lazy val module = new LazyModuleImp(this){

    val vlb_req_i  = IO(Vec(Width, Flipped(Decoupled(new VLBReq (P)))))
    val vlb_resp_o = IO(Vec(Width,             Valid(new VMA    (P))))

    val mem_req_o  = IO(               Decoupled(new MemReq (P)))
    val mem_resp_i = IO(       Flipped(Decoupled(new MemResp(P))))

    val satp_i     = IO(                   Input(UInt(P.mcnBits.W)))
    val idle_o     = IO(                  Output(Bool()))

    val MidgardFSPTW_i = Module(new MidgardFSPTW(N = Width, P = P))

    MidgardFSPTW_i.vlb_req_i <> vlb_req_i
    MidgardFSPTW_i.vlb_resp_o <> vlb_resp_o
    MidgardFSPTW_i.mem_req_o <> mem_req_o
    MidgardFSPTW_i.mem_resp_i <> mem_resp_i
    MidgardFSPTW_i.satp_i <> satp_i
    MidgardFSPTW_i.idle_o <> idle_o

    val (mem, edge) = node.out.head


    val memRead =  edge.Get(
    fromSource = 0.U,
    // toAddress  = memAddr(log2Up(CacheLineSize / 2 / 8) - 1, 0),
    toAddress  = blockBytes_align(MidgardFSPTW_i.mem_req_o.bits.mcn ## 0.U(6.W)),  //Confirm if address alignment is correct or not
    lgSize     = log2Up(l2tlbParams.blockBytes).U
  )._2

    //Need to take care of what kind of data is received by the ptw and 
    //extract the necessary data from it.
    mem.a.bits := memRead
    //Set the rest of the fields as well!
    mem.a.valid := MidgardFSPTW_i.mem_req_o.valid
    MidgardFSPTW_i.mem_req_o.ready := mem.a.ready

    //!Data sent by Tile link is 256 bits wide while the PTW will take 512 bits of data
    //So write a logic which combines two 256 bits data before asserting valid on the memory response channel

    val PTWEntry = Wire(UInt(P.clBits.W))   //clBits is 512 bits long

    //En of FF
    val lowerEn = Wire(Bool())
    //DFF with reset value
    // val lowerBits = Wire(UInt(mem.d.bits.data.getWidth.W))
    val lowerBits = (RegEnable(mem.d.bits.data, 0.U, lowerEn))

    // val higherBits = Wire(UInt(mem.d.bits.data.getWidth.W))
    val higherBits = mem.d.bits.data

    PTWEntry := higherBits ## lowerBits
    MidgardFSPTW_i.mem_resp_i.bits.data := PTWEntry

    //Counter
    val counterEn = Wire(Bool())
    val counterNext = Wire(UInt(1.W))
    val counter = (RegEnable(counterNext, 0.U, counterEn))
    counterNext := counter + 1.U
    counterEn := mem.d.fire && counter === 0.U || counter === 1.U && MidgardFSPTW_i.mem_resp_i.fire

    lowerEn := counterEn

    MidgardFSPTW_i.mem_resp_i.valid := counter === 1.U && mem.d.valid
    mem.d.ready := counter === 0.U || counter === 1.U && MidgardFSPTW_i.mem_resp_i.ready  //Accept data only when the FS PTW is ready to accept data
    }
}