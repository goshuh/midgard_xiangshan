package xiangshan.cache

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import midgard._
import midgard.util._
import freechips.rocketchip.tilelink._
import system._
import xiangshan._

class FSBCReq(implicit p: Parameters) extends DCacheBundle {
  val paddr = Bits(PAddrBits.W)
  val wmask = Bits(cfg.blockBytes.W)
  val data  = UInt((DCacheBanks * DCacheSRAMRowBits).W)
  val id    = UInt(reqIdWidth.W)
}

class FSBC(edge: TLEdgeOut)(implicit p: Parameters)
  extends DCacheModule
{
  val io = IO(new Bundle() {
    val req         = Flipped(DecoupledIO (new FSBCReq))
    val resp        =         ValidIO     (new DCacheLineResp)
    val mem_acquire =         DecoupledIO (new TLBundleA(edge.bundle))
    val mem_grant   = Flipped(DecoupledIO (new TLBundleD(edge.bundle)))
    val ise         =         Output      (new Bool()) // store buffer interrupt pending
    val fsbc        =                      new FSBCIO()
  })

  val s_idle :: s_write_data_req :: s_write_data_resp :: s_write_meta_req :: s_write_meta_resp :: s_send_resp :: Nil = Enum(6)
  val state           = RegInit(s_idle)
  val req             = RegInit(-1.S.asTypeOf(new FSBCReq))
  val req_data        = RegInit(-1.S(512.W).asUInt)
  val req_meta        = RegInit(-1.S(128.W).asUInt)

  // --------------------------------------------
  //  defaults
  io.req.ready          := (state === s_idle)
  io.resp.valid         := false.B
  io.resp.bits          := DontCare
  io.mem_acquire.valid  := false.B
  io.mem_acquire.bits   := DontCare
  io.mem_grant.ready    := false.B
  io.ise                := false.B

  val fsbc_tail_q = dontTouch(Wire(UInt(16.W)))

  fsbc_tail_q  := RegEnable(io.fsbc.mask & (fsbc_tail_q + 1.U(16.W)),
                            0.U,
                            io.ise)

  io.fsbc.tail := fsbc_tail_q

  // os should guarantee that the db is properly aligned
  val fsbc_addr = io.fsbc.base | (fsbc_tail_q ## (state === s_write_meta_req) ## 0.U(6.W))

  // --------------------------------------------
  //  state machine 
  switch (state) {

    is (s_idle) {
      // accept exception requests and transition to mem write
      req           := io.req.fire() ?? io.req.bits             :: -1.S.asTypeOf(new FSBCReq)
      req_data      := io.req.fire() ?? io.req.bits.data.asUInt :: -1.S.asUInt
      state         := io.req.fire() ?? s_write_data_req        :: s_idle // spin in current state if no request comming in
    }

    is (s_write_data_req) {
      // send mem write req
      io.mem_acquire.valid := true.B
      io.mem_acquire.bits  := edge.Put(
        fromSource = 100.U(8.W),
        toAddress  = fsbc_addr,
        lgSize     = 6.U, // 64B cacheline
        data       = req_data,
        // mask       = req.wmask // if no mask, it is putFull, else it is putPartial
      )._2 // TODO: Check legal?

      req_data    := io.mem_acquire.fire()  ?? (req_data >> 64.U) :: req_data // bmmu.ctl.node accpets 64bits at a time
      state       := io.mem_acquire.fire()  ?? s_write_data_resp  ::  // sent last beat
                                               s_write_data_req       // still sending 
    }

    is (s_write_data_resp) {
      io.mem_grant.ready := true.B
      state := io.mem_grant.fire() ?? s_write_meta_req :: s_write_data_resp // spin until we have mem grant

      // also prepare data for next payload..
      val req_addr      = req.paddr.asTypeOf(UInt(64.W))
      req_meta         := Cat(req.wmask, req_addr)
    }

    is (s_write_meta_req) {
      io.mem_acquire.valid := true.B
      io.mem_acquire.bits  := edge.Put(
        fromSource = 100.U(8.W),
        toAddress  = fsbc_addr,
        lgSize     = 4.U, // 16B <mask><addr>
        data       = req_meta,
      )._2

      req_meta  := io.mem_acquire.fire() ?? (req_meta >> 64.U)  :: req_meta
      state     := io.mem_acquire.fire() ?? s_write_meta_resp   ::  // sent last beat
                                            s_write_meta_req        // still sending 
    }

    is (s_write_meta_resp) {
      io.mem_grant.ready := true.B
      state := io.mem_grant.fire() ?? s_send_resp :: s_write_meta_resp // spin until we have mem grant
    }

    is (s_send_resp) {
      // send hit resp to sbuffer
      io.resp.valid       := true.B
      io.resp.bits.data   := DontCare
      io.resp.bits.miss   := false.B
      io.resp.bits.replay := false.B
      io.resp.bits.id     := req.id
      io.resp.bits.err    := false.B
      io.ise              := true.B
      state               := s_idle
    }
  }
}
