package xiangshan.cache

import  chisel3._
import  chisel3.util._
import  midgard._
import  midgard.util._

import  xiangshan._

import  freechips.rocketchip.tilelink._
import  chipsalliance.rocketchip.config._


class FSBCReq(implicit p: Parameters) extends DCacheBundle {
  val id   = UInt(reqIdWidth.W)
  val addr = Bits(PAddrBits.W)
  val data = UInt((DCacheBanks * DCacheSRAMRowBits).W)
  val mask = Bits(cfg.blockBytes.W)
}

class FSBC(edge: TLEdgeOut)(implicit p: Parameters)
  extends DCacheModule
{
  val io = IO(new Bundle() {
    val req   = Flipped(Decoupled(new FSBCReq))
    val res   =             Valid(new DCacheLineResp)
    val bus_a =         Decoupled(new TLBundleA(edge.bundle))
    val bus_d = Flipped(Decoupled(new TLBundleD(edge.bundle)))
    val ise   =            Output(new Bool())
    val fsb   =                    new FSBIO()
  })

  val
     (fsm_idle     ::
      fsm_data_req ::
      fsm_data_res ::
      fsm_meta_req ::
      fsm_meta_res ::
      fsm_res      ::
      fsm_null) = Enum(6)

  val fsm_en  = Pin(Bool())
  val fsm_nxt = Pin(UInt(3.W))
  val fsm_q   = Pin(UInt(3.W))

  fsm_en  := false.B
  fsm_nxt := fsm_q

  switch (fsm_q) {
    is (fsm_idle) {
      fsm_en  := io.req.valid
      fsm_nxt := fsm_data_req
    }
    is (fsm_data_req) {
      fsm_en  := io.bus_a.ready
      fsm_nxt := fsm_data_res
    }
    is (fsm_data_res) {
      fsm_en  := io.bus_d.valid
      fsm_nxt := fsm_meta_req
    }
    is (fsm_meta_req) {
      fsm_en  := io.bus_a.ready
      fsm_nxt := fsm_meta_res
    }
    is (fsm_meta_res) {
      fsm_en  := io.bus_d.valid
      fsm_nxt := fsm_res
    }
    is (fsm_res) {
      fsm_en  := true.B
      fsm_nxt := fsm_idle
    }
  }

  fsm_q := RegEnable(fsm_nxt, fsm_idle, fsm_en)

  val fsm_is_idle     = fsm_q === fsm_idle
  val fsm_is_data_req = fsm_q === fsm_data_req
  val fsm_is_data_res = fsm_q === fsm_data_res
  val fsm_is_meta_req = fsm_q === fsm_meta_req
  val fsm_is_meta_res = fsm_q === fsm_meta_res
  val fsm_is_res      = fsm_q === fsm_res

  val tail_q = Pin(UInt(16.W))

  tail_q := RegEnable(io.fsb.mask & (tail_q + 1.U(16.W)),
                      0.U,
                      io.ise)

  val data_q = RegEnable(io.req.bits.data,                     io.req.fire)
  val meta_q = RegEnable(io.req.bits.mask ## io.req.bits.addr, io.req.fire)

  // os should guarantee that the fsb is properly aligned
  io.bus_a.valid     := fsm_is_data_req || fsm_is_meta_req
  io.bus_a.bits      := edge.Put(64.U(8.W),
                                 io.fsb.base | (tail_q ## fsm_is_meta_req ## 0.U(6.W)),
                                 fsm_is_data_req ?? 6.U    :: 4.U,
                                 fsm_is_data_req ?? data_q :: meta_q)._2

  io.bus_d.ready     := fsm_is_data_res || fsm_is_meta_res

  io.req.ready       := fsm_is_idle

  io.res.valid       := fsm_is_res
  io.res.bits        := DontCare
  io.res.bits.miss   := false.B
  io.res.bits.replay := false.B
  io.res.bits.id     := RegEnable(io.req.bits.id, io.req.fire)
  io.res.bits.err    := false.B

  io.ise             := fsm_is_res

  io.fsb.tail        := tail_q
}
