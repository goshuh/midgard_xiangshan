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

package system

import chipsalliance.rocketchip.config.{Field, Parameters}
import chisel3._
import chisel3.util._
import device.{DebugModule, TLPMA, TLPMAIO}
import freechips.rocketchip.devices.tilelink.{CLINT, CLINTParams, DevNullParams, PLICParams, TLError, TLPLIC}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.interrupts.{IntSourceNode, IntSourcePortSimple}
import freechips.rocketchip.regmapper.{RegField, RegFieldAccessType, RegFieldDesc, RegFieldGroup}
import utils.{BinaryArbiter, TLEdgeBuffer}
import xiangshan.{DebugOptionsKey, HasXSParameter, XSBundle, XSCore, XSCoreParameters, XSTileKey}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.tilelink._
import top.BusPerfMonitor
import xiangshan.backend.fu.PMAConst
import huancun._
import huancun.debug.TLLogger

case object SoCParamsKey extends Field[SoCParameters]

object SoCResourceAnchors {
  val chosen = new Device {
    def describe(resources: ResourceBindings): Description = {
      Description("chosen", Map(
        "bootargs"    -> resources("bootargs").map(_.value),
        "stdout-path" -> resources("stdout"  ).map(_.value)
      ))
    }
  }

  val reserved_memory = new Device { r =>
    def describe(resources: ResourceBindings): Description = {
      Description("reserved-memory", Map(
        "#address-cells" -> resources("width").map(_.value),
        "#size-cells"    -> resources("width").map(_.value),
        "ranges"         -> Nil
      ))
    }
  }
}

case class SoCParameters
(
  EnableILA: Boolean = false,
  PAddrBits: Int = 36,
  extIntrs: Int = 64,
  timebase: Int = 10000000,
  L3NBanks: Int = 4,
  L3CacheParamsOpt: Option[HCCacheParameters] = Some(HCCacheParameters(
    name = "l3",
    level = 3,
    ways = 8,
    sets = 2048 // 1MB per bank
  ))
){
  // L3 configurations
  val L3InnerBusWidth = 256
  val L3BlockSize = 64
  // on chip network configurations
  val L3OuterBusWidth = 256
}

trait HasSoCParameter {
  implicit val p: Parameters

  val soc = p(SoCParamsKey)
  val debugOpts = p(DebugOptionsKey)
  val tiles = p(XSTileKey)

  val mgBSParam = p(MidgardKey)

  val NumCores = tiles.size
  val EnableILA = soc.EnableILA

  // L3 configurations
  val L3InnerBusWidth = soc.L3InnerBusWidth
  val L3BlockSize = soc.L3BlockSize
  val L3NBanks = soc.L3NBanks

  // on chip network configurations
  val L3OuterBusWidth = soc.L3OuterBusWidth

  val NrExtIntr = soc.extIntrs
}

class ILABundle extends Bundle {}


abstract class BaseSoC()(implicit p: Parameters) extends LazyModule with HasSoCParameter {
  val bankedNode = BankBinder(L3NBanks, L3BlockSize)
  val peripheralXbar = TLXbar()
  val l3_xbar = TLXbar()
  val l3_banked_xbar = TLXbar()

  val clint: CLINT
  val plic:  TLPLIC
}

// We adapt the following three traits from rocket-chip.
// Source: rocket-chip/src/main/scala/subsystem/Ports.scala
trait HaveSlaveAXI4Port {
  this: BaseSoC =>

  val idBits = 14

  val l3FrontendAXI4Node = AXI4MasterNode(Seq(AXI4MasterPortParameters(
    Seq(AXI4MasterParameters(
      name = "dma",
      id = IdRange(0, 1 << idBits)
    ))
  )))
  private val errorDevice = LazyModule(new TLError(
    params = DevNullParams(
      address = Seq(AddressSet(0x0, 0x7fffffffL)),
      maxAtomic = 8,
      maxTransfer = 64),
    beatBytes = L3InnerBusWidth / 8
  ))
  private val error_xbar = TLXbar()

  l3_xbar :=
    TLFIFOFixer() :=
    TLWidthWidget(32) :=
    TLBuffer() :=
    AXI4ToTL() :=
    AXI4Buffer() :=
    AXI4UserYanker(Some(32)) :=
    AXI4Fragmenter() :=
    AXI4Buffer() :=
    AXI4IdIndexer(1) :=
    l3FrontendAXI4Node
  errorDevice.node := l3_xbar

  val dma = InModuleBody {
    l3FrontendAXI4Node.makeIOs()
  }
}

trait HaveAXI4MemPort {
  this: BaseSoC =>
  val device = new MemoryDevice
  val memRange = AddressSet(0x00000000L, (1L << mgBSParam.paBits) - 1L).subtract(AddressSet(0x0L, 0x7fffffffL))
  val memAXI4SlaveNode = AXI4SlaveNode(Seq(
    AXI4SlavePortParameters(
      slaves = Seq(
        AXI4SlaveParameters(
          address = memRange,
          regionType = RegionType.UNCACHED,
          executable = true,
          supportsRead = TransferSizes(1, L3BlockSize),
          supportsWrite = TransferSizes(1, L3BlockSize),
          interleavedId = Some(0),
          resources = device.reg("mem")
        )
      ),
      beatBytes = L3OuterBusWidth / 8
    )
  ))

  val mem_xbar = TLXbar()

  val mmu = if (mgBSParam.en) Some(LazyModule(new BSMMUWrapper())) else None

  val mem_nodes =
    Seq(mem_xbar, TLBuffer()) ++
      mmu.map(e => Seq(TLWidthWidget(64), e.adp_node)).getOrElse(Seq(TLCacheCork())) ++
    Seq(TLWidthWidget(32), bankedNode)

  mem_nodes.reduce(_ :=* _)

  mem_xbar :=
    TLWidthWidget(8) :=
    TLBuffer() :=
    peripheralXbar

  mmu.foreach(_.ctl_node := TLWidthWidget(8) := peripheralXbar)

  memAXI4SlaveNode :=
    AXI4Buffer() :=
    AXI4IdIndexer(idBits = 4) :=
    AXI4UserYanker() :=
    AXI4Deinterleaver(L3BlockSize) :=
    TLToAXI4() :=
    TLSourceShrinker(16) :=
    TLWidthWidget(L3OuterBusWidth / 8) :=
    TLBuffer() :=
    mem_xbar

  val memory = InModuleBody {
    memAXI4SlaveNode.makeIOs()
  }
}

class SoCMisc()(implicit p: Parameters) extends BaseSoC
  with HaveAXI4MemPort
  with PMAConst
  with HaveSlaveAXI4Port
{
  val peripheral_ports = Array.fill(NumCores) { TLTempNode() }
  val core_to_l3_ports = Array.fill(NumCores) { TLTempNode() }

  val l3_in = TLTempNode()
  val l3_out = TLTempNode()
  val l3_mem_pmu = BusPerfMonitor(enable = !debugOpts.FPGAPlatform)

  l3_in :*= TLEdgeBuffer(_ => true, Some("L3_in_buffer")) :*= l3_banked_xbar
  bankedNode :*= TLLogger("MEM_L3", debugOpts.EnableTLLogger) :*= l3_mem_pmu :*= l3_out

  if(soc.L3CacheParamsOpt.isEmpty){
    l3_out :*= l3_in
  }

  for(port <- peripheral_ports) {
    peripheralXbar := TLBuffer() := port
  }

  for ((core_out, i) <- core_to_l3_ports.zipWithIndex){
    l3_banked_xbar :=*
      TLLogger(s"L3_L2_$i", debugOpts.EnableTLLogger) :=*
      TLBuffer() :=
      core_out
  }
  l3_banked_xbar := TLBuffer() := l3_xbar

  val clint = LazyModule(new CLINT(CLINTParams(0x38000000L), 8))
  clint.node := peripheralXbar

  class IntSourceNodeToModule(val num: Int)(implicit p: Parameters) extends LazyModule {
    val sourceNode = IntSourceNode(IntSourcePortSimple(num, ports = 1, sources = 1))
    lazy val module = new LazyModuleImp(this){
      val in = IO(Input(Vec(num, Bool())))
      in.zip(sourceNode.out.head._1).foreach{ case (i, s) => s := i }
    }
  }

  val plic = LazyModule(new TLPLIC(PLICParams(0x3c000000L), 8))
  val plicSource = LazyModule(new IntSourceNodeToModule(NrExtIntr))

  plic.intnode := plicSource.sourceNode
  plic.node := peripheralXbar

  val pll_node = TLRegisterNode(
    address = Seq(AddressSet(0x3a000000L, 0xfff)),
    device = new SimpleDevice("pll_ctrl", Seq()),
    beatBytes = 8,
    concurrency = 1
  )
  pll_node := peripheralXbar

  val debugModule = LazyModule(new DebugModule(NumCores)(p))
  debugModule.debug.node := peripheralXbar
  debugModule.debug.dmInner.dmInner.sb2tlOpt.foreach { sb2tl  =>
    l3_xbar := TLBuffer() := TLWidthWidget(1) := sb2tl.node
  }

  val pma = LazyModule(new TLPMA)
  pma.node :=
    TLBuffer() :=
    peripheralXbar

  lazy val module = new LazyModuleImp(this){

    val debug_module_io = IO(chiselTypeOf(debugModule.module.io))
    val ext_intrs = IO(Input(UInt(NrExtIntr.W)))
    val pll0_lock = IO(Input(Bool()))
    val pll0_ctrl = IO(Output(Vec(6, UInt(32.W))))
    val cacheable_check = IO(new TLPMAIO)

    debugModule.module.io <> debug_module_io

    // sync external interrupts
    require(plicSource.module.in.length == ext_intrs.getWidth)
    for ((plic_in, interrupt) <- plicSource.module.in.zip(ext_intrs.asBools)) {
      val ext_intr_sync = RegInit(0.U(3.W))
      ext_intr_sync := Cat(ext_intr_sync(1, 0), interrupt)
      plic_in := ext_intr_sync(2)
    }

    pma.module.io <> cacheable_check

    // div-4
    val cnt_q = Wire(UInt(2.W))

    cnt_q := RegNext(cnt_q + 1.U, 0.U)

    clint.module.io.rtcTick := cnt_q(1).asBool

    val pll_ctrl_regs = Seq.fill(6){ RegInit(0.U(32.W)) }
    val pll_lock = RegNext(next = pll0_lock, init = false.B)

    pll0_ctrl <> VecInit(pll_ctrl_regs)

    pll_node.regmap(
      0x000 -> RegFieldGroup(
        "Pll", Some("PLL ctrl regs"),
        pll_ctrl_regs.zipWithIndex.map{
          case (r, i) => RegField(32, r, RegFieldDesc(
            s"PLL_ctrl_$i",
            desc = s"PLL ctrl register #$i"
          ))
        } :+ RegField.r(32, Cat(0.U(31.W), pll_lock), RegFieldDesc(
          "PLL_lock",
          "PLL lock register"
        ))
      )
    )
  }
}
