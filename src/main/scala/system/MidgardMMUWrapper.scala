package system

import chipsalliance.rocketchip.config._
import chisel3._
import chisel3.util._
import freechips.rocketchip.regmapper.{RegField, RegFieldDesc, RegisterRouter, RegisterRouterParams}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.tilelink._

import midgard._
import midgard.util._


class MidgardMMUWrapper(implicit p: Parameters) extends LazyModule{
    val P = p(MidgardKey)

    val cfg_node = TLManagerNode(
                   Seq(TLSlavePortParameters.v1(
                        Seq(TLSlaveParameters.v1(
                                address         = Seq(AddressSet(P.ctlBase, P.ctlSize)),
                                regionType      = RegionType.UNCACHED,
                                supportsGet     = TransferSizes(1, 8),
                                supportsPutFull = TransferSizes(1, 8),
                                fifoId          = Some(0))),
                        beatBytes = 8)))

    val node = TLAdapterNode(
    clientFn  = { case cp =>
      cp.v1copy(clients = cp.clients.map { c => c.v1copy(
        supportsProbe = TransferSizes.none,
        sourceId = IdRange(c.sourceId.start*2, c.sourceId.end*2))})},
    managerFn = { case mp =>
      mp.v1copy(
        endSinkId = if (mp.managers.exists(_.regionType == RegionType.UNCACHED)) 8 else 0,
        managers = mp.managers.map { m => m.v1copy(
          supportsAcquireB = if (m.regionType == RegionType.UNCACHED) m.supportsGet     else m.supportsAcquireB,
          supportsAcquireT = if (m.regionType == RegionType.UNCACHED) m.supportsPutFull.intersect(m.supportsGet) else m.supportsAcquireT,
          alwaysGrantsT    = if (m.regionType == RegionType.UNCACHED) m.supportsPutFull else m.alwaysGrantsT)})})

    lazy val module = new LazyModuleImp(this) {
        (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>

            //Config Channel:
            val cfg = cfg_node.in.head._1

            val clients = edgeIn.client.clients
            val caches = clients.filter(_.supports.probe)

            val midgardMMU = Module(new backside.MMU(P.copy(llcIdx = edgeIn.bundle.sourceBits)))

            //LLC should have another cache level below it. So configure it as such.

            //In case of a release, don't send request to MMU, directly send a releaseAck on D channel

            //LLC output A and C(Eviction) channel to MMU llc_req_i channel adapter:
            //?What will be the difference in behaviour of AcquireBlock and AcquirePerm?
            // assert(in.a.bits.opcode === TLMessages.AcquireBlock)    //Since we are always sending toT over D channel, AcquirePerm request should never come to the MMU from A channel
            val releaseRequest = in.c.valid && (in.c.bits.opcode === TLMessages.Release)
            val releaseDataRequest = in.c.valid && (in.c.bits.opcode === TLMessages.ReleaseData)
            val clashingRequest = in.a.valid && (releaseRequest || releaseDataRequest)  //If A and C both send the request at the same time

            //LLC Flush Logic
            val isLLCFlush                  = cfg.a.bits.address(6 , 3) > 0x7.U    //LLCFlush Register is the 9th 64-bit register
            val LLCFlushAddress             = RegEnable(cfg.a.bits.data, cfg.a.fire && isLLCFlush)  //Address to be flushed fromt the LLC
            val outstandingFlush            = RegInit(false.B)  //True if there is a pending flush
            val flushRequestAccepted        = RegInit(false.B)  //When the flush request is accepted by B Channel
            val receivedWriteBackData       = RegInit(false.B)  //Flush data is received from the LLC over C channel as a ProbeAckData
            val writeBackData               = RegInit(0.U(P.clBits.W))  //Data that needs to written back(received over C channel)
            val writeBackSentToMMU          = RegInit(false.B)  //Flush request is now sent to MMU llc_req_i for Midgard Address translation 
                                                                //and subsequent Writeback
            val acceptFakeLLCRespFromMMU        = RegInit(false.B)  //Accept a Fake response from the MMU llc_resp_o so that as it was not initiated by the LLC
            val writeBackSentToMemory       = RegInit(false.B)  //Writeback is finally sent to the DDR Memory Channel

            val sendFakeResponseToCfgD      = RegInit(false.B)  //Fake response to Cfg D channel after Cfg A request is accepted in case of a flush

            val llc_a_to_llc_d = Wire(in.d.bits.cloneType)
            val llc_c_to_llc_d = Wire(in.d.bits.cloneType)

            val outstandingAcquireCounter = RegInit(0.U(32.W))           //If A accepts n Acquire request, this is asserted till the request is served


            midgardMMU.llc_req_i.valid      := ((in.a.valid && (in.a.bits.opcode === TLMessages.AcquireBlock)
                                                || releaseDataRequest       //Request to be sent to MMU only on a ReleaseData on C
                                                || (receivedWriteBackData && !writeBackSentToMMU))   //When WB data is received for flushing, its address is sent to the mmuReq channel for Midgard address translation
                                                && outstandingAcquireCounter  === 0.U)  //Only 1 Ack at a time 
           
            midgardMMU.llc_req_i.bits.mcn   := Mux((receivedWriteBackData && !writeBackSentToMMU), 
                                                LLCFlushAddress >> P.clWid, 
                                                Mux(releaseDataRequest, 
                                                    ((in.c.bits.address >> P.clWid)), 
                                                    ((in.a.bits.address >> P.clWid))))  //C is given priority over A
            midgardMMU.llc_req_i.bits.rnw   := !(receivedWriteBackData && !writeBackSentToMMU) && 
                                                !(releaseDataRequest || releaseRequest) //Request over A channel will always be Read Requests
            
            midgardMMU.llc_req_i.bits.idx   := Mux((receivedWriteBackData && !writeBackSentToMMU), 
                                                0xfff.U,    //This source id distinguishes a Flush from a normal request on llc_req_i channel
                                                            //It is later used to tear down the flush request
                                                Mux(releaseDataRequest, 
                                                    in.c.bits.source, 
                                                    in.a.bits.source)) //Connect to Source/Sink
           
            midgardMMU.llc_req_i.bits.pcn   := DontCare
            
            midgardMMU.llc_req_i.bits.data  := Mux((receivedWriteBackData && !writeBackSentToMMU), 
                                                writeBackData, 
                                                in.c.bits.data)   //Only ReleaseData over C channel will send Data, requests over A channel don't send data
            
            in.a.ready                      :=  
                                                !(receivedWriteBackData && !writeBackSentToMMU) && //A Flush is being accepted
                                                !(releaseRequest || releaseDataRequest) && //There is a release request over C channel
                                                outstandingAcquireCounter  === 0.U &&  //Multiple Outstanding Acquires not supported
                                                midgardMMU.llc_req_i.ready   //In case of a clash as in the above two conditions, A's request will not be accepted

            //C's ready will be generated later when Probe Response is present

            /*LLC Output D channel from MMU llc_resp_o channel adapter
             *Response over D channel can be due to the following:
                *Grant/Grant Data as a resonse to Acquire on A channel:
                    This A channel request is sent to llc_req_i.
                    A channel will send only AcquireBlock request so
                    the response will always be Grant Data over D.
                    Permission transfer will always be toT to ensure AcquirePerm is never sent by A
                * Release Ack as a response to Release/Release Data on C channel:
                
                A Release(Same block) is not issued by the Master if there is a Grant pending.
                Since we are prioritizing Release over Acquire(C over A),
                a collission should never happen.
                First finish grant then issue release.
            */
            
            when(in.a.fire){
                outstandingAcquireCounter := outstandingAcquireCounter + 1.U
            }.elsewhen(outstandingAcquireCounter =/= 0.U && in.d.fire){    //At this point, the Acquire has been served
                outstandingAcquireCounter := outstandingAcquireCounter - 1.U
            }


            when(isLLCFlush && cfg.a.fire){
                outstandingFlush            := true.B  
            }.elsewhen(midgardMMU.llc_resp_o.fire && midgardMMU.llc_resp_o.bits.idx.andR()){  
                outstandingFlush            := false.B  //Flush writeback is distinguished from a normal writeback by the sourceID
            }

            when(isLLCFlush && cfg.a.fire){
                sendFakeResponseToCfgD            := true.B  
            }.elsewhen(cfg.d.fire && sendFakeResponseToCfgD){  
                sendFakeResponseToCfgD            := false.B    //Deassert it as soon as response is sent over Cfg D
                                                                //Fake Cfg D response is given priority over other responses
            }

            when(in.b.fire && outstandingFlush){
                flushRequestAccepted        := true.B
            }.elsewhen(midgardMMU.llc_resp_o.fire && midgardMMU.llc_resp_o.bits.idx.andR()){
                flushRequestAccepted        := false.B
            }

            when(in.c.fire && flushRequestAccepted){
                receivedWriteBackData       := true.B
                writeBackData               := in.c.bits.data
            }.elsewhen(midgardMMU.llc_resp_o.fire && midgardMMU.llc_resp_o.bits.idx.andR()){
                receivedWriteBackData       := false.B
            }

            when(midgardMMU.llc_req_i.fire && midgardMMU.llc_req_i.bits.idx.andR && receivedWriteBackData){
                writeBackSentToMMU       := true.B
            }.elsewhen(midgardMMU.llc_resp_o.fire && midgardMMU.llc_resp_o.bits.idx.andR()){
                writeBackSentToMMU       := false.B
            }

            when(out.a.fire && receivedWriteBackData){
                writeBackSentToMemory       := true.B
            }.elsewhen(midgardMMU.llc_resp_o.fire && midgardMMU.llc_resp_o.bits.idx.andR()){
                writeBackSentToMemory       := false.B
            }

            when(midgardMMU.llc_req_i.fire && midgardMMU.llc_req_i.bits.idx.andR && receivedWriteBackData){
                acceptFakeLLCRespFromMMU        := true.B
            }.elsewhen(midgardMMU.llc_resp_o.fire && midgardMMU.llc_resp_o.bits.idx.andR()){
                acceptFakeLLCRespFromMMU        := false.B
            }

            //LLC doesn't expect any response in case of ProbeAck for Flush 
            llc_a_to_llc_d.corrupt     := 0.U
            llc_a_to_llc_d.data        := midgardMMU.llc_resp_o.bits.data
            llc_a_to_llc_d.denied      := 0.U
            llc_a_to_llc_d.opcode      := TLMessages.GrantData  //Always Grant Data never just Grant
            llc_a_to_llc_d.param       := TLPermissions.toT      //?toT
            llc_a_to_llc_d.sink        := 0.U
            llc_a_to_llc_d.size        := P.clWid.U
            llc_a_to_llc_d.source      := midgardMMU.llc_resp_o.bits.idx

            llc_c_to_llc_d.corrupt     := 0.U
            llc_c_to_llc_d.data        := 0.U
            llc_c_to_llc_d.denied      := 0.U
            llc_c_to_llc_d.opcode      := TLMessages.ReleaseAck
            llc_c_to_llc_d.param       := 0.U
            llc_c_to_llc_d.sink        := 0.U
            llc_c_to_llc_d.size        := P.clWid.U
            llc_c_to_llc_d.source      := Mux(releaseRequest, in.c.bits.source, midgardMMU.llc_resp_o.bits.idx)


            midgardMMU.llc_resp_o.ready     := in.d.ready || acceptFakeLLCRespFromMMU  //Confirm
            in.d.valid                      := ((midgardMMU.llc_resp_o.valid && 
                                                !(acceptFakeLLCRespFromMMU && midgardMMU.llc_resp_o.bits.idx.andR()))   //If a Fake LLC resp needs to be sent, it is only sent when the idx are all 1s, otherwise it will mask a legit Grant Response
                                                || (releaseRequest || releaseDataRequest))  //Directly send a ReleaseAck in case of a releaseRequest on C channel
            in.d.bits                       := Mux(outstandingAcquireCounter =/= 0.U, 
                                                llc_a_to_llc_d, 
                                                llc_c_to_llc_d)  //Finish Grant first



            //LLC Input B channel from MMU llc_req_o channel Adapter:
            in.b.valid                      := ((midgardMMU.llc_req_o.valid && !outstandingFlush) || 
                                                (outstandingFlush && !flushRequestAccepted))     //Both flush and MMU send this request. 
                                                // && !outstandingAcquire                           //flushRequestAccepted is addded to avoid sending multiple requests fro same release to B channel
            midgardMMU.llc_req_o.ready      := in.b.ready && !(outstandingFlush && !flushRequestAccepted)  //Unless flush request is accepted over B channel, the MMU llc_req_o request is stalled
            
            in.b.bits.address               := Mux(outstandingFlush, 
                                                    LLCFlushAddress, 
                                                    (midgardMMU.llc_req_o.bits.mcn ## 0.U(P.clWid.W)))
            
            in.b.bits.corrupt               := 0.U
            in.b.bits.data                  := DontCare
            
            in.b.bits.mask                  := MaskGen(in.b.bits.address, P.clWid.U, 64)//?
            
            in.b.bits.opcode                := TLMessages.Probe //Always Probe the entire block, not just the permission
            
            in.b.bits.param                 := Mux(outstandingFlush, 
                                                    TLPermissions.toN, 
                                                    TLPermissions.toT)  //Invalidate the LLC line in case of a Flush
            
            in.b.bits.size                  := P.clWid.U
            // in.b.bits.source                := Mux(outstandingFlush, 
            //                                         0xfff.U, 
            //                                         edgeIn.master.endSourceId.U)  //Source ID is all 1s in case of Flush to distingish it from a non-flush request

            // in.b.bits.source                := edgeIn.master.endSourceId.U
            in.b.bits.source                := 0.U


            //LLC Output C channel to MMU llc_resp_i channel Adapter
            //assert(!(in.c.bits.opcode === TLMessages.ProbeAck))
            val isProbeAck                   = (in.c.bits.opcode === TLMessages.ProbeAckData || 
                                                in.c.bits.opcode === TLMessages.ProbeAck)


            midgardMMU.llc_resp_i.valid     := in.c.valid && isProbeAck && !flushRequestAccepted    //If flush Request was accepted, the response over C will not be sent to MMU llc_resp_i.
                                                                                                    //Instead, it will be sent to MMU llc_req_i for Address Translation
            
            midgardMMU.llc_resp_i.bits.data := in.c.bits.data
            midgardMMU.llc_resp_i.bits.hit  := !(in.c.bits.param === TLPermissions.TtoN 
                                            || in.c.bits.param === TLPermissions.NtoN 
                                            || in.c.bits.param === TLPermissions.BtoN)  //?Confirm

            in.c.ready                      := releaseRequest || midgardMMU.llc_resp_i.ready || midgardMMU.llc_req_i.ready //Not sure......but should be ok

            in.e.ready                      := 1.U  //Always ready to accept the acknowledgement over E
            //Memory MMU interface
            //A Channel
            out.a.valid                     := (midgardMMU.mem_req_o.valid)
            midgardMMU.mem_req_o.ready      := out.a.ready

            out.a.bits.address              := midgardMMU.mem_req_o.bits.pcn ## 0.U(P.clWid.W)
            out.a.bits.corrupt              := 0.U
            
            out.a.bits.data                 := Mux(midgardMMU.mem_req_o.bits.rnw, 
                                                    0.U, 
                                                    midgardMMU.mem_req_o.bits.data)
            
            out.a.bits.mask                 := MaskGen(in.a.bits.address, P.clWid.U, 64)   //Use Mask Gen
            
            out.a.bits.opcode               := Mux(!midgardMMU.mem_req_o.bits.rnw, 
                                                    TLMessages.PutFullData, //Always PutFullData
                                                    TLMessages.Get)
            
            out.a.bits.param                := 0.U
            out.a.bits.size                 := P.clWid.U
            out.a.bits.source               := midgardMMU.mem_req_o.bits.idx

            //D Channel
            midgardMMU.mem_resp_i.valid     := out.d.valid
            out.d.ready                     := midgardMMU.mem_resp_i.ready

            assert(!(out.d.bits.opcode =/= TLMessages.AccessAckData && out.d.bits.opcode =/= TLMessages.AccessAck && out.d.valid))
            // assert(!(out.d.bits.source =/= 0.U && out.d.valid))    //Since 0 is sent as source ID over A channel, we would expect 0 on D channel as well
            midgardMMU.mem_resp_i.bits.rnw  := out.d.bits.opcode === TLMessages.AccessAckData
            midgardMMU.mem_resp_i.bits.data := out.d.bits.data
            midgardMMU.mem_resp_i.bits.err  := out.d.bits.denied
            midgardMMU.mem_resp_i.bits.idx  := out.d.bits.source

            
            // a
            cfg.a.ready                     :=  midgardMMU.ctl_req_i.ready && !outstandingFlush //A new request is accepted only after flush is served

            midgardMMU.ctl_req_i.valid      :=  cfg.a.valid &&  !isLLCFlush
            midgardMMU.ctl_req_i.bits.rnw   := (cfg.a.bits.opcode === TLMessages.Get)
            midgardMMU.ctl_req_i.bits.addr  :=  cfg.a.bits.address(6, 3)    //Since each register is 64-bits long
            midgardMMU.ctl_req_i.bits.data  :=  cfg.a.bits.data

            // // b
            // cfg.b.valid               :=  0.U

            // // c
            // cfg.c.ready               :=  0.U

            // d
            cfg.d.valid               :=  midgardMMU.ctl_resp_o.valid || sendFakeResponseToCfgD
            cfg.d.bits.opcode         :=  Mux(!midgardMMU.ctl_resp_o.bits.rnw || sendFakeResponseToCfgD,
                                            TLMessages.AccessAck,
                                            TLMessages.AccessAckData)
            cfg.d.bits.param          :=  0.U
            cfg.d.bits.size           :=  3.U   //8-Bytes
            cfg.d.bits.source         :=  RegEnable(cfg.a.bits.source, cfg.a.fire())
            cfg.d.bits.sink           :=  0.U
            cfg.d.bits.denied         :=  0.U
            cfg.d.bits.data           :=  midgardMMU.ctl_resp_o.bits.data
            cfg.d.bits.corrupt        :=  0.U

            midgardMMU.ctl_resp_o.ready    :=  cfg.d.ready   && !sendFakeResponseToCfgD //Fake Cfg D response is given priority over other responses

            // // e
            // cfg.e.ready               :=  0.U
        }
    }
}