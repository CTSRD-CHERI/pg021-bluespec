package AXI4_DMA_Register_Module;

// Bluespec imports
import Vector :: *;

// AXI imports
import AXI :: *;
import SourceSink :: *;

// local imports
import AXI4_DMA_Types :: *;
import AXI4_DMA_Internal_Reg_Module :: *;

import CHERICap :: *;
import CHERICC_Fat :: *;

typedef enum {
   RESET,
   HALTED,
   RUNNING,
   WAIT_SECOND_WRITE
} Reg_State deriving (Bits, FShow, Eq);

interface AXI4_DMA_Register_Module_IFC #(numeric type id_
                                        ,numeric type addr_
                                        ,numeric type data_
                                        ,numeric type awuser_
                                        ,numeric type wuser_
                                        ,numeric type buser_
                                        ,numeric type aruser_
                                        ,numeric type ruser_);
   interface AXI4_Slave #(id_, addr_, data_, awuser_, wuser_, buser_, aruser_, ruser_) axi4_slave;

   // Becomes valid on the cycle when there is a write to one of the trigger registers
   // The trigger registers are the _TAILDESC registers when in 32-bit addressing mode,
   // and _TAILDESC_MBS when in 64-bit addressing mode
   (* always_ready *)
   method Maybe #(DMA_Dir) trigger;

   method Action set_verbosity (Bit #(4) new_verb);
   method Action reset;

   (* always_ready *)
   method Bool get_halt_to_idle;
   method Action halt_to_idle;
endinterface


module mkAXI4_DMA_Register_Module #(AXI4_DMA_Int_Reg_IFC dma_int_reg,
                                    Reg #(Bool) rg_mm2s_hit_tail,
                                    Reg #(Bool) rg_s2mm_hit_tail)
                                   (AXI4_DMA_Register_Module_IFC #(id_, addr_, data_, awuser_, wuser_, buser_, aruser_, ruser_))
                                   provisos ( Add #(a__, SizeOf #(DMA_Reg_Index), addr_)
                                            , Add #(b__, SizeOf #(DMA_Reg_Word), data_)
                                            , Add #(c__, data_, 128)
                                            , Add #(d__, data_, 129)
                                            , Add #(e__, c__, 129)
                                            , Add #(f__, wuser_, 1));

   // TODO not sure if this is the cleanest way to specify this in Bluespec.
   // Would like to have a compile-time variable here which contains the trigger
   // register index
   let trigger_index_mm2s = valueof (addr_) > 32 ? DMA_MM2S_TAILDESC_MSB : DMA_MM2S_TAILDESC;
   let trigger_index_s2mm = valueof (addr_) > 32 ? DMA_S2MM_TAILDESC_MSB : DMA_S2MM_TAILDESC;

   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);

   AXI4_Shim #(id_, addr_, data_,
               awuser_, wuser_, buser_,
               aruser_, ruser_) shim <- mkAXI4ShimFF;
   AXI4_Master #(id_, addr_, data_,
                 awuser_, wuser_, buser_,
                 aruser_, ruser_) serialiser <- mkSerialiser (shim.master);

   Reg #(Reg_State) rg_state <- mkReg (RESET);

   // aggregate the capability that is currently being written
   Reg #(CapMem) rg_cap <- mkReg (nullCap);
   Reg #(Bit #(1)) rg_cap_valid <- mkReg (0);

   // Used to write triggers
   RWire #(DMA_Dir) rw_trigger <- mkRWire;

   PulseWire pw_halt_to_idle <- mkPulseWire;

   rule rl_debug;
      if (rg_verbosity > 2) begin
         $display ("register module");
         $display ("    state: ", fshow (rg_state));
      end
   endrule

   rule rl_reset (rg_state == RESET);
      if (rg_verbosity > 0) begin
         $display ("AXI4 DMA Register Reset");
      end

      shim.clear;
      // TODO might need to reset serialiser, because there might be something stuck
      // in the FIFO, but the serialiser has no "clear" method

      rg_state <= HALTED;
   endrule

   // behaviour: when writing the _CAP registers
   //               only accept 2-flit 64-bit requests
   //               TODO probably want to change this for 32-bit
   //            otherwise
   //               only accept single-flit DMA_Reg_Word-sized write requests
   //
   //            only accept transactions when both the AW and W signals are valid
   rule rl_handle_write ((rg_state == RUNNING
                          || rg_state == HALTED)
                         && serialiser.aw.canPeek
                         && serialiser.w.canPeek);
      let awflit = serialiser.aw.peek;
      let wflit = serialiser.w.peek;
      DMA_Reg_Index idx = unpack (truncate (awflit.awaddr >> 2));

      // AW flit sanitization
      let is_aligned = awflit.awaddr[1:0] == 0;
      let is_single_flit = awflit.awlen == 0;
      let is_double_flit = awflit.awlen == 1;
      let is_cap_reg = idx == DMA_MM2S_CURDESC_CAP
                       || idx == DMA_S2MM_CURDESC_CAP;
      let is_correct_burst_type = awflit.awburst == FIXED || awflit.awburst == INCR; // WRAP requires a length >= 2 flits
      // our words are either 4 bytes or 8 bytes depending on what register
      // is being written
      let is_correct_size = is_cap_reg ? awflit.awsize == 8
                                       : awflit.awsize == 4;
      let is_correct_num_of_flits = is_cap_reg ? awflit.awlen == 1
                                               : awflit.awlen == 0;

      // don't care about lock, cache, prot, qos or region
      let is_valid_request = is_aligned
                             && is_correct_num_of_flits
                             && is_correct_burst_type
                             && is_correct_size;

      // TODO handle buses that aren't 32 or 64 bits?
      // not sure that is required for this slave port
      let word_lower = awflit.awaddr[2] == 1'b0;
      DMA_Reg_Word newval = word_lower ? truncate (wflit.wdata)
                                       : truncateLSB (wflit.wdata);

      if (is_valid_request) begin
         // update the register value written
         if (is_cap_reg) begin
            // TODO this assumes 64-bit-wide bus
            rg_cap <= zeroExtend (wflit.wdata);
            rg_cap_valid <= zeroExtend (wflit.wuser);
            rg_state <= WAIT_SECOND_WRITE;
         end else begin
            dma_int_reg.external_write (idx, newval);
         end
         if (idx == DMA_MM2S_DMACR || idx == DMA_S2MM_DMACR) begin
            if (newval[0] == 1) begin
               // transition DMA engine into an idle state from halted state
               pw_halt_to_idle.send;
            end
         end
         if (rg_verbosity > 1) begin
            // the internal register file already displays the value that is passed in
            // also display the raw value for debugging when required
            $display ("   raw flit value: ", fshow (wflit.wdata));
         end
      end else begin
         $display ("dma slave write failed, index ", fshow (idx), " with value ", fshow (newval));
         $display ("conditions:");
         $display ("   flit: ", fshow (awflit));
         $display ("   is_aligned: ", fshow (is_aligned));
         $display ("      awflit.awaddr: ", fshow (awflit.awaddr));
         $display ("   is_single_flit: ", fshow (is_single_flit));
         $display ("      awflit.awlen: ", fshow (awflit.awlen));
         $display ("   is_correct_burst_type: ", fshow (is_correct_burst_type));
         $display ("      awflit.awburst: ", fshow (awflit.awburst));
         $display ("   is_correct_size: ", fshow (is_correct_size));
         $display ("      awflit.awsize: ", fshow (awflit.awsize));
      end

      // craft and send reply, and drop awflit if this was a single-flit write
      // TODO for now, we just always return OKAY
      // We should return a SLVERR when the request is not valid
      if (is_single_flit) begin
         AXI4_BFlit #(id_, buser_) bflit = AXI4_BFlit { bid: awflit.awid
                                                      //, bresp: is_valid_request ? OKAY : SLVERR
                                                      , bresp: OKAY
                                                      , buser: 0 };
         serialiser.b.put (bflit);
         serialiser.aw.drop;
      end

      serialiser.w.drop;

      // handle detecting writes to special trigger registers
      // trigger_index_s2mm and trigger_index_mm2s should be compile-time constant values
      // based on whether the address field is 32 or 64 bits
      if (is_valid_request) begin
         if (idx == trigger_index_s2mm) begin
            // only trigger if we had already hit the tail
            // TODO handle 32-bit addresses
            let cur_addr = {pack (dma_int_reg.s2mm_curdesc_msb), pack (dma_int_reg.s2mm_curdesc)};
            let cur_tail = {pack (newval), pack (dma_int_reg.s2mm_taildesc)};

            if (rg_s2mm_hit_tail && cur_tail > cur_addr) begin
               // We had hit the previous tail, and the new tail is at a higher
               // memory address than our current address, so we can fetch
               // a new buffer descriptor
               rw_trigger.wset (S2MM);
               rg_s2mm_hit_tail <= False;
               if (rg_verbosity > 0) begin
                  $display ("triggering s2mm SG fetch");
               end
            end else begin
               // Print debug information in the case where we skip the SG fetch trigger
               if (rg_verbosity > 0) begin
                  $display ("skipping s2mm SG fetch trigger");
               end
               if (rg_verbosity > 1) begin
                  $display ("    cur_addr: ", fshow (cur_addr));
                  $display ("    cur_tail: ", fshow (cur_tail));
               end
            end
         end else if (idx == trigger_index_mm2s) begin
            // only trigger if we had already run out of BDs
            // TODO handle 32-bit addresses
            let cur_addr = {pack (dma_int_reg.s2mm_curdesc_msb), pack (dma_int_reg.s2mm_curdesc)};
            let cur_tail = {pack (newval), pack (dma_int_reg.s2mm_taildesc)};

            if (rg_mm2s_hit_tail && cur_tail > cur_addr) begin
               // We had hit the previous tail, and the new tail is at a higher
               // memory address than our current address, so we can fetch
               // a new buffer descriptor
               rw_trigger.wset (MM2S);
               rg_mm2s_hit_tail <= False;
               if (rg_verbosity > 0) begin
                  $display ("triggering mm2s SG fetch");
               end
            end else begin
               // Print debug information in the case where we skip the SG fetch trigger
               if (rg_verbosity > 0) begin
                  $display ("skipping mm2s SG fetch trigger");
               end
               if (rg_verbosity > 1) begin
                  $display ("    cur_addr: ", fshow (cur_addr));
                  $display ("    curdesc_msb: ", fshow (pack (dma_int_reg.s2mm_curdesc_msb)));
                  $display ("    curdesc: ", fshow (pack (dma_int_reg.s2mm_curdesc)));
                  $display ("    cur_tail: ", fshow (cur_tail));
               end
            end
         end
      end
   endrule

   rule rl_handle_write_2 (rg_state == WAIT_SECOND_WRITE
                           && serialiser.aw.canPeek
                           && serialiser.w.canPeek);
      let awflit = serialiser.aw.peek;
      let wflit = serialiser.w.peek;
      DMA_Reg_Index idx = unpack (truncate (awflit.awaddr >> 2));

      // craft and send reply
      // TODO maybe return something that is not OKAY
      AXI4_BFlit #(id_, buser_) bflit = AXI4_BFlit { bid: awflit.awid
                                                   , bresp: OKAY
                                                   , buser: 0 };
      serialiser.b.put (bflit);
      serialiser.aw.drop;
      serialiser.w.drop;

      // TODO this assumes that the bus is 64-bits and a capability is 128-bits
      Bit #(1) is_valid = zeroExtend (wflit.wuser) & rg_cap_valid;
      CapMem newCap = unpack ({is_valid, // only valid if both flits had wuser == 1
                               wflit.wdata,
                               truncate(rg_cap)});
      CapPipe newCapFat = cast (newCap);

      if (idx == DMA_S2MM_CURDESC_CAP) begin
         dma_int_reg.s2mm_curdesc_cap_write (newCapFat);
      end
      else if (idx == DMA_MM2S_CURDESC_CAP) begin
         dma_int_reg.mm2s_curdesc_cap_write (newCapFat);
      end


      if (rg_verbosity > 0) begin
         $display ("wrote capability into rg_cap");
         $display ("    new cap: ", fshow (newCapFat));
         $display ("    index: ", fshow (idx));
      end
      rg_state <= RUNNING;
   endrule

   // behaviour: only accept single-flit reads where the size is DMA_Reg_Word-sized
   //            and the address is 32bit aligned
   rule rl_handle_read ((rg_state == RUNNING
                         || rg_state == HALTED)
                        && serialiser.ar.canPeek);
      serialiser.ar.drop;
      let arflit = serialiser.ar.peek;
      let is_aligned = arflit.araddr[1:0] == 0;
      let is_single_flit = arflit.arlen == 0;
      let is_correct_burst_type = arflit.arburst == FIXED || arflit.arburst == INCR; // WRAP requires a length >= 2 flits
      let is_correct_size = arflit.arsize == 4; // our words are always 4 bytes
      // don't care about lock, cache, prot, qos or region
      let is_valid_request = is_aligned
                             && is_single_flit
                             && is_correct_burst_type
                             && is_correct_size;
      let word_lower = arflit.araddr[2] == 1'b0;

      // We support only accesses that are properly aligned and 32bit wide
      DMA_Reg_Index idx = unpack (truncate (arflit.araddr >> 2));
      DMA_Reg_Word reg_val = dma_int_reg.external_read (idx);

      // TODO for now we always return OKAY in the response field
      //      we should return a SLVERR when the request is not valid
      AXI4_RFlit #(id_, data_, ruser_) rflit = AXI4_RFlit { rid: arflit.arid
                                                          , rdata: word_lower ? zeroExtend (reg_val) : {reg_val, 0}
                                                          //, rresp: is_valid_request ? OKAY : SLVERR
                                                          , rresp: OKAY
                                                          , rlast: True
                                                          , ruser: 0 };
      if (is_valid_request) begin
         if (rg_verbosity > 0) begin
            $display ("dma slave read request succeeded for register ", fshow (idx),
                      ", returning value ", fshow (reg_val));
            $display ("    read address: ", fshow (arflit.araddr));
         end
      end else begin
         $display ("dma slave read request failed for register ", fshow (idx));
         $display ("    is_aligned: ", fshow (is_aligned));
         $display ("    is_single_flit: ", fshow (is_aligned));
         $display ("    is_correct_burst_type: ", fshow (is_correct_burst_type));
         $display ("    is_correct_size: ", fshow (is_correct_size));
         if (!is_correct_size) begin
            $display ("        flit size: ", fshow (arflit.arsize));
         end
      end

      serialiser.r.put (rflit);
   endrule


   // output this signal always, the receivers of these signals will deal with what
   // happens if they are active when they shouldn't be
   method trigger = rw_trigger.wget;

   interface axi4_slave = shim.slave;

   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
   endmethod

   method Action reset;
      rg_state <= RESET;
   endmethod

   method Bool get_halt_to_idle = pw_halt_to_idle;

   method Action halt_to_idle if (rg_state == HALTED);
      rg_state <= RUNNING;
      if (rg_verbosity > 1) begin
         $display ("reg module halt_to_idle");
         $display ("    old state: ", fshow (rg_state));
      end
   endmethod
endmodule

endpackage
