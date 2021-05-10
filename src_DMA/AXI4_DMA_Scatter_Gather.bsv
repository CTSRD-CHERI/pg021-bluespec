package AXI4_DMA_Scatter_Gather;

// Bluespec imports
import FIFOF :: *;
import Vector :: *;
import DReg :: *;

// AXI imports
import AXI4 :: *;
import SourceSink :: *;

// local imports
import AXI4_DMA_Types :: *;

interface AXI4_DMA_Scatter_Gather_IFC#(numeric type id_,
                                       numeric type addr_,
                                       numeric type data_,
                                       numeric type awuser_,
                                       numeric type wuser_,
                                       numeric type buser_,
                                       numeric type aruser_,
                                       numeric type ruser_);
   method Action              req_reset;

   // The argument for this function is the address at which the first
   // word of the buffer descriptor can be found
   // Starts reading a Buffer Descriptor from memory, and writing it into
   // v_v_rg_bd
   method Action bd_read_from_mem (DMA_Dir dir, Bit #(addr_) start_address);

   // Fetch the next buffer descriptor from memory (using DMA_NXTDESC)
   method Action bd_fetch_next (DMA_Dir dir);

   // The argument first for this function is the address at which the first
   // word of the buffer descriptor can be found
   // The second argument is the new contents of the buffer descriptor
   // This will write the Buffer Descriptor that is in v_v_rg_bd back to memory
   method Action bd_write_to_mem (DMA_Dir dir, Bit #(addr_) start_address);

   // Becomes True for one cycle when the Scatter Gather Unit finishes
   // reading a MM2S Buffer Descriptor
   (* always_ready *)
   method Bool trigger_callback;

   // Becomes valid with a direction for one cycle when the Scatter Gather
   // Unit finishes writing back a Buffer Descriptor in either direction
   (* always_ready *)
   method Maybe #(DMA_Dir) trigger_interrupt;

   // The master used for reading and writing Buffer Descriptors
   interface AXI4_Master #(id_, addr_, data_,
                           awuser_, wuser_, buser_,
                           aruser_, ruser_) axi4_master;

   method Action set_verbosity (Bit #(4) new_verb);

   method Action reset;

   // Used to transition the Scatter Gather Unit from the Halted state
   // to the Idle state
   method Action halt_to_idle;
endinterface

typedef enum {
   DMA_RESET,
   DMA_HALTED,
   DMA_IDLE,
   DMA_MAIN_READ_RSP_OUTSTANDING,
   DMA_APP_READ_RSP_OUTSTANDING,
   DMA_MAIN_WRITE_START,
   DMA_APP_WRITE_START,
   DMA_MAIN_WRITE_LOOP,
   DMA_APP_WRITE_LOOP,
   DMA_MAIN_WRITE_RSP_OUTSTANDING,
   DMA_APP_WRITE_RSP_OUTSTANDING
} DMA_SG_State deriving (Eq, Bits, FShow);


// TODO revisit these provisos
// The internal registers are defined to be 32 bits, so to simplify
//  the DMA engine the fabric should support at least 32 bits of data
// The addresses, however, don't have as many restrictions in terms of
//  simplification. It should be possible to instantiate this such that
//  the address field is either smaller than or greater than the internal
//  address registers. However, I'm not sure how to write the code for this,
//  so here it is assumed that the addresses are >= the DMA word size
module mkAXI4_DMA_Scatter_Gather
         #(Vector #(n_, Vector #(m_, Reg #(DMA_BD_Word))) v_v_rg_bd)
          (AXI4_DMA_Scatter_Gather_IFC #(id_, addr_, data_,
                                         awuser_, wuser_, buser_,
                                         aruser_, ruser_))
          provisos (Add #(a__, SizeOf #(DMA_BD_Word), addr_),
                    Add #(b__, SizeOf #(DMA_BD_Word), data_),
                    Add #(c__, 4, TDiv #(data_, 8))); // TODO this is required because of setting
                                                     // the wstrb to zeroExtend (4'b1111).
                                                     // not sure why this is necessary, given that
                                                     // we already have the proviso that data_ is
                                                     // greater than 32, and 32/8 = 4

   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);

   let read_app_words = True;

   Reg #(DMA_SG_State) rg_state <- mkReg(DMA_HALTED);

   // The start address of the current buffer descriptor
   Reg #(Bit #(addr_)) rg_addr_bd_start <- mkRegU;

   // The address of the current buffer descriptor word (used when
   // reading and writing memory)
   Reg #(Bit #(addr_)) rg_addr_bd_cur <- mkRegU;

   // The BD index that is currently being worked on
   Reg #(DMA_BD_Index) rg_bd_index <- mkRegU;

   // The direction of the BD currently being handled
   Reg #(DMA_Dir) crg_dir[2] <- mkCReg (2, MM2S);
   let cur_dir = pack (crg_dir[1]);

   // This holds (the number of requests) - 1
   Reg #(AXI4_Len) rg_req_len <- mkRegU;

   // for reads, counts the number of responses received
   // for reads, this should be in sync with rg_bd_index
   // for writes, counts the number of write flits that have been sent
   Reg #(AXI4_Len) rg_rsp_count <- mkReg (0);

   // Whether the current transaction gave us an error response
   Reg #(Bool) rg_received_err_rsp <- mkReg (False);

   Reg #(Bool) drg_sg_finished <- mkDReg (False);

   RWire #(DMA_Dir) rw_trigger_interrupt <- mkRWire;

   rule rl_debug_finished (drg_sg_finished);
      if (rg_verbosity > 0) begin
         $display ("drg_sg_finished is True");
         $display ("    direction: ", fshow (cur_dir));
      end
   endrule

   AXI4_Shim #(id_, addr_, data_,
               awuser_, wuser_, buser_,
               aruser_, ruser_) shim <- mkAXI4ShimFF;

   Bit #(id_) base_id = 0;

   // In order to work with the current tag controller, DMA Scatter Gather
   // memory operations are limited to 8 flits.
   // At 32bits, two bursts are used. One burst contains the DMA control words,
   // and the other contains the application words.
   // app_words controls whether we read application words or DMA control words.
   function Tuple2 #(AXI4_ARFlit #(id_, addr_, aruser_), AXI4_Len) axi4_ar_burst_flit (Bit #(addr_) address, Bool app_words);
      // TODO remove hardwired 5?
      AXI4_Len len = app_words ? 5     // read only the app words
                                 - 1   // burst size is arlen + 1
                               : fromInteger (valueOf (DMA_Num_Words))
                                 - 5   // don't read app words
                                 - 1;  // burst size is arlen + 1


      AXI4_ARFlit #(id_, addr_, aruser) ar_flit = AXI4_ARFlit {
         arid : base_id,
         araddr : address,
         arlen : len,

         // TODO read 8 bytes instead, when the data width permits it?
         arsize : 4, // the size is 4 bytes, this gets transformed to the
                      // actual value of 3'b010 within the type

         // The rest of these are just the same as the Flute core's default values
         arburst  : INCR,
         arlock   : NORMAL,
         arcache  : arcache_dev_nonbuf,
         arprot   : axi4Prot(DATA, SECURE, UNPRIV),
         arqos    : 0,
         arregion : 0,
         aruser   : 0
      };
      return tuple2(ar_flit, len);
   endfunction



   rule rl_reset (rg_state == DMA_RESET);
      if (rg_verbosity > 0) begin
         $display ("AXI4 DMA SG Reset");
      end
      crg_dir[0] <= MM2S;
      rg_rsp_count <= 0;
      rg_received_err_rsp <= False;
      shim.clear;

      rg_state <= DMA_HALTED;
   endrule

   rule rl_debug;
      if (rg_verbosity > 2) begin
         $display("sg state: ", fshow(rg_state));
      end
   endrule


   // handle read responses and write them into the appropriate register
   rule rl_handle_read_rsp ((rg_state == DMA_MAIN_READ_RSP_OUTSTANDING
                             || (rg_state == DMA_APP_READ_RSP_OUTSTANDING
                                 && read_app_words))
                            && shim.slave.r.canPeek);
      AXI4_RFlit #(id_, data_, ruser_) rflit = shim.slave.r.peek;
      shim.slave.r.drop;
      rg_addr_bd_cur <= rg_addr_bd_cur + 4;
      let rsp_count_new = rg_rsp_count;

      if (rflit.rresp == OKAY || rflit.rresp == EXOKAY) begin
         // this handles the 32bit and 64bit cases
         // in 32bit, we don't truncate anything because word size is 32 bits,
         // in 64bit we truncate the upper or lower bits depending on the index
         // Buffer descriptors must be 16 word aligned (pg021, page 20), so the bottom
         // bits of the index also give us the address alignment
         // TODO handle differently-sized cases?
         DMA_BD_Word rdata_active = (pack (rg_bd_index))[0] == 1'b0 ? truncate (rflit.rdata)
                                                                    : truncateLSB (rflit.rdata);

         v_v_rg_bd[cur_dir][pack (rg_bd_index)] <= rdata_active;
         rg_bd_index <= unpack (pack (rg_bd_index) + 1);
         rsp_count_new = rg_rsp_count + 1;
         if (rg_verbosity > 0) begin
            $display ("updated register ", fshow(rg_bd_index), " with data : ", fshow(rdata_active));
            $display ("    original data: ", fshow (rflit.rdata));
            $display ("    cur_dir: ", fshow (cur_dir));
         end
         // user field should be empty for now
         // TODO handle user field when capabilities are required
      end

      if (rflit.rresp == EXOKAY) begin
         // this shouldn't happen since we don't make exclusive requests
         $display ("got an unexpected EXOKAY from memory");
      end

      if (rflit.rresp == SLVERR || rflit.rresp == DECERR) begin
         // something bad happened
         // We should set some fields somewhere, but for now we just do nothing
         // TODO handle DMA scatter-gather read errors
         $display ("AXI4 SG Unit: ERROR IN DMA: ", fshow(rflit.rresp));
      end

      if (rflit.rlast) begin
         if (read_app_words && rg_bd_index == DMA_STATUS) begin
            // DMA_STATUS is the last non-app word
            // we've finished reading all non-app words and now we need
            // to read the app words

            if (rg_state != DMA_MAIN_READ_RSP_OUTSTANDING) begin
               $display ("DMA SG ERROR: something's gone wrong. we've read past the\n",
                         "    end of the main words but still thought we were reading\n",
                         "    main words");
            end

            match {.arflit, .len} = axi4_ar_burst_flit (rg_addr_bd_cur + 4, True);
            shim.slave.ar.put (arflit);
            if (rg_verbosity > 0) begin
               $display ("AXI4 SG Unit: DMA sent AR flit to read app words");
               $display ("    flit: ", fshow(arflit));
            end
            rg_req_len <= len;
            rsp_count_new = 0;
            rg_state <= DMA_APP_READ_RSP_OUTSTANDING;
         end else begin
            if (pack (rg_rsp_count) != truncate (rg_req_len)) begin
               // not sure what to do here...
               // We got fewer responses from memory than we should have
               $display ("AXI4 SG Unit: ERROR: memory didn't give us the right number of responses");
               $display ("    rg_bd_index: ", fshow(pack(rg_bd_index)),
                         " rg_req_len: ", fshow(rg_req_len));
            end else begin
               rg_state <= DMA_IDLE;
               if (rg_verbosity > 0) begin
                  $display ("got enough responses from memory");
               end
               if (cur_dir == pack (MM2S)) begin
                  if (rg_verbosity > 0) begin
                     $display ("AXI4 SG Unit: triggering mm2s transfer after finishing SG read");
                  end
                  drg_sg_finished <= True;
               end
            end

            // check that we wrote the last index when rlast is true
            // This checks that our requests were the right size
            if ((read_app_words && rg_bd_index != maxBound)
                || (!read_app_words && rg_bd_index != DMA_STATUS)) begin
               DMA_BD_Index expected_max = read_app_words ? maxBound : DMA_STATUS;
               $display ("AXI4 DMA SG: index is not the expected value at end of transfer");
               $display ("    index: ", fshow (rg_bd_index));
               $display ("    expected: ", fshow (expected_max));
            end
         end
      end

      rg_rsp_count <= rsp_count_new;

      if (rflit.rid != base_id) begin
         $display ("AXI4 SG Unit: ERROR: something went wrong - DMA got a response with an ID that it didn't request");
      end

   endrule

   rule rl_write (rg_state == DMA_MAIN_WRITE_START
                  || rg_state == DMA_APP_WRITE_START);
      // TODO at the moment some transactions are wasted. Maybe only write the words
      // that have changed? This would require more internal state, not sure how it would
      // affect bus contention
      // NOTE burst length = awlen + 1
      let write_app = rg_state == DMA_APP_WRITE_START;
      // TODO remove hardwired 5?
      AXI4_Len len = write_app ? 5    // write only the application words
                                 - 1  // burst size is arlen + 1
                               : fromInteger (valueOf (DMA_Num_Words))
                                 - 5  // don't write application words
                                 - 1; // burst size is arlen + 1

      // TODO write more data for higher bus widths
      AXI4_AWFlit #(id_, addr_, awuser_) awflit = AXI4_AWFlit {
         awid     : base_id,
         awaddr   : zeroExtend (rg_addr_bd_cur),
         awlen    : len, // NOTE burst length = awlen + 1
         awsize   : 4, // the size is 4 bytes, this gets transformed to the
                       // actual value of 3'b010 within the type
         awburst  : INCR,
         awlock   : NORMAL,
         awcache  : awcache_dev_nonbuf,
         awprot   : axi4Prot (DATA, SECURE, UNPRIV),
         awqos    : 0,
         awregion : 0,
         awuser   : 0
      };
      shim.slave.aw.put(awflit);

      // this handles the 32bit and 64bit cases
      // TODO handle other cases?
      Bool write_lower = (pack (rg_bd_index))[0] == 1'b0;
      //Bit #(data_) val_to_write = write_lower ? zeroExtend (v_v_rg_bd[cur_dir][pack (rg_bd_index)])
      //                                        : reverseBits (zeroExtend (reverseBits (v_v_rg_bd[cur_dir][pack (rg_bd_index)])));
      Bit #(data_) val_to_write = write_lower ? {0, v_v_rg_bd[cur_dir][pack (rg_bd_index)]}
                                              : {v_v_rg_bd[cur_dir][pack (rg_bd_index)], 0};
      Bit #(TDiv #(data_, 8)) strb = write_lower ? {0, 4'b1111}
                                                 // TODO this doesn't work for
                                                 // strb requiring >8 bits
                                                 : {4'b1111, 0};
      // TODO write more data for higher bus widths
      AXI4_WFlit #(data_, wuser_) wflit = AXI4_WFlit {
         wdata : val_to_write,
         wstrb : strb,
         wlast : False,
         wuser : 0
      };
      shim.slave.w.put(wflit);
      rg_bd_index <= unpack (pack (rg_bd_index) + 1);
      rg_addr_bd_cur <= rg_addr_bd_cur + 4;
      rg_req_len <= zeroExtend (len);
      rg_rsp_count <= 1; // we send a wflit in this rule
      rg_received_err_rsp <= False;

      if (rg_verbosity > 1) begin
         $display ("requested burst write starting at ", fshow (rg_addr_bd_cur),
                   " with a length of ", fshow (len));
         $display ("   first data: ", fshow (val_to_write));
         $display ("   request flit: ", fshow(awflit));
         $display ("   data flit: ", fshow(wflit));
         $display ("   write_app: ", fshow (write_app));
      end
      rg_state <= write_app ? DMA_APP_WRITE_LOOP
                            : DMA_MAIN_WRITE_LOOP;
   endrule

   rule rl_write_loop (rg_state == DMA_MAIN_WRITE_LOOP
                       || rg_state == DMA_APP_WRITE_LOOP);
      Bool islast = rg_rsp_count == rg_req_len;
      // this handles the 32bit and 64bit cases
      // TODO handle other cases?
      // TODO handle writing more data when the data bus supports it?
      Bool write_lower = (pack (rg_bd_index))[0] == 1'b0;
      Bit #(data_) val_to_write = write_lower ? {0, v_v_rg_bd[cur_dir][pack (rg_bd_index)]}
                                              : {v_v_rg_bd[cur_dir][pack (rg_bd_index)], 0};
      Bit #(TDiv #(data_, 8)) strb = write_lower ? {0, 4'b1111}
                                                 // TODO this doesn't work for
                                                 // strb requiring >8 bits
                                                 : {4'b1111, 0};
      AXI4_WFlit #(data_, wuser_) wflit = AXI4_WFlit {
         wdata : val_to_write,
         wstrb : strb,
         wlast : islast,
         wuser : 0
      };
      shim.slave.w.put(wflit);
      if (rg_verbosity > 1) begin
         $display ("AXI4 SG Unit: rl_write_loop:");
         $display ("   data flit: ", fshow(wflit));
      end

      if (islast) begin
         if (rg_verbosity > 1) begin
            $display ("finished requesting write to last descriptor word");
            $display ("last index written: ", fshow(rg_bd_index));
         end
         let write_app = rg_state == DMA_APP_WRITE_LOOP;
         rg_state <= write_app ? DMA_APP_WRITE_RSP_OUTSTANDING
                               : DMA_MAIN_WRITE_RSP_OUTSTANDING;
      end

      rg_bd_index <= unpack (pack (rg_bd_index) + 1);
      rg_addr_bd_cur <= rg_addr_bd_cur + 4;
      rg_rsp_count <= rg_rsp_count + 1;
   endrule

   rule rl_handle_write_rsp_debug (rg_state == DMA_MAIN_WRITE_RSP_OUTSTANDING
                                   || rg_state == DMA_APP_WRITE_RSP_OUTSTANDING);
      if (rg_verbosity > 1) begin
         $display ("write response outstanding");
      end
   endrule

   rule rl_handle_b_canpeek (shim.slave.b.canPeek);
      if (rg_verbosity > 1) begin
         $display ("write response received");
         $display ("state: ", fshow (rg_state));
         $display ("response: ", fshow (shim.slave.b.peek));
      end
   endrule

   // handle receipt of outstanding write response
   rule rl_handle_write_rsp ((rg_state == DMA_MAIN_WRITE_RSP_OUTSTANDING
                              || rg_state == DMA_APP_WRITE_RSP_OUTSTANDING)
                             && shim.slave.b.canPeek);
      AXI4_BFlit #(id_, buser_) bflit = shim.slave.b.peek;
      shim.slave.b.drop;

      let iserr = False;

      if (bflit.bid != base_id) begin
         $display ("AXI4 SG Unit: ERROR: something went wrong - got write response with id that wasn't requested");
      end

      if (bflit.bresp != OKAY) begin
         // it's only an error if we got a SLVERR or DECERR
         iserr = bflit.bresp != EXOKAY;
         case (bflit.bresp)
            EXOKAY: begin
               $display ("AXI4 SG Unit: something went wrong in the DMA write response");
               $display ("   got an exclusive OK for a write request that wasn't exclusive");
            end

            SLVERR: begin
               $display ("AXI4 SG Unit: ERROR: something went wrong in the DMA write response");
               $display ("   got a slave error");
            end

            DECERR: begin
               $display ("AXI4 SG Unit: ERROR: something went wrong in the DMA write response");
               $display ("   got a decoding error");
            end

            default: begin
               // this should never happen - the only other case is excluded above by the
               // if (bflit != OKAY) check
               $display ("AXI4 SG Unit: ERROR: something is very broken in the DMA, got to a\n",
                         "state that shouldn't be possible");
            end
         endcase
      end

      if (rg_verbosity > 1) begin
         $display ("received write response");
      end
      // TODO if we receive an error response, need to deal with it properly
      let write_app = rg_state == DMA_APP_WRITE_RSP_OUTSTANDING;
      rg_state <= write_app ? iserr ? DMA_IDLE
                                    : DMA_IDLE
                            : DMA_APP_WRITE_START;
      rw_trigger_interrupt.wset (crg_dir[0]);
      if (iserr) begin
         $display ("AXI4 SG Unit: ERROR: DMA RECEIVED WRITE ERROR RESPONSE");
      end
   endrule


   // detect issues
   rule rl_detect_rresp_in_wrong_state (shim.slave.r.canPeek
                                        && rg_state != DMA_MAIN_READ_RSP_OUTSTANDING
                                        && rg_state != DMA_APP_READ_RSP_OUTSTANDING);
      $display ("AXI4 SG Unit: ERROR: DMA detected read response when DMA is in the wrong state");
   endrule

   rule rl_detect_wresp_in_wrong_state (shim.slave.b.canPeek
                                        && rg_state != DMA_MAIN_WRITE_RSP_OUTSTANDING
                                        && rg_state != DMA_APP_WRITE_RSP_OUTSTANDING);
      $display ("AXI4 SG Unit: ERROR: DMA detected write response when DMA is in the wrong state");
   endrule


   /////////////////////////
   //
   // Interface
   //
   /////////////////////////


   method Action req_reset;
      rg_addr_bd_start <= ?;
      rg_addr_bd_cur <= ?;
      rg_state <= DMA_IDLE;
      rg_bd_index <= ?;
      rg_rsp_count <= 0;
   endmethod

   /*
    * Request that a BD be read from memory and written into the registers
    */
   method Action bd_read_from_mem (DMA_Dir dir, Bit #(addr_) start_addr) if (shim.slave.ar.canPut
                                                                             && rg_state == DMA_IDLE);

      // update direction
      crg_dir[0] <= dir;

      // bus transaction
      // don't read app words yet
      match {.arflit, .len} = axi4_ar_burst_flit (start_addr, False);
      shim.slave.ar.put (arflit);
      if (rg_verbosity > 0) begin
         $display ("AXI4 SG Unit: DMA sent AR flit: ", fshow(arflit));
      end

      // internal bookkeeping
      rg_addr_bd_start <= start_addr;
      rg_addr_bd_cur <= start_addr;
      rg_bd_index <= minBound;
      rg_req_len <= len;
      rg_rsp_count <= 0;
      rg_received_err_rsp <= False;

      rg_state <= DMA_MAIN_READ_RSP_OUTSTANDING;
   endmethod

   /*
    * Request that the BD currently held in the local registers be written out to
    * main memory.
    */
   method Action bd_write_to_mem (DMA_Dir dir, Bit #(addr_) start_addr) if (shim.slave.ar.canPut
                                                                            && rg_state == DMA_IDLE);
      // update direction
      crg_dir[0] <= dir;

      if (rg_verbosity > 0) begin
         $display ("AXI4 SG Unit: requested write at ", fshow(start_addr));
         $display ("              write value: ", fshow(readVReg (v_v_rg_bd[pack (dir)])));
      end

      rg_addr_bd_start <= start_addr;
      rg_addr_bd_cur <= start_addr;
      rg_state <= DMA_MAIN_WRITE_START;
      rg_bd_index <= minBound;
      rg_rsp_count <= 0;
   endmethod

   // TODO update this to handle 64-bit
   // (ie use NXTDESC_MSB)
   // This is currently unused and untested
   method Action bd_fetch_next (DMA_Dir dir) if (shim.slave.ar.canPut
                                                 && rg_state == DMA_IDLE);
      // update direction
      crg_dir[0] <= dir;

      if (rg_verbosity > 0) begin
         $display ("AXI4 SG Unit: requested fetch of next buffer descriptor");
      end

      Bit #(addr_) addr = zeroExtend (v_v_rg_bd[pack (dir)][pack (DMA_NXTDESC)]);

      // bus transaction
      // don't read app words yet
      match {.arflit, .len} = axi4_ar_burst_flit (addr, False);
      shim.slave.ar.put (arflit);

      // internal bookkeeping
      rg_addr_bd_start <= addr;
      rg_addr_bd_cur <= addr;
      rg_bd_index <= minBound;
      rg_req_len <= len;
      rg_rsp_count <= 0;
      rg_received_err_rsp <= False;

      rg_state <= DMA_MAIN_READ_RSP_OUTSTANDING;
   endmethod

   /*
    * Signal that we have finished doing an MM2S read, and the BD in
    * the registers is correct
    */
   method Bool trigger_callback;
      return drg_sg_finished;
   endmethod

   /*
    * Signal that we have finished writing back a BD to memory
    */
   method Maybe #(DMA_Dir) trigger_interrupt;
      return rw_trigger_interrupt.wget;
   endmethod

   interface axi4_master = shim.master;

   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
   endmethod

   method Action reset;
      rg_state <= DMA_RESET;
   endmethod

   method Action halt_to_idle;
      if (rg_state != DMA_HALTED) begin
         $display ("AXI DMA Scatter Gather Unit: ERROR: halted_to_idle when not halted1");
      end
      rg_state <= DMA_IDLE;
   endmethod

endmodule


endpackage
