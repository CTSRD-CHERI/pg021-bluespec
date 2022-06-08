package AXI4_DMA_Scatter_Gather;

// Bluespec imports
import FIFOF :: *;
import Vector :: *;
import DReg :: *;
import GetPut :: *;
import ClientServer :: *;

// AXI imports
import AXI4 :: *;
import SourceSink :: *;
import Connectable :: *;

// local imports
import AXI4_DMA_Types :: *;

`ifdef DMA_CHERI
import CHERICap :: *;
import CHERICC_Fat :: *;
`endif

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

   // Becomes Valid for one cycle when the Scatter Gather Unit finishes
   // reading a Buffer Descriptor
   (* always_ready *)
   method Maybe #(DMA_Dir) trigger_callback;

   // Becomes valid with a direction for one cycle when the Scatter Gather
   // Unit finishes writing back a Buffer Descriptor in either direction
   (* always_ready *)
   method Maybe #(DMA_Dir) trigger_interrupt;

   // The direction in which the SG unit is currently operationg
   (* always_ready *)
   method DMA_Dir current_dir;

   // The master used for reading and writing Buffer Descriptors
   interface AXI4_Master #(id_, addr_, data_,
                           awuser_, wuser_, TAdd #(Checker_Resp_U_Bits, buser_),
                           aruser_, TAdd #(Checker_Resp_U_Bits, ruser_)) axi4_master;

   method Action set_verbosity (Bit #(4) new_verb);

   method Action reset;

   // Used to transition the Scatter Gather Unit from the Halted state
   // to the Idle state
   method Action halt_to_idle;

   // Transitions the Scatter Gather unit to halted mode when something goes wrong
   interface Server #(Bit #(0), Bit #(0)) srv_halt;

   // Something went wrong in the scatter gather unit; signal appropriately
   (* always_ready *)
   method Maybe #(DMA_Err_Cause) enq_halt_o;
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
/*
 * Some notes on reading/writing capabilities: In order to read/write
   capabilities, the tag controller requires 64-bit accesses. This means
   that any capabilities that we put in the buffer descriptors must be read
   and written using 64-bit flits, which might cause some issues here
 */
module mkAXI4_DMA_Scatter_Gather
         #(Vector #(n_, Vector #(m_, Reg #(DMA_BD_TagWord))) v_v_rg_bd)
          (AXI4_DMA_Scatter_Gather_IFC #(id_, addr_, data_,
                                         awuser_, wuser_, buser_,
                                         aruser_, ruser_))
          provisos (Add #(a__, SizeOf #(DMA_BD_Word), addr_),
                    Add #(b__, SizeOf #(DMA_BD_Word), data_),
                    Add #(c__, 4, TDiv #(data_, 8)),  // TODO this is required because of setting
                                                     // the wstrb to zeroExtend (4'b1111).
                                                     // not sure why this is necessary, given that
                                                     // we already have the proviso that data_ is
                                                     // greater than 32, and 32/8 = 4
                    Mul #(d__, 32, data_),
                    Add #(e__, TLog#(TDiv#(data_, 32)), 5),
                    Add #(f__, 1, ruser_));

   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);

   let read_app_words = True;

   Reg #(DMA_SG_State) rg_state <- mkReg(DMA_HALTED);

   FIFOF #(Bit #(0)) ugfifo_halt <- mkUGFIFOF1;

   // The start address of the current buffer descriptor
   Reg #(Bit #(addr_)) rg_addr_bd_start <- mkRegU;

   // The address of the current buffer descriptor word (used when
   // reading and writing memory)
   Reg #(Bit #(addr_)) rg_addr_bd_cur <- mkRegU;
   let rg_addr_bd_incr = rg_addr_bd_cur + 4;

   // The BD index that is currently being worked on
   Reg #(DMA_BD_Index) rg_bd_index <- mkRegU;

   // The direction of the BD currently being handled
   Reg #(DMA_Dir) crg_dir[2] <- mkCReg (2, MM2S);
   let cur_dir = pack (crg_dir[1]);

   // This holds (the number of requests) - 1
   Reg #(AXI4_Len) rg_req_len <- mkRegU;
   Reg #(AXI4_Size) rg_req_size <- mkRegU;

   // for reads, counts the number of responses received
   // for reads, this should be in sync with rg_bd_index
   // for writes, counts the number of write flits that have been sent
   Reg #(AXI4_Len) rg_rsp_count <- mkReg (0);

   // Whether the current transaction gave us an error response
   Reg #(Bool) rg_received_err_rsp <- mkReg (False);

   Reg #(Maybe #(DMA_Dir)) drg_sg_finished <- mkDReg (Invalid);

   RWire #(DMA_Dir) rw_trigger_interrupt <- mkRWire;
   RWire #(DMA_Err_Cause) rw_enq_halt_o <- mkRWire;

   rule rl_debug_finished (isValid (drg_sg_finished));
      if (rg_verbosity > 0) begin
         $display ("drg_sg_finished is Valid");
         $display ("    direction: ", fshow (fromMaybe (?, drg_sg_finished)));
      end
   endrule

   let shim <- mkAXI4ShimFF;
   let ugshim_slave <- toUnguarded_AXI4_Slave (shim.slave);

   Bit #(id_) base_id = 0;

   // In order to work with the current tag controller, DMA Scatter Gather
   // memory operations are limited to 8 flits.
   // At 32bits, two bursts are used. One burst contains the DMA control words,
   // and the other contains the application words.
   // app_words controls whether we read application words or DMA control words.
   // Cases:
   //    ptr_words == True, app_words == True
   //       For 64-bit, fetch the entire buffer descriptor in 1 burst, because
   //       we have enough bandwidth for it
   //       For 32-bit, fetch only the pointer words (ie treat as though
   //       app_words is false
   //    ptr_words == True, app_words == False
   //       Fetch only the pointer words
   //       For 32-bit, this will fetch 16 bytes
   //       For 64-bit, this will fetch 32 bytes
   //    ptr_words == False, app_words == True
   //       Fetch both the application words (DMA_APP0 .. DMA_APP4) _and_ the
   //       control words (DMA_BD_CONTROL and DMA_BD_STATUS)
   //    ptr_words == False, app_words == False
   //       Fetch only the control words (DMA_BD_CONTROL and DMA_BD_STATUS)
   function Tuple2 #(AXI4_ARFlit #(id_, addr_, aruser_), AXI4_Len) axi4_ar_burst_flit (Bit #(addr_) address, Bool ptr_words, Bool app_words);
      Bit #(5) num_words_to_read = ?;
      if (ptr_words && app_words) begin
         if (valueOf (data_) >= 64) begin
            // if we have a bus that is at least 64 bits, then we can fetch the
            // entire buffer descriptor in one max-length (8-flit) burst
            num_words_to_read = fromInteger (valueOf (DMA_Num_Total_Words));
         end else begin
            // if our bus is not at least 64 bits, then we have to issue
            // multiple bursts. The first one will read only the pointer
            // words
            num_words_to_read = fromInteger (valueOf (DMA_Num_Ptr_Words));
         end
      end else if (ptr_words && !app_words) begin
         num_words_to_read = fromInteger (valueOf (DMA_Num_Ptr_Words));
      end else if (!ptr_words && app_words) begin
`ifdef DMA_CHERI
         num_words_to_read = fromInteger (valueOf (DMA_Num_Control_App_Res_Words));
`else
         num_words_to_read = fromInteger (valueOf (DMA_Num_Control_App_Words));
`endif
      end else if (!ptr_words && !app_words) begin
         num_words_to_read = fromInteger (valueOf (DMA_Num_Control_Words));
      end


      let bus_size_words = valueOf (TDiv #(data_, SizeOf #(DMA_BD_Word)));
      let log_bus_size_words = valueOf (TLog #(TDiv #(data_, SizeOf #(DMA_BD_Word))));

      // TODO this artifically limits the supported bus size to 128-bit
      //      because of the truncate and because we impose a limit on
      //      the number of words that can be read
      Bit #(TLog #(TDiv #(data_, SizeOf #(DMA_BD_Word)))) num_words_bottom
         = truncate (num_words_to_read);

      let flit_num = ?;
      let flit_size = ?;
      if (log_bus_size_words != 0 // this is known at compile time, so if it is
                                  // false then this if statement should generate
                                  // no extra logic
          && num_words_bottom == 0) begin
         // the number of words we want evenly divides by the number of words
         // we can get per flit, so we can send a bus-wide request
         flit_num = num_words_to_read >> log_bus_size_words;
         flit_size = fromInteger (valueOf (TDiv #(data_, 8)));
      end else begin
         // it is not possible to read the exact number of words we want using
         // bus-wide flits, so default to 32-bit reads
         flit_num = num_words_to_read;
         flit_size = fromInteger (4);
      end

      AXI4_Len flit_len = unpack (zeroExtend (flit_num - 1));

      AXI4_ARFlit #(id_, addr_, aruser) ar_flit = AXI4_ARFlit {
         arid : base_id,
         araddr : address,
         arlen : flit_len,
         arsize : flit_size,

         // The rest of these are just the same as the Flute core's default values
         arburst  : INCR,
         arlock   : NORMAL,
         arcache  : arcache_dev_nonbuf,
         arprot   : axi4Prot(DATA, SECURE, UNPRIV),
         arqos    : 0,
         arregion : 0,
         aruser   : 0
      };
      return tuple2(ar_flit, flit_len);
   endfunction


   function Vector #(TDiv #(data_, 32), Bool) fn_size_to_active_reg (AXI4_Size size);
      Vector #(TDiv #(data_, 32), Bool) ret_vec = replicate(False);
      let size_bytes = fromAXI4_Size (size);
      let size_words = size_bytes >> 2;
      for (Integer i = 0; i < valueOf (TDiv #(data_, 32)); i = i+1) begin
         if (fromInteger (i) < size_words) begin
            ret_vec[i] = True;
         end
      end

      return ret_vec;
   endfunction


   rule rl_reset (rg_state == DMA_RESET
                  && !ugfifo_halt.notEmpty);
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
   // when the read request size was greater than 32bits, write all the
   // appropriate registers with the data from memory
   rule rl_handle_read_rsp ((rg_state == DMA_MAIN_READ_RSP_OUTSTANDING
                             || (rg_state == DMA_APP_READ_RSP_OUTSTANDING
                                 && read_app_words))
                            && ugshim_slave.r.canPeek
                            && ugshim_slave.ar.canPut
                            && !ugfifo_halt.notEmpty);
      if (rg_verbosity > 1) begin
         $display ("rl_handle_read_rsp");
      end
      let rflit = shim.slave.r.peek;
      ugshim_slave.r.drop;
      rg_addr_bd_cur <= rg_addr_bd_incr;
      let rsp_count_new = rg_rsp_count;
      DMA_BD_Index next_bd_index = unpack (pack (rg_bd_index)
                                           + truncate (fromAXI4_Size (rg_req_size) >> 2));
      DMA_BD_Index latest_bd_index = unpack (pack (rg_bd_index) - 1
                                             + truncate (fromAXI4_Size (rg_req_size) >> 2)
                                             );

      if (rflit.rresp == OKAY || rflit.rresp == EXOKAY) begin

         // TODO this section has been tested with 64-bit and works
         //      it has been written such that it should work with any bus size
         //      which is a multiple of 32, but has not been tested in non-64-bit
         //      busses
         Bit #(TLog #(data_)) base_offset
            = pack (rg_bd_index)[valueOf (TSub #(TLog #(TDiv #(data_, SizeOf #(DMA_BD_Word))), 1)):0] << 5;

         let active = fn_size_to_active_reg (rg_req_size);
         for (Integer i = 0; i < valueOf (TDiv #(data_, 32)); i = i+1) begin
            if (active[i]) begin
               Bit #(TLog #(data_)) offset = base_offset + fromInteger (i) * 32;
               DMA_BD_Word word_to_write = unpack (rflit.rdata[offset+31:offset]);
               // TODO multiple user bits?
`ifdef DMA_CHERI
               Bit #(ruser_) expected_ruser = signExtend (1'b1);
               Bool tag_to_write = truncate (rflit.ruser) == expected_ruser;
`endif

               DMA_BD_TagWord val_to_write = DMA_BD_TagWord { word: word_to_write
`ifdef DMA_CHERI
                                                            , tag: tag_to_write
`endif
                                                            };
               DMA_BD_Index reg_to_write = unpack (pack (rg_bd_index) + fromInteger (i));
               v_v_rg_bd[cur_dir][pack (reg_to_write)] <= val_to_write;
               if (rg_verbosity > 0) begin
                  $display ("AXI DMA SG Register Write");
                  $display ("    register: ", fshow (reg_to_write));
                  $display ("    value: ", fshow (val_to_write));
`ifdef DMA_CHERI
                  $display ("    expected_ruser: ", fshow (expected_ruser));
`endif
               end
            end
         end
         if (rg_verbosity > 1) begin
            $display ("    raw flit: ", fshow (rflit));
         end
         rsp_count_new = rg_rsp_count + 1;
         rg_bd_index <= next_bd_index;
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
         $display ("    flit: ", fshow (rflit));
`ifdef DMA_CHERI
         let cherierr = truncateLSB (rflit.ruser) == 1'b1;
         rw_enq_halt_o.wset ( cherierr              ? CHERIERR
                            : rflit.rresp == DECERR ? DECERR
                                                    : SLVERR);
`else
         rw_enq_halt_o.wset ( rflit.rresp == DECERR ? DECERR
                                                    : SLVERR);
`endif
      end

      if (rflit.rlast) begin
         // TODO change this to latest_bd_index and DMA_STATUS?
         if (read_app_words && next_bd_index == DMA_APP0) begin
            // we've finished reading all non-app words and now we need
            // to read the app words

            if (rg_state != DMA_MAIN_READ_RSP_OUTSTANDING) begin
               $display ("DMA SG ERROR: something's gone wrong. we've read past the\n",
                         "    end of the main words but still thought we were reading\n",
                         "    main words");
            end

            match {.arflit, .len} = axi4_ar_burst_flit (rg_addr_bd_incr, False, True);
            ugshim_slave.ar.put (arflit);
            if (rg_verbosity > 0) begin
               $display ("AXI4 SG Unit: DMA sent AR flit to read app words");
               $display ("    flit: ", fshow(arflit));
            end
            rg_req_len <= len;
            rg_req_size <= arflit.arsize;
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
               drg_sg_finished <= tagged Valid (crg_dir[1]);
               if (rg_verbosity > 0) begin
                  $display ("got enough responses from memory");
                  $display ("AXI4 SG Unit: notifying that SG is finished reading, direction: ", fshow (crg_dir[1]));
               end
            end

            // Maximum expected values for latest_bd_index
            // Explicitly set here to avoid type ambiguity
`ifdef DMA_CHERI
            DMA_BD_Index app_max = rg_req_size > toAXI4_Size (4).Valid ? DMA_RESERVED_4
                                                                       : DMA_APP4;
`else
            DMA_BD_Index app_max = DMA_APP4;
`endif
            DMA_BD_Index ctrl_max = DMA_STATUS;
            // check that we wrote the last index when rlast is true
            // This checks that our requests were the right size
            // TODO a more hardware-efficient way of doing this would probably
            // be to check that next_bd_index != (DMA_APP4 + 1)
            //              and next_bd_index != DMA_APP0
            if ((read_app_words && pack (latest_bd_index) != pack (app_max))
                || (!read_app_words && pack (latest_bd_index) != pack (ctrl_max))) begin
               DMA_BD_Index expected_max = read_app_words ? maxBound : app_max;
               $display ("AXI4 DMA SG: index is not the expected value at end of transfer");
               $display ("    index: ", fshow (latest_bd_index));
               $display ("    expected: ", fshow (expected_max));
            end
         end
      end

      rg_rsp_count <= rsp_count_new;

      if (rflit.rid != base_id) begin
         $display ("AXI4 SG Unit: ERROR: something went wrong - DMA got a response with an ID that it didn't request");
      end

   endrule

   rule rl_write ((rg_state == DMA_MAIN_WRITE_START
                   || rg_state == DMA_APP_WRITE_START)
                  && ugshim_slave.aw.canPut
                  && ugshim_slave.w.canPut
                  && !ugfifo_halt.notEmpty);
      // TODO at the moment some transactions are wasted. Maybe only write the words
      // that have changed? This would require more internal state, not sure how it would
      // affect bus contention
      // NOTE burst length = awlen + 1
      let write_app = rg_state == DMA_APP_WRITE_START;
      // TODO remove hardwired 5?
      AXI4_Len len = write_app ? 8    // write only the application words
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
      ugshim_slave.aw.put(awflit);
      $display ("aw put: ", fshow (awflit));

      // this handles the 32bit and 64bit cases
      // TODO handle other cases?
      Bool write_lower = (pack (rg_bd_index))[0] == 1'b0;
      //Bit #(data_) val_to_write = write_lower ? zeroExtend (v_v_rg_bd[cur_dir][pack (rg_bd_index)])
      //                                        : reverseBits (zeroExtend (reverseBits (v_v_rg_bd[cur_dir][pack (rg_bd_index)])));
      Bit #(data_) val_to_write = write_lower ? {0, v_v_rg_bd[cur_dir][pack (rg_bd_index)].word}
                                              : {v_v_rg_bd[cur_dir][pack (rg_bd_index)].word, 0};
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
      ugshim_slave.w.put(wflit);
      rg_bd_index <= unpack (pack (rg_bd_index) + 1);
      rg_addr_bd_cur <= rg_addr_bd_incr;
      rg_req_len <= zeroExtend (len);
      rg_req_size <= awflit.awsize;
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

   rule rl_write_loop ((rg_state == DMA_MAIN_WRITE_LOOP
                        || rg_state == DMA_APP_WRITE_LOOP)
                       && ugshim_slave.w.canPut
                       && !ugfifo_halt.notEmpty);
      Bool islast = rg_rsp_count == rg_req_len;
      // this handles the 32bit and 64bit cases
      // TODO handle other cases?
      // TODO handle writing more data when the data bus supports it?
      Bool write_lower = (pack (rg_bd_index))[0] == 1'b0;
      Bit #(data_) val_to_write = write_lower ? {0, v_v_rg_bd[cur_dir][pack (rg_bd_index)].word}
                                              : {v_v_rg_bd[cur_dir][pack (rg_bd_index)].word, 0};
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
      ugshim_slave.w.put(wflit);
      if (rg_verbosity > 1) begin
         $display ("AXI4 SG Unit: rl_write_loop:");
         $display ("   data flit: ", fshow(wflit));
         $display ("   index: ", fshow (rg_bd_index));
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
      rg_addr_bd_cur <= rg_addr_bd_incr;
      rg_rsp_count <= rg_rsp_count + 1;
   endrule

   rule rl_handle_write_rsp_debug (rg_state == DMA_MAIN_WRITE_RSP_OUTSTANDING
                                   || rg_state == DMA_APP_WRITE_RSP_OUTSTANDING);
      if (rg_verbosity > 1) begin
         $display ("write response outstanding");
      end
   endrule

   rule rl_handle_b_canpeek (ugshim_slave.b.canPeek);
      if (rg_verbosity > 1) begin
         $display ("write response received");
         $display ("state: ", fshow (rg_state));
         $display ("response: ", fshow (ugshim_slave.b.peek));
      end
   endrule

   // handle receipt of outstanding write response
   rule rl_handle_write_rsp ((rg_state == DMA_MAIN_WRITE_RSP_OUTSTANDING
                              || rg_state == DMA_APP_WRITE_RSP_OUTSTANDING)
                             && ugshim_slave.b.canPeek
                             && !ugfifo_halt.notEmpty);
      let bflit = shim.slave.b.peek;
      ugshim_slave.b.drop;

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
`ifdef DMA_CHERI
               let cherierr = truncateLSB (bflit.buser) == 1'b1;
               rw_enq_halt_o.wset (cherierr ? CHERIERR
                                            : SLVERR);
`else
               rw_enq_halt_o.wset (SLVERR);
`endif
            end

            DECERR: begin
               $display ("AXI4 SG Unit: ERROR: something went wrong in the DMA write response");
               $display ("   got a decoding error");
`ifdef DMA_CHERI
               let cherierr = truncateLSB (bflit.buser) == 1'b1;
               rw_enq_halt_o.wset (cherierr ? CHERIERR
                                            : DECERR);
`else
               rw_enq_halt_o.wset (DECERR);
`endif
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

   rule rl_handle_halt (ugfifo_halt.notEmpty);
      if (rg_verbosity > 0) begin
         $display ("DMA SG Unit rl_handle_halt");
      end
      case (rg_state)
         DMA_RESET, DMA_IDLE, DMA_HALTED, DMA_MAIN_WRITE_START, DMA_APP_WRITE_START: begin
            // no state needs to be handled; just go to halted
            ugfifo_halt.deq;
            rg_state <= DMA_HALTED;
            if (rg_verbosity > 1) begin
               $display ("    No state needs handled, going straight to halted");
            end
         end
         DMA_MAIN_READ_RSP_OUTSTANDING, DMA_APP_READ_RSP_OUTSTANDING: begin
            if (ugshim_slave.r.canPeek) begin
               if (rg_verbosity > 1) begin
                  $display ("    Was reading, dropping flit: ", fshow (ugshim_slave.r.peek));
               end
               ugshim_slave.r.drop;
               if (ugshim_slave.r.peek.rlast) begin
                  rg_state <= DMA_HALTED;
                  ugfifo_halt.deq;
                  if (rg_verbosity > 1) begin
                     $display ("    Dropped last flit");
                  end
               end
            end
         end
         DMA_MAIN_WRITE_RSP_OUTSTANDING, DMA_APP_WRITE_RSP_OUTSTANDING: begin
            if (ugshim_slave.b.canPeek) begin
               ugshim_slave.b.drop;
               rg_state <= DMA_HALTED;
               ugfifo_halt.deq;
               if (rg_verbosity > 1) begin
                  $display ("    Was waiting for write response, dropping flit: ", fshow (ugshim_slave.b.peek));
               end
            end
         end
         DMA_MAIN_WRITE_LOOP, DMA_APP_WRITE_LOOP: begin
            Bool islast = rg_rsp_count == rg_req_len;
            AXI4_WFlit #(data_, wuser_) dummy_wflit = AXI4_WFlit {
               wdata : ?,
               wstrb : 0,
               wlast : islast,
               wuser : 0
            };
            if (rg_verbosity > 1) begin
               $display ("    Was writing, sending flit: ", fshow (dummy_wflit));
               $display ("    count: ", fshow (rg_rsp_count));
            end

            rg_rsp_count <= rg_rsp_count + 1;
            if (islast) begin
               // need to wait for B response
               rg_state <= DMA_MAIN_WRITE_RSP_OUTSTANDING;
               if (rg_verbosity > 1) begin
                  $display ("    Last write flit sent");
               end
            end
         end
      endcase
   endrule

   // detect issues
   rule rl_detect_rresp_in_wrong_state (ugshim_slave.r.canPeek
                                        && rg_state != DMA_MAIN_READ_RSP_OUTSTANDING
                                        && rg_state != DMA_APP_READ_RSP_OUTSTANDING);
      $display ("AXI4 SG Unit: ERROR: DMA detected read response when DMA is in the wrong state");
   endrule

   rule rl_detect_wresp_in_wrong_state (ugshim_slave.b.canPeek
                                        && rg_state != DMA_MAIN_WRITE_RSP_OUTSTANDING
                                        && rg_state != DMA_APP_WRITE_RSP_OUTSTANDING);
      $display ("AXI4 SG Unit: ERROR: DMA detected write response when DMA is in the wrong state");
   endrule

   rule rl_debug_r_canpeek (ugshim_slave.r.canPeek);
      $display ("SG canpeek");
      $display ("    state: ", fshow (rg_state));
      $display ("    read_app_words: ", fshow (read_app_words));
      $display ("    input flit: ", fshow (ugshim_slave.r.peek));
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
   method Action bd_read_from_mem (DMA_Dir dir, Bit #(addr_) start_addr) if (ugshim_slave.ar.canPut
                                                                             && rg_state == DMA_IDLE);

      // update direction
      crg_dir[0] <= dir;

      // bus transaction
      // don't read app words yet
      match {.arflit, .len} = axi4_ar_burst_flit (start_addr, True, True);
      ugshim_slave.ar.put (arflit);
      if (rg_verbosity > 0) begin
         $display ("AXI4 SG Unit: DMA sent AR flit: ", fshow(arflit));
      end

      // internal bookkeeping
      rg_addr_bd_start <= start_addr;
      rg_addr_bd_cur <= start_addr;
      rg_bd_index <= minBound;
      rg_req_len <= len;
      rg_req_size <= arflit.arsize;
      rg_rsp_count <= 0;
      rg_received_err_rsp <= False;

      rg_state <= DMA_MAIN_READ_RSP_OUTSTANDING;
   endmethod

   /*
    * Request that the BD currently held in the local registers be written out to
    * main memory.
    */
   method Action bd_write_to_mem (DMA_Dir dir, Bit #(addr_) start_addr) if (rg_state == DMA_IDLE);
      // update direction
      crg_dir[0] <= dir;

      if (rg_verbosity > 0) begin
         $display ("AXI4 SG Unit: requested write at ", fshow(start_addr));
         $display ("              write value: ", fshow(readVReg (v_v_rg_bd[pack (dir)])));
      end

`ifdef DMA_CHERI
      rg_addr_bd_start <= start_addr + 'h20;
      rg_addr_bd_cur <= start_addr + 'h20;
      rg_state <= DMA_APP_WRITE_START;
      // TODO this will break with 32bit
      rg_bd_index <= DMA_CONTROL;
`else
      rg_addr_bd_start <= start_addr;
      rg_addr_bd_cur <= start_addr;
      rg_state <= DMA_MAIN_WRITE_START;
      rg_bd_index <= minBound;
`endif
      rg_rsp_count <= 0;
   endmethod

   // TODO update this to handle 64-bit
   // (ie use NXTDESC_MSB)
   // This is currently unused and untested
   method Action bd_fetch_next (DMA_Dir dir) if (ugshim_slave.ar.canPut
                                                 && rg_state == DMA_IDLE);
      // update direction
      crg_dir[0] <= dir;

      if (rg_verbosity > 0) begin
         $display ("AXI4 SG Unit: requested fetch of next buffer descriptor");
      end

`ifdef DMA_CHERI
      DMA_BD_Index nxtdesc_0_idx = DMA_NXTDESC_0;
`else
      DMA_BD_Index nxtdesc_0_idx = DMA_NXTDESC;
`endif
      Bit #(addr_) addr = zeroExtend (v_v_rg_bd[pack (dir)][pack (nxtdesc_0_idx)].word);

      // bus transaction
      // don't read app words yet
      match {.arflit, .len} = axi4_ar_burst_flit (addr, True, True);
      ugshim_slave.ar.put (arflit);

      // internal bookkeeping
      rg_addr_bd_start <= addr;
      rg_addr_bd_cur <= addr;
      rg_bd_index <= minBound;
      rg_req_len <= len;
      rg_req_size <= arflit.arsize;
      rg_rsp_count <= 0;
      rg_received_err_rsp <= False;

      rg_state <= DMA_MAIN_READ_RSP_OUTSTANDING;
   endmethod

   /*
    * Signal that we have finished doing an MM2S read, and the BD in
    * the registers is correct
    */
   method Maybe #(DMA_Dir) trigger_callback;
      return drg_sg_finished;
   endmethod

   /*
    * Signal that we have finished writing back a BD to memory
    */
   method Maybe #(DMA_Dir) trigger_interrupt;
      return rw_trigger_interrupt.wget;
   endmethod

   method DMA_Dir current_dir = crg_dir[1];

   interface axi4_master = shim.master;

   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
   endmethod

   method Action reset;
      rg_state <= DMA_RESET;
   endmethod

   method Action halt_to_idle if (rg_state == DMA_HALTED);
      rg_state <= DMA_IDLE;
      if (rg_verbosity > 1) begin
         $display ("sg module halt_to_idle");
         $display ("    old state: ", fshow (rg_state));
      end
   endmethod

   interface Server srv_halt;
      interface Put request;
         method Action put (Bit #(0) none) if (ugfifo_halt.notFull);
            if (rg_verbosity > 0) begin
               $display ("DMA Scatter Gather Halt request received");
            end
            ugfifo_halt.enq (?);
         endmethod
      endinterface
      interface Get response;
         method ActionValue #(Bit #(0)) get if (!ugfifo_halt.notEmpty);
            if (rg_verbosity > 0) begin
               $display ("DMA Scatter Gather Halt response sent");
            end
            return (?);
         endmethod
      endinterface
   endinterface

   method Maybe #(DMA_Err_Cause) enq_halt_o = rw_enq_halt_o.wget;
endmodule


endpackage
