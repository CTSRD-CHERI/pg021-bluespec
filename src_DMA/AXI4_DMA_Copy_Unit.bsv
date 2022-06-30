package AXI4_DMA_Copy_Unit;

// Bluespec imports
import Vector :: *;
import FIFOF :: *;
import SpecialFIFOs :: *;
import FIFOLevel :: *;
import GetPut :: *;
import ClientServer :: *;

// AXI imports
import AXI :: *;
import AXI4Stream_Types :: *;
import AXI4Stream_Utils :: *;
import SourceSink :: *;

// Local imports
import AXI4_DMA_Types :: *;
import AXI4_DMA_Internal_Reg_Module :: *;
import AXI4_DMA_Utils :: *;

interface AXI4_DMA_Copy_Unit_IFC #(numeric type id_
                                  ,numeric type addr_
                                  ,numeric type data_
                                  ,numeric type awuser_
                                  ,numeric type wuser_
                                  ,numeric type buser_
                                  ,numeric type aruser_
                                  ,numeric type ruser_
                                  // axi4 stream parameters
                                  ,numeric type sid_
                                  ,numeric type sdata_
                                  ,numeric type sdest_
                                  ,numeric type suser_);

   interface AXI4_Master #(id_, addr_, data_,
                           awuser_, wuser_, TAdd #(Checker_Resp_U_Bits, buser_),
                           aruser_, TAdd #(Checker_Resp_U_Bits, ruser_)) axi4_master;

   interface AXI4Stream_Master #(sid_, sdata_, sdest_, suser_) axi4s_data_master;
   interface AXI4Stream_Master #(sid_, sdata_, sdest_, suser_) axi4s_meta_master;

   interface AXI4Stream_Slave #(sid_, sdata_, sdest_, suser_) axi4s_data_slave;
   interface AXI4Stream_Slave #(sid_, sdata_, sdest_, suser_) axi4s_meta_slave;

   method Action trigger;

   // True when the stream has just started sending data, otherwise false
   method Bool s2mm_start_trigger;

   method Maybe #(DMA_Dir) end_trigger;

   method DMA_Dir current_dir;

   method Action notify_bd;

   method Action set_verbosity (Bit #(4) new_verb);

   method Action reset;
   method Bool reset_done;
   method Action halt_to_idle;
   interface Server #(Bit #(0), Bit #(0)) srv_halt;
   method Maybe #(DMA_Err_Cause) enq_halt_o;
endinterface

typedef enum {
   RESET,
   IDLE,
   HALTED,
   META_RECEIVE_S2MM,
   META_SEND_MM2S,
   COPYING,
   WAIT_FOR_FINAL_DEQ
} State deriving (Eq, Bits, FShow);

// This module performs a single memory transfer (either MM2S or S2MM)
// when triggered.
// It reads and writes the configuration registers that are passed in
// at instantiation.
// TODO for now this assumes that all buffers start at addresses that are at least 32-bit aligned
module mkAXI4_DMA_Copy_Unit #(Vector #(n_, Vector #(m_, Reg #(DMA_BD_TagWord))) v_v_rg_bd,
                              DMA_State state_outer,
                              AXI4_DMA_Int_Reg_IFC dma_int_reg)
                            (AXI4_DMA_Copy_Unit_IFC #(id_, addr_, data_,
                                                      awuser_, wuser_, buser_,
                                                      aruser_, ruser_, sid_, sdata_, sdest_, suser_))
                            provisos ( Add #(a__, 12, addr_)
                                     , Add #(b__, 3, addr_)
                                     , Add #(c__, SizeOf #(DMA_Copy_Word), data_)
                                     , Add #(d__, SizeOf #(DMA_Copy_Word), addr_)
                                     , Add #(e__, 9, addr_)
                                     , Add #(f__, 32, addr_)
                                     , Add #(g__, 8, addr_)
                                     , Add #(h__, 26, addr_)
                                     , Add #(0, SizeOf #(DMA_Copy_Word), sdata_)
                                     , Add #(i__, SizeOf #(DMA_Copy_Word), data_)
                                     , Add #(j__, 10, addr_)
                                     , Add #(4, k__, TDiv#(data_, 8))
                                     );

   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);

   let shim <- mkAXI4ShimFF;
   let ugshim_slave <- toUnguarded_AXI4_Slave (shim.slave);

   // AXI 4 Stream slave shim
   // We are the master of this slave, and we write flits into it
   // This is the shim through which the actual data passes
   AXI4Stream_Shim #(sid_, sdata_, sdest_, suser_) axi4s_s_data_shim <- mkAXI4StreamShimFF;
   let axi4s_s_data_ugshim_slave <- toUnguarded_AXI4Stream_Slave (axi4s_s_data_shim.slave);

   // This is the shim through which metadata passes
   // Used to communicate with the ethernet block
   // For the ethernet block, the metadata is transferred first, and then the actual
   // data is transferred
   AXI4Stream_Shim #(sid_, sdata_, sdest_, suser_) axi4s_s_meta_shim <- mkAXI4StreamShimFF;
   let axi4s_s_meta_ugshim_slave <- toUnguarded_AXI4Stream_Slave (axi4s_s_meta_shim.slave);

   // AXI 4 Stream master shims
   // We are the slave of this master, and we read flits from it
   // This is the shim through which the actual data passes
   AXI4Stream_Shim #(sid_, sdata_, sdest_, suser_) axi4s_m_data_shim <- mkAXI4StreamShimFF;
   let axi4s_m_data_ugshim_master <- toUnguarded_AXI4Stream_Master (axi4s_m_data_shim.master);

   // This is the shim through which metadata passes
   // Used to communicate with the ethernet block
   // For the ethernet block, the metadata is transferred first, and then the actual
   // data is transferred
   AXI4Stream_Shim #(sid_, sdata_, sdest_, suser_) axi4s_m_meta_shim <- mkAXI4StreamShimFF;
   let axi4s_m_meta_ugshim_master <- toUnguarded_AXI4Stream_Master (axi4s_m_meta_shim.master);

   Reg #(DMA_Dir) crg_dir[2] <- mkCReg (2, MM2S);
   Reg #(State) rg_state <- mkReg (HALTED);
   FIFOF #(Bit #(0)) ugfifo_halt <- mkUGFIFOF1;

   // Use a FIFO with a level, so that we can see how much data is in it
   // We only send read requests when there is space for the full request,
   // and we only send write requests when there enough data in the fifo
   // to feed the full request
   // This is used for transfers in both directions
   FIFOCountIfc #(DMA_Copy_Word, 16) fifo_data <- mkFIFOCount;

   // TODO make some of these registers RegU?
   // length of the buffer being transferred in bytes
   // for MM2S transactions, this value is in the buffer descriptor
   // for S2MM transactions, we calculate this value by counting how
   // many flits we get from the stream interface, and then write it
   // into memory
   Reg #(Bit #(26)) rg_buf_len <- mkReg (0);

   // amount of bytes already transferred
   // for MM2S transactions, this is incremented when we receive data from
   // memory, and used to keep track of how many bytes we have received from
   // main memory
   // for S2MM transactions, this is incremented when we produce a WFlit
   // to write to memory, and used to keep track of how many bytes have
   // been written to main memory
   Reg #(Bit #(26)) rg_buf_cur <- mkReg (0);

   // amount of bytes left to transfer
   // for MM2S transactions, this keeps track of how many more bytes we
   // still have to read from memory, and can only ever decrease
   // For S2MM transactions, this can increase or decrease since we
   // don't know the size of the transaction ahead of time
   let amt_buf_left = rg_buf_len - rg_buf_cur;

   // address of next byte needed from memory
   // for MM2S transactions, this keeps track of the address of the next
   // byte we need to receive
   // for S2MM transactions, this keeps track of the address of the start
   // of the next write to memory, and does not track correctly with the
   // actual address being written during the middle of a write burst
   Reg #(Bit #(addr_)) rg_addr_next_byte <- mkReg (0);

   // stores whether there is currently a transaction in flight
   // becomes True when we enqueue a transaction, and False when we get
   // the last response from that transaction
   // for MM2S transactions, this becomes True when we make a read request,
   // and becomes False when we subsequently receive a RFlit where rlast is
   // set to true
   // for S2MM transactions, this becomes true when we make a write request,
   // and becomes false when we have sent enough WFlits
   Reg #(Bool) rg_txion_in_flight <- mkReg (False);

   // Whether we're expecting to receive a bresp or not
   // Only valid during S2MM transactions
   // becomes True when we produce the last WFlit in a transaction (ie
   // when rg_txion_in_flight becomes False)
   // becomes False when we receive a BResp
   Reg #(Bool) rg_bresp_required <- mkReg (False);

   // Whether the bresp we're expecting is the last one for this s2mm
   // transaction or not
   Reg #(Bool) rg_bresp_last <- mkRegU;

   // Stores the arlen of the previously sent request, for both MM2S and S2MM
   // This is actually 1 less than the number of responses required
   Reg #(AXI4_Len) rg_prev_arlen <- mkReg (0);

   // Tracks how far into the current transaction we are
   Reg #(AXI4_Len) rg_txion_counter <- mkReg (0);

   // Stores the number of metadata words that have been received
   // We should always receive exactly 6 words from the ethernet
   Reg #(Bit #(3)) rg_meta_counter <- mkReg (0);

   // Whether the next data will be in the upper bits or the lower bits of
   // the 64bit word, for both S2MM and MM2S
   Reg #(Bool) rg_word_in_upper <- mkRegU;

   // Keeps track of how many bytes have been sent out of the stream this transaction
   Reg #(Bit #(26)) rg_stream_out_count <- mkReg (0);

   // Keeps track of whether a new Buffer Descriptor has been fetched by the
   // Scatter Gather module for S2MM, so that we can block reception until a
   // BD is available.
   Reg #(Bool) rg_bd_available <- mkReg (False);

   RWire #(DMA_Dir) rw_end_trigger <- mkRWireSBR;
   RWire #(DMA_Err_Cause) rw_enq_halt_o <- mkRWire;

   // True when the stream has just started sending data, otherwise false
   Wire #(Bool) dw_s2mm_start_trigger <- mkDWire (False);

   // Whether making a request for (len+1) words would cross a 4KB boundary
   // (crossing 4KB boundaries in a burst is not allowed in AXI)
   // Alternative phrasing: whether the last byte that will be requested by a transaction
   // starting at base and ending at (base + (len * word_size)) will be in the same 4KB region as
   // the base
   function Bool fn_crosses_4k_boundary (Bit #(addr_) base, Bit #(3) len);
      Bit #(addr_) last = base + (zeroExtend (len) * (32 / 8)); // assumes 32-bit data
      let retval = False;
      if (base[12] != last[12]) begin
         retval = True;
      end
      return retval;
   endfunction

   // returns the maximum AXI len you can request while still staying within a 4KB boundary
   // returns len up to 7 (meaning maximum 8 flits)
   // assumes that the address is size-aligned
   // (ie if the size is 64bits/8bytes then addr[2:0] is 0
   // For now, assume the size of each word being copied is 32 bits
   // 4KB boundary corresponds to the bottom 12 bits
   // ie if the bottom 12 bits are 0, the request is 4KB aligned
   function Bit #(3) fn_arlen_from_4k_boundary (Bit #(addr_) addr);
      let ones = ~0;
      // this is where the 32-bit assumption is made
      // We take the bottom 12 bits, and of those we ignore the bottom 2 or 3 bits (depending
      // on whether we're doing 32bit or 64bit accesses) because we assume the address is
      // size-aligned
      Bit #(TSub #(12, TLog #(TDiv #(32, 8)))) addr_lsb = truncate (addr >> valueOf (TLog #(TDiv #(32, 8))));

      // "ones" represents the last word that we could write without crossing
      // the 4KB boundary
      // Here we find out how far away we are from that word address
      Bit #(TSub #(12, TLog #(TDiv #(32, 8)))) sub = ones - addr_lsb;

      let retval = 3'b111;
      if (sub > zeroExtend (3'b111)) begin
         // We are more than 8 words away from the boundary, so we can write 8 words
         retval = 3'b111;
      end else begin
         // We are fewer than 8 words away from the boundary
         retval = truncate (sub);
      end
      return retval;
   endfunction

   rule rl_reset (rg_state == RESET);
      if (rg_verbosity > 0) begin
         $display ("AXI4 DMA Copy Unit Reset");
      end
      shim.clear;
      axi4s_s_data_shim.clear;
      axi4s_s_meta_shim.clear;
      axi4s_m_data_shim.clear;
      axi4s_m_meta_shim.clear;
      crg_dir[0] <= MM2S;

      fifo_data.clear;
      rg_buf_len <= 0;
      rg_buf_cur <= 0;
      rg_addr_next_byte <= 0;
      rg_txion_in_flight <= False;
      rg_prev_arlen <= 0;
      rg_txion_counter <= 0;
      rg_bd_available <= False;

      rg_state <= HALTED;
   endrule

   // The ethernet block will first send the metadata, and only will only
   // send the data afterwards.
   // If we are in IDLE and we can dequeue from the metadata master shim,
   // then the ethernet block is trying to send us data, and we can handle
   // the data transmission.
   // Main memory transactions are deferred until we have buffered some data
   // for bursting.
   // There is no useful information in the first word (apart from the flag,
   // which for correct transactions is always the same) so we can just throw
   // it away
   // TODO this assumes that whatever buffer descriptor we have is valid and
   // we can write to it
   rule rl_s2mm_metadata_transfer_start (rg_state == IDLE
                                         && state_outer == DMA_IDLE
                                         && rg_bd_available
                                         && dma_int_reg.s2mm_dmasr.halted == 1'b0
                                         && axi4s_m_meta_ugshim_master.canPeek);
      if (rg_verbosity > 0) begin
         $display ("AXI4 DMA Copy Unit receiving S2MM metadata");
         if (rg_verbosity > 1) begin
            $display ("    flit: ", fshow (axi4s_m_meta_ugshim_master.peek));
         end
      end
      if (truncate (axi4s_m_meta_ugshim_master.peek.tstrb) != 4'hf) begin
         $display ("AXI4 DMA Copy Unit: Error: Received non-full word from ethernet");
      end
      if (axi4s_m_meta_ugshim_master.peek.tlast) begin
         $display ("AXI4 DMA Copy Unit: Error: First word from ethernet was also last");
      end
      if (truncate (axi4s_m_meta_ugshim_master.peek.tkeep) != 4'hf) begin
         $display ("AXI4 DMA Copy Unit: Error: Received null byte from ethernet");
      end
      if (axi4s_m_meta_ugshim_master.peek.tdata[31:28] != 'h5) begin
         $display ("AXI4 DMA Copy Unit: Error: Received wrong transfer type from ethernet");
      end

      axi4s_m_meta_ugshim_master.drop;
      rg_state <= META_RECEIVE_S2MM;
      dw_s2mm_start_trigger <= True;
      // start the counter at zero for code clarity (we can just add the counter to
      // the address of _APP0 to get which word we are changing now)
      rg_meta_counter <= 0;
   endrule

   // Receive the rest of the metadata for this ethernet frame, and
   // update the Buffer Descriptor application fields accordingly
   rule rl_s2mm_metadata_receive (rg_state == META_RECEIVE_S2MM
                                  && state_outer == DMA_RUNNING_S2MM_COPY
                                  && axi4s_m_meta_ugshim_master.canPeek);
      axi4s_m_meta_ugshim_master.drop;
      if (rg_verbosity > 1) begin
         $display ("AXI4 DMA Copy Unit rl_s2mm_metadata_receive");
         $display ("    flit: ", fshow (axi4s_m_meta_ugshim_master.peek));
      end

      v_v_rg_bd[pack (S2MM)][pack (DMA_APP0) + zeroExtend (rg_meta_counter)]
         <= fn_to_untagged_tagword (truncate (axi4s_m_meta_ugshim_master.peek.tdata));

      // We start the counter at zero for clarity above, which means that
      // it is actually one lower than the real number of words received.
      // If when we check it here, it has a value of 4, then that means that
      // 5 words had previously been received and this is the sixth (and last)
      if (rg_meta_counter == 4) begin
         // No need to change state - the next rule looks for this same state and
         // for the correct counter value
         // We do update the RXSOF and RXEOF since for this version of the DMA engine
         // we just always set both to 1
         // Also update the Cmplt bit
         // TODO need to update with the length later
         // can't just set it to the value received in the status stream, as the one in
         // the status stream is a non-overflowing counter with a lower maximum than the longest
         // possible DMA transfer
         v_v_rg_bd[pack (S2MM)][pack (DMA_STATUS)] <=
            fn_to_untagged_tagword (v_v_rg_bd[pack (S2MM)][pack (DMA_STATUS)].word
                                    | {1'b1, 3'b0, 1'b1, 1'b1, 26'b0});
         if (!axi4s_m_meta_ugshim_master.peek.tlast) begin
            $display ("AXI4 DMA Copy Unit: Error: Received more than 6 metadata words");
         end
      end
      rg_meta_counter <= rg_meta_counter + 1;
   endrule


   // If we are in META_RECEIVE_S2MM, that means we have started to receive
   // the metadata. If our counter has 5 in it, that means we have received
   // all 6 words and so are ready to start receiving actual data.
   // If we can dequeue from our master shim then our stream master is trying to
   // send us data and we can handle the data transmission
   // We need to buffer some of this data in order to know how big our
   // AWFlit should be, so we don't yet send out an AWFlit
   rule rl_s2mm_data_transfer_start (rg_state == META_RECEIVE_S2MM
                                     && state_outer == DMA_RUNNING_S2MM_COPY
                                     && rg_meta_counter == 5
                                     && axi4s_m_data_ugshim_master.canPeek);
      if (rg_verbosity > 0) begin
         $display ("AXI4 DMA Copy Unit starting S2MM transfer");
      end
      rg_addr_next_byte
`ifdef DMA_CHERI
                        <= zeroExtend (pack (v_v_rg_bd[pack (S2MM)][pack (DMA_BUFFER_ADDRESS_0)].word));
`else
                        <= zeroExtend (pack (v_v_rg_bd[pack (S2MM)][pack (DMA_BUFFER_ADDRESS)].word));
`endif
      rg_buf_len <= 0;
      rg_buf_cur <= 0;
      crg_dir[0] <= S2MM;
      rg_state <= COPYING;
   endrule

   // When there is data in the input stream and there is space in the
   // internal FIFOs for it, transfer it from the stream to the FIFOs
   // The Ethernet block specification (pg138) specifies that null strobes
   // are not allowed, and that all except the last word transferred must
   // have tkeep = 'hf and tstrb = 'hf
   rule rl_s2mm_deq_from_stream (rg_state == COPYING
                                 && state_outer == DMA_RUNNING_S2MM_COPY
                                 && crg_dir[0] == S2MM
                                 && axi4s_m_data_ugshim_master.canPeek
                                 && fifo_data.notFull);
      if (rg_verbosity > 0) begin
         $display ("AXI4 DMA Copy Unit dequeueing data from stream");
      end
      if (rg_verbosity > 1) begin
         $display ("    data: ", fshow (axi4s_m_data_ugshim_master.peek.tdata));
      end
      // handle transferring the data
      axi4s_m_data_ugshim_master.drop;
      // TODO not sure if this should be done with tkeep or tstrb
      let tkeep = axi4s_m_data_ugshim_master.peek.tkeep;
      let mask = { tkeep[3] == 1 ? 8'hff : 8'h0
                 , tkeep[2] == 1 ? 8'hff : 8'h0
                 , tkeep[1] == 1 ? 8'hff : 8'h0
                 , tkeep[0] == 1 ? 8'hff : 8'h0};
      let bytes_rcvd = tkeep[3] == 1 ? 4
                     : tkeep[2] == 1 ? 3
                     : tkeep[1] == 1 ? 2 : 1;
      fifo_data.enq (axi4s_m_data_ugshim_master.peek.tdata & mask);
      // TODO this assumes 32bit transfers
      rg_buf_len <= rg_buf_len + bytes_rcvd;

      // handle the last flit
      if (axi4s_m_data_ugshim_master.peek.tlast) begin
         // This is the last flit of data
         rg_state <= WAIT_FOR_FINAL_DEQ;
         if (rg_buf_len + bytes_rcvd != zeroExtend (v_v_rg_bd[pack (S2MM)][pack (DMA_APP4)].word[15:0])) begin
            $display ("AXI4 DMA Copy Unit: Error: Received the wrong number of data bytes");
            // TODO this assumes 32bit transfers
            $display ("    Real bytes received: ", fshow (rg_buf_len + bytes_rcvd));
            $display ("    Number of bytes that should have been received: ",
                      fshow (v_v_rg_bd[pack (S2MM)][pack (DMA_APP4)].word[15:0]));
         end
         if (rg_verbosity > 0) begin
            $display ("           received tlast");
         end
      end
   endrule

   // Produce an AWFlit when there is enough data in the FIFOs for a
   // full-length (8 flit) burst or when we have received the last
   // stream flit, and we have not already sent off a transaction
   rule rl_s2mm_produce_awflit (crg_dir[0] == S2MM
                                && ((rg_state == COPYING && fifo_data.count > 7) // there is enough data for an 8-flit burst
                                    || (rg_state == WAIT_FOR_FINAL_DEQ))       // we have received the last flit from the stream
                                && state_outer == DMA_RUNNING_S2MM_COPY
                                && fifo_data.notEmpty
                                && ugshim_slave.aw.canPut
                                && !rg_txion_in_flight
                                && !rg_bresp_required);
      AXI4_Len len = min (7, zeroExtend (pack (fifo_data.count - 1)));
      let len_from_4k = fn_arlen_from_4k_boundary (rg_addr_next_byte);
      let constrained_by_4k = len > zeroExtend (len_from_4k);
      len = min (len, zeroExtend (len_from_4k));
      if (constrained_by_4k) begin
         if (rg_verbosity > 1) begin
            $display ("DMA Copy Unit: AWFlit would have crossed 4k boundary");
            $display ("               Reducing size to avoid crossing 4k boundary");
         end
      end

      if (fifo_data.count == 0) begin
         $display ("DMA Copy Unit: ERROR: trying to produce awflit when there is no data");
      end

      if (rg_state == WAIT_FOR_FINAL_DEQ && fifo_data.count <= 8 && !constrained_by_4k) begin
         // we can output the rest of the data in the FIFO with this AWFlit
         // and there is no more data that will be put into the FIFO, so
         // the BResp for this AWFlit will be the last one for this
         // S2MM transfer
         rg_bresp_last <= True;
      end

      AXI4_AWFlit #(id_, addr_, awuser_) awflit = AXI4_AWFlit {
         awid     : 0,
         awaddr   : zeroExtend (rg_addr_next_byte),
         awlen    : len, // NOTE burst length = awlen + 1
         awsize   : 4, // the size is 4 bytes, this gets transformed to the
                       // actual value of 3'b011 within the type
                       // this assumes 32bit transfers
         awburst  : INCR,
         awlock   : NORMAL,
         awcache  : awcache_dev_nonbuf,
         awprot   : axi4Prot (DATA, SECURE, UNPRIV),
         awqos    : 0,
         awregion : 0,
         awuser   : 0
      };
      // TODO this assumes 32bit transfers
      rg_addr_next_byte <= rg_addr_next_byte + zeroExtend ((len + 1)*4);
      rg_prev_arlen <= zeroExtend (len);
      rg_txion_in_flight <= True;
      rg_txion_counter <= 0;
      rg_word_in_upper <= rg_addr_next_byte[2] == 1;

      if (rg_verbosity > 0) begin
         $display ("DMA Copy Unit producing S2MM AWFlit");
         $display ("   flit: ", fshow (awflit));
      end
      ugshim_slave.aw.put (awflit);
      if (rg_bresp_required) begin
         $display ("DMA Copy Unit: ERROR: Producing AWFlit without having received BResp from");
         $display ("                      previous AWFlit");
      end
   endrule

   // While we are copying from the stream to memory and there is a
   // transaction in flight and there is data in the FIFOs, dequeue
   // from the FIFOs and push a WFlit to our AXI4 slave
   rule rl_s2mm_produce_wflit (crg_dir[0] == S2MM
                              && (rg_state == COPYING || rg_state == WAIT_FOR_FINAL_DEQ)
                              && state_outer == DMA_RUNNING_S2MM_COPY
                              && rg_txion_in_flight
                              && ugshim_slave.w.canPut
                              && fifo_data.notEmpty);
      if (rg_prev_arlen == rg_txion_counter) begin
         // After this flit, we will have sent off enough data that we
         // will have finished the last AWFlit sent off, so there will
         // be no transaction in flight
         rg_txion_in_flight <= False;
         rg_bresp_required <= True;
      end
      fifo_data.deq;

      AXI4_WFlit #(data_, wuser_) wflit = AXI4_WFlit {
         wdata : rg_word_in_upper ? {fifo_data.first, 0} : {0, fifo_data.first},
         wstrb : rg_word_in_upper ? {4'hf, 0} : {0, 4'hf},
         wlast : rg_prev_arlen == rg_txion_counter,
         wuser : 0
      };
      ugshim_slave.w.put (wflit);
      if (rg_verbosity > 0) begin
         $display ("DMA Copy Unit producing wflit");
         $display ("   wflit: ", fshow (wflit));
      end

      rg_txion_counter <= rg_txion_counter + 1;
      // TODO this assumes 32bit transfers
      rg_buf_cur <= rg_buf_cur + 4;
      rg_word_in_upper <= !rg_word_in_upper;
   endrule

   // Handle receiving B responses from memory
   // if this is the last B response we expect to receive, transition into the idle
   // state, write back the required registers, and update the trigger
   rule rl_s2mm_handle_bresp ((rg_state == COPYING || rg_state == WAIT_FOR_FINAL_DEQ)
                              && state_outer == DMA_RUNNING_S2MM_COPY
                              && rg_bresp_required
                              && crg_dir[0] == S2MM
                              && ugshim_slave.b.canPeek);
      rg_bresp_required <= False;
      ugshim_slave.b.drop;

      let bflit = ugshim_slave.b.peek;
`ifdef DMA_CHERI
      let cheri_err = bflit.bresp == SLVERR && truncateLSB(bflit.buser) == 1'b1;
`endif

      if (bflit.bresp == SLVERR || bflit.bresp == DECERR) begin
         ugfifo_halt.enq (?);
         if (rg_verbosity > 0) begin
            $display ("DMA Copy Unit: got errored B response");
            if (rg_verbosity > 1) begin
               $display ("    flit: ", fshow (bflit));
            end
`ifdef DMA_CHERI
            if (cheri_err) begin
               if (rg_verbosity > 0) begin
                  $display ("    Errored due to CHERI capability not authorising access");
               end
               rw_enq_halt_o.wset (CHERIERR);
            end else
`endif
            if (bflit.bresp == SLVERR) begin
               if (rg_verbosity > 0) begin
                  $display ("    Errored due to SLVERR");
               end
               rw_enq_halt_o.wset (SLVERR);
            end else begin
               if (rg_verbosity > 0) begin
                  $display ("    Errored due to DECERR");
               end
               rw_enq_halt_o.wset (DECERR);
            end
         end
      end else begin
         if (rg_bresp_last) begin
            if (rg_verbosity > 0) begin
               $display ("DMA Copy Unit: finished s2mm transfer, going back to idle, outer state: ", fshow (state_outer));
            end
            rg_state <= IDLE;
            rg_bd_available <= False;
            rw_end_trigger.wset (S2MM);
            let sts_to_write = fn_to_untagged_tagword (v_v_rg_bd[pack (S2MM)][pack (DMA_STATUS)].word
                                                       | {1'b1, 3'b0, 1'b1, 1'b1, zeroExtend (rg_buf_len)});
            v_v_rg_bd[pack (S2MM)][pack (DMA_STATUS)] <= sts_to_write;
            if (rg_verbosity > 1) begin
               $display ("DMA Copy Unit: Set S2MM Status to ", fshow (sts_to_write));
               //$display ("               Set DMA_APP4 to ", fshow (app4_to_write));
            end
            rg_bresp_last <= False;
         end
         if (rg_verbosity > 0) begin
            $display ("DMA Copy Unit: Received and acknowledged BResp");
         end
      end
   endrule



   // MM2S Transactions are started using the Action method in this module's interface
   // so none of the MM2S rules start off at IDLE

   // Start off with transferring metadata on the metadata stream
   rule rl_mm2s_meta_transfer (rg_state == META_SEND_MM2S
                               && state_outer == DMA_RUNNING_MM2S_COPY
                               && axi4s_s_meta_ugshim_slave.canPut);
      AXI4Stream_Flit #(sid_, sdata_, sdest_, suser_) flit = AXI4Stream_Flit {
         tdata: zeroExtend (v_v_rg_bd[pack (MM2S)][pack (DMA_APP0) + zeroExtend (rg_meta_counter)].word),
         tstrb: ~0,
         tkeep: ~0,
         tlast: rg_meta_counter == 4, // See below where we check if it is 4
         tid:   0,
         tdest: 0,
         tuser: 0
      };
      axi4s_s_meta_ugshim_slave.put (flit);

      // We start this counter at 0 when we send the first byte in order
      // to clarify when selecting which APP word we are writing, so when
      // it has a 4 then that means that we have actually sent 5 of our
      // 6 words and this is the last one.
      if (rg_meta_counter == 4) begin
         rg_state <= COPYING;
      end
      rg_meta_counter <= rg_meta_counter + 1;
   endrule

   // Copy data from internal FIFOs into the stream interface when possible
   rule rl_mm2s_fifof_deq ((rg_state == COPYING
                            || rg_state == WAIT_FOR_FINAL_DEQ)
                          && state_outer == DMA_RUNNING_MM2S_COPY
                          && crg_dir[0] == MM2S
                          && axi4s_s_data_ugshim_slave.canPut
                          && rg_stream_out_count < rg_buf_len);
      fifo_data.deq;
      if (rg_verbosity > 1) begin
         $display ("DMA Copy Unit: fifo_data dequeue: ", fshow (fifo_data.first));
      end
      let diff = rg_buf_len - rg_stream_out_count;
      let active_bytes = diff > 3  ? 4'hf
                       : diff == 3 ? 4'h7
                       : diff == 2 ? 4'h3
                       : diff == 1 ? 4'h1
                       :             4'h0;
      AXI4Stream_Flit #(sid_, sdata_, sdest_, suser_) flit = AXI4Stream_Flit {
         tdata: zeroExtend (fifo_data.first),
         tstrb: active_bytes,
         tkeep: active_bytes,
         tlast: rg_stream_out_count + 4 >= rg_buf_len,
         tid: 0,
         tdest: 0,
         tuser: 0
      };
      if (active_bytes == 4'h0) begin
         $display ("AXI Copy Unit: ERROR: writing flit to stream with no active bytes");
      end
      axi4s_s_data_ugshim_slave.put (flit);
      rg_stream_out_count <= rg_stream_out_count + 4;

      if (rg_verbosity > 1) begin
         $display ("DMA Copy Unit: enqueuing flit to stream");
         $display ("    flit: ", fshow (flit));
      end
      if (rg_verbosity > 0 && rg_stream_out_count + 4 >= rg_buf_len) begin
         $display ("DMA Copy Unit: sent last stream flit");
      end
   endrule

   // handle updating the state back to idle after all data has been
   // transferred out to the stream interface
   // This has been split off from the rule above to avoid a scheduling
   // conflict between that rule and rl_handle_read_rsp, caused by both
   // rules writing rg_state.
   // Even after adding a condition that prevented both rules from
   // writing rg_state in the same cycle (ie checking that the state
   // in the rule above was WAIT_FOR_FINAL_DEQ) and with aggressive
   // conditions the conflict was still present
   rule rl_mm2s_update_state (rg_state == WAIT_FOR_FINAL_DEQ
                              && state_outer == DMA_RUNNING_MM2S_COPY
                              && crg_dir[0] == MM2S
                              && rg_stream_out_count >= rg_buf_len);
      $display ("DMA Copy Unit: finished stream transfer, going to IDLE");
      // Set BD status complete and transferred bytes fields
      v_v_rg_bd[pack (MM2S)][pack (DMA_STATUS)] <=
         fn_to_untagged_tagword ({1'b1, 5'b0, rg_buf_len});

      rg_state <= IDLE;
      rw_end_trigger.wset (MM2S);
   endrule

   // This rule handles refilling the FIFO when there is enoug space in it
   // for a burst read.
   rule rl_mm2s_refill_fifo (fifo_data.notFull
                            && rg_state == COPYING
                            && state_outer == DMA_RUNNING_MM2S_COPY
                            && crg_dir[0] == MM2S
                            && rg_buf_cur < rg_buf_len // ie amt_buf_left > 0
                            && !rg_txion_in_flight
                            && ugshim_slave.ar.canPut);
      Bit #(3) len = fn_arlen_from_4k_boundary (rg_addr_next_byte);
      if (fn_crosses_4k_boundary (rg_addr_next_byte, len)) begin
         // if we get here, then it means that fn_crosses_4k_boundary thinks that our address will
         // cross a 4k boundary.
         // this should never happen; if it does, then it means that either fn_crosses_4k_boundary
         // or fn_arlen_from_4k_boundary is incorrect
         $display ("DMA Copy Unit: ERROR: functions disagree about whether this is a safe request length");
      end

      if (len != 7) begin
         if (rg_verbosity > 0) begin
            $display ("DMA Copy Unit: arlen is not 7, because of request crossing 4K boundary");
         end
      end


      // TODO this section assumes 4byte accesses
      // (the assumption means that we remove the bottom 2 bits)
      // if the amount of bytes left to be read is smaller than the next request's lenght
      // then reduce the number of bytes to be read, in order to not read past the end of the
      // buffer
      if (amt_buf_left >> 2 < zeroExtend (len) + 1) begin
         // We need to reduce the number of words we will be fetching in order to not fetch
         // past the end of the buffer
         if (rg_verbosity > 0) begin
            $display ("DMA Copy Unit: Reducing fetch length to avoid going over the buffer size");
         end
         if (rg_verbosity > 1) begin
            $display ("    original length: ", fshow (len));
         end

         if (amt_buf_left[1:0] != 0) begin
            // TODO this assumes 4byte accesses
            // The whole DMA request was not a multiple of 4 bytes
            // We need to extend the request to get the rest of the data
            // This line works because the len we want is actually one lower than the
            // number of requests we want to make.
            // Since amt_buf_left[1:0] is not 0, we know we will want to request one
            // more read than (amt_buf_left >> 2) to account for the leftover bytes
            len = truncate (amt_buf_left >> 2);
         end else begin
            // We know that there is at least 1 byte left to fetch, because that is one
            // of the conditions of this rule.
            // We know that amt_buf_left[1:0] == 0, so there must be at least one 1 in the
            // upper bits of amt_buf_left, so we cannot underflow here
            // We know that amt_buf_left >> 2 is at most 1 less than len, so len cannot
            // increase here (and so we are safe from flowing into the next 4KB boundary)
            len = truncate (amt_buf_left >> 2) - 1;
         end

         if (rg_verbosity > 1) begin
            $display ("    new length: ", fshow (len));
         end
      end

      rg_prev_arlen <= zeroExtend (len);
      rg_txion_counter <= 0;

      AXI4_ARFlit #(id_, addr_, aruser_) arflit = AXI4_ARFlit { arid:     0
                                                              , araddr:   rg_addr_next_byte
                                                              , arlen:    unpack (zeroExtend (len))
                                                              , arsize:   4 // assumes 4-byte accesses
                                                                            // TODO change this?
                                                              , arburst:  INCR
                                                              , arlock:   NORMAL
                                                              , arcache:  arcache_dev_nonbuf
                                                              , arprot:    axi4Prot(DATA, SECURE, UNPRIV)
                                                              , arqos:    0
                                                              , arregion: 0
                                                              , aruser:   0 };
      rg_word_in_upper <= rg_addr_next_byte[2] == 1;

      if (rg_verbosity > 0) begin
         $display ("DMA Copy Unit: making memory request");
         $display ("    ", fshow (arflit));
      end
      if (rg_verbosity > 0) begin
         $display ("    rg_buf_len: ", fshow (rg_buf_len));
         $display ("    rg_buf_cur: ", fshow (rg_buf_cur));
         $display ("    amt_buf_left: ", fshow (amt_buf_left));
      end
      ugshim_slave.ar.put (arflit);
      rg_txion_in_flight <= True;
   endrule

   // handle read responses when copying in the MM2S direction
   rule rl_handle_read_rsp (rg_state == COPYING
                           && state_outer == DMA_RUNNING_MM2S_COPY
                           && crg_dir[0] == MM2S
                           && ugshim_slave.r.canPeek
                           );
      ugshim_slave.r.drop;
      let rflit = ugshim_slave.r.peek;
`ifdef DMA_CHERI
      let cheri_err = rflit.rresp == SLVERR && truncateLSB (rflit.ruser) == 1'b1;
`endif

      if (rflit.rresp == SLVERR || rflit.rresp == DECERR) begin
         ugfifo_halt.enq (?);
         if (rg_verbosity > 0) begin
            $display ("DMA Copy Unit: got errored R response");
            if (rg_verbosity > 1) begin
               $display ("    flit: ", fshow (rflit));
            end
`ifdef DMA_CHERI
            if (cheri_err) begin
               if (rg_verbosity > 0) begin
                  $display ("    Errored due to CHERI capability not authorising access");
               end
               rw_enq_halt_o.wset (CHERIERR);
            end else
`endif
            if (rflit.rresp == SLVERR) begin
               if (rg_verbosity > 0) begin
                  $display ("    Errored due to SLVERR");
               end
               rw_enq_halt_o.wset (SLVERR);
            end else begin
               if (rg_verbosity > 0) begin
                  $display ("    Errored due to DECERR");
               end
               rw_enq_halt_o.wset (DECERR);
            end
         end
      end else begin
         DMA_Copy_Word data = rg_word_in_upper ? truncateLSB (rflit.rdata) : truncate (rflit.rdata);
         rg_word_in_upper <= !rg_word_in_upper;

         if (rg_verbosity > 0) begin
            $display ("DMA copy unit got response: ", fshow (rflit));
         end
         if (rg_verbosity > 0) begin
            $display ("    enqueueing data: ", fshow (data));
            $display ("    rg_buf_len: ", fshow (rg_buf_len));
            $display ("    rg_buf_cur: ", fshow (rg_buf_cur));
         end
         fifo_data.enq (data);

         // TODO this assumes 32bit transfers
         rg_addr_next_byte <= rg_addr_next_byte + 4;
         rg_buf_cur <= rg_buf_cur + 4;
         rg_txion_counter <= rg_txion_counter + 1;

         if (rflit.rlast) begin
            if (rg_txion_counter != rg_prev_arlen) begin
               $display ("DMA Copy Unit: ERROR: did not receive enough responses from memory");
            end else begin
               $display ("DMA Copy Unit: received enough read responses from memory");
            end
            rg_txion_in_flight <= False;
            // TODO this assumes 32bit transfers
            if (rg_buf_cur >= rg_buf_len-4) begin
               rg_state <= WAIT_FOR_FINAL_DEQ;
            end
         end
      end
   endrule

   rule rl_handle_halt (ugfifo_halt.notEmpty);
      if (rg_verbosity > 1) begin
         $display ("Copy Unit rl_handle_halt");
      end
      case (rg_state)
         // not sure that this should happen in the reset state as well
         RESET, IDLE, HALTED: begin
            ugfifo_halt.deq;
            rg_state <= HALTED;
            if (rg_verbosity > 0) begin
               $display ("DMA Copy Unit halt process finished, going to HALTED state");
            end
         end
         META_RECEIVE_S2MM: begin
            if (rg_verbosity > 1) begin
               $display ("    in META_RECEIVE_S2MM");
            end
            if (axi4s_m_meta_ugshim_master.canPeek) begin
               axi4s_m_meta_ugshim_master.drop;
               if (rg_verbosity > 1) begin
                  $display ("DMA Copy Unit dropped s2mm meta flit: ", fshow (axi4s_m_meta_ugshim_master.peek));
               end
               if (axi4s_m_meta_ugshim_master.peek.tlast) begin
                  rg_state <= COPYING;
                  crg_dir[0] <= S2MM;
                  if (rg_verbosity > 1) begin
                     $display ("   finished dropping s2mm meta flits.");
                  end
               end
            end
         end
         META_SEND_MM2S: begin
            if (rg_verbosity > 1) begin
               $display ("    in META_SEND_MM2S");
            end
            let is_last = rg_meta_counter == 4;
            AXI4Stream_Flit #(sid_, sdata_, sdest_, suser_) flit = AXI4Stream_Flit {
               tdata: zeroExtend (v_v_rg_bd[pack (MM2S)][pack (DMA_APP0) + zeroExtend (rg_meta_counter)].word),
               tstrb: 0,
               tkeep: ~0,
               tlast: is_last,
               tid:   0,
               tdest: 0,
               tuser: 0
            };
            axi4s_s_meta_ugshim_slave.put (flit);
            rg_meta_counter <= rg_meta_counter + 1;
            if (rg_verbosity > 1) begin
               $display ("    produced rflit: ", fshow (flit));
            end
            if (is_last) begin
               rg_state <= COPYING;
               if (rg_verbosity > 1) begin
                  $display ("    moving to COPYING state");
               end
            end
         end
         WAIT_FOR_FINAL_DEQ: begin
            if (rg_verbosity > 1) begin
               $display ("    in WAIT_FOR_FINAL_DEQ");
            end
            if (crg_dir[0] == S2MM) begin
               if (rg_txion_in_flight) begin
                  // we need to finish providing w flits
                  if (ugshim_slave.w.canPut) begin
                     rg_txion_counter <= rg_txion_counter + 1;
                     let islast = rg_prev_arlen == rg_txion_counter;
                     AXI4_WFlit #(data_, wuser_) dummy_wflit = AXI4_WFlit {
                        wdata : ?,
                        wstrb : 0,
                        wlast : islast,
                        wuser : 0
                     };
                     ugshim_slave.w.put (dummy_wflit);
                     if (rg_verbosity > 1) begin
                        $display ("    producing wflit: ", fshow (dummy_wflit));
                     end
                     if (islast) begin
                        rg_txion_in_flight <= False;
                        rg_bresp_required <= True;
                        fifo_data.clear;
                        if (rg_verbosity > 1) begin
                           $display ("    finished current w txion");
                        end
                     end
                  end
               end else if (rg_bresp_required) begin
                  // this is the last bresp, so we can just wait for it and halt
                  if (ugshim_slave.b.canPeek) begin
                     ugshim_slave.b.drop;
                     rg_state <= HALTED;
                     ugfifo_halt.deq;
                     if (rg_verbosity > 1) begin
                        $display ("    dropped b flit: ", fshow (ugshim_slave.b.peek));
                        $display ("    going to HALTED");
                     end
                  end
               end else begin
                  // If we had not halted, we would be about to make the final
                  // write request into memory; we can just not perform this
                  // write and halt, since there is nothing more to get from
                  // the stream
                  rg_state <= HALTED;
                  ugfifo_halt.deq;
                  fifo_data.clear;
                  if (rg_verbosity > 1) begin
                     $display ("    not producing aw flit");
                     $display ("    going to HALTED");
                  end
               end
            end else begin
               fifo_data.clear;
               if (rg_stream_out_count < rg_buf_len && axi4s_s_data_ugshim_slave.canPut) begin
                  AXI4Stream_Flit #(sid_, sdata_, sdest_, suser_) dummy_flit = AXI4Stream_Flit {
                     tdata: ?,
                     tstrb: 0,
                     tkeep: ~0,
                     tlast: True,
                     tid: 0,
                     tdest: 0,
                     tuser: 0
                  };
                  axi4s_s_data_ugshim_slave.put (dummy_flit);
                  rg_state <= HALTED;
                  ugfifo_halt.deq;
                  if (rg_verbosity > 1) begin
                     $display ("    produced stream flit: ", fshow (dummy_flit));
                     $display ("    going to HALTED");
                  end
               end else if (rg_stream_out_count >= rg_buf_len) begin
                  rg_state <= HALTED;
                  ugfifo_halt.deq;
                  if (rg_verbosity > 1) begin
                     $display ("    not producing stream flit");
                     $display ("    going to HALTED");
                  end
               end
            end
         end
         COPYING: begin
            if (rg_verbosity > 1) begin
               $display ("    in COPYING");
            end
            if (crg_dir[0] == S2MM) begin
               // keep dequeueing until there's nothing left. then let
               // WAIT_FOR_FINAL_DEQ to clean up
               if (axi4s_m_data_ugshim_master.canPeek) begin
                  axi4s_m_data_ugshim_master.drop;
                  if (rg_verbosity > 1) begin
                     $display ("    dropping data stream flit: ", fshow (axi4s_m_data_ugshim_master.peek));
                  end
                  if (axi4s_m_data_ugshim_master.peek.tlast) begin
                     if (rg_verbosity > 1) begin
                        $display ("    going to WAIT_FOR_FINAL_DEQ");
                     end
                     rg_state <= WAIT_FOR_FINAL_DEQ;
                  end
               end
            end else begin
               if (rg_stream_out_count < rg_buf_len && axi4s_s_data_ugshim_slave.canPut) begin
                  // need to send last flit to stream
                  fifo_data.clear;
                  AXI4Stream_Flit #(sid_, sdata_, sdest_, suser_) dummy_flit = AXI4Stream_Flit {
                     tdata: ?,
                     tstrb: 0,
                     tkeep: ~0,
                     tlast: True,
                     tid: 0,
                     tdest: 0,
                     tuser: 0
                  };
                  axi4s_s_data_ugshim_slave.put (dummy_flit);
                  rg_stream_out_count <= rg_buf_len;
                  if (rg_verbosity > 1) begin
                     $display ("    producing stream data flit: ", fshow (dummy_flit));
                  end
               end
               if (rg_txion_in_flight && ugshim_slave.r.canPeek) begin
                  // need to clear all remaining read responses
                  ugshim_slave.r.drop;
                  if (rg_verbosity > 1) begin
                     $display ("    dropping r flit: ", fshow (ugshim_slave.r.peek));
                  end
                  if (ugshim_slave.r.peek.rlast) begin
                     rg_txion_in_flight <= False;
                     if (rg_verbosity > 1) begin
                        $display ("    setting rg_txion_in_flight to false");
                     end
                  end
               end
               if (!rg_txion_in_flight && rg_stream_out_count >= rg_buf_len) begin
                  rg_state <= HALTED;
                  ugfifo_halt.deq;
                  if (rg_verbosity > 1) begin
                     $display ("    goinng to HALTED");
                  end
               end
            end
         end
      endcase
   endrule


   rule rl_debug (rg_verbosity > 2);
      $display ("DMA Copy Unit debug info:");
      $display ("   rg_state: ", fshow (rg_state));
      $display ("   crg_dir[0]: ", fshow (crg_dir[0]));
      $display ("   fifo_data.notFull: ", fshow (fifo_data.notFull));
      $display ("   fifo_data.notEmpty: ", fshow (fifo_data.notEmpty));
      $display ("   fifo_data.count: ", fshow (fifo_data.count));
      $display ("   axi4s_m_data_ugshim_master.canPeek: ", fshow (axi4s_m_data_ugshim_master.canPeek));
      $display ("   rg_buf_len: ", fshow (rg_buf_len));
      $display ("   rg_buf_cur: ", fshow (rg_buf_cur));
   endrule

   rule rl_debug_fifo_not_empty_while_idle (rg_state == IDLE
                                           && fifo_data.notEmpty
                                           );
      $display ("DMA Copy Unit: ERROR: FIFOFs not empty when idle");
      $display ("   fifo_data.notEmpty: ", fshow (fifo_data.notEmpty));
      $display ("   fifo_data.count: ", fshow (fifo_data.count));
   endrule

   rule rl_debug_cannot_enqueue (ugshim_slave.r.canPeek && !fifo_data.notFull);
      $display ("DMA Copy Unit: ERROR: we have received data from memory but can't enqueue it");
      $display ("                      Something went wrong the last time we made a request; we");
      $display ("                      thought we would have enough space for the data but we didn't");
   endrule

   // detect bresp when not in a correct state
   rule rl_detect_incorrect_bresp (ugshim_slave.b.canPeek
                                   && (rg_state == IDLE
                                       || rg_state == RESET
                                       || !rg_bresp_required));
      $display ("DMA Copy Unit: ERROR: got bresp in incorrect state");
   endrule

   Reg #(Bool) rg_mm2s_throughput <- mkReg (False);
   rule rl_debug_mm2s_throughput_start (axi4s_s_data_shim.master.canPeek
                                        && !rg_mm2s_throughput
                                        && rg_verbosity > 1);
      $display ("%m tock 1");
      rg_mm2s_throughput <= True;
   endrule
   rule rl_debug_mm2s_throughput_print (rg_mm2s_throughput
                                        && rg_verbosity > 1);
      $display ("%m tock");
      $display ("    axi4s_s_data_ugshim_slave.canPut: ", fshow (axi4s_s_data_ugshim_slave.canPut));
      $display ("    fifo_data.notFull: ", fshow (fifo_data.notFull));
      $display ("    fifo_data.notEmpty: ", fshow (fifo_data.notEmpty));
      $display ("    ugshim_slave.r.canPeek: ", fshow (ugshim_slave.r.canPeek));
      $display ("    rg_state: ", fshow (rg_state));
   endrule
   rule rl_debug_mm2s_throughput_end (axi4s_s_data_shim.master.canPeek
                                      && axi4s_s_data_shim.master.peek.tlast
                                      && rg_verbosity > 1);
      rg_mm2s_throughput <= False;
   endrule


   Reg #(Bool) rg_s2mm_throughput <- mkReg (False);
   rule rl_debug_s2mm_throughput_start (axi4s_m_data_ugshim_master.canPeek
                                        && !rg_s2mm_throughput
                                        && rg_verbosity > 1);
      $display ("%m tick 1");
      rg_s2mm_throughput <= True;
   endrule
   rule rl_debug_s2mm_throughput_print (rg_s2mm_throughput
                                        && rg_verbosity > 1);
      $display ("%m tick");
      $display ("    axi4s_m_meta_ugshim_master.canPeek: ", fshow (axi4s_m_meta_ugshim_master.canPeek));
      $display ("    axi4s_m_data_ugshim_master.canPeek: ", fshow (axi4s_m_data_ugshim_master.canPeek));
      $display ("    fifo_data.notFull: ", fshow (fifo_data.notFull));
      $display ("    fifo_data.notEmpty: ", fshow (fifo_data.notEmpty));
      $display ("    ugshim_slave.w.canPut: ", fshow (ugshim_slave.w.canPut));
      $display ("    rg_state: ", fshow (rg_state));
      $display ("    rg_meta_counter: ", fshow (rg_meta_counter));
   endrule
   rule rl_debug_s2mm_throughput_end (axi4s_m_data_ugshim_master.canPeek
                                      && axi4s_m_data_ugshim_master.peek.tlast
                                      && rg_verbosity > 1);
      rg_s2mm_throughput <= False;
   endrule


   interface axi4_master = shim.master;

   interface axi4s_data_master = axi4s_s_data_shim.master;
   interface axi4s_meta_master = axi4s_s_meta_shim.master;

   interface axi4s_data_slave = axi4s_m_data_shim.slave;
   interface axi4s_meta_slave = axi4s_m_meta_shim.slave;

   // Trigger a MM2S copy using the values currently in the v_v_rg_bd register bank
   method Action trigger if (rg_state == IDLE
                             && dma_int_reg.mm2s_dmasr.halted == 1'b0);
      if (rg_verbosity > 0) begin
         $display ("DMA Copy Unit received MM2S trigger");
      end
      crg_dir[0] <= MM2S;
      rg_addr_next_byte
`ifdef DMA_CHERI
                        <= zeroExtend (pack (v_v_rg_bd[pack (MM2S)][pack (DMA_BUFFER_ADDRESS_0)].word));
`else
                        <= zeroExtend (pack (v_v_rg_bd[pack (MM2S)][pack (DMA_BUFFER_ADDRESS)].word));
`endif
      rg_buf_cur <= 0;
      rg_buf_len <= zeroExtend ((pack (v_v_rg_bd[pack (MM2S)][pack (DMA_CONTROL)].word))[25:0]);
      rg_stream_out_count <= 0;
      rg_state <= META_SEND_MM2S;

      rg_meta_counter <= 0;
      Bit #(32) flag_b32 = {4'ha, 0};
      // Start sending the stream metadata immediately
      // The first flit just contains the flag and nothing else
      AXI4Stream_Flit #(sid_, sdata_, sdest_, suser_) flit = AXI4Stream_Flit {
         tdata: zeroExtend (flag_b32),
         tstrb: ~0,
         tkeep: ~0,
         tlast: False,
         tid:   0,
         tdest: 0,
         tuser: 0
      };
      if (axi4s_s_meta_ugshim_slave.canPut) begin
         axi4s_s_meta_ugshim_slave.put (flit);
      end
   endmethod

   method Bool s2mm_start_trigger = dw_s2mm_start_trigger;

   method Maybe #(DMA_Dir) end_trigger = rw_end_trigger.wget;

   method DMA_Dir current_dir = crg_dir[1];

   method Action notify_bd;
      rg_bd_available <= True;
   endmethod

   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
   endmethod

   method Action reset;
      rg_state <= RESET;
   endmethod

   method Bool reset_done;
      return rg_state != RESET;
   endmethod

   method Action halt_to_idle if (rg_state == HALTED);
      rg_state <= IDLE;
      if (rg_verbosity > 1) begin
         $display ("copy unit halt_to_idle");
         $display ("    old state: ", fshow (rg_state));
      end
   endmethod

   interface Server srv_halt;
      interface Put request;
         method Action put (Bit #(0) none) if (ugfifo_halt.notFull);
            if (rg_verbosity > 0) begin
               $display ("DMA Copy Unit Halt request received");
            end
            ugfifo_halt.enq (?);
         endmethod
      endinterface
      interface Get response;
         method ActionValue #(Bit #(0)) get if (!ugfifo_halt.notEmpty);
            if (rg_verbosity > 0) begin
               $display ("DMA Copy Unit Halt response sent");
            end
            return (?);
         endmethod
      endinterface
   endinterface

   method Maybe #(DMA_Err_Cause) enq_halt_o = rw_enq_halt_o.wget;

endmodule



endpackage
