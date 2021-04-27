package AXI4_DMA_Utils;

// Bluespec imports
import FIFOF :: *;

// AXI imports
import AXI :: *;
import AXI4_Stream_Types :: *;
import AXI4S_Utils :: *;
import SourceSink :: *;

interface AXI4S_Data_Source #(numeric type strm_id_
                             ,numeric type sdata_
                             ,numeric type sdest_
                             ,numeric type suser_);
   interface AXI4S_Master #(strm_id_, sdata_, sdest_, suser_) axi4s_master;
   method Action set_verbosity (Bit #(4) new_verb);
endinterface
module mkAXI4S_Data_Source (AXI4S_Data_Source #(strm_id_, sdata_, sdest_, suser_))
            provisos ( Add#(a__, 4, TDiv#(sdata_, 8))
                     , Add#(b__, 32, sdata_)
                     );
   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);
   AXI4S_Shim #(strm_id_, sdata_, sdest_, suser_) shim <- mkAXI4SShimFF1;
   Reg #(Bit #(32)) rg_counter <- mkReg (0);
   Reg #(Bit #(32)) rg_tx_count <- mkReg (0);
   Reg #(Bit #(32)) rg_tx_count_max <- mkReg (3);
   Reg #(Bool) rg_txfer_started <- mkReg (False);
   Reg #(Bool) rg_txfer_end_trigger <- mkReg (False);
   AXI4S_TFlit #(strm_id_, sdata_, sdest_, suser_) base_tflit = AXI4S_TFlit {
      tdata: zeroExtend (32'hcafecafe),
      tstrb: zeroExtend (4'hf),
      tkeep: zeroExtend (4'hf),
      tlast: False,
      tid  : 0,
      tdest: 0,
      tuser: 0
   };
   rule rl_debug (rg_verbosity > 3);
      $display ("AXI4 Data Source");
      $display ("   rg_counter: ", fshow (rg_counter));
      $display ("   rg_tx_count: ", fshow (rg_tx_count));
      $display ("   rg_tx_count_max: ", fshow (rg_tx_count_max));
      $display ("   rg_txfer_started: ", fshow (rg_txfer_started));
      $display ("   rg_txfer_end_trigger: ", fshow (rg_txfer_end_trigger));
      $display ("   shim.slave.t.canPut: ", fshow (shim.slave.t.canPut));
   endrule
   rule rl_incr_counter;
      rg_counter <= rg_counter + 1;
   endrule
   rule rl_txfer_loop (rg_counter != 0
                      && (rg_counter[11:0] == 12'h800 || rg_txfer_started)
                      && !rg_txfer_end_trigger);
      if (!rg_txfer_started) begin
         if (rg_verbosity > 0) begin
            $display ("DMA S2MM transfer started");
         end
      end else begin
         if (rg_verbosity > 1) begin
            $display ("DMA S2MM transfer continuing");
         end
      end
      shim.slave.t.put (base_tflit);
      rg_txfer_started <= True;
      rg_tx_count <= rg_tx_count + 1;
   endrule
   rule rl_txfer_trigger_end (rg_txfer_started
                              && !rg_txfer_end_trigger
                              && rg_tx_count >= rg_tx_count_max);
      rg_txfer_end_trigger <= True;
   endrule
   rule rl_txfer_end (rg_txfer_started && rg_txfer_end_trigger);
      if (rg_verbosity > 0) begin
         $display ("DMA S2MM transfer ended");
      end
      let last_tflit = base_tflit;
      last_tflit.tlast = True;
      shim.slave.t.put (last_tflit);
      rg_txfer_started <= False;
      rg_txfer_end_trigger <= False;
      rg_tx_count <= 0;
      rg_tx_count_max <= rg_tx_count_max + 1;
   endrule
   interface axi4s_master = shim.master;
   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
   endmethod
endmodule


interface AXI4_Stream_Delay_Loopback_IFC #(numeric type strm_id_,
                                           numeric type sdata_,
                                           numeric type sdest_,
                                           numeric type suser_);
   interface AXI4S_Master #(strm_id_, sdata_, sdest_, suser_) axi4s_data_master;
   interface AXI4S_Master #(strm_id_, sdata_, sdest_, suser_) axi4s_meta_master;
   interface AXI4S_Slave #(strm_id_, sdata_, sdest_, suser_) axi4s_data_slave;
   interface AXI4S_Slave #(strm_id_, sdata_, sdest_, suser_) axi4s_meta_slave;
   method Action set_verbosity (Bit #(4) new_verb);
endinterface
module mkAXI4_Stream_Delay_Loopback (AXI4_Stream_Delay_Loopback_IFC #(strm_id_, sdata_, sdest_, suser_));
   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);
   FIFOF #(Bit #(sdata_)) data_fifo <- mkSizedFIFOF (1024);
   FIFOF #(Bit #(sdata_)) meta_fifo <- mkSizedFIFOF (8);
   Reg #(Bool) rg_meta_input_started <- mkReg (False);
   Reg #(Bool) rg_meta_output_started <- mkReg (False);
   Reg #(Bool) rg_input_started <- mkReg (False);
   Reg #(Bool) rg_input_finished <- mkReg (False);
   Reg #(Bool) rg_output_trigger <- mkReg (False);
   Reg #(Bit #(32)) rg_tick_counter <- mkReg (0);
   Reg #(Bit #(32)) rg_txfer_counter <- mkReg (0);
   Reg #(Bit #(3)) rg_meta_counter <- mkReg (0);
   AXI4S_Shim #(strm_id_, sdata_, sdest_, suser_) axi4s_m_data_shim <- mkAXI4SShimFF1;
   AXI4S_Shim #(strm_id_, sdata_, sdest_, suser_) axi4s_m_meta_shim <- mkAXI4SShimFF1;
   AXI4S_Shim #(strm_id_, sdata_, sdest_, suser_) axi4s_s_data_shim <- mkAXI4SShimFF1;
   AXI4S_Shim #(strm_id_, sdata_, sdest_, suser_) axi4s_s_meta_shim <- mkAXI4SShimFF1;
   rule rl_debug (rg_verbosity > 2);
      $display ("AXI DMA Stream Delay Loopback rl_debug");
      $display ("    rg_input_started: ", fshow (rg_input_started));
      $display ("    rg_input_finished: ", fshow (rg_input_finished));
      $display ("    rg_output_trigger: ", fshow (rg_output_trigger));
      $display ("    rg_meta_input_started: ", fshow (rg_meta_input_started));
      $display ("    rg_meta_output_started: ", fshow (rg_meta_output_started));
      $display ("    axi4s_s_meta_shim.slave.t.canPut: ", fshow (axi4s_s_meta_shim.slave.t.canPut));
      $display ("    data_fifo.notEmpty: ", fshow (data_fifo.notEmpty));
      $display ("    meta_fifo.notEmpty: ", fshow (data_fifo.notEmpty));
   endrule
   rule rl_get_input_meta (axi4s_m_meta_shim.master.t.canPeek);
      axi4s_m_meta_shim.master.t.drop;
      if (!rg_meta_input_started) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback started receiving metadata");
         end
         rg_meta_input_started <= True;
      end
      if (rg_verbosity > 0) begin
         $display ("AXI4 Stream Delay Loopback metadata: ", fshow (axi4s_m_meta_shim.master.t.peek.tdata));
      end
      meta_fifo.enq (axi4s_m_meta_shim.master.t.peek.tdata);
      rg_meta_counter <= rg_meta_counter + 1;
   endrule
   rule rl_get_input_data (axi4s_m_data_shim.master.t.canPeek
                           && !rg_input_finished);
      if (!rg_input_started) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback started receiving");
         end
         rg_input_started <= True;
      end
      data_fifo.enq (axi4s_m_data_shim.master.t.peek.tdata);
      axi4s_m_data_shim.master.t.drop;
      if (axi4s_m_data_shim.master.t.peek.tlast) begin
         rg_input_finished <= True;
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback finished receiving");
         end
      end
      rg_txfer_counter <= rg_txfer_counter + 1;
   endrule

   rule rl_incr_tick (rg_input_started && rg_input_finished && !rg_output_trigger);
      rg_tick_counter <= rg_tick_counter + 1;
      if (rg_tick_counter > 300000) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback: delay finished");
         end
         rg_output_trigger <= True;
      end
   endrule

   rule rl_push_output_meta (rg_input_started
                             && rg_input_finished
                             && rg_output_trigger
                             && axi4s_s_meta_shim.slave.t.canPut
                             && meta_fifo.notEmpty);
      if (!rg_meta_output_started) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback started sending metadata");
         end
         rg_meta_output_started <= True;
      end

      AXI4S_TFlit #(strm_id_, sdata_, sdest_, suser_) tflit = AXI4S_TFlit {
         tdata: meta_fifo.first,
         tstrb: ~0,
         tkeep: ~0,
         tlast: rg_meta_counter == 1,
         tid: 0,
         tdest: 0,
         tuser: 0
      };
      if (rg_meta_counter == 1) begin
         $display ("AXI4 Stream Delay Loopback finished sending metadata");
      end
      axi4s_s_meta_shim.slave.t.put (tflit);
      rg_meta_counter <= rg_meta_counter - 1;
      meta_fifo.deq;
   endrule
   rule rl_push_output_data (rg_input_started
                             && rg_input_finished
                             && rg_output_trigger
                             && !meta_fifo.notEmpty
                             && axi4s_s_data_shim.slave.t.canPut
                             && data_fifo.notEmpty);
      data_fifo.deq;
      AXI4S_TFlit #(strm_id_, sdata_, sdest_, suser_) tflit = AXI4S_TFlit {
         tdata: data_fifo.first,
         tstrb: ~0,
         tkeep: ~0,
         tlast: rg_txfer_counter == 1,
         tid: 0,
         tdest: 0,
         tuser: 0
      };
      axi4s_s_data_shim.slave.t.put (tflit);
      rg_txfer_counter <= rg_txfer_counter - 1;
      if (rg_txfer_counter == 1) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay loopback finished sending");
         end
         rg_input_started <= False;
         rg_input_finished <= False;
         rg_output_trigger <= False;
         rg_meta_input_started <= False;
         rg_meta_output_started <= False;
         rg_tick_counter <= 0;
      end
   endrule

   interface axi4s_meta_slave = axi4s_m_meta_shim.slave;
   interface axi4s_data_slave = axi4s_m_data_shim.slave;
   interface axi4s_meta_master = axi4s_s_meta_shim.master;
   interface axi4s_data_master = axi4s_s_data_shim.master;

   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
   endmethod
endmodule

endpackage
