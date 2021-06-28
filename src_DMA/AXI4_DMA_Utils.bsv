package AXI4_DMA_Utils;

// Bluespec imports
import FIFOF :: *;

// AXI imports
import AXI :: *;
import AXI4Stream_Types :: *;
import AXI4Stream_Utils :: *;
import SourceSink :: *;

// local imports
import AXI4_DMA_Types :: *;

interface AXI4Stream_Data_Source #(numeric type strm_id_
                             ,numeric type sdata_
                             ,numeric type sdest_
                             ,numeric type suser_);
   interface AXI4Stream_Master #(strm_id_, sdata_, sdest_, suser_) axi4s_master;
   method Action set_verbosity (Bit #(4) new_verb);
endinterface
module mkAXI4Stream_Data_Source (AXI4Stream_Data_Source #(strm_id_, sdata_, sdest_, suser_))
            provisos ( Add#(a__, 4, TDiv#(sdata_, 8))
                     , Add#(b__, 32, sdata_)
                     );
   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);
   AXI4Stream_Shim #(strm_id_, sdata_, sdest_, suser_) shim <- mkAXI4StreamShimFF1;
   Reg #(Bit #(32)) rg_counter <- mkReg (0);
   Reg #(Bit #(32)) rg_tx_count <- mkReg (0);
   Reg #(Bit #(32)) rg_tx_count_max <- mkReg (3);
   Reg #(Bool) rg_txfer_started <- mkReg (False);
   Reg #(Bool) rg_txfer_end_trigger <- mkReg (False);
   AXI4Stream_Flit #(strm_id_, sdata_, sdest_, suser_) base_flit = AXI4Stream_Flit {
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
      $display ("   shim.slave.canPut: ", fshow (shim.slave.canPut));
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
      shim.slave.put (base_flit);
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
      let last_flit = base_flit;
      last_flit.tlast = True;
      shim.slave.put (last_flit);
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
   interface AXI4Stream_Master #(strm_id_, sdata_, sdest_, suser_) axi4s_data_master;
   interface AXI4Stream_Master #(strm_id_, sdata_, sdest_, suser_) axi4s_meta_master;
   interface AXI4Stream_Slave #(strm_id_, sdata_, sdest_, suser_) axi4s_data_slave;
   interface AXI4Stream_Slave #(strm_id_, sdata_, sdest_, suser_) axi4s_meta_slave;
   method Action set_verbosity (Bit #(4) new_verb);
   method Action reset;
endinterface
module mkAXI4_Stream_Delay_Loopback (AXI4_Stream_Delay_Loopback_IFC #(strm_id_, sdata_, sdest_, suser_));
   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);
   FIFOF #(Bit #(sdata_)) data_fifo <- mkUGSizedFIFOF (1024);
   FIFOF #(Bit #(sdata_)) meta_fifo <- mkUGSizedFIFOF (8);
   Reg #(Bool) rg_meta_input_started <- mkReg (False);
   Reg #(Bool) rg_meta_output_started <- mkReg (False);
   Reg #(Bool) rg_input_started <- mkReg (False);
   Reg #(Bool) rg_input_finished <- mkReg (False);
   Reg #(Bool) rg_output_trigger <- mkReg (False);
   Reg #(Bit #(32)) rg_tick_counter <- mkReg (0);
   Reg #(Bit #(32)) rg_txfer_counter <- mkReg (0);
   Reg #(Bit #(3)) rg_meta_counter <- mkReg (0);
   AXI4Stream_Shim #(strm_id_, sdata_, sdest_, suser_) axi4s_m_data_shim <- mkAXI4StreamShimFF;
   AXI4Stream_Shim #(strm_id_, sdata_, sdest_, suser_) axi4s_m_meta_shim <- mkAXI4StreamShimFF;
   AXI4Stream_Shim #(strm_id_, sdata_, sdest_, suser_) axi4s_s_data_shim <- mkAXI4StreamShimFF;
   AXI4Stream_Shim #(strm_id_, sdata_, sdest_, suser_) axi4s_s_meta_shim <- mkAXI4StreamShimFF;
   let axi4s_m_data_shim_master <- toUnguarded_AXI4Stream_Master (axi4s_m_data_shim.master);
   let axi4s_m_meta_shim_master <- toUnguarded_AXI4Stream_Master (axi4s_m_meta_shim.master);
   let axi4s_s_data_shim_slave <- toUnguarded_AXI4Stream_Slave (axi4s_s_data_shim.slave);
   let axi4s_s_meta_shim_slave <- toUnguarded_AXI4Stream_Slave (axi4s_s_meta_shim.slave);
   rule rl_debug (rg_verbosity > 2);
      $display ("AXI DMA Stream Delay Loopback rl_debug");
      $display ("    rg_input_started: ", fshow (rg_input_started));
      $display ("    rg_input_finished: ", fshow (rg_input_finished));
      $display ("    rg_output_trigger: ", fshow (rg_output_trigger));
      $display ("    rg_meta_input_started: ", fshow (rg_meta_input_started));
      $display ("    rg_meta_output_started: ", fshow (rg_meta_output_started));
      $display ("    axi4s_s_meta_shim_slave.canPut: ", fshow (axi4s_s_meta_shim_slave.canPut));
      $display ("    data_fifo.notEmpty: ", fshow (data_fifo.notEmpty));
      $display ("    meta_fifo.notEmpty: ", fshow (meta_fifo.notEmpty));
   endrule
   rule rl_get_input_meta (axi4s_m_meta_shim_master.canPeek
                           && meta_fifo.notFull);
      axi4s_m_meta_shim_master.drop;
      if (!rg_meta_input_started) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback started receiving metadata");
         end
         rg_meta_input_started <= True;
      end
      if (rg_verbosity > 0) begin
         $display ("AXI4 Stream Delay Loopback metadata: ", fshow (axi4s_m_meta_shim_master.peek));
      end
      meta_fifo.enq (axi4s_m_meta_shim_master.peek.tdata);
      rg_meta_counter <= rg_meta_counter + 1;
   endrule
   rule rl_get_input_data (axi4s_m_data_shim_master.canPeek
                           && data_fifo.notFull
                           && !rg_input_finished);
      if (!rg_input_started) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback started receiving");
         end
         rg_input_started <= True;
      end
      data_fifo.enq (axi4s_m_data_shim_master.peek.tdata);
      axi4s_m_data_shim_master.drop;
      if (axi4s_m_data_shim_master.peek.tlast) begin
         rg_input_finished <= True;
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback finished receiving");
         end
      end
      rg_txfer_counter <= rg_txfer_counter + 1;
   endrule

   rule rl_incr_tick (rg_input_started && rg_input_finished && !rg_output_trigger);
      rg_tick_counter <= rg_tick_counter + 1;
      if (rg_tick_counter > 300) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback: delay finished");
         end
         rg_output_trigger <= True;
      end
   endrule

   rule rl_push_output_meta (rg_input_started
                             && rg_input_finished
                             && rg_output_trigger
                             && axi4s_s_meta_shim_slave.canPut
                             && meta_fifo.notEmpty);
      if (!rg_meta_output_started) begin
         if (rg_verbosity > 0) begin
            $display ("AXI4 Stream Delay Loopback started sending metadata");
         end
         rg_meta_output_started <= True;
      end

      AXI4Stream_Flit #(strm_id_, sdata_, sdest_, suser_) flit = AXI4Stream_Flit {
         tdata: meta_fifo.first,
         tstrb: ~0,
         tkeep: ~0,
         tlast: rg_meta_counter == 1,
         tid: 0,
         tdest: 0,
         tuser: 0
      };
      if (rg_verbosity > 1) begin
         $display ("Stream delay loopback flit");
         $display ("    flit: ", fshow (flit));
      end
      if (rg_meta_counter == 1) begin
         $display ("AXI4 Stream Delay Loopback finished sending metadata");
      end
      axi4s_s_meta_shim_slave.put (flit);
      rg_meta_counter <= rg_meta_counter - 1;
      meta_fifo.deq;
   endrule
   rule rl_push_output_data (rg_input_started
                             && rg_input_finished
                             && rg_output_trigger
                             && !meta_fifo.notEmpty
                             && axi4s_s_data_shim_slave.canPut
                             && data_fifo.notEmpty);
      data_fifo.deq;
      AXI4Stream_Flit #(strm_id_, sdata_, sdest_, suser_) flit = AXI4Stream_Flit {
         tdata: data_fifo.first,
         tstrb: ~0,
         tkeep: ~0,
         tlast: rg_txfer_counter == 1,
         tid: 0,
         tdest: 0,
         tuser: 0
      };
      axi4s_s_data_shim_slave.put (flit);
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

   method Action reset;
      axi4s_m_meta_shim.clear;
      axi4s_m_data_shim.clear;
      axi4s_s_meta_shim.clear;
      axi4s_s_data_shim.clear;
   endmethod
endmodule

function DMA_BD_TagWord fn_to_untagged_tagword (DMA_BD_Word word)
   = DMA_BD_TagWord { word: word, tag: False };

function AXI4_Master #(id_, addr_, data_,
                       awuser_o_, wuser_, buser_,
                       aruser_o_, ruser_) fn_extend_ar_aw_user_fields (AXI4_Master #(id_, addr_, data_,
                                                                                     awuser_i_, wuser_, buser_,
                                                                                     aruser_i_, ruser_) m,
                                                                       Bit #(n_) val)
   provisos ( Add #(n_, awuser_i_, awuser_o_)
            , Add #(n_, aruser_i_, aruser_o_));
   return interface AXI4_Master;
      interface Source aw;
         method drop = m.aw.drop;
         method canPeek = m.aw.canPeek;
         method peek;
            let x = m.aw.peek;
            return AXI4_AWFlit { awid:     x.awid
                               , awaddr:   x.awaddr
                               , awlen:    x.awlen
                               , awsize:   x.awsize
                               , awburst:  x.awburst
                               , awlock:   x.awlock
                               , awcache:  x.awcache
                               , awprot:   x.awprot
                               , awqos:    x.awqos
                               , awregion: x.awregion
                               , awuser:   {val, x.awuser}};
         endmethod
      endinterface
      interface Source w = m.w;
      interface Sink b = m.b;
      interface Source ar;
         method drop = m.ar.drop;
         method canPeek = m.ar.canPeek;
         method peek;
            let x = m.ar.peek;
            return AXI4_ARFlit { arid:     x.arid
                               , araddr:   x.araddr
                               , arlen:    x.arlen
                               , arsize:   x.arsize
                               , arburst:  x.arburst
                               , arlock:   x.arlock
                               , arcache:  x.arcache
                               , arprot:   x.arprot
                               , arqos:    x.arqos
                               , arregion: x.arregion
                               , aruser:   {val, x.aruser}};
         endmethod
      endinterface
      interface Sink r = m.r;
   endinterface;
endfunction

function AXI4_Master #(id_, addr_, data_,
                       awuser_o_, wuser_, buser_,
                       aruser_o_, ruser_) fn_truncate_ar_aw_user_fields (AXI4_Master #(id_, addr_, data_,
                                                                                       awuser_i_, wuser_, buser_,
                                                                                       aruser_i_, ruser_) m)
   provisos ( Add #(a_, awuser_o_, awuser_i_)
            , Add #(b_, aruser_o_, aruser_i_));
   return interface AXI4_Master;
      interface Source aw;
         method drop = m.aw.drop;
         method canPeek = m.aw.canPeek;
         method peek;
            let x = m.aw.peek;
            return AXI4_AWFlit { awid:     x.awid
                               , awaddr:   x.awaddr
                               , awlen:    x.awlen
                               , awsize:   x.awsize
                               , awburst:  x.awburst
                               , awlock:   x.awlock
                               , awcache:  x.awcache
                               , awprot:   x.awprot
                               , awqos:    x.awqos
                               , awregion: x.awregion
                               , awuser:   truncate (x.awuser)};
         endmethod
      endinterface
      interface Source w = m.w;
      interface Sink b = m.b;
      interface Source ar;
         method drop = m.ar.drop;
         method canPeek = m.ar.canPeek;
         method peek;
            let x = m.ar.peek;
            return AXI4_ARFlit { arid:     x.arid
                               , araddr:   x.araddr
                               , arlen:    x.arlen
                               , arsize:   x.arsize
                               , arburst:  x.arburst
                               , arlock:   x.arlock
                               , arcache:  x.arcache
                               , arprot:   x.arprot
                               , arqos:    x.arqos
                               , arregion: x.arregion
                               , aruser:   truncate (x.aruser)};
         endmethod
      endinterface
      interface Sink r = m.r;
   endinterface;
endfunction
endpackage
