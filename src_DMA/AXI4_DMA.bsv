/*-
 * Copyright (c) 2021 Ivan Ribeiro
 * All rights reserved.
 *
 * This hardware was developed by University of Cambridge Computer Laboratory
 * (Department of Computer Science and Technology) under EPSRC award
 * EP/S030867/1 ("SIPP"); and by SRI International and the University of
 * Cambridge Computer Laboratory (Department of Computer Science and
 * Technology) under DARPA contract HR0011-18-C-0016 ("ECATS"), as part of the
 * DARPA SSITH research programme.
 *
 * @BERI_LICENSE_HEADER_START@
 *
 * Licensed to BERI Open Systems C.I.C. (BERI) under one or more contributor
 * license agreements.  See the NOTICE file distributed with this work for
 * additional information regarding copyright ownership.  BERI licenses this
 * file to you under the BERI Hardware-Software License, Version 1.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at:
 *
 *   http://www.beri-open-systems.org/legal/license-1-0.txt
 *
 * Unless required by applicable law or agreed to in writing, Work distributed
 * under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * @BERI_LICENSE_HEADER_END@
 */

package AXI4_DMA;

// Bluespec imports
import FIFOF :: *;
import Vector :: *;

// AXI imports
import AXI :: *;
import AXI4Stream_Types :: *;
import AXI4Stream_Utils :: *;
import Connectable :: *;
import SourceSink :: *;

// local imports
import AXI4_DMA_Scatter_Gather   :: *;
import AXI4_DMA_Register_Module :: *;
import AXI4_DMA_Copy_Unit :: *;
import AXI4_DMA_Types :: *;
import AXI4_DMA_Internal_Reg_Module :: *;
import AXI4_DMA_Utils :: *;

typedef enum {
   RESET,
   HALTED,
   IDLE,
   MM2S,
   S2MM
} DMA_State deriving (Bits, FShow, Eq);

interface AXI4_DMA_IFC #(numeric type mid_,  // master id
                         numeric type sid_,  // slave id
                         numeric type addr_,
                         numeric type data_,
                         numeric type awuser_,
                         numeric type wuser_,
                         numeric type buser_,
                         numeric type aruser_,
                         numeric type ruser_,
                         // AXI4 Stream sizes
                         numeric type strm_id_,
                         numeric type sdata_,
                         numeric type sdest_,
                         numeric type suser_);
   (* always_ready *) method Bool s2mm_interrupt_req;
   (* always_ready *) method Bool mm2s_interrupt_req;
   // AXI4 master used for Buffer Descriptor fetching and updating
   interface AXI4_Master #(mid_, addr_, data_,
                           awuser_, wuser_, buser_,
                           aruser_, ruser_) axi_sg_master;
   // AXI4 slave for writing and reading the DMA registers
   interface AXI4_Slave #(sid_, addr_, data_,
                          awuser_, wuser_, buser_,
                          aruser_, ruser_) axi_reg_slave;
   // AXI4 master for reading and writing the data being copied by the DMA
   interface AXI4_Master #(mid_, addr_, data_,
                           awuser_, wuser_, buser_,
                           aruser_, ruser_) axi_copy_master;

   // AXI4 Stream masters for writing data and metadata
   interface AXI4Stream_Master #(strm_id_, sdata_, sdest_, suser_) axi4s_data_master;
   interface AXI4Stream_Master #(strm_id_, sdata_, sdest_, suser_) axi4s_meta_master;

   // AXI4 Stream slaves for receiving data and metadata
   interface AXI4Stream_Slave #(strm_id_, sdata_, sdest_, suser_) axi4s_data_slave;
   interface AXI4Stream_Slave #(strm_id_, sdata_, sdest_, suser_) axi4s_meta_slave;

   method Action set_verbosity (Bit #(4) new_verb);
   method Action reset;
endinterface

module mkAXI4_DMA (AXI4_DMA_IFC #(mid_, sid_, addr_, data_,
                                  awuser_, wuser_, buser_,
                                  aruser_, ruser_,
                                  strm_id_, sdata_, sdest_, suser_))
                  provisos ( Add #(a__, SizeOf #(DMA_BD_Word), data_)
                           , Add #(b__, SizeOf #(DMA_BD_Word), addr_)
                           , Add #(c__, 4, TDiv #(data_, 8)) // See scatter-gather for info on
                                                             // these provisos
                           , Add #(d__, SizeOf #(DMA_Reg_Index), addr_)
                           , Add #(e__, 3, addr_)
                           , Add #(f__, 12, addr_)
                           , Add #(g__, 9, addr_)
                           , Add #(h__, 64, addr_)
                           , Add #(i__, 64, data_)
                           , Add #(j__, 8, addr_)
                           , Add #(k__, 26, addr_)
                           , Add #(l__, 10, addr_)
                           , Add #(0, 64, data_)
                           , Add #(0, SizeOf #(DMA_Copy_Word), sdata_) // TODO remove this?
                           , Add #(m__, 4, TDiv#(sdata_, 8))
                           , Add #(n__, 32, sdata_)
                           , Add #(o__, addr_, 64)
                           );

   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);
   Reg #(DMA_State) rg_state <- mkReg (HALTED);

   Reg #(Bool) rg_mm2s_first_fetch <- mkReg (True);
   Reg #(Bool) rg_s2mm_first_fetch <- mkReg (True);
   Reg #(Bool) rg_mm2s_hit_tail <- mkReg (True);
   Reg #(Bool) rg_s2mm_hit_tail <- mkReg (True);
   Reg #(Bool) rg_fetch_after_intr <- mkReg (False);

   // A 2D Vector of registers, which contain the MM2S and S2MM Buffer Descriptors
   // that are currently being worked on
   Vector #(TExp #(SizeOf #(DMA_Dir)), Vector #(TExp #(SizeOf #(DMA_BD_Index)), Reg #(DMA_BD_Word))) v_v_rg_bd;
   v_v_rg_bd <- replicateM (replicateM (mkRegU));

   AXI4_DMA_Int_Reg_IFC dma_int_reg <- mkAXI4_DMA_Int_Reg;

   AXI4_DMA_Scatter_Gather_IFC #(mid_, addr_, data_,
                                 awuser_, wuser_, buser_,
                                 aruser_, ruser_) axi_sg <- mkAXI4_DMA_Scatter_Gather (v_v_rg_bd);

   AXI4_DMA_Register_Module_IFC #(sid_, addr_, data_,
                                  awuser_, wuser_, buser_,
                                  aruser_, ruser_) dma_reg <- mkAXI4_DMA_Register_Module (dma_int_reg,
                                                                                          rg_mm2s_hit_tail,
                                                                                          rg_s2mm_hit_tail);

   AXI4_DMA_Copy_Unit_IFC #(mid_, addr_, data_,
                            awuser_, wuser_, buser_,
                            aruser_, ruser_,
                            strm_id_, sdata_, sdest_, suser_) dma_copy_unit <- mkAXI4_DMA_Copy_Unit (v_v_rg_bd, dma_int_reg);


   // FIFO containing triggers from the Register Module
   FIFOF #(DMA_Dir) fifo_trigger <- mkFIFOF1;

   // FIFO containing triggers from the Copy Unit
   FIFOF #(DMA_Dir) fifo_copy_unit_end_trigger <- mkFIFOF1;

   Reg #(Bit #(64)) rg_counter <- mkReg (0);

   function Action fa_reset;
      action
         dma_reg.reset;
         axi_sg.reset;
         dma_copy_unit.reset;
         dma_int_reg.reset;
         rg_state <= RESET;
         rg_mm2s_first_fetch <= True;
         rg_s2mm_first_fetch <= True;
         rg_mm2s_hit_tail <= True;
         rg_s2mm_hit_tail <= True;
         rg_fetch_after_intr <= False;
      endaction
   endfunction

   rule rl_detect_reset (rg_state != RESET
                         && (dma_int_reg.mm2s_dmacr.reset == 1'b1
                             || dma_int_reg.s2mm_dmacr.reset == 1'b1));
      if (rg_verbosity > 0) begin
         $display ("AXI DMA Unit: Reset detected in registers");
      end
      if (rg_verbosity > 1) begin
         $display ("    dma_int_reg.mm2s_dmacr.reset: ", fshow (dma_int_reg.mm2s_dmacr.reset));
         $display ("    dma_int_reg.s2mm_dmacr.reset: ", fshow (dma_int_reg.s2mm_dmacr.reset));
      end
      fa_reset;
   endrule

   rule rl_reset (rg_state == RESET);
      if (rg_verbosity > 0) begin
         $display ("AXI4 DMA Reset");
      end
      rg_counter <= 0;
      rg_state <= HALTED;
   endrule

   rule rl_increment_counter (rg_state != RESET
                              && rg_state != HALTED);
      rg_counter <= rg_counter + 1;
   endrule

   rule rl_debug_halt_to_idle (dma_reg.get_halt_to_idle);
      if (rg_verbosity > 0) begin
         $display ("toplevel received halt to idle");
         $display ("   rg_state: ", fshow (rg_state));
      end
   endrule

   // When the halt_to_idle signal is received, transition the modules
   // into their idle state
   rule rl_halt_to_idle (rg_state == HALTED
                         && dma_reg.get_halt_to_idle);
      if (rg_verbosity > 1) begin
         $display ("DMA Unit: toplevel halt to idle");
      end
      rg_state <= IDLE;
      dma_reg.halt_to_idle;
      axi_sg.halt_to_idle;
      dma_copy_unit.halt_to_idle;
   endrule

   // Handle enqueueing triggers received from the Register Unit
   // These happen when the tail descriptor registers are written to
   // with a value that causes us to have to do something
   rule rl_enq_reg_trigger (rg_state != HALTED
                            && rg_state != RESET
                            && isValid (dma_reg.trigger));
      let dir = fromMaybe (?, dma_reg.trigger);
      if (rg_verbosity > 0) begin
         $display ("DMA Unit: enqueued to trigger fifo, direction: ", fshow (dir));
      end
      fifo_trigger.enq (dir);
   endrule

   rule rl_debug_trigger_mm2s (isValid (dma_reg.trigger));
      if (rg_verbosity > 1) begin
         $display ("AXI DMA trigger");
         $display ("    dir: ", fshow (fromMaybe (?, dma_reg.trigger)));
         $display ("    state: ", fshow (rg_state));
      end
   endrule

   // Handle DMA transfer trigger
   // The trigger is enqueued into the fifo above.
   // This rule deals with dequeueing it and handling the operations that need
   // to be done because of the trigger
   rule rl_handle_trigger (rg_state != HALTED
                           && rg_state != RESET);
      let dir = fifo_trigger.first;
      fifo_trigger.deq;

      if (dir == MM2S) begin
         // There have been instances in the past where the bluespec compiler has not collapsed
         // two signals that should be the same (here dir_local and dir) into one. This is done
         // to avoid that
         DMA_Dir dir_local = MM2S;
         // TODO handle 32bit addresses
         Bit #(addr_) address = rg_mm2s_first_fetch ? truncate ({pack (dma_int_reg.mm2s_curdesc_msb)
                                                      ,pack (dma_int_reg.mm2s_curdesc)})
                                                    : truncate ({v_v_rg_bd[pack (dir_local)][pack (DMA_NXTDESC_MSB)]
                                                      ,v_v_rg_bd[pack (dir_local)][pack (DMA_NXTDESC)]});
         dma_int_reg.mm2s_curdesc_write (unpack (truncate (address)));
         dma_int_reg.mm2s_curdesc_msb_write (unpack (truncateLSB (address)));
         if (rg_verbosity > 0) begin
            $display ("MM2S trigger handling");
            $display ("    addr: ", fshow(address));
         end
         axi_sg.bd_read_from_mem(MM2S, address);
         rg_mm2s_first_fetch <= False;
      end else begin
         // There have been instances in the past where the bluespec compiler has not collapsed
         // two signals that should be the same (here dir_local and dir) into one. This is done
         // to avoid that
         DMA_Dir dir_local = S2MM;
         // TODO handle 32bit addresses
         Bit #(addr_) address = rg_s2mm_first_fetch ? truncate ({pack (dma_int_reg.s2mm_curdesc_msb)
                                                      ,pack (dma_int_reg.s2mm_curdesc)})
                                                    : truncate ({v_v_rg_bd[pack (dir_local)][pack (DMA_NXTDESC_MSB)]
                                                      ,v_v_rg_bd[pack (dir_local)][pack (DMA_NXTDESC)]});
         dma_int_reg.s2mm_curdesc_write (unpack (truncate (address)));
         dma_int_reg.s2mm_curdesc_msb_write (unpack (truncateLSB (address)));
         if (rg_verbosity > 0) begin
            $display ("S2MM trigger handling");
            $display ("    addr: ", fshow(address));
         end
         axi_sg.bd_read_from_mem(S2MM, address);
         rg_s2mm_first_fetch <= False;
      end
   endrule

   // The Scatter Gather unit sets the trigger_callback signal high when it finishes
   // an MM2S SG fetch.
   // When the SG fetch is finished, we trigger the copying of the data
   rule rl_handle_sg_trigger_callback (rg_state != HALTED
                                       && rg_state != RESET
                                       && axi_sg.trigger_callback);
      dma_copy_unit.trigger;
   endrule

   // Trigger
   rule rl_enq_copy_unit_end_trigger (rg_state != HALTED
                                      && rg_state != RESET
                                      && isValid (dma_copy_unit.end_trigger));
      fifo_copy_unit_end_trigger.enq (fromMaybe (?, dma_copy_unit.end_trigger));
   endrule

   // Handle writing back the buffer descriptor after the end of a memory copy.
   // All the BD data should already have been updated
   rule rl_handle_copy_unit_end_trigger (rg_state != HALTED
                                         && rg_state != RESET);
      fifo_copy_unit_end_trigger.deq;
      let dir = fifo_copy_unit_end_trigger.first;

      // TODO handle 32-bit addresses
      let cur_addr = dir == MM2S ? {pack (dma_int_reg.mm2s_curdesc_msb), pack (dma_int_reg.mm2s_curdesc)}
                                 : {pack (dma_int_reg.s2mm_curdesc_msb), pack (dma_int_reg.s2mm_curdesc)};
      let cur_tail = dir == MM2S ? {pack (dma_int_reg.mm2s_taildesc_msb), pack (dma_int_reg.mm2s_taildesc)}
                                 : {pack (dma_int_reg.s2mm_taildesc_msb), pack (dma_int_reg.s2mm_taildesc)};
      axi_sg.bd_write_to_mem (dir, truncate (cur_addr));
      if (cur_addr >= cur_tail) begin
         if (rg_verbosity > 0) begin
            $display ("DMA Unit: ", fshow (dir), " hit tail descriptor");
         end
         if (dir == MM2S) begin
            rg_mm2s_hit_tail <= True;
         end else begin
            rg_s2mm_hit_tail <= True;
         end
      end

      if (rg_verbosity > 0) begin
         $display ("AXI DMA: Handling copy unit end trigger, writing ", fshow (dir), " BD to address ", fshow (cur_addr));
      end
   endrule

   // Handle Scatter Gather interrupt triggers
   // These happen every time the Scatter Gather Unit finishes writing
   // a BD back to memory.
   // At the moment, this happens after every BD write request.
   // For now, we always trigger an external interrupt when we finish.
   // To trigger the external interrupt, we write to the interrupt bits
   // in the appropriate registers
   // We also fetch the next available BD
   // TODO implement interrupt thresholds and delay interrupts
   rule rl_handle_sg_interrupt (isValid (axi_sg.trigger_interrupt));
      let dir = fromMaybe (?, axi_sg.trigger_interrupt);
      if (rg_verbosity > 0) begin
         $display ("AXI DMA: handling SG interrupt trigger in direction ", fshow (dir));
      end
      if (dir == MM2S) begin
         let val_to_write = dma_int_reg.mm2s_dmasr;
         val_to_write.ioc_irq = 1'b1;
         dma_int_reg.mm2s_dmasr_write (val_to_write);
         if (rg_verbosity > 1) begin
            $display ("    old DMASR value: ", fshow (dma_int_reg.mm2s_dmasr));
            $display ("    value written: ", fshow (val_to_write));
         end
      end else begin
         let val_to_write = dma_int_reg.s2mm_dmasr;
         val_to_write.ioc_irq = 1'b1;
         dma_int_reg.s2mm_dmasr_write (val_to_write);
         rg_fetch_after_intr <= True;
         if (rg_verbosity > 1) begin
            $display ("    old DMASR value: ", fshow (dma_int_reg.s2mm_dmasr));
            $display ("    value written: ", fshow (val_to_write));
         end
         if (rg_verbosity > 0) begin
            $display ("DMA Unit: reading next BD");
         end
      end
   endrule

   // Used in S2MM transactions
   // Runs the cycle after receiving a SG interrupt, which happens when
   // the SG unit finishes writing a BD back to main memory.
   // Gets the new BD from memory, to be ready for the next transaction
   rule rl_handle_sg_fetch_after_write_s2mm (rg_fetch_after_intr);
      DMA_Dir dir_local = S2MM;
      rg_fetch_after_intr <= False;
      // bsc is not able to disambiguate the type if we use pack (S2MM) so we introduce
      // dir_local instead
      Bit #(addr_) address = truncate ({v_v_rg_bd[pack (dir_local)][pack (DMA_NXTDESC_MSB)]
                                       ,v_v_rg_bd[pack (dir_local)][pack (DMA_NXTDESC)]});
      dma_int_reg.s2mm_curdesc_write (unpack (truncate (address) & pack (s2mm_curdesc_rw_mask_halted)));
      dma_int_reg.s2mm_curdesc_msb_write (unpack (truncateLSB (address) & pack (s2mm_curdesc_rw_mask)));
      axi_sg.bd_read_from_mem (S2MM, address);
   endrule

   rule rl_debug_enq_copy_unit_end_trigger (isValid (dma_copy_unit.end_trigger)
                                               && !fifo_copy_unit_end_trigger.notFull);
      $display ("AXI DMA: Error: Missed copy unit end trigger");
   endrule

   rule rl_debug_trigger_callback (axi_sg.trigger_callback);
      if (rg_verbosity > 1) begin
         $display ("dma toplevel sees trigger_callback");
      end
   endrule

   rule rl_interrupt_debug (rg_verbosity > 0
                            && isValid (axi_sg.trigger_interrupt));
      $display ("AXI DMA: interrupt detected in direction ", fshow (fromMaybe (?, axi_sg.trigger_interrupt)));
   endrule



   // Uncomment this when compiling for outside use
   //interface axi4s_data_master = dma_copy_unit.axi4s_data_master;
   //interface axi4s_meta_master = dma_copy_unit.axi4s_meta_master;

   //interface axi4s_data_slave = dma_copy_unit.axi4s_data_slave;
   //interface axi4s_meta_slave = dma_copy_unit.axi4s_meta_slave;

   // Uncomment this when compiling for simulation with loopback
   AXI4_Stream_Delay_Loopback_IFC #(strm_id_, sdata_, sdest_, suser_) axi4s_loopback <- mkAXI4_Stream_Delay_Loopback;
   mkConnection (dma_copy_unit.axi4s_data_master, axi4s_loopback.axi4s_data_slave);
   mkConnection (dma_copy_unit.axi4s_meta_master, axi4s_loopback.axi4s_meta_slave);
   mkConnection (dma_copy_unit.axi4s_data_slave, axi4s_loopback.axi4s_data_master);
   mkConnection (dma_copy_unit.axi4s_meta_slave, axi4s_loopback.axi4s_meta_master);

   method s2mm_interrupt_req = ( ( dma_int_reg.s2mm_dmasr.err_irq & dma_int_reg.s2mm_dmacr.err_irqen )
                               | ( dma_int_reg.s2mm_dmasr.dly_irq & dma_int_reg.s2mm_dmacr.dly_irqen )
                               | ( dma_int_reg.s2mm_dmasr.ioc_irq & dma_int_reg.s2mm_dmacr.ioc_irqen )
                               ) == 1'b1;

   method mm2s_interrupt_req = ( ( dma_int_reg.mm2s_dmasr.err_irq & dma_int_reg.mm2s_dmacr.err_irqen )
                               | ( dma_int_reg.mm2s_dmasr.dly_irq & dma_int_reg.mm2s_dmacr.dly_irqen )
                               | ( dma_int_reg.mm2s_dmasr.ioc_irq & dma_int_reg.mm2s_dmacr.ioc_irqen )
                               ) == 1'b1;

   interface axi_sg_master = axi_sg.axi4_master;

   interface axi_reg_slave = dma_reg.axi4_slave;

   interface axi_copy_master = dma_copy_unit.axi4_master;

   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
      axi_sg.set_verbosity             (new_verb);
      dma_reg.set_verbosity            (new_verb);
      dma_copy_unit.set_verbosity      (new_verb);
      dma_int_reg.set_verbosity        (new_verb);
      //axi4s_loopback.set_verbosity     (new_verb);
   endmethod

   method Action reset;
      fa_reset;
      axi4s_loopback.reset;
   endmethod

endmodule


endpackage
