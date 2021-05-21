package AXI4_DMA_Internal_Reg_Module;

import AXI4_DMA_Types :: *;
import CHERICap :: *;
import CHERICC_Fat :: *;

typedef enum {
   RESET,
   IDLE
} State deriving (Bits, FShow, Eq);


interface AXI4_DMA_Int_Reg_IFC;
   // The methods to read and write each unique value are unfiltered and
   // use the raw data
   method MM2S_DMACR        mm2s_dmacr;
   method Action            mm2s_dmacr_write        (MM2S_DMACR newval);

   method MM2S_DMASR        mm2s_dmasr;
   method Action            mm2s_dmasr_write        (MM2S_DMASR newval);

   method MM2S_CURDESC      mm2s_curdesc;
   method Action            mm2s_curdesc_write      (MM2S_CURDESC newval);

   method MM2S_CURDESC_MSB  mm2s_curdesc_msb;
   method Action            mm2s_curdesc_msb_write  (MM2S_CURDESC_MSB newval);

   method MM2S_TAILDESC     mm2s_taildesc;
   method Action            mm2s_taildesc_write     (MM2S_TAILDESC newval);

   method MM2S_TAILDESC_MSB mm2s_taildesc_msb;
   method Action            mm2s_taildesc_msb_write (MM2S_TAILDESC_MSB newval);

   method CapPipe           mm2s_curdesc_cap;
   method Action            mm2s_curdesc_cap_write  (CapPipe newval);


   method S2MM_DMACR        s2mm_dmacr;
   method Action            s2mm_dmacr_write        (S2MM_DMACR newval);

   method S2MM_DMASR        s2mm_dmasr;
   method Action            s2mm_dmasr_write        (S2MM_DMASR newval);

   method S2MM_CURDESC      s2mm_curdesc;
   method Action            s2mm_curdesc_write      (S2MM_CURDESC newval);

   method S2MM_CURDESC_MSB  s2mm_curdesc_msb;
   method Action            s2mm_curdesc_msb_write  (S2MM_CURDESC_MSB newval);

   method S2MM_TAILDESC     s2mm_taildesc;
   method Action            s2mm_taildesc_write     (S2MM_TAILDESC newval);

   method S2MM_TAILDESC_MSB s2mm_taildesc_msb;
   method Action            s2mm_taildesc_msb_write (S2MM_TAILDESC_MSB newval);

   method CapPipe           s2mm_curdesc_cap;
   method Action            s2mm_curdesc_cap_write  (CapPipe newval);


   // These methods should be used by the slave interface to write the appropriate
   // values. These methods apply the proper RW masks and ensure that the proper
   // old values are propagated
   method DMA_Reg_Word external_read  (DMA_Reg_Index index);
   method Action       external_write (DMA_Reg_Index index, DMA_Reg_Word newval);

   method Action set_verbosity (Bit #(4) new_verb);
   method Action reset;
endinterface


module mkAXI4_DMA_Int_Reg (AXI4_DMA_Int_Reg_IFC);
   Reg #(State) rg_state <- mkReg (RESET);
   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);

   Reg #(MM2S_DMACR)        rg_mm2s_dmacr        <- mkReg (mm2s_dmacr_default);
   Reg #(MM2S_DMASR)        rg_mm2s_dmasr        <- mkReg (mm2s_dmasr_default);
   Reg #(MM2S_CURDESC)      rg_mm2s_curdesc      <- mkReg (mm2s_curdesc_default);
   Reg #(MM2S_CURDESC_MSB)  rg_mm2s_curdesc_msb  <- mkReg (mm2s_curdesc_msb_default);
   Reg #(MM2S_TAILDESC)     rg_mm2s_taildesc     <- mkReg (mm2s_taildesc_default);
   Reg #(MM2S_TAILDESC_MSB) rg_mm2s_taildesc_msb <- mkReg (mm2s_taildesc_msb_default);
   Reg #(CapPipe)           rg_mm2s_curdesc_cap  <- mkReg (nullCap);

   Reg #(S2MM_DMACR)        rg_s2mm_dmacr        <- mkReg (s2mm_dmacr_default);
   Reg #(S2MM_DMASR)        rg_s2mm_dmasr        <- mkReg (s2mm_dmasr_default);
   Reg #(S2MM_CURDESC)      rg_s2mm_curdesc      <- mkReg (s2mm_curdesc_default);
   Reg #(S2MM_CURDESC_MSB)  rg_s2mm_curdesc_msb  <- mkReg (s2mm_curdesc_msb_default);
   Reg #(S2MM_TAILDESC)     rg_s2mm_taildesc     <- mkReg (s2mm_taildesc_default);
   Reg #(S2MM_TAILDESC_MSB) rg_s2mm_taildesc_msb <- mkReg (s2mm_taildesc_msb_default);
   Reg #(CapPipe)           rg_s2mm_curdesc_cap  <- mkReg (nullCap);

   rule rl_reset (rg_state == RESET);
      if (rg_verbosity > 0) begin
         $display ("DMA Internal Register Reset");
      end
      rg_state <= IDLE;
      rg_mm2s_dmacr        <= mm2s_dmacr_default;
      rg_mm2s_dmasr        <= mm2s_dmasr_default;
      rg_mm2s_curdesc      <= mm2s_curdesc_default;
      rg_mm2s_curdesc_msb  <= mm2s_curdesc_msb_default;
      rg_mm2s_taildesc     <= mm2s_taildesc_default;
      rg_mm2s_taildesc_msb <= mm2s_taildesc_msb_default;
      rg_mm2s_curdesc_cap  <= nullCap;

      rg_s2mm_dmacr        <= s2mm_dmacr_default;
      rg_s2mm_dmasr        <= s2mm_dmasr_default;
      rg_s2mm_curdesc      <= s2mm_curdesc_default;
      rg_s2mm_curdesc_msb  <= s2mm_curdesc_msb_default;
      rg_s2mm_taildesc     <= s2mm_taildesc_default;
      rg_s2mm_taildesc_msb <= s2mm_taildesc_msb_default;
      rg_s2mm_curdesc_cap  <= nullCap;
   endrule


   method        mm2s_dmacr = rg_mm2s_dmacr;
   method Action mm2s_dmacr_write (MM2S_DMACR newval) if (rg_state != RESET);
      rg_mm2s_dmacr <= newval;
   endmethod

   method        mm2s_dmasr = rg_mm2s_dmasr;
   method Action mm2s_dmasr_write (MM2S_DMASR newval) if (rg_state != RESET);
      rg_mm2s_dmasr <= newval;
   endmethod

   method        mm2s_curdesc = rg_mm2s_curdesc;
   method Action mm2s_curdesc_write (MM2S_CURDESC newval) if (rg_state != RESET);
      rg_mm2s_curdesc <= newval;
   endmethod

   method        mm2s_curdesc_msb = rg_mm2s_curdesc_msb;
   method Action mm2s_curdesc_msb_write (MM2S_CURDESC_MSB newval) if (rg_state != RESET);
      rg_mm2s_curdesc_msb <= newval;
   endmethod

   method        mm2s_taildesc = rg_mm2s_taildesc;
   method Action mm2s_taildesc_write (MM2S_TAILDESC newval) if (rg_state != RESET);
      rg_mm2s_taildesc <= newval;
   endmethod

   method        mm2s_taildesc_msb = rg_mm2s_taildesc_msb;
   method Action mm2s_taildesc_msb_write (MM2S_TAILDESC_MSB newval) if (rg_state != RESET);
      rg_mm2s_taildesc_msb <= newval;
   endmethod

   method        mm2s_curdesc_cap = rg_mm2s_curdesc_cap;
   method Action mm2s_curdesc_cap_write (CapPipe newval) if (rg_state != RESET);
      rg_mm2s_curdesc_cap <= newval;
   endmethod


   method        s2mm_dmacr = rg_s2mm_dmacr;
   method Action s2mm_dmacr_write (S2MM_DMACR newval) if (rg_state != RESET);
      rg_s2mm_dmacr <= newval;
   endmethod

   method        s2mm_dmasr = rg_s2mm_dmasr;
   method Action s2mm_dmasr_write (S2MM_DMASR newval) if (rg_state != RESET);
      rg_s2mm_dmasr <= newval;
   endmethod

   method        s2mm_curdesc = rg_s2mm_curdesc;
   method Action s2mm_curdesc_write (S2MM_CURDESC newval) if (rg_state != RESET);
      rg_s2mm_curdesc <= newval;
   endmethod

   method        s2mm_curdesc_msb = rg_s2mm_curdesc_msb;
   method Action s2mm_curdesc_msb_write (S2MM_CURDESC_MSB newval) if (rg_state != RESET);
      rg_s2mm_curdesc_msb <= newval;
   endmethod

   method        s2mm_taildesc = rg_s2mm_taildesc;
   method Action s2mm_taildesc_write (S2MM_TAILDESC newval) if (rg_state != RESET);
      rg_s2mm_taildesc <= newval;
   endmethod

   method        s2mm_taildesc_msb = rg_s2mm_taildesc_msb;
   method Action s2mm_taildesc_msb_write (S2MM_TAILDESC_MSB newval) if (rg_state != RESET);
      rg_s2mm_taildesc_msb <= newval;
   endmethod

   method        s2mm_curdesc_cap = rg_s2mm_curdesc_cap;
   method Action s2mm_curdesc_cap_write (CapPipe newval) if (rg_state != RESET);
      rg_s2mm_curdesc_cap <= newval;
   endmethod



   method DMA_Reg_Word external_read (DMA_Reg_Index index);
      case (index)
         DMA_MM2S_DMACR:         return pack (rg_mm2s_dmacr);
         DMA_MM2S_DMASR:         return pack (rg_mm2s_dmasr);
         DMA_MM2S_CURDESC:       return pack (rg_mm2s_curdesc);
         DMA_MM2S_CURDESC_MSB:   return pack (rg_mm2s_curdesc_msb);
         DMA_MM2S_TAILDESC:      return pack (rg_mm2s_taildesc);
         DMA_MM2S_TAILDESC_MSB:  return pack (rg_mm2s_taildesc_msb);
         DMA_S2MM_DMACR:         return pack (rg_s2mm_dmacr);
         DMA_S2MM_DMASR:         return pack (rg_s2mm_dmasr);
         DMA_S2MM_CURDESC:       return pack (rg_s2mm_curdesc);
         DMA_S2MM_CURDESC_MSB:   return pack (rg_s2mm_curdesc_msb);
         DMA_S2MM_TAILDESC:      return pack (rg_s2mm_taildesc);
         DMA_S2MM_TAILDESC_MSB:  return pack (rg_s2mm_taildesc_msb);
         default:                return ?;
      endcase
   endmethod

   method Action external_write (DMA_Reg_Index index, DMA_Reg_Word newval) if (rg_state != RESET);

      let raw_val = ?;
      if (rg_verbosity > 0) begin
         $display ("DMA Internal Reg Unit: external write to ", fshow (index));
         $display ("    raw value: ", fshow (newval));
      end
      case (index)
         DMA_MM2S_DMACR: begin
            // TODO should we also set bits in DMASR and other registers based on changes to this?
            // ie halted bit, reset bit etc
            raw_val = (~(pack (mm2s_dmacr_rw_mask)) & pack (rg_mm2s_dmacr)) // preserve read-only values
                        | ( (pack (mm2s_dmacr_rw_mask)) & newval);              // write the writable bits
            rg_mm2s_dmacr <= unpack (raw_val);
            if (raw_val[0] == 1'b1) begin
               let val_to_write = rg_mm2s_dmasr;
               val_to_write.halted = 1'b0;
               rg_mm2s_dmasr <= val_to_write;
               if (rg_verbosity > 1) begin
                  $display ("    also updating ", fshow (DMA_MM2S_DMASR));
                  $display ("        from ", fshow (rg_mm2s_dmasr));
                  $display ("        to ", fshow (val_to_write));
               end
            end
         end
         DMA_MM2S_DMASR: begin
            // The bits in this register are either read-only or write-clear
            raw_val = (~(pack (mm2s_dmasr_rw_mask)) & pack (rg_mm2s_dmasr))            // preserve the read-only bits
                        | ( (pack (mm2s_dmasr_rw_mask)) & pack (rg_mm2s_dmasr) & ~newval); // preserve the bits that we didn't clear
            rg_mm2s_dmasr <= unpack (raw_val);
         end
         DMA_MM2S_CURDESC: begin
            // The behaviour of this register depends on whether this channel is halted
            let mask = rg_mm2s_dmasr.halted == 1'b1 ? pack (mm2s_curdesc_rw_mask_halted)
                                                    : pack (mm2s_curdesc_rw_mask);
            raw_val = (~mask & pack (rg_mm2s_curdesc)) // preserve the read-only bits
                        | ( mask & newval);                // write the writable bits
            rg_mm2s_curdesc <= unpack (raw_val);
         end
         DMA_MM2S_CURDESC_MSB: begin
            // The behaviour of this register depends on whether this channel is halted
            let mask = rg_mm2s_dmasr.halted == 1'b1 ? pack (mm2s_curdesc_msb_rw_mask_halted)
                                                    : pack (mm2s_curdesc_msb_rw_mask);
            raw_val = (~mask & pack (rg_mm2s_curdesc_msb)) // preserve the read-only bits
                        | ( mask & newval);                    // write the writable bits
            rg_mm2s_curdesc_msb <= unpack (raw_val);
         end
         DMA_MM2S_TAILDESC: begin
            raw_val = (~(pack (mm2s_taildesc_rw_mask)) & pack (rg_mm2s_taildesc))
                        | ( (pack (mm2s_taildesc_rw_mask)) & newval);
            rg_mm2s_taildesc <= unpack (raw_val);
         end
         DMA_MM2S_TAILDESC_MSB: begin
            raw_val = (~(pack (mm2s_taildesc_msb_rw_mask)) & pack (rg_mm2s_taildesc_msb))
                        | ( (pack (mm2s_taildesc_msb_rw_mask)) & newval);
            rg_mm2s_taildesc_msb <= unpack (raw_val);
         end
         DMA_MM2S_CURDESC_CAP: begin
            $display ("DMA Internal Register Module Error: should not try to write\n",
                      "    capabilities using external_write");
         end

         DMA_S2MM_DMACR: begin
            raw_val = (~(pack (s2mm_dmacr_rw_mask)) & pack (rg_s2mm_dmacr))
                        | ( (pack (s2mm_dmacr_rw_mask)) & newval);
            rg_s2mm_dmacr <= unpack (raw_val);
            if (raw_val[0] == 1'b1) begin
               let val_to_write = rg_s2mm_dmasr;
               val_to_write.halted = 1'b0;
               rg_s2mm_dmasr <= val_to_write;
               if (rg_verbosity > 1) begin
                  $display ("    also updating ", fshow (DMA_MM2S_DMASR));
                  $display ("        from ", fshow (rg_s2mm_dmasr));
                  $display ("        to ", fshow (val_to_write));
               end
            end
         end
         DMA_S2MM_DMASR: begin
            raw_val = (~(pack (s2mm_dmasr_rw_mask)) & pack (rg_s2mm_dmasr))
                        | ( (pack (s2mm_dmasr_rw_mask)) & pack (rg_s2mm_dmasr) & ~newval); // preserve the bits that we didn't clear
            rg_s2mm_dmasr <= unpack (raw_val);
         end
         DMA_S2MM_CURDESC: begin
            // The behaviour of this register depends on whether this channel is halted
            let mask = rg_s2mm_dmasr.halted == 1'b1 ? pack (s2mm_curdesc_rw_mask_halted)
                                                    : pack (s2mm_curdesc_rw_mask);
            raw_val = (~mask & pack (rg_s2mm_curdesc)) // preserve the read-only bits
                        | ( mask & newval);                // write the writable bits
            rg_s2mm_curdesc <= unpack (raw_val);
         end
         DMA_S2MM_CURDESC_MSB: begin
            // The behaviour of this register depends on whether this channel is halted
            let mask = rg_s2mm_dmasr.halted == 1'b1 ? pack (s2mm_curdesc_msb_rw_mask_halted)
                                                    : pack (s2mm_curdesc_msb_rw_mask);
            raw_val = (~mask & pack (rg_s2mm_curdesc_msb)) // preserve the read-only bits
                        | ( mask & newval);                    // write the writable bits
            rg_s2mm_curdesc_msb <= unpack (raw_val);
         end
         DMA_S2MM_TAILDESC: begin
            raw_val = (~(pack (s2mm_taildesc_rw_mask)) & pack (rg_s2mm_taildesc))
                        | ( (pack (s2mm_taildesc_rw_mask)) & newval);
            rg_s2mm_taildesc <= unpack (raw_val);
         end
         DMA_S2MM_TAILDESC_MSB: begin
            raw_val = (~(pack (s2mm_taildesc_msb_rw_mask)) & pack (rg_s2mm_taildesc_msb))
                        | ( (pack (s2mm_taildesc_msb_rw_mask)) & newval);
            rg_s2mm_taildesc_msb <= unpack (raw_val);
         end
      endcase
      if (rg_verbosity > 0) begin
         $display ("    written value: ", fshow (raw_val));
      end
   endmethod

   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
   endmethod

   method Action reset if (rg_state != RESET);
      rg_state <= RESET;
   endmethod
endmodule

endpackage
