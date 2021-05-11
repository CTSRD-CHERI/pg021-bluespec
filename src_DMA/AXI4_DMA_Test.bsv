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

package AXI4_DMA_Test;

// This contains a very simple initial test module

// bluespec imports
import DReg :: *;

// AXI imports
import AXI4 :: *;
import SourceSink :: *;
import Connectable :: *;

// local imports
import AXI4_DMA_Scatter_Gather :: *;
import AXI4_DMA_Register_Module :: *;
import AXI4_DMA_Types :: *;



typedef 0 MID_sz;
typedef 64 ADDR_sz;
typedef 32 DATA_sz;
typedef 0 AWUSER_sz;
typedef 0 WUSER_sz;
typedef 0 BUSER_sz;
typedef 0 ARUSER_sz;
typedef 0 RUSER_sz;


module mkFake_Mem_Slave (AXI4_Slave #(id_, addr_, data_, awuser_, wuser_, buser_, aruser_, ruser_));
   AXI4_Shim #(id_, addr_, data_, awuser_, wuser_, buser_, aruser_, ruser_) shim <- mkAXI4Shim;

   let m = shim.master;
   Reg #(Bit #(16)) rg_count <- mkReg (0);
   Reg #(Bool) rg_wresp_outstanding <- mkReg (False);
   Reg #(Bool) rg_rresp_outstanding <- mkReg (False);

   //rule rl_debug;
   //   let m = shim.master;
   //   if (m.aw.canPeek) begin
   //      $display ("noted m.aw.canPeek is true in fake mem");
   //   end
   //endrule

   (* fire_when_enabled *)
   rule rl_ar_drop;
      if (m.ar.canPeek) begin
         $display ("address read channel access");
         $display ("    flit: ", fshow(m.ar.peek));
         m.ar.drop;
         rg_count <= zeroExtend (m.ar.peek.arlen);
         rg_rresp_outstanding <= True;
      end
   endrule

   rule rl_handle_read (rg_rresp_outstanding);
      let islast = rg_count == 0;
      AXI4_RFlit #(id_, data_, user_) rflit = AXI4_RFlit {
         rid : 0,
         rdata : 'hdeadbeef,
         rresp : OKAY,
         rlast : islast,
         ruser : ?
      };
      m.r.put (rflit);
      rg_count <= rg_count - 1;
      if (islast) begin
         rg_rresp_outstanding <= False;
      end
   endrule

   rule rl_aw_drop;
      if (m.aw.canPeek) begin
         $display ("address write channel access");
         $display ("    flit: ", fshow(m.aw.peek));
         m.aw.drop;
         rg_wresp_outstanding <= True;
         rg_count <= zeroExtend (m.aw.peek.awlen);
      end
   endrule

   rule rl_w_drop;
      if (m.w.canPeek) begin
         $display ("write channel access");
         $display ("    flit: ", fshow(m.w.peek));
         m.w.drop;
      end
   endrule

   rule rl_handle_write (rg_wresp_outstanding);
      let islast = rg_count == 0;
      AXI4_BFlit #(id_, user_) bflit = AXI4_BFlit {
         bid : 0,
         bresp : OKAY,
         buser : ?
      };
      m.b.put (bflit);
      rg_count <= rg_count - 1;
      if (islast) begin
         rg_wresp_outstanding <= False;
      end
   endrule


   return shim.slave;
endmodule


module mkAXI4_DMA_Test (Empty);
   AXI4_DMA_Scatter_Gather_IFC #(
      MID_sz, ADDR_sz, DATA_sz, AWUSER_sz, WUSER_sz, BUSER_sz, ARUSER_sz, RUSER_sz
   ) sg <- mkAXI4_DMA_Scatter_Gather;

   AXI4_Slave #(0, ADDR_sz, DATA_sz, AWUSER_sz, WUSER_sz, BUSER_sz, ARUSER_sz, RUSER_sz) axi4_slave <- mkFake_Mem_Slave;

   mkConnection (sg.axi4_master, axi4_slave);

   DMA_BD test_bd_val = DMA_BD {
      nxt_desc             : 32'h1010_1000,
      nxtdesc_msb          : 32'h1010_2000,
      buffer_address       : 32'h1010_3000,
      buffer_address_msb   : 32'h1010_4000,
      reserved0            : 32'h1010_5000,
      reserved1            : 32'h1010_6000,
      control              : 32'h1010_7000,
      status               : 32'h1010_8000,
      app0                 : 32'h1010_9000,
      app1                 : 32'h1010_A000,
      app2                 : 32'h1010_B000,
      app3                 : 32'h1010_C000,
      app4                 : 32'h1010_D000,
      fill0                : 32'h1010_E000,
      fill1                : 32'h1010_E000,
      fill2                : 32'h1010_E000
   };



   Reg #(Bit #(32)) counter <- mkReg (0);
   Reg #(Bool) writing_loop <- mkReg (False);
   Reg #(Bool) trigger_write_to_mem <- mkDReg (False);

   rule rl_debug;
      //$display ("counter: ", fshow(counter));
      //$display ("writing_loop: ", fshow(writing_loop));
   endrule

   rule rl_counter_incr;
      counter <= counter + 1;
   endrule


   rule rl_dma_read (counter[7:0] == 'h0);
      $display ("read trigger");
      sg.bd_read_from_mem ('h1000);
   endrule

   rule rl_dma_bd_fetch_next (counter [7:0] == 'h60);
      $display ("fetching next bd");
      sg.bd_fetch_next;
   endrule

   rule rl_dma_write_bd (counter[7:0] == 'h40);
      $display ("writing bd register with index: ", fshow(counter[3:0]));
      sg.reg_write (unpack (counter[3:0]), 32'h1010_0000 + (zeroExtend (counter[4:0]) << 12));
      writing_loop <= True;
   endrule

   rule rl_dma_write_bd_loop (writing_loop);
      $display ("writing bd register with index: ", fshow(counter[3:0]));
      sg.reg_write (unpack (counter[3:0]), 32'h1010_0000 + (zeroExtend (counter[4:0]) << 12));
      DMA_BD_Index dma_max = maxBound;
      if (counter[3:0] == pack (dma_max)) begin
         trigger_write_to_mem <= True;
         $display ("writing trigger register");
         writing_loop <= False;
      end
   endrule

   rule rl_dma_write_to_mem (trigger_write_to_mem);
      $display ("write trigger");
      sg.bd_write_to_mem ('h2000);
   endrule

endmodule

endpackage
