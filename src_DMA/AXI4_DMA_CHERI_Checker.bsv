package AXI4_DMA_CHERI_Checker;

import DReg :: *;

import AXI :: *;
import CHERICap :: *;
import CHERICC_Fat :: *;
import SourceSink :: *;

typedef enum {
   AW,
   AR
} Mem_Op deriving (Bits, Eq, FShow);

typedef enum {
   IDLE,
   RUNNING,
   FORWARD_R,
   DENY_R,
   FORWARD_W,
   DENY_W
} State deriving (Bits, Eq, FShow);

// This interface is similar to an AXI4_Shim interface, but the slave port
// (which is designed to face internally into the design) has wider user
// fields than the master port (which is designed to face externally into the
// bus).
// The awuser and aruser fields are extended by the size of a capability
// including the tag, and expect the top bits to contain a tag in the MSB
// followed by a capability. This capability is used for authorisation.
// The ruser and buser fields are extended by adding a new MSB, used to signal
// whether a CHERI error has occurred.
// For bursts, the full burst size is calculated and checked against the top
// and bottom of the capability; if all bytes are authorised by the capability,
// the request is forwarded onto the bus, otherwise the request stops here and
// never reaches the bus.
// When the capability does authorise the request, it is removed from the
// awflit and this new awflit is put out onto the bus. Once the capability is
// verified to authorise the request, the behaviour of the CHERI checker
// becomes transparent.
// When the capability does not authorise the request, it is not forwarded to
// the external bus. The CHERI checker will generate failing responses and
// send them to the internal requester on the CHERI checker's slave port.
// CHERI authorisation failures are signalled by setting the appropriate rresp
// or bresp field to SLVERR, and the top bit of the appropriate ruser or buser
// field is set to 1.
interface CHERI_Checker_IFC #( numeric type id_
                             , numeric type addr_
                             , numeric type data_
                             , numeric type awuser_o_
                             , numeric type wuser_
                             , numeric type buser_o_
                             , numeric type aruser_o_
                             , numeric type ruser_o_);
   interface AXI4_Master #(id_, addr_, data_,
                           awuser_o_, wuser_, buser_o_,
                           aruser_o_, ruser_o_) master;
   interface AXI4_Slave #(id_, addr_, data_,
                          TAdd #(SizeOf #(CapMem), awuser_o_), wuser_, TAdd #(1, buser_o_),
                          TAdd #(SizeOf #(CapMem), aruser_o_), TAdd #(1, ruser_o_)) slave;
   method Action clear;
   method Action set_verbosity (Bit #(4) new_verb);
endinterface

module mkCHERI_Checker (CHERI_Checker_IFC #(id_, addr_, data_, awuser_o_, wuser_, buser_o_, aruser_o_, ruser_o_))
                      provisos ( Add #(c__, 7, addr_)
                               , Add #(d__, addr_, 64));

   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);

   // connected to the outside
   let inShim <- mkAXI4ShimFF;
   let outShim <- mkAXI4ShimFF;
   // serialised input
   let inSerial <- mkSerialiser (inShim.master);


   Reg #(State) rg_state <- mkReg (IDLE);

   Reg #(Bit #(TAdd #(SizeOf #(AXI4_Len), 1))) rg_count <- mkRegU;

   Reg #(Mem_Op) rg_mem_op <- mkRegU;

   Wire #(Bool) dw_req_allowed <- mkDReg (False);

   // TODO this doesn't support WRAP bursts at all
   function Bool fn_allow_flit ( Bit #(addr_) addr
                               , AXI4_Len len
                               , AXI4_Size size
                               , AXI4_Burst burst
                               , CapMem cap);
      CapPipe cap_fat = cast (cap);
      let valid_tag = isValidCap (cap_fat);
      let cap_base = getBase (cap_fat);
      let cap_top = getTop (cap_fat);
      let size_full = fromAXI4_Size (size);

      // The byte _after_ the last one that will be read
      // Size is increased to deal with possible overflow
      // A well-behaved peripheral should never issue transactions that overflow,
      // since AXI prohibits bursts crossing 4KB boundaries, but a badly behaved
      // peripheral might not respect this on purpose and could access it's not
      // supposed to access using this
      Bit #(TAdd #(addr_, 1)) next_byte = ?;
      if (burst == INCR) begin
         next_byte = zeroExtend (addr) + zeroExtend ((len + 1) << pack (size));
      end else begin
         next_byte = zeroExtend (addr) + zeroExtend (size_full);
      end

      // capability semantics: bottom address is inclusive, top address is not
      let is_allowed = zeroExtend (addr) >= cap_base
                       && zeroExtend (next_byte) <= cap_top
                       && valid_tag;
      return is_allowed;
   endfunction

   // reduce instantiations of fn_allow_flit by using it only in one place
   rule rl_handle_cur_req;
      let use_aw = inSerial.aw.canPeek;
      let use_ar = inSerial.ar.canPeek;
      let awflit = inSerial.aw.peek;
      let arflit = inSerial.ar.peek;
      let txion_allowed = fn_allow_flit ( use_aw ? awflit.awaddr  : use_ar ? arflit.araddr  : ?
                                        , use_aw ? awflit.awlen   : use_ar ? arflit.arlen   : ?
                                        , use_aw ? awflit.awsize  : use_ar ? arflit.arsize  : ?
                                        , use_aw ? awflit.awburst : use_ar ? arflit.arburst : ?
                                        , use_aw ? unpack (truncateLSB (awflit.awuser)) :
                                          use_ar ? unpack (truncateLSB (arflit.aruser)) :
                                          ?);
      dw_req_allowed <= txion_allowed;
   endrule

   rule rl_handle_new_aw (rg_state == IDLE
                          && inSerial.aw.canPeek
                          && outShim.slave.aw.canPut);
      let awflit = inSerial.aw.peek;
      rg_count <= 0;
      rg_mem_op <= AW;
      let txion_allowed = dw_req_allowed;

      rg_state <= txion_allowed ? FORWARD_W
                                : DENY_W;

      AXI4_AWFlit #(id_, addr_, awuser_o_) out_awflit
         = AXI4_AWFlit { awid    : awflit.awid
                       , awaddr  : awflit.awaddr
                       , awlen   : awflit.awlen
                       , awsize  : awflit.awsize
                       , awburst : awflit.awburst
                       , awlock  : awflit.awlock
                       , awcache : awflit.awcache
                       , awprot  : awflit.awprot
                       , awqos   : awflit.awqos
                       , awregion: awflit.awregion
                       , awuser  : truncate (awflit.awuser)};

      if (txion_allowed) begin
         outShim.slave.aw.put (out_awflit);
         if (rg_verbosity > 1) begin
            $display ("CHERI Checker putting AW flit: ", fshow (out_awflit));
         end
      end

      if (rg_verbosity > 0) begin
         $display ("CHERI Checker AW request");
         $display ("    accepted: ", fshow (txion_allowed));
      end
      if (rg_verbosity >  1) begin
         CapMem cap = unpack (truncate (awflit.awuser));
         CapPipe capPipe = cast (cap);
         $display ("    flit: ", fshow (awflit));
         $display ("    capPipe: ", fshow (capPipe));
         $display ("    out_awflit: ", fshow (out_awflit));
      end
   endrule

   (* mutually_exclusive="rl_handle_new_ar,rl_handle_new_aw" *)
   rule rl_handle_new_ar (rg_state == IDLE && inSerial.ar.canPeek);
      let arflit = inSerial.ar.peek;
      rg_count <= 0;
      rg_mem_op <= AR;
      let txion_allowed = dw_req_allowed;

      rg_state <= txion_allowed ? FORWARD_R
                                : DENY_R;

      AXI4_ARFlit #(id_, addr_, aruser_o_) out_arflit
         = AXI4_ARFlit { arid    : arflit.arid
                       , araddr  : arflit.araddr
                       , arlen   : arflit.arlen
                       , arsize  : arflit.arsize
                       , arburst : arflit.arburst
                       , arlock  : arflit.arlock
                       , arcache : arflit.arcache
                       , arprot  : arflit.arprot
                       , arqos   : arflit.arqos
                       , arregion: arflit.arregion
                       , aruser  : truncate (arflit.aruser)};

      if (txion_allowed) begin
         outShim.slave.ar.put (out_arflit);
      end

      if (rg_verbosity > 0) begin
         $display ("CHERI Checker AR request");
         $display ("    accepted: ", fshow (txion_allowed));
      end
      if (rg_verbosity >  1) begin
         CapMem cap = unpack (truncate (arflit.aruser));
         CapPipe capPipe = cast (cap);
         $display ("    flit: ", fshow (arflit));
         $display ("    capPipe: ", fshow (capPipe));
      end
   endrule

   rule rl_forward_w (rg_state == FORWARD_W
                      && inSerial.w.canPeek
                      && outShim.slave.w.canPut
                      && rg_count <= zeroExtend (inSerial.aw.peek.awlen));
      inSerial.w.drop;
      outShim.slave.w.put (inSerial.w.peek);
      rg_count <= rg_count + 1;
      if (rg_verbosity > 1) begin
         $display ("CHERI Checker W Forward flit: ", fshow (inSerial.w.peek));
         $display ("    rg_count: ", fshow (rg_count));
         $display ("    len: ", fshow (inSerial.aw.peek.awlen));
      end
      if (rg_count == zeroExtend (inSerial.aw.peek.awlen)) begin
         if (rg_verbosity > 0) begin
            $display ("CHERI Checker finished forwarding W flits");
         end
      end
   endrule

   rule rl_deny_w (rg_state == DENY_W
                   && inSerial.w.canPeek
                   && rg_count <= zeroExtend (inSerial.aw.peek.awlen));
      inSerial.w.drop;
      rg_count <= rg_count + 1;
      if (rg_verbosity > 0) begin
         $display ("CHERI Checker W flit denied");
      end
      if (rg_verbosity > 1) begin
         $display ("    flit: ", fshow (inSerial.w.peek));
      end
   endrule


   rule rl_finish_forward_w (rg_state == FORWARD_W
                             && rg_count > zeroExtend (inSerial.aw.peek.awlen)
                             && outShim.slave.b.canPeek
                             && inSerial.b.canPut);
      outShim.slave.b.drop;
      let bflit_in = outShim.slave.b.peek;
      AXI4_BFlit #(id_, TAdd #(1, buser_o_)) bflit_out
         = AXI4_BFlit { bid: bflit_in.bid
                      , bresp: bflit_in.bresp
                      , buser: zeroExtend (bflit_in.buser)};
      inSerial.b.put (bflit_out);

      inSerial.aw.drop;
      rg_state <= IDLE;
      if (rg_verbosity > 0) begin
         $display ("CHERI Checker forwarded B flit");
      end
      if (rg_verbosity > 1) begin
         $display ("    bflit_in: ", fshow (bflit_in));
         $display ("    bflit_out: ", fshow (bflit_out));
      end
   endrule

   rule rl_finish_deny_w (rg_state == DENY_W
                          && rg_count > zeroExtend (inSerial.aw.peek.awlen)
                          && inSerial.b.canPut);
      inSerial.aw.drop;
      AXI4_BFlit #(id_, TAdd #(1, buser_o_)) bflit_out
         = AXI4_BFlit { bid: inSerial.aw.peek.awid
                      , bresp: SLVERR
                      , buser: {1'b1, 0}};

      inSerial.b.put (bflit_out);

      rg_state <= IDLE;
      if (rg_verbosity > 0) begin
         $display ("CHERI Checker finished denied AW, returning B flit");
      end
      if (rg_verbosity > 1) begin
         $display ("    request flit: ", fshow (inSerial.aw.peek));
         $display ("    response bflit: ", fshow (bflit_out));
      end
   endrule

   rule rl_forward_r (rg_state == FORWARD_R
                      && outShim.slave.r.canPeek
                      && inSerial.r.canPut
                      && rg_count <= zeroExtend (inSerial.ar.peek.arlen));
      outShim.slave.r.drop;
      let rflit_in = outShim.slave.r.peek;
      AXI4_RFlit #(id_, data_, TAdd #(1, ruser_o_)) rflit_out
         = AXI4_RFlit { rid: rflit_in.rid
                      , rdata: rflit_in.rdata
                      , rresp: rflit_in.rresp
                      , rlast: rflit_in.rlast
                      , ruser: zeroExtend (rflit_in.ruser)};
      inSerial.r.put (rflit_out);
      rg_count <= rg_count + 1;
      if (rg_verbosity > 1) begin
         $display ("CHERI Checker R Forward flit: ", fshow (rflit_out));
         $display ("                original flit: ", fshow (rflit_in));
      end
      if (rflit_in.rlast) begin
         rg_state <= IDLE;
         inSerial.ar.drop;
         if (rg_verbosity > 0) begin
            $display ("CHERI Checker finished forwarding R flits");
         end
         if (rg_count != zeroExtend (inSerial.ar.peek.arlen)) begin
            $display ("CHERI Checker ERROR: Unexpected number of R flits");
         end
      end
   endrule

   rule rl_deny_r (rg_state == DENY_R
                   && inSerial.r.canPut
                   && rg_count <= zeroExtend (inSerial.ar.peek.arlen));
      AXI4_RFlit #(id_, data_, TAdd #(1, ruser_o_)) rflit_out
         = AXI4_RFlit { rid: inSerial.ar.peek.arid
                      , rdata: ?
                      , rresp: SLVERR
                      , rlast: rg_count == zeroExtend (inSerial.ar.peek.arlen)
                      , ruser: {1'b1, 0}};
      inSerial.r.put (rflit_out);
      rg_count <= rg_count + 1;
      if (rg_count == zeroExtend (inSerial.ar.peek.arlen)) begin
         rg_state <= IDLE;
         inSerial.ar.drop;
         if (rg_verbosity > 0) begin
            $display ("CHERI checker sent last denied R response");
         end
         if (rg_verbosity > 1) begin
            $display ("    flit: ", fshow (rflit_out));
         end
      end
      if (rg_count < zeroExtend (inSerial.ar.peek.arlen)) begin
         if (rg_verbosity > 1) begin
            $display ("CHERI checker sent denied R response");
            $display ("    flit: ", fshow (rflit_out));
         end
      end
   endrule

   rule rl_debug_forward_w (rg_state == FORWARD_W);
      if (rg_verbosity > 1) begin
         $display ("rl_debug_forward_w");
         $display ("    rg_count: ", fshow (rg_count));
         $display ("    len: ", fshow (inSerial.aw.peek.awlen));
         $display ("    inSerial.b.canPut: ", fshow (inSerial.b.canPut));
         $display ("    inSerial.w.canPeek: ", fshow (inSerial.w.canPeek));
         $display ("    outShim.slave.b.canPeek: ", fshow (outShim.slave.b.canPeek));
         $display ("    outShim.slave.w.canPut: ", fshow (outShim.slave.w.canPut));
         $display ("    outShim.slave.aw.canPut: ", fshow (outShim.slave.aw.canPut));
      end
   endrule


   interface slave = inShim.slave;
   interface master = outShim.master;

   method Action clear;
      inShim.clear;
      outShim.clear;
      rg_state <= IDLE;
      // no need to reset rg_count and rg_mem_op since these are both RegU
   endmethod

   method Action set_verbosity (Bit #(4) new_verb);
      rg_verbosity <= new_verb;
   endmethod
endmodule

endpackage
