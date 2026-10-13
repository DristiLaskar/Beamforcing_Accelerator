//------------------------------------------------------------------------------
// BFA - Beamforming Accelerator
// File        : bfa_rnd_sat.v
// Description : Combinational arithmetic right shift with round-half-up,
//               followed by saturation to OUT_W bits two's complement.
//
//               dout = sat( (acc_in + 2^(shift-1)) >>> shift )
//               with  shift == 0  meaning no rounding term is added.
//
//               Rounding is "round half towards +infinity" (add half, then
//               arithmetic shift right). This is NOT round-half-away-from-zero
//               and NOT convergent rounding; -1.5 rounds to -1.
//
//               Saturation clamps to [-2^(OUT_W-1), 2^(OUT_W-1)-1] and reports
//               `ovf` whenever clamping was required.
//
// Language    : Verilog-2001
//------------------------------------------------------------------------------

module bfa_rnd_sat #(
  parameter integer ACC_W  = 40,   // Accumulator width  (signed)
  parameter integer OUT_W  = 16,   // Output width       (signed)
  parameter integer SHFT_W = 6     // Shift control width
) (
  input  wire signed [ACC_W-1:0]  acc_in,
  input  wire        [SHFT_W-1:0] shift,
  output wire signed [OUT_W-1:0]  dout,
  output wire                     ovf
);

  // Two extra bits: one for the rounding carry, one to keep the sum
  // unambiguously signed with headroom.
  localparam integer EXT_W = ACC_W + 2;

  wire signed [EXT_W-1:0] acc_ext;
  wire signed [EXT_W-1:0] one_ext;
  wire signed [EXT_W-1:0] rnd_add;
  wire signed [EXT_W-1:0] sum;
  wire signed [EXT_W-1:0] shifted;
  wire signed [EXT_W-1:0] max_v;
  wire signed [EXT_W-1:0] min_v;

  assign acc_ext = {{2{acc_in[ACC_W-1]}}, acc_in};
  assign one_ext = {{(EXT_W-1){1'b0}}, 1'b1};

  // When shift == 0 the (shift-1) expression underflows, but that branch of the
  // ternary is not selected, and the shift amount is bounded by SHFT_W bits so
  // the discarded value is a well-defined 0 rather than X.
  assign rnd_add = (shift == {SHFT_W{1'b0}}) ? {EXT_W{1'b0}}
                                             : (one_ext <<< (shift - 1'b1));

  assign sum     = acc_ext + rnd_add;
  assign shifted = sum >>> shift;

  //  max_v = +2^(OUT_W-1) - 1   ->  (EXT_W-OUT_W+1) zeros, (OUT_W-1) ones
  //  min_v = -2^(OUT_W-1)       ->  (EXT_W-OUT_W+1) ones,  (OUT_W-1) zeros
  assign max_v = {{(EXT_W-OUT_W+1){1'b0}}, {(OUT_W-1){1'b1}}};
  assign min_v = {{(EXT_W-OUT_W+1){1'b1}}, {(OUT_W-1){1'b0}}};

  assign ovf  = (shifted > max_v) || (shifted < min_v);

  assign dout = (shifted > max_v) ? max_v[OUT_W-1:0] :
                (shifted < min_v) ? min_v[OUT_W-1:0] :
                                    shifted[OUT_W-1:0];

endmodule
