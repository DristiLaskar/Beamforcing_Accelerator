//------------------------------------------------------------------------------
// BFA - Beamforming Accelerator
// File        : bfa_cmac.v
// Description : Complex multiply-accumulate leaf cell.
//
//                 acc += x * w   where x = x_re + j.x_im
//                                      w = w_re + j.w_im
//
//               Real part : acc_re += (x_re*w_re) - (x_im*w_im)
//               Imag part : acc_im += (x_re*w_im) + (x_im*w_re)
//
//               No conjugation is applied. Software that wants a conjugate
//               beamformer must program conjugated coefficients.
//
// Pipeline    : 2 stages.
//                 en    asserted -> products registered at next edge
//                 en_q  (internal) -> accumulator updated at following edge
//               So the accumulator reflects a sample 2 cycles after `en`.
//
// Language    : Verilog-2001
//------------------------------------------------------------------------------

module bfa_cmac #(
  parameter integer DATA_W = 16,   // Per-component sample width  (signed)
  parameter integer COEF_W = 16,   // Per-component coeff  width  (signed)
  parameter integer ACC_W  = 40    // Accumulator width           (signed)
) (
  input  wire                      ACLK,
  input  wire                      ARESETn,

  input  wire                      acc_clr,   // Synchronous accumulator clear
  input  wire                      en,        // Sample+coeff pair valid

  input  wire signed [DATA_W-1:0]  x_re,
  input  wire signed [DATA_W-1:0]  x_im,
  input  wire signed [COEF_W-1:0]  w_re,
  input  wire signed [COEF_W-1:0]  w_im,

  output wire signed [ACC_W-1:0]   acc_re,
  output wire signed [ACC_W-1:0]   acc_im
);

  localparam integer PROD_W = DATA_W + COEF_W;

  //----------------------------------------------------------------------------
  // Stage 1 : four real products
  //
  // The LHS width (PROD_W) is the context width, so both signed operands are
  // sign-extended to PROD_W before the multiply. All operands are declared
  // signed, therefore the multiply is signed. IEEE 1364-2001 s5.4 / s5.5.1.
  //----------------------------------------------------------------------------
  reg signed [PROD_W-1:0] p_rr, p_ii, p_ri, p_ir;
  reg                     en_q;

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      p_rr <= {PROD_W{1'b0}};
      p_ii <= {PROD_W{1'b0}};
      p_ri <= {PROD_W{1'b0}};
      p_ir <= {PROD_W{1'b0}};
      en_q <= 1'b0;
    end else begin
      // acc_clr also kills the in-flight accumulate enable. Without this a
      // soft reset asserted while a product was in stage 1 would accumulate
      // that stale product one cycle AFTER the accumulator had been cleared.
      en_q <= en & ~acc_clr;
      if (en) begin
        p_rr <= x_re * w_re;
        p_ii <= x_im * w_im;
        p_ri <= x_re * w_im;
        p_ir <= x_im * w_re;
      end
    end
  end

  //----------------------------------------------------------------------------
  // Stage 2 : accumulate
  //
  // acc_clr takes priority over en_q. The controller only asserts acc_clr when
  // the pipeline is drained, so no accumulation is ever silently discarded.
  //----------------------------------------------------------------------------
  reg signed [ACC_W-1:0] acc_re_q, acc_im_q;

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      acc_re_q <= {ACC_W{1'b0}};
      acc_im_q <= {ACC_W{1'b0}};
    end else if (acc_clr) begin
      acc_re_q <= {ACC_W{1'b0}};
      acc_im_q <= {ACC_W{1'b0}};
    end else if (en_q) begin
      acc_re_q <= acc_re_q + p_rr - p_ii;
      acc_im_q <= acc_im_q + p_ri + p_ir;
    end
  end

  assign acc_re = acc_re_q;
  assign acc_im = acc_im_q;

endmodule
