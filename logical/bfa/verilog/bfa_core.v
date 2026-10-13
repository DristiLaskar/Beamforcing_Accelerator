//------------------------------------------------------------------------------
// BFA - Beamforming Accelerator
// File        : bfa_core.v
// Description : Beamforming datapath and AXI4-Stream controller.
//
// FUNCTION
//   For each input "snapshot" of N element samples the block computes B beam
//   outputs:
//
//       y[b] = SUM over e of ( w[b][e] * x[e] )     b = 0..B-1
//
//   All B beams are computed concurrently: every accepted input beat is
//   broadcast to B complex MACs, each with its own coefficient memory.
//
// STREAM FORMAT
//   RX : one beat per antenna element, N beats per packet, TLAST on beat N-1.
//   TX : one beat per beam, B beats per packet, TLAST on beat B-1, TID = beam.
//   TDATA on both = { imag[DATA_W-1:0], real[DATA_W-1:0] }, two's complement.
//
// PIPELINE ALIGNMENT (this is the part that is easy to get wrong)
//   cycle 0 : RX handshake. Coefficient RAM read issued at raddr = elem_cnt.
//   cycle 1 : RAM rdata valid and aligned with the registered sample (s1).
//             cmac `en` asserted.
//   cycle 2 : cmac product registers loaded.
//   cycle 3 : cmac accumulator updated; final value readable this cycle.
//   The valid shift register s1_v/v_m/v_a tracks exactly this, and results are
//   only captured once all three are clear, giving one cycle of margin.
//
// THROUGHPUT
//   One snapshot occupies N + 4 + B cycles. RX and TX do not overlap: TREADY is
//   held low while results drain. See docs/bfa_integration.md for the rationale
//   and for what an overlapped variant would require.
//
// Language    : Verilog-2001
//------------------------------------------------------------------------------

module bfa_core #(
  parameter integer NUM_ELEMENTS = 8,
  parameter integer NUM_BEAMS    = 4,
  parameter integer DATA_W       = 16,
  parameter integer COEF_W       = 16,
  parameter integer ACC_W        = 40,
  parameter integer EL_AW        = 3,   // ceil(log2(NUM_ELEMENTS)), min 1
  parameter integer BM_AW        = 2,   // ceil(log2(NUM_BEAMS)),    min 1
  parameter integer SHFT_W       = 6
) (
  input  wire                      ACLK,
  input  wire                      ARESETn,

  // Configuration (static while CTRL.EN = 1)
  input  wire                      cfg_en,
  input  wire                      core_rst,      // 1-cycle soft reset pulse
  input  wire [EL_AW-1:0]          cfg_nelem_m1,
  input  wire [BM_AW-1:0]          cfg_nbeam_m1,
  input  wire [SHFT_W-1:0]         cfg_shift,

  // Coefficient write port (from APB register block, same clock)
  input  wire                      wgt_we,
  input  wire [BM_AW-1:0]          wgt_beam,
  input  wire [EL_AW-1:0]          wgt_elem,
  input  wire [2*COEF_W-1:0]       wgt_wdata,

  // AXI4-Stream sink : element samples
  input  wire [2*DATA_W-1:0]       RX_TDATA,
  input  wire                      RX_TVALID,
  output wire                      RX_TREADY,
  input  wire                      RX_TLAST,

  // AXI4-Stream source : beam samples
  output wire [2*DATA_W-1:0]       TX_TDATA,
  output wire                      TX_TVALID,
  input  wire                      TX_TREADY,
  output wire                      TX_TLAST,
  output wire [BM_AW-1:0]          TX_TID,

  // Status / events (single-cycle pulses)
  output wire                      stat_busy,
  output wire                      evt_done,
  output wire                      evt_ovf,
  output wire                      evt_perr
);

  //----------------------------------------------------------------------------
  // Elaboration-time check that the derived widths were not overridden with
  // inconsistent values at instantiation (simulation only).
  //----------------------------------------------------------------------------
`ifndef SYNTHESIS
  initial begin
    if ((1 << EL_AW) < NUM_ELEMENTS) begin
      $display("BFA ERROR: EL_AW=%0d too small for NUM_ELEMENTS=%0d.", EL_AW, NUM_ELEMENTS);
      $finish;
    end
    if ((1 << BM_AW) < NUM_BEAMS) begin
      $display("BFA ERROR: BM_AW=%0d too small for NUM_BEAMS=%0d.", BM_AW, NUM_BEAMS);
      $finish;
    end
    if (ACC_W < (DATA_W + COEF_W)) begin
      $display("BFA ERROR: ACC_W=%0d must be at least DATA_W+COEF_W=%0d.",
               ACC_W, DATA_W+COEF_W);
      $finish;
    end
  end
`endif

  //----------------------------------------------------------------------------
  // FSM encoding
  //----------------------------------------------------------------------------
  localparam [1:0] ST_ACC    = 2'd0;  // Accepting and accumulating a snapshot
  localparam [1:0] ST_FLUSH  = 2'd1;  // Waiting for the MAC pipeline to drain
  localparam [1:0] ST_OUT    = 2'd2;  // Draining beam results onto TX
  localparam [1:0] ST_EDRAIN = 2'd3;  // Error: discarding beats up to TLAST

  reg  [1:0]           state;
  reg                  discard;       // Flush leads back to ST_ACC, not ST_OUT
  reg  [EL_AW-1:0]     elem_cnt;
  reg  [BM_AW-1:0]     out_cnt;

  //----------------------------------------------------------------------------
  // Handshakes
  //----------------------------------------------------------------------------
  wire rx_hs;
  wire pipe_en;

  assign RX_TREADY = cfg_en & ((state == ST_ACC) | (state == ST_EDRAIN));
  assign rx_hs     = RX_TVALID & RX_TREADY;
  assign pipe_en   = rx_hs & (state == ST_ACC);   // ST_EDRAIN beats are dropped

  //----------------------------------------------------------------------------
  // Stage 1 : sample registered to align with coefficient RAM read latency
  //----------------------------------------------------------------------------
  reg                  s1_v;
  reg  [2*DATA_W-1:0]  s1_data;
  reg                  v_m;   // cmac product stage occupied
  reg                  v_a;   // one extra cycle of drain margin
  wire                 pipe_busy;

  assign pipe_busy = s1_v | v_m | v_a;

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      s1_v    <= 1'b0;
      v_m     <= 1'b0;
      v_a     <= 1'b0;
      s1_data <= {(2*DATA_W){1'b0}};
    end else if (core_rst) begin
      s1_v    <= 1'b0;
      v_m     <= 1'b0;
      v_a     <= 1'b0;
    end else begin
      s1_v <= pipe_en;
      v_m  <= s1_v;
      v_a  <= v_m;
      if (pipe_en) begin
        s1_data <= RX_TDATA;
      end
    end
  end

  //----------------------------------------------------------------------------
  // Accumulator clear
  //----------------------------------------------------------------------------
  reg  acc_clr_q;
  wire acc_clr;
  assign acc_clr = acc_clr_q | core_rst;

  //----------------------------------------------------------------------------
  // Per-beam coefficient memory, complex MAC and output scaling
  //----------------------------------------------------------------------------
  wire [NUM_BEAMS*2*COEF_W-1:0] wgt_rdata;
  wire [NUM_BEAMS*ACC_W-1:0]    acc_re;
  wire [NUM_BEAMS*ACC_W-1:0]    acc_im;
  wire [NUM_BEAMS*DATA_W-1:0]   sat_re;
  wire [NUM_BEAMS*DATA_W-1:0]   sat_im;
  wire [NUM_BEAMS-1:0]          ovf_re;
  wire [NUM_BEAMS-1:0]          ovf_im;

  genvar gb;
  generate
    for (gb = 0; gb < NUM_BEAMS; gb = gb + 1) begin : g_beam

      wire beam_we = wgt_we & (wgt_beam == gb[BM_AW-1:0]);

      bfa_wgt_ram #(
        .AW    (EL_AW),
        .DW    (2*COEF_W)
      ) u_wgt_ram (
        .ACLK  (ACLK),
        .we    (beam_we),
        .waddr (wgt_elem),
        .wdata (wgt_wdata),
        .re    (pipe_en),
        .raddr (elem_cnt),
        .rdata (wgt_rdata[gb*2*COEF_W +: 2*COEF_W])
      );

      bfa_cmac #(
        .DATA_W  (DATA_W),
        .COEF_W  (COEF_W),
        .ACC_W   (ACC_W)
      ) u_cmac (
        .ACLK    (ACLK),
        .ARESETn (ARESETn),
        .acc_clr (acc_clr),
        .en      (s1_v),
        .x_re    (s1_data[0        +: DATA_W]),
        .x_im    (s1_data[DATA_W   +: DATA_W]),
        .w_re    (wgt_rdata[gb*2*COEF_W          +: COEF_W]),
        .w_im    (wgt_rdata[gb*2*COEF_W + COEF_W +: COEF_W]),
        .acc_re  (acc_re[gb*ACC_W +: ACC_W]),
        .acc_im  (acc_im[gb*ACC_W +: ACC_W])
      );

      bfa_rnd_sat #(
        .ACC_W  (ACC_W),
        .OUT_W  (DATA_W),
        .SHFT_W (SHFT_W)
      ) u_sat_re (
        .acc_in (acc_re[gb*ACC_W +: ACC_W]),
        .shift  (cfg_shift),
        .dout   (sat_re[gb*DATA_W +: DATA_W]),
        .ovf    (ovf_re[gb])
      );

      bfa_rnd_sat #(
        .ACC_W  (ACC_W),
        .OUT_W  (DATA_W),
        .SHFT_W (SHFT_W)
      ) u_sat_im (
        .acc_in (acc_im[gb*ACC_W +: ACC_W]),
        .shift  (cfg_shift),
        .dout   (sat_im[gb*DATA_W +: DATA_W]),
        .ovf    (ovf_im[gb])
      );

    end
  endgenerate

  //----------------------------------------------------------------------------
  // Overflow is only reported for beams that are actually enabled
  //----------------------------------------------------------------------------
  reg  [NUM_BEAMS-1:0] beam_mask;
  integer bi;
  always @(*) begin
    beam_mask = {NUM_BEAMS{1'b0}};
    for (bi = 0; bi < NUM_BEAMS; bi = bi + 1) begin
      beam_mask[bi] = (bi <= {{(32-BM_AW){1'b0}}, cfg_nbeam_m1});
    end
  end

  wire any_ovf = |(ovf_re & beam_mask) | |(ovf_im & beam_mask);

  //----------------------------------------------------------------------------
  // Result holding registers, TX output registers and the control FSM
  //----------------------------------------------------------------------------
  reg [2*DATA_W-1:0] res_q [0:NUM_BEAMS-1];

  reg [2*DATA_W-1:0] tx_data_q;
  reg                tx_valid_q;
  reg                tx_last_q;
  reg [BM_AW-1:0]    tx_id_q;

  reg                evt_done_q;
  reg                evt_ovf_q;
  reg                evt_perr_q;

  integer ri;

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      state      <= ST_ACC;
      discard    <= 1'b0;
      elem_cnt   <= {EL_AW{1'b0}};
      out_cnt    <= {BM_AW{1'b0}};
      acc_clr_q  <= 1'b0;
      tx_data_q  <= {(2*DATA_W){1'b0}};
      tx_valid_q <= 1'b0;
      tx_last_q  <= 1'b0;
      tx_id_q    <= {BM_AW{1'b0}};
      evt_done_q <= 1'b0;
      evt_ovf_q  <= 1'b0;
      evt_perr_q <= 1'b0;
      for (ri = 0; ri < NUM_BEAMS; ri = ri + 1) begin
        res_q[ri] <= {(2*DATA_W){1'b0}};
      end
    end else if (core_rst) begin
      // Soft reset behaves as a local reset. It may abort an in-flight TX
      // packet; the receiver must treat SRST like ARESETn.
      state      <= ST_ACC;
      discard    <= 1'b0;
      elem_cnt   <= {EL_AW{1'b0}};
      out_cnt    <= {BM_AW{1'b0}};
      acc_clr_q  <= 1'b0;
      tx_valid_q <= 1'b0;
      tx_last_q  <= 1'b0;
      evt_done_q <= 1'b0;
      evt_ovf_q  <= 1'b0;
      evt_perr_q <= 1'b0;
    end else begin
      // Defaults: all pulses are single cycle
      acc_clr_q  <= 1'b0;
      evt_done_q <= 1'b0;
      evt_ovf_q  <= 1'b0;
      evt_perr_q <= 1'b0;

      case (state)

        //----------------------------------------------------------------------
        ST_ACC: begin
          if (rx_hs) begin
            if (RX_TLAST) begin
              elem_cnt <= {EL_AW{1'b0}};
              state    <= ST_FLUSH;
              if (elem_cnt == cfg_nelem_m1) begin
                discard <= 1'b0;              // Well formed packet
              end else begin
                discard    <= 1'b1;           // TLAST too early: short packet
                evt_perr_q <= 1'b1;
              end
            end else if (elem_cnt == cfg_nelem_m1) begin
              // Element budget spent but no TLAST: packet is too long.
              elem_cnt   <= {EL_AW{1'b0}};
              state      <= ST_EDRAIN;
              evt_perr_q <= 1'b1;
            end else begin
              elem_cnt <= elem_cnt + 1'b1;
            end
          end
        end

        //----------------------------------------------------------------------
        ST_EDRAIN: begin
          // Consume the remainder of the malformed packet without accumulating.
          if (rx_hs && RX_TLAST) begin
            discard <= 1'b1;
            state   <= ST_FLUSH;
          end
        end

        //----------------------------------------------------------------------
        ST_FLUSH: begin
          if (!pipe_busy) begin
            acc_clr_q <= 1'b1;
            if (discard) begin
              discard <= 1'b0;
              state   <= ST_ACC;
            end else begin
              // Latch every beam result, then present beam 0 on TX.
              for (ri = 0; ri < NUM_BEAMS; ri = ri + 1) begin
                res_q[ri] <= {sat_im[ri*DATA_W +: DATA_W],
                              sat_re[ri*DATA_W +: DATA_W]};
              end
              tx_data_q  <= {sat_im[0 +: DATA_W], sat_re[0 +: DATA_W]};
              tx_id_q    <= {BM_AW{1'b0}};
              tx_last_q  <= (cfg_nbeam_m1 == {BM_AW{1'b0}});
              tx_valid_q <= 1'b1;
              out_cnt    <= {BM_AW{1'b0}};
              evt_done_q <= 1'b1;
              evt_ovf_q  <= any_ovf;
              state      <= ST_OUT;
            end
          end
        end

        //----------------------------------------------------------------------
        ST_OUT: begin
          if (tx_valid_q && TX_TREADY) begin
            if (out_cnt == cfg_nbeam_m1) begin
              tx_valid_q <= 1'b0;
              tx_last_q  <= 1'b0;
              state      <= ST_ACC;
            end else begin
              out_cnt   <= out_cnt + 1'b1;
              tx_data_q <= res_q[out_cnt + 1];
              tx_id_q   <= out_cnt + 1'b1;
              tx_last_q <= ((out_cnt + 1) == {{(32-BM_AW){1'b0}}, cfg_nbeam_m1});
            end
          end
        end

        //----------------------------------------------------------------------
        default: begin
          state <= ST_ACC;
        end

      endcase
    end
  end

  //----------------------------------------------------------------------------
  // Outputs
  //
  // TX_TVALID is a register and has no combinational dependency on TX_TREADY,
  // as required of an AXI4-Stream source. Payload only changes on a completed
  // handshake or while TVALID is low.
  //----------------------------------------------------------------------------
  assign TX_TDATA  = tx_data_q;
  assign TX_TVALID = tx_valid_q;
  assign TX_TLAST  = tx_last_q;
  assign TX_TID    = tx_id_q;

  assign stat_busy = (state != ST_ACC) | pipe_busy | (elem_cnt != {EL_AW{1'b0}});

  assign evt_done  = evt_done_q;
  assign evt_ovf   = evt_ovf_q;
  assign evt_perr  = evt_perr_q;

endmodule
