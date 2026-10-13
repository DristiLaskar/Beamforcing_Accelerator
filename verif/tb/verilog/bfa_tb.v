//------------------------------------------------------------------------------
// BFA - Beamforming Accelerator
// File        : bfa_tb.v
// Description : Self-checking testbench.
//
//               The golden model is computed independently in 64-bit integer
//               arithmetic from the same stimulus and then rounded/saturated
//               with the same rule the RTL implements. It reuses no RTL signal,
//               so a datapath error cannot cancel itself out.
//
//               T1  identification registers
//               T2  reset values and HWCFG
//               T3  directed beamforming, back to back, no wait states
//               T4  randomised data with random RX and TX stalls
//               T5  output shift, saturation and the OVF event
//               T6  runtime geometry change (5 elements, 3 beams)
//               T7  protocol error: TLAST too early
//               T8  protocol error: TLAST missing, packet too long
//               T9  PSLVERR cases and register locking
//               T10 interrupt set / enable / write-1-to-clear, SNAPCNT
//               T11 soft reset recovery from an aborted packet
//
//               A free running AXI4-Stream monitor checks that the source never
//               drops TVALID before TREADY and never changes payload mid beat.
//
// Language    : Verilog-2001
//------------------------------------------------------------------------------

`timescale 1ns/1ps
`include "bfa_defs.vh"

module bfa_tb;
   initial begin
     $dumpfile("bfa_tb.vcd");
     $dumpvars(0, bfa_tb);
   end

  //----------------------------------------------------------------------------
  // Configuration under test
  //----------------------------------------------------------------------------
  // Overridable from the command line, e.g. iverilog -DBFA_TB_ELEMENTS=16
`ifndef BFA_TB_ELEMENTS
  `define BFA_TB_ELEMENTS 8
`endif
`ifndef BFA_TB_BEAMS
  `define BFA_TB_BEAMS 4
`endif

  localparam integer NUM_ELEMENTS = `BFA_TB_ELEMENTS;
  localparam integer NUM_BEAMS    = `BFA_TB_BEAMS;
  localparam integer DATA_W       = 16;
  localparam integer COEF_W       = 16;
  localparam integer ACC_W        = 40;
  localparam integer SHFT_W       = 6;
  localparam integer CLK_PERIOD   = 10;

  // Same derivation the DUT uses.
  localparam integer EL_AW = (NUM_ELEMENTS <=  2) ? 1 :
                             (NUM_ELEMENTS <=  4) ? 2 :
                             (NUM_ELEMENTS <=  8) ? 3 :
                             (NUM_ELEMENTS <= 16) ? 4 :
                             (NUM_ELEMENTS <= 32) ? 5 : 6;
  localparam integer BM_AW = (NUM_BEAMS    <=  2) ? 1 :
                             (NUM_BEAMS    <=  4) ? 2 :
                             (NUM_BEAMS    <=  8) ? 3 :
                             (NUM_BEAMS    <= 16) ? 4 :
                             (NUM_BEAMS    <= 32) ? 5 : 6;

  //----------------------------------------------------------------------------
  // DUT connections
  //----------------------------------------------------------------------------
  reg                 ACLK;
  reg                 ARESETn;

  reg                 PSEL;
  reg                 PENABLE;
  reg                 PWRITE;
  reg  [11:0]         PADDR;
  reg  [31:0]         PWDATA;
  reg  [3:0]          PSTRB;
  reg  [2:0]          PPROT;
  wire [31:0]         PRDATA;
  wire                PREADY;
  wire                PSLVERR;

  reg  [2*DATA_W-1:0] RX_TDATA;
  reg                 RX_TVALID;
  wire                RX_TREADY;
  reg                 RX_TLAST;

  wire [2*DATA_W-1:0] TX_TDATA;
  wire                TX_TVALID;
  reg                 TX_TREADY;
  wire                TX_TLAST;
  wire [BM_AW-1:0]    TX_TID;

  wire                IRQ;

  bfa #(
    .NUM_ELEMENTS (NUM_ELEMENTS),
    .NUM_BEAMS    (NUM_BEAMS),
    .DATA_W       (DATA_W),
    .COEF_W       (COEF_W),
    .ACC_W        (ACC_W),
    .SHFT_W       (SHFT_W)
  ) u_dut (
    .ACLK (ACLK), .ARESETn (ARESETn),
    .PSEL (PSEL), .PENABLE (PENABLE), .PWRITE (PWRITE), .PADDR (PADDR),
    .PWDATA (PWDATA), .PSTRB (PSTRB), .PPROT (PPROT),
    .PRDATA (PRDATA), .PREADY (PREADY), .PSLVERR (PSLVERR),
    .RX_TDATA (RX_TDATA), .RX_TVALID (RX_TVALID), .RX_TREADY (RX_TREADY),
    .RX_TLAST (RX_TLAST),
    .TX_TDATA (TX_TDATA), .TX_TVALID (TX_TVALID), .TX_TREADY (TX_TREADY),
    .TX_TLAST (TX_TLAST), .TX_TID (TX_TID),
    .IRQ (IRQ)
  );

  initial begin
    ACLK = 1'b0;
    forever #(CLK_PERIOD/2) ACLK = ~ACLK;
  end

  //----------------------------------------------------------------------------
  // Scoreboard
  //----------------------------------------------------------------------------
  integer errors;
  integer checks;
  integer proto_errs;

  task check_eq32;
    input [255:0] name;
    input [31:0]  got;
    input [31:0]  exp;
    begin
      checks = checks + 1;
      if (got !== exp) begin
        errors = errors + 1;
        $display("  FAIL %0s : got 0x%08h expected 0x%08h (t=%0t)", name, got, exp, $time);
      end
    end
  endtask

  task check_true;
    input [255:0] name;
    input         cond;
    begin
      checks = checks + 1;
      if (cond !== 1'b1) begin
        errors = errors + 1;
        $display("  FAIL %0s (t=%0t)", name, $time);
      end
    end
  endtask

  //----------------------------------------------------------------------------
  // APB4 master model. PSLVERR and PRDATA are sampled inside the access phase.
  //----------------------------------------------------------------------------
  reg        last_slverr;
  reg [31:0] apb_rdata;

  task apb_xfer;
    input        is_write;
    input [11:0] addr;
    input [31:0] data;
    input [3:0]  strb;
    begin
      @(posedge ACLK);
      PSEL    <= 1'b1;
      PENABLE <= 1'b0;
      PWRITE  <= is_write;
      PADDR   <= addr;
      PWDATA  <= data;
      PSTRB   <= strb;
      @(posedge ACLK);
      PENABLE <= 1'b1;
      #(CLK_PERIOD - 1);              // settle within the access phase
      apb_rdata   = PRDATA;
      last_slverr = PSLVERR;
      @(posedge ACLK);
      PSEL    <= 1'b0;
      PENABLE <= 1'b0;
      PWRITE  <= 1'b0;
      PSTRB   <= 4'b1111;
    end
  endtask

  task apb_write;
    input [11:0] addr;
    input [31:0] data;
    begin
      apb_xfer(1'b1, addr, data, 4'b1111);
    end
  endtask

  task apb_write_strb;
    input [11:0] addr;
    input [31:0] data;
    input [3:0]  strb;
    begin
      apb_xfer(1'b1, addr, data, strb);
    end
  endtask

  task apb_read;
    input [11:0] addr;
    begin
      apb_xfer(1'b0, addr, 32'h0, 4'b1111);
    end
  endtask

  //----------------------------------------------------------------------------
  // Golden model
  //----------------------------------------------------------------------------
  reg signed [DATA_W-1:0] smp_re [0:NUM_ELEMENTS-1];
  reg signed [DATA_W-1:0] smp_im [0:NUM_ELEMENTS-1];
  reg signed [COEF_W-1:0] cof_re [0:NUM_BEAMS-1][0:NUM_ELEMENTS-1];
  reg signed [COEF_W-1:0] cof_im [0:NUM_BEAMS-1][0:NUM_ELEMENTS-1];

  reg signed [DATA_W-1:0] exp_re [0:NUM_BEAMS-1];
  reg signed [DATA_W-1:0] exp_im [0:NUM_BEAMS-1];
  reg                     exp_ovf;

  // Round half toward +infinity then saturate. Mirrors bfa_rnd_sat.
  function signed [63:0] gold_shift;
    input signed [63:0] acc;
    input integer       sh;
    reg signed [63:0]   sum;
    begin
      if (sh == 0) sum = acc;
      else         sum = acc + (64'sd1 <<< (sh - 1));
      gold_shift = sum >>> sh;
    end
  endfunction

  function signed [DATA_W-1:0] gold_rndsat;
    input signed [63:0] acc;
    input integer       sh;
    reg signed [63:0]   v, maxv, minv;
    begin
      v    = gold_shift(acc, sh);
      maxv = (64'sd1 <<< (DATA_W-1)) - 64'sd1;
      minv = -(64'sd1 <<< (DATA_W-1));
      if      (v > maxv) gold_rndsat = maxv[DATA_W-1:0];
      else if (v < minv) gold_rndsat = minv[DATA_W-1:0];
      else               gold_rndsat = v[DATA_W-1:0];
    end
  endfunction

  function gold_ovf;
    input signed [63:0] acc;
    input integer       sh;
    reg signed [63:0]   v;
    begin
      v = gold_shift(acc, sh);
      gold_ovf = (v > ((64'sd1 <<< (DATA_W-1)) - 64'sd1)) ||
                 (v < -(64'sd1 <<< (DATA_W-1)));
    end
  endfunction

  task compute_golden;
    input integer nelem;
    input integer nbeam;
    input integer sh;
    integer b, e;
    reg signed [63:0] ar, ai;
    begin
      exp_ovf = 1'b0;
      for (b = 0; b < nbeam; b = b + 1) begin
        ar = 64'sd0;
        ai = 64'sd0;
        for (e = 0; e < nelem; e = e + 1) begin
          ar = ar + ($signed(smp_re[e]) * $signed(cof_re[b][e]))
                  - ($signed(smp_im[e]) * $signed(cof_im[b][e]));
          ai = ai + ($signed(smp_re[e]) * $signed(cof_im[b][e]))
                  + ($signed(smp_im[e]) * $signed(cof_re[b][e]));
        end
        exp_re[b] = gold_rndsat(ar, sh);
        exp_im[b] = gold_rndsat(ai, sh);
        if (gold_ovf(ar, sh) || gold_ovf(ai, sh)) exp_ovf = 1'b1;
      end
    end
  endtask

  //----------------------------------------------------------------------------
  // Stimulus generation
  //----------------------------------------------------------------------------
  integer data_seed;
  integer rx_seed;
  integer tx_seed;

  task randomise_samples;
    input integer nelem;
    input integer amp;
    integer e;
    begin
      for (e = 0; e < NUM_ELEMENTS; e = e + 1) begin
        smp_re[e] = {DATA_W{1'b0}};
        smp_im[e] = {DATA_W{1'b0}};
      end
      for (e = 0; e < nelem; e = e + 1) begin
        smp_re[e] = ($random(data_seed) % amp);
        smp_im[e] = ($random(data_seed) % amp);
      end
    end
  endtask

  task randomise_coeffs;
    input integer nelem;
    input integer nbeam;
    input integer amp;
    integer b, e;
    begin
      for (b = 0; b < NUM_BEAMS; b = b + 1) begin
        for (e = 0; e < NUM_ELEMENTS; e = e + 1) begin
          cof_re[b][e] = {COEF_W{1'b0}};
          cof_im[b][e] = {COEF_W{1'b0}};
        end
      end
      for (b = 0; b < nbeam; b = b + 1) begin
        for (e = 0; e < nelem; e = e + 1) begin
          cof_re[b][e] = ($random(data_seed) % amp);
          cof_im[b][e] = ($random(data_seed) % amp);
        end
      end
    end
  endtask

  task load_coeffs;
    input integer nelem;
    input integer nbeam;
    integer b, e, idx;
    begin
      for (b = 0; b < nbeam; b = b + 1) begin
        for (e = 0; e < nelem; e = e + 1) begin
          idx = (b << EL_AW) + e;
          apb_write(12'h200 + idx[9:0] * 4, {cof_im[b][e], cof_re[b][e]});
        end
      end
    end
  endtask

  task program_and_enable;
    input integer nelem;
    input integer nbeam;
    input integer sh;
    reg [31:0] cfgval;
    begin
      cfgval = ((nbeam - 1) << 8) | (nelem - 1);
      apb_write(`BFA_OFF_CTRL,  32'h0);
      apb_write(`BFA_OFF_CFG,   cfgval);
      apb_write(`BFA_OFF_SHIFT, sh);
      load_coeffs(nelem, nbeam);
      apb_write(`BFA_OFF_CTRL,  32'h1);
    end
  endtask

  //----------------------------------------------------------------------------
  // AXI4-Stream sink driver. Payload is presented at a clock edge and held
  // until the handshake completes, so beats are back to back when stall_pct=0.
  //----------------------------------------------------------------------------
  task send_snapshot;
    input integer nbeats;
    input integer stall_pct;
    input         emit_last;
    integer e, gap;
    begin
      @(posedge ACLK);
      for (e = 0; e < nbeats; e = e + 1) begin
        if (stall_pct > 0) begin
          gap = 0;
          while (($unsigned($random(rx_seed)) % 100) < stall_pct) gap = gap + 1;
          if (gap > 0) begin
            RX_TVALID <= 1'b0;
            RX_TLAST  <= 1'b0;
            repeat (gap) @(posedge ACLK);
          end
        end
        RX_TVALID <= 1'b1;
        RX_TDATA  <= {smp_im[e], smp_re[e]};
        RX_TLAST  <= (emit_last && (e == nbeats-1)) ? 1'b1 : 1'b0;
        @(posedge ACLK);
        while (!((RX_TVALID === 1'b1) && (RX_TREADY === 1'b1))) @(posedge ACLK);
      end
      RX_TVALID <= 1'b0;
      RX_TLAST  <= 1'b0;
    end
  endtask

  //----------------------------------------------------------------------------
  // AXI4-Stream source collector
  //----------------------------------------------------------------------------
  reg [2*DATA_W-1:0] got_data [0:NUM_BEAMS-1];
  reg [BM_AW-1:0]    got_id   [0:NUM_BEAMS-1];
  integer            got_n;
  reg                got_last_ok;

  task collect_beams;
    input integer nbeam;
    input integer stall_pct;
    integer k, gap;
    begin
      got_n       = 0;
      got_last_ok = 1'b1;
      @(posedge ACLK);
      for (k = 0; k < nbeam; k = k + 1) begin
        if (stall_pct > 0) begin
          gap = 0;
          while (($unsigned($random(tx_seed)) % 100) < stall_pct) gap = gap + 1;
          if (gap > 0) begin
            TX_TREADY <= 1'b0;
            repeat (gap) @(posedge ACLK);
          end
        end
        TX_TREADY <= 1'b1;
        @(posedge ACLK);
        while (!((TX_TVALID === 1'b1) && (TX_TREADY === 1'b1))) @(posedge ACLK);
        got_data[k] = TX_TDATA;
        got_id[k]   = TX_TID;
        if (TX_TLAST !== ((k == nbeam-1) ? 1'b1 : 1'b0)) got_last_ok = 1'b0;
        got_n = got_n + 1;
      end
      TX_TREADY <= 1'b0;
    end
  endtask

  task check_beams;
    input [255:0] tag;
    input integer nbeam;
    integer k;
    reg signed [31:0] g_re, g_im, e_re, e_im;
    begin
      check_true(tag, got_last_ok);
      check_eq32(tag, got_n, nbeam);
      for (k = 0; k < nbeam; k = k + 1) begin
        g_re = $signed(got_data[k][DATA_W-1:0]);
        g_im = $signed(got_data[k][2*DATA_W-1:DATA_W]);
        e_re = $signed(exp_re[k]);
        e_im = $signed(exp_im[k]);
        check_eq32(tag, g_re, e_re);
        check_eq32(tag, g_im, e_im);
        check_eq32(tag, {{(32-BM_AW){1'b0}}, got_id[k]}, k);
      end
    end
  endtask

  //----------------------------------------------------------------------------
  // AXI4-Stream source protocol monitor
  //----------------------------------------------------------------------------
  reg                mon_pend;
  reg [2*DATA_W-1:0] mon_data;
  reg                mon_last;
  reg [BM_AW-1:0]    mon_id;

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      mon_pend <= 1'b0;
    end else begin
      if (mon_pend === 1'b1) begin
        if (TX_TVALID !== 1'b1) begin
          proto_errs = proto_errs + 1;
          $display("  FAIL AXIS: TVALID dropped before TREADY (t=%0t)", $time);
        end else if ((TX_TDATA !== mon_data) || (TX_TLAST !== mon_last) ||
                     (TX_TID   !== mon_id)) begin
          proto_errs = proto_errs + 1;
          $display("  FAIL AXIS: payload changed while stalled (t=%0t)", $time);
        end
      end
      mon_pend <= TX_TVALID & ~TX_TREADY;
      mon_data <= TX_TDATA;
      mon_last <= TX_TLAST;
      mon_id   <= TX_TID;
    end
  end

  //----------------------------------------------------------------------------
  // Main sequence
  //----------------------------------------------------------------------------
  integer t, e, b;
  integer n6e, n6b;
  reg [31:0] tmp;

  initial begin
    errors     = 0;
    checks     = 0;
    proto_errs = 0;
    rx_seed    = 32'h1234_5678;
    tx_seed    = 32'h0BAD_C0DE;
    data_seed  = 32'h0EED_BEEF;

    PSEL      = 1'b0;
    PENABLE   = 1'b0;
    PWRITE    = 1'b0;
    PADDR     = 12'h0;
    PWDATA    = 32'h0;
    PSTRB     = 4'b1111;
    PPROT     = 3'b000;
    RX_TDATA  = {(2*DATA_W){1'b0}};
    RX_TVALID = 1'b0;
    RX_TLAST  = 1'b0;
    TX_TREADY = 1'b0;
    ARESETn   = 1'b0;

    repeat (5) @(posedge ACLK);
    ARESETn = 1'b1;
    repeat (3) @(posedge ACLK);

    $display("");
    $display("=== BFA testbench : %0d elements, %0d beams ===", NUM_ELEMENTS, NUM_BEAMS);

    //--------------------------------------------------------------------------
    $display("T1  identification registers");
    apb_read(`BFA_OFF_PERIPHID0); check_eq32("T1 PERIPHID0", apb_rdata, 32'h0000_001F);
    apb_read(`BFA_OFF_PERIPHID1); check_eq32("T1 PERIPHID1", apb_rdata, 32'h0000_001B);
    apb_read(`BFA_OFF_PERIPHID2); check_eq32("T1 PERIPHID2", apb_rdata, 32'h0000_0004);
    apb_read(`BFA_OFF_PERIPHID3); check_eq32("T1 PERIPHID3", apb_rdata, 32'h0000_0000);
    apb_read(`BFA_OFF_PCELLID0);  check_eq32("T1 PCELLID0",  apb_rdata, 32'h0000_000D);
    apb_read(`BFA_OFF_PCELLID1);  check_eq32("T1 PCELLID1",  apb_rdata, 32'h0000_00F0);
    apb_read(`BFA_OFF_PCELLID2);  check_eq32("T1 PCELLID2",  apb_rdata, 32'h0000_0005);
    apb_read(`BFA_OFF_PCELLID3);  check_eq32("T1 PCELLID3",  apb_rdata, 32'h0000_00B1);

    //--------------------------------------------------------------------------
    $display("T2  reset values and HWCFG");
    apb_read(`BFA_OFF_CTRL);    check_eq32("T2 CTRL",    apb_rdata, 32'h0);
    apb_read(`BFA_OFF_STATUS);  check_eq32("T2 STATUS",  apb_rdata, 32'h0);
    apb_read(`BFA_OFF_CFG);     check_eq32("T2 CFG",     apb_rdata,
                                  ((NUM_BEAMS-1) << 8) | (NUM_ELEMENTS-1));
    apb_read(`BFA_OFF_SHIFT);   check_eq32("T2 SHIFT",   apb_rdata, 32'h0);
    apb_read(`BFA_OFF_IRQSTAT); check_eq32("T2 IRQSTAT", apb_rdata, 32'h0);
    apb_read(`BFA_OFF_IRQEN);   check_eq32("T2 IRQEN",   apb_rdata, 32'h0);
    apb_read(`BFA_OFF_SNAPCNT); check_eq32("T2 SNAPCNT", apb_rdata, 32'h0);
    apb_read(`BFA_OFF_HWCFG);   check_eq32("T2 HWCFG",   apb_rdata,
                                  (ACC_W << 24) | (DATA_W << 16) |
                                  (NUM_BEAMS << 8) | NUM_ELEMENTS);

    //--------------------------------------------------------------------------
    $display("T3  directed beamforming, no wait states");
    randomise_coeffs(NUM_ELEMENTS, NUM_BEAMS, 4096);
    program_and_enable(NUM_ELEMENTS, NUM_BEAMS, 15);
    for (t = 0; t < 4; t = t + 1) begin
      randomise_samples(NUM_ELEMENTS, 8192);
      compute_golden(NUM_ELEMENTS, NUM_BEAMS, 15);
      fork
        send_snapshot(NUM_ELEMENTS, 0, 1'b1);
        collect_beams(NUM_BEAMS, 0);
      join
      check_beams("T3", NUM_BEAMS);
    end

    //--------------------------------------------------------------------------
    $display("T4  randomised data with random RX and TX stalls");
    for (t = 0; t < 20; t = t + 1) begin
      randomise_coeffs(NUM_ELEMENTS, NUM_BEAMS, 8192);
      randomise_samples(NUM_ELEMENTS, 16384);
      apb_write(`BFA_OFF_CTRL, 32'h0);
      load_coeffs(NUM_ELEMENTS, NUM_BEAMS);
      apb_write(`BFA_OFF_CTRL, 32'h1);
      compute_golden(NUM_ELEMENTS, NUM_BEAMS, 15);
      fork
        send_snapshot(NUM_ELEMENTS, 40, 1'b1);
        collect_beams(NUM_BEAMS, 50);
      join
      check_beams("T4", NUM_BEAMS);
    end

    //--------------------------------------------------------------------------
    $display("T5  saturation and OVF event");
    for (b = 0; b < NUM_BEAMS; b = b + 1) begin
      for (e = 0; e < NUM_ELEMENTS; e = e + 1) begin
        cof_re[b][e] = 16'sh7FFF;
        cof_im[b][e] = 16'sh0000;
      end
    end
    for (e = 0; e < NUM_ELEMENTS; e = e + 1) begin
      smp_re[e] = 16'sh7FFF;
      smp_im[e] = 16'sh7FFF;
    end
    program_and_enable(NUM_ELEMENTS, NUM_BEAMS, 0);
    apb_write(`BFA_OFF_IRQSTAT, 32'h7);
    compute_golden(NUM_ELEMENTS, NUM_BEAMS, 0);
    check_true("T5 golden predicts ovf", exp_ovf);
    fork
      send_snapshot(NUM_ELEMENTS, 0, 1'b1);
      collect_beams(NUM_BEAMS, 0);
    join
    check_beams("T5 sat", NUM_BEAMS);
    repeat (3) @(posedge ACLK);
    apb_read(`BFA_OFF_IRQSTAT);
    check_true("T5 OVF set",  apb_rdata[`BFA_IRQ_OVF]);
    check_true("T5 DONE set", apb_rdata[`BFA_IRQ_DONE]);

    apb_write(`BFA_OFF_IRQSTAT, 32'h7);
    program_and_enable(NUM_ELEMENTS, NUM_BEAMS, 20);
    compute_golden(NUM_ELEMENTS, NUM_BEAMS, 20);
    check_true("T5 golden predicts no ovf", !exp_ovf);
    fork
      send_snapshot(NUM_ELEMENTS, 0, 1'b1);
      collect_beams(NUM_BEAMS, 0);
    join
    check_beams("T5 scaled", NUM_BEAMS);
    repeat (3) @(posedge ACLK);
    apb_read(`BFA_OFF_IRQSTAT);
    check_true("T5 OVF clear", !apb_rdata[`BFA_IRQ_OVF]);

    //--------------------------------------------------------------------------
    n6e = (NUM_ELEMENTS > 3) ? (NUM_ELEMENTS - 3) : NUM_ELEMENTS;
    n6b = (NUM_BEAMS    > 1) ? (NUM_BEAMS    - 1) : NUM_BEAMS;
    $display("T6  runtime geometry: %0d elements, %0d beams", n6e, n6b);
    randomise_coeffs(n6e, n6b, 4096);
    randomise_samples(n6e, 8192);
    program_and_enable(n6e, n6b, 12);
    apb_read(`BFA_OFF_CFG);
    check_eq32("T6 CFG readback", apb_rdata, ((n6b-1) << 8) | (n6e-1));
    compute_golden(n6e, n6b, 12);
    fork
      send_snapshot(n6e, 20, 1'b1);
      collect_beams(n6b, 20);
    join
    check_beams("T6", n6b);

    //--------------------------------------------------------------------------
    $display("T7  protocol error: TLAST too early");
    randomise_coeffs(NUM_ELEMENTS, NUM_BEAMS, 4096);
    randomise_samples(NUM_ELEMENTS, 8192);
    program_and_enable(NUM_ELEMENTS, NUM_BEAMS, 15);
    // An early TLAST can only exist when the frame is longer than one beat.
    // With NUM_ELEMENTS == 1 the scenario is unreachable, so it is skipped
    // rather than approximated with stimulus that means something else.
    if (NUM_ELEMENTS > 1) begin
      apb_write(`BFA_OFF_IRQSTAT, 32'h7);
      send_snapshot(NUM_ELEMENTS - 1, 0, 1'b1);
      repeat (12) @(posedge ACLK);
      apb_read(`BFA_OFF_IRQSTAT);
      check_true("T7 PERR set",   apb_rdata[`BFA_IRQ_PERR]);
      check_true("T7 no DONE",   !apb_rdata[`BFA_IRQ_DONE]);
      check_true("T7 no TX beat", (TX_TVALID === 1'b0));
    end else begin
      $display("    (skipped: early TLAST unreachable with 1 element)");
    end

    apb_write(`BFA_OFF_IRQSTAT, 32'h7);
    randomise_samples(NUM_ELEMENTS, 8192);
    compute_golden(NUM_ELEMENTS, NUM_BEAMS, 15);
    fork
      send_snapshot(NUM_ELEMENTS, 0, 1'b1);
      collect_beams(NUM_BEAMS, 0);
    join
    check_beams("T7 recovery", NUM_BEAMS);

    //--------------------------------------------------------------------------
    $display("T8  protocol error: packet too long");
    apb_write(`BFA_OFF_IRQSTAT, 32'h7);
    randomise_samples(NUM_ELEMENTS, 8192);
    send_snapshot(NUM_ELEMENTS+3, 0, 1'b1);
    repeat (12) @(posedge ACLK);
    apb_read(`BFA_OFF_IRQSTAT);
    check_true("T8 PERR set",   apb_rdata[`BFA_IRQ_PERR]);
    check_true("T8 no DONE",   !apb_rdata[`BFA_IRQ_DONE]);
    check_true("T8 no TX beat", (TX_TVALID === 1'b0));

    apb_write(`BFA_OFF_IRQSTAT, 32'h7);
    randomise_samples(NUM_ELEMENTS, 8192);
    compute_golden(NUM_ELEMENTS, NUM_BEAMS, 15);
    fork
      send_snapshot(NUM_ELEMENTS, 0, 1'b1);
      collect_beams(NUM_BEAMS, 0);
    join
    check_beams("T8 recovery", NUM_BEAMS);

    //--------------------------------------------------------------------------
    $display("T9  PSLVERR and register locking");
    apb_write(`BFA_OFF_CTRL, 32'h0);
    check_true("T9 CTRL write ok", !last_slverr);

    apb_read(12'h002);
    check_true("T9 unaligned read err", last_slverr);
    apb_write(12'h006, 32'h0);
    check_true("T9 unaligned write err", last_slverr);

    apb_read(12'h0A0);
    check_true("T9 undecoded read err", last_slverr);

    apb_write_strb(`BFA_OFF_SHIFT, 32'h5, 4'b0011);
    check_true("T9 partial strobe err", last_slverr);

    apb_read(12'h200);
    check_true("T9 coeff read err", last_slverr);

    apb_write(12'h200 + (NUM_BEAMS * (1 << EL_AW)) * 4, 32'h0);
    check_true("T9 coeff oob err", last_slverr);

    apb_write(`BFA_OFF_CFG, NUM_ELEMENTS);
    check_true("T9 CFG range err", last_slverr);

    apb_write(`BFA_OFF_SHIFT, ACC_W);
    check_true("T9 SHIFT range err", last_slverr);

    apb_write(`BFA_OFF_STATUS, 32'hFFFF_FFFF);
    check_true("T9 RO write no err", !last_slverr);
    apb_read(`BFA_OFF_STATUS);
    check_eq32("T9 STATUS unchanged", apb_rdata, 32'h0);

    apb_write(`BFA_OFF_CTRL, 32'h1);
    check_true("T9 enable ok", !last_slverr);
    apb_write(`BFA_OFF_CFG, 32'h0);
    check_true("T9 CFG locked", last_slverr);
    apb_write(`BFA_OFF_SHIFT, 32'h0);
    check_true("T9 SHIFT locked", last_slverr);
    apb_write(12'h200, 32'h1234_5678);
    check_true("T9 coeff locked", last_slverr);
    apb_write(`BFA_OFF_CTRL, 32'h0);
    check_true("T9 disable ok", !last_slverr);

    //--------------------------------------------------------------------------
    $display("T10 interrupts and SNAPCNT");
    randomise_coeffs(NUM_ELEMENTS, NUM_BEAMS, 2048);
    program_and_enable(NUM_ELEMENTS, NUM_BEAMS, 18);
    apb_write(`BFA_OFF_IRQSTAT, 32'h7);
    apb_write(`BFA_OFF_IRQEN,   32'h0);
    apb_read(`BFA_OFF_SNAPCNT);
    tmp = apb_rdata;

    randomise_samples(NUM_ELEMENTS, 4096);
    compute_golden(NUM_ELEMENTS, NUM_BEAMS, 18);
    fork
      send_snapshot(NUM_ELEMENTS, 0, 1'b1);
      collect_beams(NUM_BEAMS, 0);
    join
    check_beams("T10", NUM_BEAMS);
    repeat (3) @(posedge ACLK);

    check_true("T10 IRQ masked low", (IRQ === 1'b0));
    apb_read(`BFA_OFF_IRQSTAT);
    check_true("T10 DONE latched", apb_rdata[`BFA_IRQ_DONE]);
    apb_write(`BFA_OFF_IRQEN, 32'h1);
    repeat (2) @(posedge ACLK);
    check_true("T10 IRQ asserted", (IRQ === 1'b1));
    apb_write(`BFA_OFF_IRQSTAT, 32'h1);
    repeat (2) @(posedge ACLK);
    check_true("T10 IRQ cleared", (IRQ === 1'b0));
    apb_read(`BFA_OFF_SNAPCNT);
    check_eq32("T10 SNAPCNT +1", apb_rdata, tmp + 32'd1);

    //--------------------------------------------------------------------------
    $display("T11 soft reset recovery from an aborted packet");
    apb_write(`BFA_OFF_IRQSTAT, 32'h7);
    @(posedge ACLK);
    RX_TLAST <= 1'b0;
    for (e = 0; e < 3; e = e + 1) begin
      RX_TVALID <= 1'b1;
      RX_TDATA  <= {smp_im[e], smp_re[e]};
      @(posedge ACLK);
      while (!((RX_TVALID === 1'b1) && (RX_TREADY === 1'b1))) @(posedge ACLK);
    end
    RX_TVALID <= 1'b0;

    apb_write(`BFA_OFF_CTRL, 32'h3);        // EN together with SRST
    repeat (6) @(posedge ACLK);
    apb_read(`BFA_OFF_SNAPCNT);
    check_eq32("T11 SNAPCNT zeroed", apb_rdata, 32'h0);
    apb_read(`BFA_OFF_IRQSTAT);
    check_eq32("T11 IRQSTAT zeroed", apb_rdata, 32'h0);
    apb_read(`BFA_OFF_STATUS);
    check_eq32("T11 idle, still enabled", apb_rdata, 32'h2);

    randomise_samples(NUM_ELEMENTS, 4096);
    compute_golden(NUM_ELEMENTS, NUM_BEAMS, 18);
    fork
      send_snapshot(NUM_ELEMENTS, 0, 1'b1);
      collect_beams(NUM_BEAMS, 0);
    join
    check_beams("T11 recovery", NUM_BEAMS);

    //--------------------------------------------------------------------------
    // Boundary geometries. The single-beam case is the only one that exercises
    // TLAST asserted on the very first output beat, and the single-element case
    // is the only one where TLAST arrives on the very first input beat.
    $display("T12 boundary geometries");

    // (a) 1 element, 1 beam
    randomise_coeffs(1, 1, 4096);
    randomise_samples(1, 8192);
    program_and_enable(1, 1, 4);
    compute_golden(1, 1, 4);
    fork
      send_snapshot(1, 0, 1'b1);
      collect_beams(1, 0);
    join
    check_beams("T12a 1elem 1beam", 1);

    // (b) 1 element, all beams
    randomise_coeffs(1, NUM_BEAMS, 4096);
    randomise_samples(1, 8192);
    program_and_enable(1, NUM_BEAMS, 4);
    compute_golden(1, NUM_BEAMS, 4);
    fork
      send_snapshot(1, 30, 1'b1);
      collect_beams(NUM_BEAMS, 30);
    join
    check_beams("T12b 1elem Nbeam", NUM_BEAMS);

    // (c) all elements, 1 beam
    randomise_coeffs(NUM_ELEMENTS, 1, 4096);
    randomise_samples(NUM_ELEMENTS, 8192);
    program_and_enable(NUM_ELEMENTS, 1, 10);
    compute_golden(NUM_ELEMENTS, 1, 10);
    fork
      send_snapshot(NUM_ELEMENTS, 30, 1'b1);
      collect_beams(1, 30);
    join
    check_beams("T12c Nelem 1beam", 1);

    // (d) With a 1 element geometry an early TLAST is impossible, so exercise
    //     the opposite error: a 2 beat packet must be rejected as too long.
    //     The geometry is programmed here rather than inherited from T12c,
    //     otherwise a 2 beat packet is well formed when NUM_ELEMENTS == 2.
    randomise_coeffs(1, 1, 4096);
    program_and_enable(1, 1, 4);
    apb_write(`BFA_OFF_IRQSTAT, 32'h7);
    send_snapshot(2, 0, 1'b1);
    repeat (12) @(posedge ACLK);
    apb_read(`BFA_OFF_IRQSTAT);
    check_true("T12d PERR on overlong 1elem packet", apb_rdata[`BFA_IRQ_PERR]);
    check_true("T12d no DONE",                      !apb_rdata[`BFA_IRQ_DONE]);

    //--------------------------------------------------------------------------
    repeat (10) @(posedge ACLK);
    errors = errors + proto_errs;
    $display("");
    $display("=== checks run : %0d ===", checks);
    if (errors == 0) $display("=== TEST PASSED : 0 errors ===");
    else             $display("=== TEST FAILED : %0d errors ===", errors);
    $display("");
    $finish;
  end

  initial begin
    #2_000_000;
    $display("=== TEST FAILED : timeout ===");
    $finish;
  end

endmodule
