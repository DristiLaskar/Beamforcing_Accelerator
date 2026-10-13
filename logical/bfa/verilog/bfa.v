//------------------------------------------------------------------------------
// BFA - Beamforming Accelerator
// File        : bfa.v
// Description : Top level. AMBA APB4 control plane, AMBA AXI4-Stream data
//               plane, PrimeCell-style identification registers, one level
//               sensitive interrupt.
//
// INTERFACES
//   APB4 slave      : control and status, 4KB aperture, 12-bit PADDR.
//   AXI4-Stream sink: RX_*, antenna element samples, N beats per packet.
//   AXI4-Stream src : TX_*, beam samples, B beats per packet, TID = beam index.
//
// CLOCKING AND RESET  -- IMPORTANT
//   This block is SINGLE CLOCK. There is one clock port (ACLK) and one
//   asynchronous, active-LOW reset port (ARESETn). The APB interface has no
//   separate PCLK/PRESETn ports because no clock domain crossing logic is
//   implemented; the APB slave must be driven from the same clock and reset as
//   the stream interfaces. Connecting a different PCLK will produce metastable
//   behaviour. See docs/bfa_integration.md.
//
// Language    : Verilog-2001
//------------------------------------------------------------------------------

module bfa #(
  parameter integer NUM_ELEMENTS = 8,    // Antenna elements per snapshot
  parameter integer NUM_BEAMS    = 4,    // Concurrent beams
  parameter integer DATA_W       = 16,   // Sample component width
  parameter integer COEF_W       = 16,   // Coefficient component width
  parameter integer ACC_W        = 40,   // Accumulator width
  parameter integer SHFT_W       = 6,    // Output shift control width

  //--------------------------------------------------------------------------
  // DERIVED. Do not override at instantiation. These are declared as
  // parameters rather than localparams only because Verilog-2001 requires
  // any identifier used in an ANSI port declaration (TX_TID) to be visible in
  // the parameter list. An elaboration check in bfa_core verifies that they
  // are consistent with NUM_ELEMENTS / NUM_BEAMS.
  //
  // ceil(log2(n)) with a floor of 1, expressed without a constant function so
  // that it is accepted in a parameter context by every tool.
  //--------------------------------------------------------------------------
  parameter integer EL_AW = (NUM_ELEMENTS <=  2) ? 1 :
                            (NUM_ELEMENTS <=  4) ? 2 :
                            (NUM_ELEMENTS <=  8) ? 3 :
                            (NUM_ELEMENTS <= 16) ? 4 :
                            (NUM_ELEMENTS <= 32) ? 5 : 6,
  parameter integer BM_AW = (NUM_BEAMS    <=  2) ? 1 :
                            (NUM_BEAMS    <=  4) ? 2 :
                            (NUM_BEAMS    <=  8) ? 3 :
                            (NUM_BEAMS    <= 16) ? 4 :
                            (NUM_BEAMS    <= 32) ? 5 : 6
) (
  //--------------------------------------------------------------------------
  // Clock and reset
  //--------------------------------------------------------------------------
  input  wire                  ACLK,
  input  wire                  ARESETn,

  //--------------------------------------------------------------------------
  // AMBA APB4 slave
  //--------------------------------------------------------------------------
  input  wire                  PSEL,
  input  wire                  PENABLE,
  input  wire                  PWRITE,
  input  wire [11:0]           PADDR,
  input  wire [31:0]           PWDATA,
  input  wire [3:0]            PSTRB,
  input  wire [2:0]            PPROT,
  output wire [31:0]           PRDATA,
  output wire                  PREADY,
  output wire                  PSLVERR,

  //--------------------------------------------------------------------------
  // AMBA AXI4-Stream sink : element samples
  //--------------------------------------------------------------------------
  input  wire [2*DATA_W-1:0]   RX_TDATA,
  input  wire                  RX_TVALID,
  output wire                  RX_TREADY,
  input  wire                  RX_TLAST,

  //--------------------------------------------------------------------------
  // AMBA AXI4-Stream source : beam samples
  //--------------------------------------------------------------------------
  output wire [2*DATA_W-1:0]   TX_TDATA,
  output wire                  TX_TVALID,
  input  wire                  TX_TREADY,
  output wire                  TX_TLAST,
  output wire [BM_AW-1:0]      TX_TID,

  //--------------------------------------------------------------------------
  // Interrupt, level sensitive, active HIGH
  //--------------------------------------------------------------------------
  output wire                  IRQ
);

  //----------------------------------------------------------------------------
  // Register block to core
  //----------------------------------------------------------------------------
  wire                cfg_en;
  wire                core_rst;
  wire [EL_AW-1:0]    cfg_nelem_m1;
  wire [BM_AW-1:0]    cfg_nbeam_m1;
  wire [SHFT_W-1:0]   cfg_shift;

  wire                wgt_we;
  wire [BM_AW-1:0]    wgt_beam;
  wire [EL_AW-1:0]    wgt_elem;
  wire [2*COEF_W-1:0] wgt_wdata;

  wire                stat_busy;
  wire                evt_done;
  wire                evt_ovf;
  wire                evt_perr;

  //----------------------------------------------------------------------------
  bfa_apb_regs #(
    .NUM_ELEMENTS (NUM_ELEMENTS),
    .NUM_BEAMS    (NUM_BEAMS),
    .DATA_W       (DATA_W),
    .COEF_W       (COEF_W),
    .ACC_W        (ACC_W),
    .EL_AW        (EL_AW),
    .BM_AW        (BM_AW),
    .SHFT_W       (SHFT_W)
  ) u_apb_regs (
    .ACLK         (ACLK),
    .ARESETn      (ARESETn),

    .PSEL         (PSEL),
    .PENABLE      (PENABLE),
    .PWRITE       (PWRITE),
    .PADDR        (PADDR),
    .PWDATA       (PWDATA),
    .PSTRB        (PSTRB),
    .PPROT        (PPROT),
    .PRDATA       (PRDATA),
    .PREADY       (PREADY),
    .PSLVERR      (PSLVERR),

    .cfg_en       (cfg_en),
    .core_rst     (core_rst),
    .cfg_nelem_m1 (cfg_nelem_m1),
    .cfg_nbeam_m1 (cfg_nbeam_m1),
    .cfg_shift    (cfg_shift),

    .wgt_we       (wgt_we),
    .wgt_beam     (wgt_beam),
    .wgt_elem     (wgt_elem),
    .wgt_wdata    (wgt_wdata),

    .stat_busy    (stat_busy),
    .evt_done     (evt_done),
    .evt_ovf      (evt_ovf),
    .evt_perr     (evt_perr),

    .irq          (IRQ)
  );

  //----------------------------------------------------------------------------
  bfa_core #(
    .NUM_ELEMENTS (NUM_ELEMENTS),
    .NUM_BEAMS    (NUM_BEAMS),
    .DATA_W       (DATA_W),
    .COEF_W       (COEF_W),
    .ACC_W        (ACC_W),
    .EL_AW        (EL_AW),
    .BM_AW        (BM_AW),
    .SHFT_W       (SHFT_W)
  ) u_core (
    .ACLK         (ACLK),
    .ARESETn      (ARESETn),

    .cfg_en       (cfg_en),
    .core_rst     (core_rst),
    .cfg_nelem_m1 (cfg_nelem_m1),
    .cfg_nbeam_m1 (cfg_nbeam_m1),
    .cfg_shift    (cfg_shift),

    .wgt_we       (wgt_we),
    .wgt_beam     (wgt_beam),
    .wgt_elem     (wgt_elem),
    .wgt_wdata    (wgt_wdata),

    .RX_TDATA     (RX_TDATA),
    .RX_TVALID    (RX_TVALID),
    .RX_TREADY    (RX_TREADY),
    .RX_TLAST     (RX_TLAST),

    .TX_TDATA     (TX_TDATA),
    .TX_TVALID    (TX_TVALID),
    .TX_TREADY    (TX_TREADY),
    .TX_TLAST     (TX_TLAST),
    .TX_TID       (TX_TID),

    .stat_busy    (stat_busy),
    .evt_done     (evt_done),
    .evt_ovf      (evt_ovf),
    .evt_perr     (evt_perr)
  );

endmodule
