//------------------------------------------------------------------------------
// BFA - Beamforming Accelerator
// File        : bfa_apb_regs.v
// Description : AMBA APB4 slave register block.
//
//               Zero wait states: PREADY is tied HIGH. A transfer therefore
//               occupies the standard two APB cycles (setup then access).
//
//               PSLVERR is asserted during the access phase for:
//                 - unaligned addresses (PADDR[1:0] != 0)
//                 - partial write strobes (PSTRB != 4'b1111)
//                 - undecoded addresses
//                 - reads of the write-only coefficient aperture
//                 - accesses to unimplemented coefficient slots
//                 - writes to CFG, SHIFT or the coefficient aperture while
//                   CTRL.EN is set (these are locked while enabled)
//                 - CFG or SHIFT writes whose value field is out of range
//
//               Writes to read-only registers are ignored WITHOUT error, which
//               is the usual Arm peripheral convention and keeps generic
//               register-restore routines working.
//
//               Reserved bits are RAZ/WI (read as zero, write ignored).
//
// Language    : Verilog-2001
//------------------------------------------------------------------------------

`include "bfa_defs.vh"

module bfa_apb_regs #(
  parameter integer NUM_ELEMENTS = 8,
  parameter integer NUM_BEAMS    = 4,
  parameter integer DATA_W       = 16,
  parameter integer COEF_W       = 16,
  parameter integer ACC_W        = 40,
  parameter integer EL_AW        = 3,
  parameter integer BM_AW        = 2,
  parameter integer SHFT_W       = 6
) (
  input  wire                   ACLK,
  input  wire                   ARESETn,

  // AMBA APB4 slave
  input  wire                   PSEL,
  input  wire                   PENABLE,
  input  wire                   PWRITE,
  input  wire [11:0]            PADDR,
  input  wire [31:0]            PWDATA,
  input  wire [3:0]             PSTRB,
  input  wire [2:0]             PPROT,
  output wire [31:0]            PRDATA,
  output wire                   PREADY,
  output wire                   PSLVERR,

  // To core
  output reg                    cfg_en,
  output reg                    core_rst,
  output reg  [EL_AW-1:0]       cfg_nelem_m1,
  output reg  [BM_AW-1:0]       cfg_nbeam_m1,
  output reg  [SHFT_W-1:0]      cfg_shift,

  output reg                    wgt_we,
  output reg  [BM_AW-1:0]       wgt_beam,
  output reg  [EL_AW-1:0]       wgt_elem,
  output reg  [2*COEF_W-1:0]    wgt_wdata,

  // From core
  input  wire                   stat_busy,
  input  wire                   evt_done,
  input  wire                   evt_ovf,
  input  wire                   evt_perr,

  // Interrupt request, level sensitive, active HIGH
  output wire                   irq
);

  localparam integer WGT_SLOTS = NUM_BEAMS * (1 << EL_AW);
  localparam [31:0]  HWCFG_VAL = (ACC_W  << 24) | (DATA_W    << 16) |
                                 (NUM_BEAMS << 8) | NUM_ELEMENTS;

  // Explicitly sized reset values so no implicit integer truncation occurs.
  localparam [EL_AW-1:0] NELEM_M1_RST = NUM_ELEMENTS - 1;
  localparam [BM_AW-1:0] NBEAM_M1_RST = NUM_BEAMS    - 1;

  //----------------------------------------------------------------------------
  // Elaboration-time sanity checks (simulation only)
  //----------------------------------------------------------------------------
`ifndef SYNTHESIS
  initial begin
    if (WGT_SLOTS > 64) begin
      $display("BFA ERROR: coefficient aperture needs %0d words, max 64.", WGT_SLOTS);
      $finish;
    end
    if ((EL_AW > 8) || (BM_AW > 8) || (SHFT_W > 8)) begin
      $display("BFA ERROR: EL_AW/BM_AW/SHFT_W must each be <= 8.");
      $finish;
    end
    if (2*COEF_W > 32) begin
      $display("BFA ERROR: 2*COEF_W must be <= 32 to fit one APB word.");
      $finish;
    end
  end
`endif

  //----------------------------------------------------------------------------
  // APB phase decode
  //----------------------------------------------------------------------------
  wire acc_phase    = PSEL & PENABLE;
  wire wr_phase     = acc_phase &  PWRITE;
  wire rd_phase     = acc_phase & ~PWRITE;
  wire addr_aligned = (PADDR[1:0] == 2'b00);
  wire full_strb    = (PSTRB == 4'b1111);

  assign PREADY = 1'b1;

  //----------------------------------------------------------------------------
  // Coefficient aperture decode
  //   word index = beam * 2**EL_AW + element
  //----------------------------------------------------------------------------
  wire            wgt_page  = (PADDR[11:8] == `BFA_WGT_PAGE);
  wire [5:0]      wgt_idx   = PADDR[7:2];
  wire [EL_AW-1:0] wgt_elem_w = wgt_idx[EL_AW-1:0];
  wire [BM_AW-1:0] wgt_beam_w = wgt_idx[EL_AW +: BM_AW];

  // wgt_idx < WGT_SLOTS already implies the bits above EL_AW+BM_AW are zero.
  wire wgt_slot_ok = wgt_page &&
                     (wgt_idx   < WGT_SLOTS) &&
                     (wgt_elem_w < NUM_ELEMENTS);

  //----------------------------------------------------------------------------
  // Register storage
  //----------------------------------------------------------------------------
  reg [`BFA_IRQ_WIDTH-1:0] irq_stat;
  reg [`BFA_IRQ_WIDTH-1:0] irq_en;
  reg [31:0]               snap_cnt;

  //----------------------------------------------------------------------------
  // Read mux and address decode
  //----------------------------------------------------------------------------
  reg [31:0] prdata_c;
  reg        csr_hit;

  always @(*) begin
    prdata_c = 32'h0000_0000;
    csr_hit  = 1'b1;
    case (PADDR)
      `BFA_OFF_CTRL     : prdata_c = {30'h0, 1'b0, cfg_en};        // SRST reads 0
      `BFA_OFF_STATUS   : prdata_c = {30'h0, cfg_en, stat_busy};
      `BFA_OFF_CFG      : prdata_c = {16'h0,
                                      {(8-BM_AW){1'b0}}, cfg_nbeam_m1,
                                      {(8-EL_AW){1'b0}}, cfg_nelem_m1};
      `BFA_OFF_SHIFT    : prdata_c = {24'h0, {(8-SHFT_W){1'b0}}, cfg_shift};
      `BFA_OFF_IRQSTAT  : prdata_c = {{(32-`BFA_IRQ_WIDTH){1'b0}}, irq_stat};
      `BFA_OFF_IRQEN    : prdata_c = {{(32-`BFA_IRQ_WIDTH){1'b0}}, irq_en};
      `BFA_OFF_SNAPCNT  : prdata_c = snap_cnt;
      `BFA_OFF_HWCFG    : prdata_c = HWCFG_VAL;
      `BFA_OFF_PERIPHID0: prdata_c = {24'h0, `BFA_PERIPHID0};
      `BFA_OFF_PERIPHID1: prdata_c = {24'h0, `BFA_PERIPHID1};
      `BFA_OFF_PERIPHID2: prdata_c = {24'h0, `BFA_PERIPHID2};
      `BFA_OFF_PERIPHID3: prdata_c = {24'h0, `BFA_PERIPHID3};
      `BFA_OFF_PCELLID0 : prdata_c = {24'h0, `BFA_PCELLID0};
      `BFA_OFF_PCELLID1 : prdata_c = {24'h0, `BFA_PCELLID1};
      `BFA_OFF_PCELLID2 : prdata_c = {24'h0, `BFA_PCELLID2};
      `BFA_OFF_PCELLID3 : prdata_c = {24'h0, `BFA_PCELLID3};
      default           : csr_hit  = 1'b0;
    endcase
  end

  assign PRDATA = rd_phase ? prdata_c : 32'h0000_0000;

  //----------------------------------------------------------------------------
  // Write value range checks
  //----------------------------------------------------------------------------
  wire cfg_val_ok   = (PWDATA[7:0]  <= (NUM_ELEMENTS-1)) &&
                      (PWDATA[15:8] <= (NUM_BEAMS-1));
  wire shift_val_ok = (PWDATA[7:0]  <= (ACC_W-1));

  //----------------------------------------------------------------------------
  // Error generation
  //----------------------------------------------------------------------------
  reg err_c;

  always @(*) begin
    err_c = 1'b0;
    if (!addr_aligned) begin
      err_c = 1'b1;
    end else if (wgt_page) begin
      if      (!wgt_slot_ok) err_c = 1'b1;   // unimplemented slot
      else if (!PWRITE)      err_c = 1'b1;   // aperture is write-only
      else if (!full_strb)   err_c = 1'b1;   // word writes only
      else if (cfg_en)       err_c = 1'b1;   // locked while enabled
    end else if (!csr_hit) begin
      err_c = 1'b1;
    end else if (PWRITE) begin
      if (!full_strb) begin
        err_c = 1'b1;
      end else if (PADDR == `BFA_OFF_CFG) begin
        if      (cfg_en)       err_c = 1'b1;
        else if (!cfg_val_ok)  err_c = 1'b1;
      end else if (PADDR == `BFA_OFF_SHIFT) begin
        if      (cfg_en)        err_c = 1'b1;
        else if (!shift_val_ok) err_c = 1'b1;
      end
    end
  end

  assign PSLVERR = acc_phase & err_c;

  wire wr_ok = wr_phase & ~err_c;

  //----------------------------------------------------------------------------
  // Control / configuration registers
  //
  // The coefficient write port is registered so that the core always sees a
  // clean one-cycle strobe and so that the APB data path does not feed the
  // memory write port combinationally.
  //----------------------------------------------------------------------------
  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      cfg_en       <= 1'b0;
      core_rst     <= 1'b0;
      cfg_nelem_m1 <= NELEM_M1_RST;
      cfg_nbeam_m1 <= NBEAM_M1_RST;
      cfg_shift    <= {SHFT_W{1'b0}};
      irq_en       <= {`BFA_IRQ_WIDTH{1'b0}};
      wgt_we       <= 1'b0;
      wgt_beam     <= {BM_AW{1'b0}};
      wgt_elem     <= {EL_AW{1'b0}};
      wgt_wdata    <= {(2*COEF_W){1'b0}};
    end else begin
      core_rst <= 1'b0;
      wgt_we   <= 1'b0;

      if (wr_ok) begin
        if (wgt_page) begin
          wgt_we    <= 1'b1;
          wgt_beam  <= wgt_beam_w;
          wgt_elem  <= wgt_elem_w;
          wgt_wdata <= PWDATA[2*COEF_W-1:0];
        end else begin
          case (PADDR)
            `BFA_OFF_CTRL: begin
              cfg_en   <= PWDATA[`BFA_CTRL_EN];
              core_rst <= PWDATA[`BFA_CTRL_SRST];
            end
            `BFA_OFF_CFG: begin
              cfg_nelem_m1 <= PWDATA[EL_AW-1:0];
              cfg_nbeam_m1 <= PWDATA[8 +: BM_AW];
            end
            `BFA_OFF_SHIFT: begin
              cfg_shift <= PWDATA[SHFT_W-1:0];
            end
            `BFA_OFF_IRQEN: begin
              irq_en <= PWDATA[`BFA_IRQ_WIDTH-1:0];
            end
            default: begin
              // Read-only registers: write ignored, no error.
            end
          endcase
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Interrupt status : write-1-to-clear, hardware set takes priority
  //----------------------------------------------------------------------------
  wire [`BFA_IRQ_WIDTH-1:0] evt_vec  = {evt_perr, evt_ovf, evt_done};
  wire [`BFA_IRQ_WIDTH-1:0] w1c_mask = (wr_ok && (PADDR == `BFA_OFF_IRQSTAT))                 
                                       ? PWDATA[`BFA_IRQ_WIDTH-1:0]
                                       : {`BFA_IRQ_WIDTH{1'b0}};

  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      irq_stat <= {`BFA_IRQ_WIDTH{1'b0}};
    end else if (core_rst) begin
      irq_stat <= {`BFA_IRQ_WIDTH{1'b0}};
    end else begin
      irq_stat <= (irq_stat & ~w1c_mask) | evt_vec;
    end
  end

  assign irq = |(irq_stat & irq_en);

  //----------------------------------------------------------------------------
  // Completed snapshot counter (free running, wraps)
  //----------------------------------------------------------------------------
  always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      snap_cnt <= 32'h0;
    end else if (core_rst) begin
      snap_cnt <= 32'h0;
    end else if (evt_done) begin
      snap_cnt <= snap_cnt + 32'h1;
    end
  end

  //----------------------------------------------------------------------------
  // Deliberately unused inputs. PPROT is accepted but no access filtering is
  // performed; see docs/bfa_integration.md if privilege checking is required.
  //----------------------------------------------------------------------------
  wire unused_ok = |{1'b0, PPROT, 1'b0};

endmodule
