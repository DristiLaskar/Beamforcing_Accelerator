//------------------------------------------------------------------------------
// BFA - Beamforming Accelerator
// File        : bfa_wgt_ram.v
// Description : Simple dual-port coefficient memory (one write port, one read
//               port, common clock). One instance is created per beam.
//
//               Read latency  : 1 ACLK cycle when `re` is asserted.
//               Write latency : 1 ACLK cycle.
//
//               Collision behaviour: if `we` and `re` target the same address
//               in the same cycle, the read returns the OLD contents
//               (read-before-write). The register block prevents this case by
//               rejecting coefficient writes while STATUS.BUSY is set, so the
//               behaviour is never relied upon.
//
//               This is written as an inferrable memory. For an ASIC flow,
//               replace this module with the equivalent compiled SRAM or
//               register-file macro and its wrapper.
//
// Language    : Verilog-2001
//------------------------------------------------------------------------------

module bfa_wgt_ram #(
  parameter integer AW = 3,        // Address width -> depth = 2**AW
  parameter integer DW = 32        // Data width
) (
  input  wire           ACLK,

  input  wire           we,
  input  wire [AW-1:0]  waddr,
  input  wire [DW-1:0]  wdata,

  input  wire           re,
  input  wire [AW-1:0]  raddr,
  output reg  [DW-1:0]  rdata
);

  localparam integer DEPTH = (1 << AW);

  reg [DW-1:0] mem [0:DEPTH-1];

`ifndef SYNTHESIS
  // Simulation-only initialisation so that reads of never-written locations
  // return 0 rather than X. Ignored by ASIC synthesis; harmless on FPGA.
  integer i;
  initial begin
    for (i = 0; i < DEPTH; i = i + 1) begin
      mem[i] = {DW{1'b0}};
    end
    rdata = {DW{1'b0}};
  end
`endif

  always @(posedge ACLK) begin
    if (we) begin
      mem[waddr] <= wdata;
    end
    if (re) begin
      rdata <= mem[raddr];
    end
  end

endmodule
