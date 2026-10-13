//------------------------------------------------------------------------------
// BFA - Beamforming Accelerator
// File        : bfa_defs.vh
// Description : Programmer's model constants (APB register offsets, bit
//               positions and PrimeCell-style identification values).
// Language    : Verilog-2001
//------------------------------------------------------------------------------
// All macros are prefixed BFA_ to avoid polluting the global `define namespace.
//------------------------------------------------------------------------------

`ifndef BFA_DEFS_VH
`define BFA_DEFS_VH

//------------------------------------------------------------------------------
// Register offsets (byte addresses within the 4KB APB slave region)
//------------------------------------------------------------------------------
`define BFA_OFF_CTRL      12'h000   // RW  Control
`define BFA_OFF_STATUS    12'h004   // RO  Status
`define BFA_OFF_CFG       12'h008   // RW  Runtime geometry
`define BFA_OFF_SHIFT     12'h00C   // RW  Output scaling
`define BFA_OFF_IRQSTAT   12'h010   // W1C Interrupt status
`define BFA_OFF_IRQEN     12'h014   // RW  Interrupt enable
`define BFA_OFF_SNAPCNT   12'h018   // RO  Completed snapshot counter
`define BFA_OFF_HWCFG     12'h01C   // RO  Build-time configuration

// Coefficient (weight) aperture. Word index = beam*NUM_ELEMENTS + element.
// Occupies PADDR[11:8] == 4'h2, i.e. 0x200..0x2FF (max 64 words).
`define BFA_WGT_PAGE      4'h2

// PrimeCell-style identification registers
`define BFA_OFF_PERIPHID0 12'hFE0
`define BFA_OFF_PERIPHID1 12'hFE4
`define BFA_OFF_PERIPHID2 12'hFE8
`define BFA_OFF_PERIPHID3 12'hFEC
`define BFA_OFF_PCELLID0  12'hFF0
`define BFA_OFF_PCELLID1  12'hFF4
`define BFA_OFF_PCELLID2  12'hFF8
`define BFA_OFF_PCELLID3  12'hFFC

//------------------------------------------------------------------------------
// CTRL bit positions
//------------------------------------------------------------------------------
`define BFA_CTRL_EN       0        // RW  Enable datapath
`define BFA_CTRL_SRST     1        // WO  Soft reset, self-clearing

//------------------------------------------------------------------------------
// STATUS bit positions
//------------------------------------------------------------------------------
`define BFA_STAT_BUSY     0        // RO  Snapshot in flight
`define BFA_STAT_EN       1        // RO  CTRL.EN readback

//------------------------------------------------------------------------------
// Interrupt bit positions (IRQSTAT / IRQEN share this layout)
//------------------------------------------------------------------------------
`define BFA_IRQ_DONE      0        // Snapshot results produced
`define BFA_IRQ_OVF       1        // Output saturation occurred
`define BFA_IRQ_PERR      2        // Stream length / TLAST protocol error
`define BFA_IRQ_WIDTH     3

//------------------------------------------------------------------------------
// Identification values
//
// NOTE: the designer field below (0x41) is the JEP106 code allocated to Arm.
// It is used here only because this block follows the Arm PrimeCell
// identification scheme. Any party other than Arm MUST substitute their own
// JEP106 designer code before taping out. See docs/bfa_integration.md.
//------------------------------------------------------------------------------
`define BFA_PART_NUM      12'hB1F  // Placeholder part number
`define BFA_DESIGNER      8'h41    // JEP106 designer code
`define BFA_REVISION      4'h0     // r0p0

`define BFA_PERIPHID0     8'h1F                    // part_num[7:0]
`define BFA_PERIPHID1     8'h1B                    // {designer[3:0], part_num[11:8]} = {4'h1,4'hB}
`define BFA_PERIPHID2     8'h04                    // {revision, designer[7:4]}       = {4'h0,4'h4}
`define BFA_PERIPHID3     8'h00                    // configuration
`define BFA_PCELLID0      8'h0D                    // 0xB105F00D
`define BFA_PCELLID1      8'hF0
`define BFA_PCELLID2      8'h05
`define BFA_PCELLID3      8'hB1

`endif // BFA_DEFS_VH
