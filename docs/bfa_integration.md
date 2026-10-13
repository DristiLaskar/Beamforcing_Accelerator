# BFA Beamforming Accelerator — Integration Guide

Read this before instantiating the block. Section 2 in particular describes a
constraint that will silently produce a broken chip if ignored.

---

## 1. Interfaces

| Group | Signals |
|-------|---------|
| Clock/reset | `ACLK`, `ARESETn` |
| APB4 slave | `PSEL`, `PENABLE`, `PWRITE`, `PADDR[11:0]`, `PWDATA[31:0]`, `PSTRB[3:0]`, `PPROT[2:0]`, `PRDATA[31:0]`, `PREADY`, `PSLVERR` |
| AXI4-Stream sink | `RX_TDATA[2*DATA_W-1:0]`, `RX_TVALID`, `RX_TREADY`, `RX_TLAST` |
| AXI4-Stream source | `TX_TDATA[2*DATA_W-1:0]`, `TX_TVALID`, `TX_TREADY`, `TX_TLAST`, `TX_TID[BM_AW-1:0]` |
| Interrupt | `IRQ` |

`ARESETn` is asynchronous assert, active low. It must be synchronously
deasserted with respect to `ACLK` by the integrator; the block does not contain
a reset synchroniser.

`IRQ` is level sensitive and active high. Connect to a GIC SPI input or an
equivalent level-triggered controller input.

### 1.1 AXI4-Stream conformance notes

- `TX_TVALID` is driven from a register and has no combinational dependency on
  `TX_TREADY`, as required of a source. Payload (`TDATA`, `TLAST`, `TID`) is
  held stable while `TVALID` is high and `TREADY` is low. A protocol monitor in
  the testbench checks both properties continuously.
- `RX_TREADY` is a function of internal registered state and `CTRL.EN` only. It
  does not depend on `RX_TVALID`, so no combinational loop can form across the
  interface.
- Optional stream signals not implemented: `TKEEP`, `TSTRB`, `TDEST`, `TUSER`.
  Every beat is a full, valid data beat; there are no null or position bytes.

---

## 2. Clocking — read this

**The block is single clock.** There is exactly one clock port, `ACLK`, and one
reset port, `ARESETn`. The APB slave deliberately has **no separate `PCLK` and
`PRESETn` ports**, because no clock domain crossing logic is implemented.

`PCLK` in your system must therefore be the same physical clock as `ACLK`, and
`PRESETn` the same reset as `ARESETn`. Driving the APB interface from a
different clock will produce metastability and silent data corruption. This is
not detectable by simulation of this deliverable.

If a separate APB clock is genuinely required, the following would need to be
added and re-verified. This work has **not** been done:

- A two-flop synchroniser plus handshake for `cfg_en`, `cfg_nelem_m1`,
  `cfg_nbeam_m1`, `cfg_shift`, treated as a single atomic configuration bundle
  so the datapath never sees a half-updated geometry.
- A request/acknowledge crossing for the coefficient write port, or a dual-clock
  memory.
- A pulse synchroniser for `core_rst` and for the three event pulses
  (`evt_done`, `evt_ovf`, `evt_perr`).
- A synchroniser on `stat_busy` for the `STATUS` read path.

---

## 3. Memory replacement

`bfa_wgt_ram.v` is written as an inferrable memory and is instantiated once per
beam, with depth `2**EL_AW` and width `2*COEF_W`. For the default build that is
four instances of 8 x 32 bits.

At these depths most ASIC flows will map this to flops or a register file
rather than a compiled SRAM. For a hard macro, replace the module with a
wrapper of the same port list. The required behaviour is:

- Write: synchronous, one cycle.
- Read: synchronous, one cycle, enabled by `re`.
- Same-address collision: the read must return the **old** contents.

The collision case never occurs in practice because coefficient writes are
rejected while `CTRL.EN = 1`, but a replacement macro should still not return
`X`.

The `initial` block that zeroes the array is guarded by `` `ifndef SYNTHESIS ``
and exists only so that reads of never-written locations return 0 instead of
`X` in simulation.

---

## 4. Known limitations

These are design decisions, not defects. Each is listed with what it would take
to change it.

**4.1 RX and TX do not overlap.** `RX_TREADY` is deasserted from the moment
`TLAST` is accepted until the final beam beat is handshaked, giving a fixed cost
of `N + B + 4` cycles per snapshot. Overlapping the two would require the input
side to begin a new snapshot while results are still draining. The accumulators
are already cleared and the results already copied into separate holding
registers by that point, so the datapath permits it; what is missing is
arbitration for the case where the next snapshot's `TLAST` arrives before the
previous packet has finished draining. That corner is why it was not done. If
`B < N` the gain would be roughly `B` cycles per snapshot.

**4.2 No accumulator overflow detection.** Only output saturation is detected
and reported. If `ACC_W` is undersized for the configured `N`, the accumulator
wraps silently and results are wrong with no error flag. Size `ACC_W` per
section 1.2 of the TRM.

**4.3 `PPROT` is ignored.** No privileged or secure access filtering is
performed; any master that can reach the APB slave can program the block. The
signal is accepted and tied into an `unused_ok` term to keep lint clean. If
access control is required, gate `PSEL` externally or add a check in
`bfa_apb_regs`.

**4.4 The coefficient aperture is write-only.** Reads return 0 and assert
`PSLVERR`. Readback would require either a second read port on each memory or a
shadow copy in flops, doubling the storage. Software should keep its own copy.

**4.5 `SRST` may abort an in-flight TX packet.** It behaves as a local reset,
so `TX_TVALID` can be deasserted mid-packet. Downstream logic must treat `SRST`
the same way it treats `ARESETn`. Software should prefer clearing `CTRL.EN` and
polling `STATUS.BUSY` for an orderly stop, and reserve `SRST` for error
recovery.

**4.6 Clearing `CTRL.EN` mid-packet stalls the source.** `RX_TREADY` drops
immediately, so a partially transmitted packet will hang. Recover with `SRST`.

**4.7 Aperture size caps the configuration.** `NUM_BEAMS * 2**EL_AW` must not
exceed 64 words. This rules out, for example, 12 elements with 6 beams
(96 words). Larger configurations need the aperture widened beyond one
`PADDR[11:8]` page.

---

## 5. Verification status

What was done:

- Self-checking testbench with an independent 64-bit golden model. 509 checks
  in the default configuration.
- Twelve parameter configurations swept, all passing: 1x1, 1x4, 2x2, 4x2, 8x4,
  16x4, 6x3, 5x5, 2x8, 16x2, 8x8, 3x7.
- Fault injection: ten deliberate RTL bugs injected, all ten detected by the
  testbench. This guards against a testbench that passes because it is not
  actually checking. Run with `./verif/scripts/run_sim.sh mutate`.
- Continuous AXI4-Stream source protocol monitor.
- Latency and throughput measured, not assumed.

What was **not** done, and should be before any production use:

- **No synthesis.** The RTL has never been through a synthesis tool. There is
  no area figure, no timing closure, no critical path analysis. The four
  parallel 16x16 multipliers per beam are the obvious timing risk and may need
  an extra pipeline stage at high frequency.
- **No lint.** Not run through Spyglass, Verilator lint, or equivalent.
- **No formal property checking**, and no commercial AMBA VIP. APB4 and
  AXI4-Stream conformance was checked by a hand-written testbench and a
  hand-written monitor only. A protocol assertion IP run is strongly
  recommended.
- **No gate-level or power simulation.**
- **No code or functional coverage measurement.** Icarus Verilog does not
  provide it. Test adequacy was argued via fault injection instead, which is
  evidence but not a substitute.
- Only Icarus Verilog 12.0 was used. Behaviour on commercial simulators, and
  in particular acceptance of the Verilog-2001 constructs used, has not been
  confirmed.

---

## 6. Reference documents

The interfaces follow the published AMBA specifications:

- AMBA APB Protocol Specification
- AMBA 4 AXI4-Stream Protocol Specification
- AMBA AXI and ACE Protocol Specification (for general AMBA handshake rules)

These are available from Arm's developer documentation site. I have not
verified the current document ID or issue letter for each of these, so check
the identifiers on Arm's site rather than relying on any number quoted from
memory. The PrimeCell identification register scheme referenced in TRM section
3.9 is described in the technical reference manuals of Arm PrimeCell
peripherals such as the PL011 UART.
