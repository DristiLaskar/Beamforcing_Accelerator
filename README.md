# BFA — Digital Beamforming Accelerator

Fixed-point digital beamforming accelerator in Verilog-2001. AMBA APB4 control plane, two AMBA AXI4-Stream data
ports, level-sensitive interrupt.

For each snapshot of `N` complex antenna samples the block computes `B` beams
concurrently:

```
y[b] = SUM over e = 0..N-1 of ( w[b][e] * x[e] )
```

Revision r0p0.

---

## Quick start

Requires Icarus Verilog (developed and tested against 12.0).

```sh
./verif/scripts/run_sim.sh            # default build, 8 elements x 4 beams
./verif/scripts/run_sim.sh sweep      # 12 parameter configurations
./verif/scripts/run_sim.sh mutate     # fault injection self-check
./verif/scripts/run_sim.sh all        # everything
```

To compile by hand at a non-default geometry:

```sh
iverilog -g2005 -Wall -DBFA_TB_ELEMENTS=16 -DBFA_TB_BEAMS=4 \
         -o bfa_tb.vvp -I logical/bfa/verilog \
         logical/bfa/verilog/*.v verif/tb/verilog/bfa_tb.v
vvp bfa_tb.vvp
```

---

## Structure

```
docs/
  bfa_trm.md                 Programmer's model: register map, data formats,
                             programming sequence, error handling, timing.
  bfa_integration.md         Clocking constraints, memory replacement, known
                             limitations, verification status. Read section 2.

logical/bfa/verilog/
  bfa.v                      Top level. Port list and submodule wiring.
  bfa_apb_regs.v             APB4 slave, control/status registers, coefficient
                             write decode, interrupt logic, PrimeCell IDs.
  bfa_core.v                 Datapath and controller FSM. Stream handshakes,
                             coefficient RAM addressing, result capture.
  bfa_cmac.v                 Complex multiply-accumulate. Four real multipliers.
  bfa_rnd_sat.v              Round-half-toward-plus-infinity, then saturate.
  bfa_wgt_ram.v              Coefficient memory, one instance per beam.
  bfa_defs.vh                Register offsets, bit positions, ID values.

verif/
  tb/verilog/bfa_tb.v        Self-checking testbench with an independent
                             golden model and an AXI4-Stream protocol monitor.
  scripts/run_sim.sh         Regression driver.
  logs/                      Simulation output.
```

The `logical/<block>/verilog` layout is modelled on Arm's IP deliverable
conventions. It is not a copy of any specific proprietary Arm directory tree.

---

## Configuration

| Parameter      | Default | Meaning                              |
|----------------|---------|--------------------------------------|
| `NUM_ELEMENTS` | 8       | Maximum elements per snapshot        |
| `NUM_BEAMS`    | 4       | Maximum concurrent beams             |
| `DATA_W`       | 16      | Sample component width, signed       |
| `COEF_W`       | 16      | Coefficient component width, signed  |
| `ACC_W`        | 40      | Accumulator width, signed            |
| `SHFT_W`       | 6       | Output shift control width           |

`EL_AW` and `BM_AW` are derived and must not be overridden.

Element and beam counts are also runtime-reducible below the build-time maximum
via the `CFG` register, so one hardware build can serve several array
geometries.

Constraint: `NUM_BEAMS * 2**EL_AW <= 64`. Elaboration fails with a clear message
if violated.

---

## Status

**Passing.** 509 checks in the default configuration, 0 errors. Twelve parameter
configurations sweep clean: 1x1, 1x4, 2x2, 4x2, 8x4, 16x4, 6x3, 5x5, 2x8, 16x2,
8x8, 3x7. Ten injected RTL faults, all ten caught by the testbench.

Throughput is `N + B + 4` cycles per snapshot, measured across all twelve
configurations rather than assumed. 16 cycles for the default build.

**Not done.** No synthesis, no lint, no timing closure, no formal property
checking, no commercial AMBA VIP, no gate-level or power simulation, no coverage
measurement. Only Icarus Verilog 12.0 has been used. See
`docs/bfa_integration.md` section 5 for the full list.

**Two things that will bite you if skipped:**

1. The block is **single clock**. `PCLK` must be the same physical clock as
   `ACLK`. There are no separate APB clock or reset ports because no CDC logic
   exists. `docs/bfa_integration.md` section 2.
2. `PERIPHID` uses designer code `0x41`, which is the JEP106 code allocated to
   Arm. Substitute your own before production use.

---

## Verification approach

The testbench carries an independent 64-bit golden model that shares no signal
with the RTL, an APB4 master model that samples `PSLVERR` and `PRDATA` inside
the access phase, randomised stall injection on both stream ports, and a
free-running AXI4-Stream protocol monitor.

Tests T1 through T12 cover identification registers, reset values, directed
beamforming, randomised snapshots with stalls, saturation and overflow, runtime
geometry change, both protocol-error paths, `PSLVERR` and register locking,
interrupts and the snapshot counter, soft-reset recovery, and boundary
geometries.

Because Icarus Verilog provides no coverage measurement, test adequacy is
argued by fault injection instead: ten known bugs are patched into the RTL one
at a time and the testbench must fail on every one. This is evidence that the
tests actually check what they claim, not a substitute for coverage.

Two findings from that campaign are worth recording:

- A mutation that broke `TLAST` on single-beam packets initially **survived** —
  a genuine gap, since no test used one beam. Test T12 was added to close it.
- A mutation that accumulates discarded beats during error drain also survives,
  but is an **equivalent mutant**: `RX_TREADY` is low in the flush and output
  states, so the two versions differ only on a path that clears the
  accumulators regardless. Gating is cleaner for power, not observably
  different.

---

