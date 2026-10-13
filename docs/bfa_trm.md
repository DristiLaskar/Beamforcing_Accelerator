# BFA Beamforming Accelerator — Technical Reference Manual

Revision r0p0. Applies to the RTL in `logical/bfa/verilog`.

---

## 1. Overview

The BFA is a fixed-point digital beamforming accelerator with an AMBA APB4
control plane and two AMBA AXI4-Stream data ports.

For each input *snapshot* of `N` antenna element samples it produces `B` beam
outputs:

```
y[b] = SUM over e = 0..N-1 of ( w[b][e] * x[e] )        for b = 0..B-1
```

where `x[e]` and `w[b][e]` are complex numbers in two's complement fixed point.
All `B` beams are computed concurrently: each accepted input beat is broadcast
to `B` complex multiply-accumulate units, each with a private coefficient
memory.

**No conjugation is applied.** `w * x` is a plain complex product. A conjugate
beamformer is obtained by programming conjugated coefficients (negate the
imaginary part in software).

### 1.1 Build-time parameters

| Parameter      | Default | Meaning                                    |
|----------------|---------|--------------------------------------------|
| `NUM_ELEMENTS` | 8       | Maximum antenna elements per snapshot       |
| `NUM_BEAMS`    | 4       | Maximum concurrent beams                    |
| `DATA_W`       | 16      | Sample component width, signed              |
| `COEF_W`       | 16      | Coefficient component width, signed         |
| `ACC_W`        | 40      | Accumulator width, signed                   |
| `SHFT_W`       | 6       | Output shift control width                  |

`EL_AW` and `BM_AW` are derived and **must not be overridden**. An elaboration
check in `bfa_core` fires if they are inconsistent.

Constraints, all checked at elaboration in simulation:

- `NUM_BEAMS * 2**EL_AW <= 64` (the coefficient aperture is 64 words)
- `ACC_W >= DATA_W + COEF_W`
- `2 * COEF_W <= 32` (one coefficient must fit in a single APB word)

### 1.2 Accumulator sizing

The accumulator must not wrap; the design does not detect accumulator overflow,
only output saturation. Required width is:

```
ACC_W >= DATA_W + COEF_W + ceil(log2(2 * NUM_ELEMENTS))
```

The `2 *` accounts for the two products summed per element in each of the real
and imaginary paths. For the defaults (16, 16, 8 elements) this gives
`16 + 16 + 4 = 36`; the default `ACC_W` of 40 leaves four bits of margin.

---

## 2. Data formats

Both stream ports carry one complex sample per beat:

```
TDATA[2*DATA_W-1 : DATA_W]  = imaginary component (Q), two's complement
TDATA[DATA_W-1   : 0]       = real component      (I), two's complement
```

Coefficients use the same packing in the APB write word:

```
PWDATA[31:16] = w_imag     PWDATA[15:0] = w_real     (for COEF_W = 16)
```

The interpretation of the binary point is entirely a software convention. The
hardware performs integer arithmetic and then applies `SHIFT` (see below), so
treating coefficients as Q1.15 simply means programming `SHIFT = 15`.

### 2.1 Packet structure

| Port | Beats per packet | TLAST                | TID              |
|------|------------------|----------------------|------------------|
| RX   | `N` (= `NELEM_M1`+1) | on beat `N-1`    | not present      |
| TX   | `B` (= `NBEAM_M1`+1) | on beat `B-1`    | beam index       |

RX beats must be presented in element order 0, 1, ... N-1. TX beats are emitted
in beam order 0, 1, ... B-1 and `TX_TID` carries the beam index of each beat.

### 2.2 Output scaling

Each accumulator is reduced to `DATA_W` bits by:

```
dout = saturate( (acc + 2^(SHIFT-1)) >>> SHIFT )      SHIFT > 0
dout = saturate(  acc                        )        SHIFT = 0
```

- Rounding is **round half toward +infinity**. It is not round-half-away-from-
  zero and not convergent rounding. `-1.5` rounds to `-1`.
- Saturation clamps to `[-2^(DATA_W-1), 2^(DATA_W-1)-1]`, i.e. `0x8000` to
  `0x7FFF` for `DATA_W = 16`.
- Any clamp on an enabled beam sets `IRQSTAT.OVF`.

---

## 3. Register map

APB4 slave, 4KB region, 12-bit `PADDR`. All registers are 32 bits and
**word-access only**. Reserved bits are RAZ/WI.

| Offset | Name       | Access | Reset      | Description                       |
|--------|------------|--------|------------|-----------------------------------|
| 0x000  | `CTRL`     | RW     | 0x00000000 | Control                           |
| 0x004  | `STATUS`   | RO     | 0x00000000 | Status                            |
| 0x008  | `CFG`      | RW     | see below  | Runtime geometry                  |
| 0x00C  | `SHIFT`    | RW     | 0x00000000 | Output scaling                    |
| 0x010  | `IRQSTAT`  | W1C    | 0x00000000 | Interrupt status                  |
| 0x014  | `IRQEN`    | RW     | 0x00000000 | Interrupt enable                  |
| 0x018  | `SNAPCNT`  | RO     | 0x00000000 | Completed snapshot counter        |
| 0x01C  | `HWCFG`    | RO     | build      | Build-time configuration          |
| 0x200+ | `WGT[n]`   | WO     | undefined  | Coefficient aperture              |
| 0xFE0  | `PERIPHID0`| RO     | 0x0000001F | Peripheral identification         |
| 0xFE4  | `PERIPHID1`| RO     | 0x0000001B |                                   |
| 0xFE8  | `PERIPHID2`| RO     | 0x00000004 |                                   |
| 0xFEC  | `PERIPHID3`| RO     | 0x00000000 |                                   |
| 0xFF0  | `PCELLID0` | RO     | 0x0000000D | PrimeCell identification          |
| 0xFF4  | `PCELLID1` | RO     | 0x000000F0 | (concatenates to 0xB105F00D)      |
| 0xFF8  | `PCELLID2` | RO     | 0x00000005 |                                   |
| 0xFFC  | `PCELLID3` | RO     | 0x000000B1 |                                   |

### 3.1 CTRL, 0x000

| Bits   | Name   | Access | Description                                     |
|--------|--------|--------|-------------------------------------------------|
| [31:2] | —      | RAZ/WI | Reserved                                        |
| [1]    | `SRST` | WO     | Soft reset. Self-clearing, always reads 0.      |
| [0]    | `EN`   | RW     | Enable. `RX_TREADY` is forced low when clear.   |

`SRST` resets the datapath, the FSM, `IRQSTAT` and `SNAPCNT`. It **preserves**
`EN`, `CFG`, `SHIFT`, `IRQEN` and the coefficient memories, so a stream can be
recovered after a protocol error without reprogramming.

### 3.2 STATUS, 0x004

| Bits   | Name   | Description                                            |
|--------|--------|--------------------------------------------------------|
| [1]    | `EN`   | Readback of `CTRL.EN`                                   |
| [0]    | `BUSY` | A snapshot is in flight (accepting, computing, or draining) |

### 3.3 CFG, 0x008

| Bits    | Name        | Description                                  |
|---------|-------------|----------------------------------------------|
| [15:8]  | `NBEAM_M1`  | Beams per snapshot, minus 1                  |
| [7:0]   | `NELEM_M1`  | Elements per snapshot, minus 1               |

Reset value is `((NUM_BEAMS-1) << 8) | (NUM_ELEMENTS-1)`, i.e. the full
build-time geometry. Writable **only while `CTRL.EN = 0`**. A write with
`NELEM_M1 > NUM_ELEMENTS-1` or `NBEAM_M1 > NUM_BEAMS-1` is rejected with
`PSLVERR` and leaves the register unchanged.

### 3.4 SHIFT, 0x00C

| Bits   | Name    | Description                                          |
|--------|---------|------------------------------------------------------|
| [7:0]  | `SHIFT` | Right shift applied before saturation, 0 to `ACC_W-1` |

Writable **only while `CTRL.EN = 0`**. Values above `ACC_W-1` are rejected with
`PSLVERR`.

### 3.5 IRQSTAT (0x010) and IRQEN (0x014)

| Bit | Name   | Set when                                                     |
|-----|--------|--------------------------------------------------------------|
| [2] | `PERR` | Stream length did not match `CFG` (see section 5)             |
| [1] | `OVF`  | Output saturation occurred on an enabled beam                 |
| [0] | `DONE` | A snapshot's results were produced                            |

`IRQSTAT` is write-1-to-clear. If hardware sets a bit in the same cycle
software clears it, **the set wins**. `IRQ` is level sensitive, active high, and
equals `|(IRQSTAT & IRQEN)`. `IRQEN` is writable at any time.

### 3.6 SNAPCNT, 0x018

Free-running 32-bit count of completed snapshots. Increments on each `DONE`
event, wraps at `0xFFFFFFFF`, and is zeroed by `SRST` only.

### 3.7 HWCFG, 0x01C

| Bits    | Field          |
|---------|----------------|
| [31:24] | `ACC_W`        |
| [23:16] | `DATA_W`       |
| [15:8]  | `NUM_BEAMS`    |
| [7:0]   | `NUM_ELEMENTS` |

For the default build this reads `0x28100408`.

### 3.8 Coefficient aperture, 0x200 onward

Write-only. Reads return zero **and assert `PSLVERR`**.

```
word_index   = beam * 2**EL_AW + element
byte_address = 0x200 + word_index * 4
```

Note the stride is `2**EL_AW`, **not** `NUM_ELEMENTS`. For the default build
`EL_AW = 3`, so each beam occupies 8 words (32 bytes) and beam `b`, element `e`
lives at `0x200 + (b*8 + e)*4`. With `NUM_ELEMENTS = 5` the stride is still 8
and words 5..7 of each beam are unimplemented.

Writable **only while `CTRL.EN = 0`**. This restriction removes any race
between an APB write and a datapath read of the same memory.

### 3.9 Identification registers

`PERIPHID` follows the Arm PrimeCell scheme: part number `0xB1F`, designer
`0x41`, revision `0x0`.

> **`0x41` is the JEP106 designer code allocated to Arm.** It appears here only
> because this block adopts the PrimeCell identification scheme. Any party
> other than Arm must substitute their own JEP106 code in `bfa_defs.vh` before
> production use.

---

## 4. Programming sequence

```c
/* 1. Disable. Geometry, scaling and coefficients are locked while enabled. */
WR(CTRL, 0);

/* 2. Geometry: N elements, B beams. */
WR(CFG, ((B-1) << 8) | (N-1));

/* 3. Output scaling. Q1.15 coefficients -> SHIFT = 15. */
WR(SHIFT, 15);

/* 4. Coefficients, packed {imag, real}. Stride is 2**EL_AW, not N. */
for (b = 0; b < B; b++)
    for (e = 0; e < N; e++)
        WR(0x200 + ((b << EL_AW) + e) * 4,
           ((uint32_t)w_imag[b][e] << 16) | (uint16_t)w_real[b][e]);

/* 5. Interrupts. */
WR(IRQSTAT, 0x7);          /* clear stale events   */
WR(IRQEN,   0x7);          /* DONE | OVF | PERR    */

/* 6. Enable, then stream. */
WR(CTRL, 1);
```

To change coefficients or geometry later, clear `CTRL.EN`, wait for
`STATUS.BUSY` to read 0, reprogram, then set `CTRL.EN` again.

---

## 5. Error handling

The block validates that each RX packet is exactly `N` beats.

| Condition                                    | Behaviour                                                        |
|----------------------------------------------|------------------------------------------------------------------|
| `TLAST` asserted before beat `N-1`           | `PERR` set; snapshot discarded; no TX packet                      |
| Beat `N-1` accepted without `TLAST`          | `PERR` set; remaining beats consumed and discarded until `TLAST`; no TX packet |

In both cases the accumulators are cleared and the block returns to accepting a
new packet, so a single malformed packet cannot corrupt the following one. This
is covered by tests T7, T8 and T12d.

Note the second case: the block keeps `RX_TREADY` asserted and swallows beats
until it sees `TLAST`. If the source never asserts `TLAST`, the block waits
indefinitely. Use `SRST` to recover.

---

## 6. Timing

Latency and throughput were measured in simulation across twelve
configurations and match exactly:

```
snapshot interval = N + B + 4 cycles
```

For the default 8-element, 4-beam build that is 16 cycles per snapshot.

| Measurement (8 elements, 4 beams)          | Cycles |
|--------------------------------------------|--------|
| RX accept span, source never stalls        | 8 (= N, one beat per cycle) |
| `TLAST` accepted to first TX beat          | 5      |
| First RX beat to last TX beat              | 16     |
| Snapshot to snapshot, source never stalls  | 16     |

RX and TX **do not overlap**: `RX_TREADY` is held low from the moment `TLAST`
is accepted until the last beam beat has been handshaked. See
`docs/bfa_integration.md` section 4 for the rationale and what an overlapped
variant would require.
