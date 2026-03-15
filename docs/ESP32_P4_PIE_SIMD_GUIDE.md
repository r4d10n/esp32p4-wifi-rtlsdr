# ESP32-P4 PIE SIMD Programming Guide

Complete reference for PIE (Processor Instruction Extensions) SIMD on ESP32-P4. This guide covers integer vector operations, multiply-accumulate instructions, complex arithmetic, and real-world DSP patterns for signal processing, AI, and SDR applications.

**Note:** This is a general-purpose reference. While examples may reference SDR/radio concepts, the patterns apply to any DSP, AI, or signal processing workload.

## Table of Contents

1. [Overview](#overview)
2. [Register Architecture](#register-architecture)
3. [Instruction Reference](#instruction-reference)
4. [What PIE Does NOT Support](#what-pie-does-not-support)
5. [Programming Patterns](#programming-patterns)
6. [esp-dsp Function Matrix](#esp-dsp-function-matrix)
7. [Performance Guidelines](#performance-guidelines)
8. [Common Pitfalls](#common-pitfalls)
9. [Comparison with VOLK](#comparison-with-volk)

---

## Overview

### What is PIE?

PIE (Processor Instruction Extensions) is a custom 128-bit integer SIMD instruction set for ESP32-P4. It operates alongside the RISC-V base ISA and provides:

- **8 vector registers** (128-bit each): QR0–QR7
- **2 accumulators**: QACC (256-bit) and XACC (40-bit scalar)
- **Integer-only data types**: u8, s8, u16, s16, u32, s32
- **No floating-point SIMD** (FPU is separate scalar unit)
- **Zero-overhead hardware loops** (esp.lp.setup) for tight kernels

### RISC-V Base + PIE Extensions

ESP32-P4 is a 400 MHz dual-core RISC-V processor. PIE adds custom instructions without breaking standard RISC-V toolchains:

```
┌─────────────────────────────────────────┐
│ RISC-V Base ISA (RV32I)                 │
│ ├─ Integer ALU (add, sub, mul, etc)    │
│ ├─ Load/Store (32-bit or 16-bit)       │
│ └─ Control flow (branch, jump)          │
└─────────────────────────────────────────┘
         ↓ (coexist peacefully)
┌─────────────────────────────────────────┐
│ PIE Custom Instructions                  │
│ ├─ Vector arithmetic (128-bit)          │
│ ├─ Complex operations (cmul, vcmulas)   │
│ └─ Accumulators (QACC, XACC)            │
└─────────────────────────────────────────┘
         ↓ (separate from)
┌─────────────────────────────────────────┐
│ Scalar FPU (flw, fsw, fadd.s, etc)      │
│ (Handles only single float)              │
└─────────────────────────────────────────┘
```

### Comparison with Other SIMD

| Feature | PIE (P4) | ARM NEON | x86 SSE | ESP32-S3 PIE |
|---------|----------|----------|---------|--------------|
| Register width | 128-bit | 128-bit | 128-bit | 128-bit |
| Registers | 8 (QR0–QR7) | 16 (Q0–Q15) | 8 (xmm0–xmm7) | 8 |
| Data types | int only | int + float | int + float | int only |
| Complex native | Yes (cmul, vcmulas) | No | No | Yes |
| Accumulators | 2 (QACC, XACC) | None | None | 1 (XACC) |
| Hardware loops | Yes | No | No | Yes |
| Clock speed | 400 MHz | 1+ GHz | 2+ GHz | 240 MHz |

---

## Register Architecture

### Vector Registers (QR0–QR7)

Each of the 8 vector registers is 128 bits wide. Layout depends on current element size:

```
QRn (128-bit register)

┌─ As 16×u8 ────────────────────────────────────────────┐
│ [0]   [1]   [2]   [3]   [4]   [5]  ...   [14]  [15]   │
│ u8 │ u8 │ u8 │ u8 │ u8 │ u8 │    │ u8 │  u8   │
└────────────────────────────────────────────────────────┘

┌─ As 8×s16 ─────────────────────────────────────────────┐
│ [0]       [1]       [2]       [3]  ...  [6]      [7]    │
│  s16  │    s16  │    s16  │    s16  │  s16  │  s16    │
└────────────────────────────────────────────────────────┘

┌─ As 4×s32 ─────────────────────────────────────────────┐
│ [0]           [1]           [2]           [3]           │
│      s32      │      s32      │      s32      │  s32     │
└────────────────────────────────────────────────────────┘
```

**Alignment:** PIE loads/stores require **16-byte (128-bit) alignment**. Always align data buffers:

```c
// Good
uint8_t aligned_buffer[1024] __attribute__((aligned(16)));

// Bad (will cause alignment fault)
uint8_t misaligned[16] = {...};  // start at random address
esp.vld.128.ip q0, x8, 16        // FAULT if not aligned
```

### QACC Accumulator (256-bit)

The 256-bit multiply-accumulate result register, split into two halves:

```
QACC (256 bits total)
┌───────────────────────────────────────────────────────────┐
│         QACC_H (upper 128 bits)                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │ Stores high parts of 4×s32 or 8×s16 products        │  │
│  └─────────────────────────────────────────────────────┘  │
├───────────────────────────────────────────────────────────┤
│         QACC_L (lower 128 bits)                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │ Stores low parts of 4×s32 or 8×s16 products         │  │
│  └─────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────┘

Used by: esp.vmulas.s16.qacc, esp.vmulas.u8.qacc, etc.
```

Extract results using `esp.srcmb.s8.qacc`, `esp.srcmb.s16.qacc` with shift/round/saturate.

### XACC Accumulator (40-bit scalar)

Narrow but deep accumulator for dot products and reductions:

```
XACC (40 bits total, signed)
┌──────────────────────────────────────┐
│  Sign-extended 40-bit integer        │
│  Holds sum of up to ~128K products   │
│  (u8×u8 at shift=0, typical range)   │
└──────────────────────────────────────┘

Used by: esp.vmulas.u8.xacc, esp.vmulas.s16.xacc, etc.
Extract: esp.srs.u.xacc / esp.srs.s.xacc (shift-round-saturate)
```

### Control Registers

**CSR_PIE_STATE_REG** (read/write via esp.movx.r.cfg, esp.movx.w.cfg):

```c
uint32_t cfg = esp.movx.r.cfg();  // Read PIE enable bits
cfg |= 0x02;                      // Enable PIE for unaligned access
esp.movx.w.cfg(cfg);              // Write back
```

Bits 0–1 typically control:
- Bit 0: PIE enable/disable
- Bit 1: Allow unaligned vector access (slower but tolerant)

---

## Instruction Reference

Instructions are organized by category with syntax and brief description. All operations are **128-bit wide** unless specified.

### Load / Store

#### Vector Load (128-bit)

```asm
esp.vld.128.ip  qd, xs, imm      ; Load 128b from xs+imm, post-inc xs by imm
esp.vld.128.xp  qd, xs, xt       ; Load 128b from xs, post-inc xs by xt
```

**Use:** Load QR registers from memory. Always requires 16-byte alignment.

```c
// Example: Load 4 int32 values
int32_t data[4] __attribute__((aligned(16))) = {10, 20, 30, 40};
asm("esp.vld.128.ip %0, %1, 4" : "=w"(qr0_val) : "r"(data));
```

#### Vector Load Broadcast (fill register)

```asm
esp.vldbc.8.ip   qd, xs, imm     ; Broadcast single byte to all 16 elements
esp.vldbc.16.ip  qd, xs, imm     ; Broadcast 16-bit value to all 8 elements
esp.vldbc.32.ip  qd, xs, imm     ; Broadcast 32-bit value to all 4 elements
```

**Use:** Fill entire register with one value. Useful for constants (e.g., saturation thresholds).

```asm
; Load 0xFF into all 16 bytes of qr1
esp.vldbc.8.ip  q1, x8, 0        ; Load *(x8) broadcast to 16×u8
```

#### Vector Store (128-bit)

```asm
esp.vst.128.ip  qd, xs, imm      ; Store 128b to xs+imm, post-inc xs
esp.vst.128.xp  qd, xs, xt       ; Store 128b to xs, post-inc xs by xt
```

**Use:** Write results back to memory.

```asm
esp.vst.128.ip  q0, x8, 16       ; Store q0 to *x8, x8 += 16
```

#### Partial Loads (64-bit, for FFT)

```asm
esp.vld.h.64.ip  qd, xs, imm     ; Load upper 64 bits
esp.vld.l.64.ip  qd, xs, imm     ; Load lower 64 bits
```

---

### Arithmetic (Integer)

#### Vector Add

```asm
esp.vadd.u8   qd, qs1, qs2      ; Add 16×u8 (wrapping)
esp.vadd.s8   qd, qs1, qs2      ; Add 16×s8 (wrapping)
esp.vadd.u16  qd, qs1, qs2      ; Add 8×u16 (wrapping)
esp.vadd.s16  qd, qs1, qs2      ; Add 8×s16 (wrapping)
esp.vadd.u32  qd, qs1, qs2      ; Add 4×u32 (wrapping)
esp.vadd.s32  qd, qs1, qs2      ; Add 4×s32 (wrapping)
```

#### Vector Subtract

```asm
esp.vsub.u8   qd, qs1, qs2      ; Subtract (wrapping)
esp.vsub.s8   qd, qs1, qs2
esp.vsub.u16  qd, qs1, qs2
esp.vsub.s16  qd, qs1, qs2
esp.vsub.u32  qd, qs1, qs2
esp.vsub.s32  qd, qs1, qs2
```

#### Vector Multiply

```asm
esp.vmul.u8   qd, qs1, qs2      ; Multiply u8×u8 → u8 (truncate upper)
esp.vmul.s8   qd, qs1, qs2      ; Multiply s8×s8 → s8
esp.vmul.u16  qd, qs1, qs2      ; Multiply u16×u16 → u16
esp.vmul.s16  qd, qs1, qs2      ; Multiply s16×s16 → s16
```

#### Widening Multiply

```asm
esp.vmul.s16.s8xs8   qd, qs1, qs2    ; Multiply s8×s8 → 8×s16 (widen)
esp.vmul.s32.s16xs16 qd, qs1, qs2    ; Multiply s16×s16 → 4×s32
```

**Use:** Prevent overflow by promoting to wider type. Critical for fixed-point DSP.

```asm
; Input: 16 int8 samples in q0, q1
; Output: 8 int16 products in q0
esp.vmul.s16.s8xs8  q0, q0, q1    ; q0 = [s8_0*s8_0, s8_1*s8_1, ...]
```

---

### Multiply-Accumulate (MAC)

The most critical instructions for DSP: multiply and add into an accumulator in one cycle.

#### MAC into XACC (scalar, 40-bit)

```asm
esp.vmulas.u8.xacc   qs1, qs2     ; qs1 × qs2 → XACC (unsigned, 16×u8)
esp.vmulas.s8.xacc   qs1, qs2     ; qs1 × qs2 → XACC (signed, 16×s8)
esp.vmulas.u16.xacc  qs1, qs2     ; qs1 × qs2 → XACC (unsigned, 8×u16)
esp.vmulas.s16.xacc  qs1, qs2     ; qs1 × qs2 → XACC (signed, 8×s16)
```

**Use:** Dot products, FIR filters. XACC accumulates across multiple MACs.

```asm
; Dot product of two 8-element int16 vectors
esp.zero.xacc                      ; Clear accumulator
esp.vmulas.s16.xacc  q0, q1        ; += q0 × q1
esp.srs.s.xacc  a0, 15             ; Extract result, shift-right by 15
```

#### MAC into QACC (vector, 256-bit)

```asm
esp.vmulas.u8.qacc   qs1, qs2      ; 16×u8 × 16×u8 → QACC
esp.vmulas.s8.qacc   qs1, qs2      ; 16×s8 × 16×s8 → QACC
esp.vmulas.u16.qacc  qs1, qs2      ; 8×u16 × 8×u16 → QACC
esp.vmulas.s16.qacc  qs1, qs2      ; 8×s16 × 8×s16 → QACC
```

Result is split: products go to QACC_H (upper) and QACC_L (lower).

#### Complex Multiply-Accumulate

```asm
esp.vcmulas.s8.qacc.h   qs1, qs2   ; Complex MAC (int8), upper half of result
esp.vcmulas.s8.qacc.l   qs1, qs2   ; Complex MAC (int8), lower half
esp.vcmulas.s16.qacc.h  qs1, qs2   ; Complex MAC (int16), upper half
esp.vcmulas.s16.qacc.l  qs1, qs2   ; Complex MAC (int16), lower half
```

**Use:** IQ mixing, NCO multiplication, radio front-end processing.

```asm
; Complex multiply IQ samples (stored as interleaved [I Q I Q ...])
; Input: q0 = [I0, Q0, I1, Q1, ...] (int8 pairs)
;        q1 = [I_nco, Q_nco, I_nco, Q_nco, ...] (NCO LO)
; Compute: out = in × LO* (conjugate)
esp.vcmulas.s8.qacc.h  q0, q1      ; I_out = I×I + Q×Q (real part)
```

---

### Complex Operations

#### Complex Multiply (result in QR register)

```asm
esp.cmul.u8   qd, qs1, qs2, rmode   ; Complex multiply u8 (rmode: 0–3)
esp.cmul.s8   qd, qs1, qs2, rmode   ; Complex multiply s8
esp.cmul.u16  qd, qs1, qs2, rmode   ; Complex multiply u16
esp.cmul.s16  qd, qs1, qs2, rmode   ; Complex multiply s16
```

rmode (rounding mode):
- 0 = R*R - I*I, R*I + I*R (real, imag, truncate)
- 1 = R*R + I*I, R*I - I*R (magnitude-squared variant)
- 2 = R*R + I*I, 2×R×I (conjugate variant)
- 3 = 2×R×I, R*R - I*I (alt variant)

**Use:** Single-cycle complex multiply for mixers, rotators.

```asm
; Multiply two complex int8 values
; q0 = [I_in, Q_in, ...]   (16 bytes = 8 complex)
; q1 = [I_lo, Q_lo, ...]   (16 bytes = 8 complex)
; q2 = result
esp.cmul.s8  q2, q0, q1, 0          ; q2 = q0 × q1 (rmode=0)
```

---

### Shift Operations

#### Vector Shift Left

```asm
esp.vsl.8   qd, qd_init           ; Shift left all 16×u8 by sar amount
esp.vsl.16  qd, qd_init
esp.vsl.32  qd, qd_init
```

SAR (shift amount register) is set via standard RISC-V or via `esp.movx.w.sar`.

#### Vector Shift Right

```asm
esp.vsr.u8  qd, qd_init           ; Unsigned shift right
esp.vsr.s8  qd, qd_init           ; Signed shift right (arithmetic)
esp.vsr.u16 qd, qd_init
esp.vsr.s16 qd, qd_init
esp.vsr.u32 qd, qd_init
esp.vsr.s32 qd, qd_init
```

#### Dynamic Shift (element-wise, different shift per element)

```asm
esp.vsld.8   qd, qs, qd_init      ; Dynamic shift left by qs[i] bits
esp.vsrd.8   qd, qs, qd_init      ; Dynamic shift right
esp.vsld.16  qd, qs, qd_init
esp.vsrd.16  qd, qs, qd_init
```

---

### Comparison & Selection

#### Vector Compare

```asm
esp.vcmp.eq.u8  qd, qs1, qs2      ; Compare ==
esp.vcmp.eq.s8  qd, qs1, qs2
esp.vcmp.eq.u16 qd, qs1, qs2
esp.vcmp.eq.s16 qd, qs1, qs2
esp.vcmp.eq.u32 qd, qs1, qs2
esp.vcmp.eq.s32 qd, qs1, qs2

esp.vcmp.lt.u8  qd, qs1, qs2      ; Compare <  (unsigned)
esp.vcmp.lt.s8  qd, qs1, qs2      ; Compare <  (signed)
...

esp.vcmp.gt.u8  qd, qs1, qs2      ; Compare >
esp.vcmp.gt.s8  qd, qs1, qs2
...
```

Result: all 1s (0xFF...) if true, all 0s if false.

#### Vector Mux (conditional select)

```asm
esp.vmux.8   qd, qs_true, qs_false, qcond  ; if qcond[i] then qs_true[i] else qs_false[i]
esp.vmux.16  qd, qs_true, qs_false, qcond
esp.vmux.32  qd, qs_true, qs_false, qcond
```

**Use:** Branchless conditionals.

```asm
esp.vcmp.lt.s16  q2, q0, q1          ; q2 = (q0 < q1) ? -1 : 0
esp.vmux.16      q3, q0, q1, q2      ; q3 = q2 ? q0 : q1 (min)
```

---

### Min / Max / Clamp

#### Vector Min/Max

```asm
esp.vmin.u8  qd, qs1, qs2     ; Element-wise minimum
esp.vmin.s8  qd, qs1, qs2
esp.vmin.u16 qd, qs1, qs2
esp.vmin.s16 qd, qs1, qs2
esp.vmin.u32 qd, qs1, qs2
esp.vmin.s32 qd, qs1, qs2

esp.vmax.u8  qd, qs1, qs2     ; Element-wise maximum
esp.vmax.s8  qd, qs1, qs2
...
```

#### Saturate

```asm
esp.vsat.u8   qd, qs, imm_lo, imm_hi  ; Clamp to [imm_lo, imm_hi]
esp.vsat.s8   qd, qs, imm_lo, imm_hi
esp.vsat.u16  qd, qs, imm_lo, imm_hi
esp.vsat.s16  qd, qs, imm_lo, imm_hi
esp.vsat.u32  qd, qs, imm_lo, imm_hi
esp.vsat.s32  qd, qs, imm_lo, imm_hi
```

#### Clamp to Signed Range

```asm
esp.vclamp.s16  qd, qs, imm_lo, imm_hi  ; Clamp s16 to range
```

**Use:** Prevent overflow in saturating arithmetic.

```asm
esp.vsat.u8  q0, q0, 0, 255        ; Clamp all bytes to [0, 255]
```

---

### Data Rearrangement

#### Unzip (Deinterleave pairs)

```asm
esp.vunzip.8   qd, qs           ; Separate odd/even u8 elements
esp.vunzip.16  qd, qs           ; Separate odd/even u16 elements
esp.vunzip.32  qd, qs           ; Separate odd/even u32 elements
```

Example: `[A0, B0, A1, B1, ...] → qd=[A0, A1, ..., B0, B1, ...]`

**Use:** Split interleaved IQ (I, Q, I, Q) into separate I and Q buffers.

```asm
; Input: q0 = [I0, Q0, I1, Q1, I2, Q2, I3, Q3] (8 int8 pairs)
; Output: q0 = [I0, I1, I2, I3, Q0, Q1, Q2, Q3]
esp.vunzip.8  q0, q0
```

#### Zip (Interleave pairs)

```asm
esp.vzip.8   qd, qs            ; Interleave odd/even u8 elements
esp.vzip.16  qd, qs
esp.vzip.32  qd, qs
```

Inverse of unzip.

#### Extend (Zero/Sign Extend)

```asm
esp.vext.u8   qd, qs, qs2, imm  ; Zero-extend u8 → u16
esp.vext.s8   qd, qs, qs2, imm  ; Sign-extend s8 → s16
esp.vext.u16  qd, qs, qs2, imm  ; Zero-extend u16 → u32
esp.vext.s16  qd, qs, qs2, imm  ; Sign-extend s16 → s32
```

**Use:** Widen small types to prevent overflow in accumulation.

```asm
; Convert 16×u8 → 8×u16 (lower half)
esp.vext.u8  q0, q0, q1, 0    ; q0 = u16(q0[0..7]), sign extended
```

---

### Accumulator Operations

#### Load/Store XACC

```asm
esp.ld.xacc.ip   xs, imm      ; Load 40-bit scalar from memory into XACC
esp.st.s.xacc.ip xs, imm      ; Store signed 40-bit XACC to memory
esp.st.u.xacc.ip xs, imm      ; Store unsigned 40-bit XACC to memory
```

#### Shift-Round-Saturate from XACC

```asm
esp.srs.u.xacc  xd, imm       ; Unsigned: XACC >> imm, round, saturate → xd (32-bit)
esp.srs.s.xacc  xd, imm       ; Signed: XACC >> imm, round, saturate → xd
```

**Use:** Extract dot product result with fixed-point scaling.

```asm
esp.zero.xacc                  ; Clear accumulator
esp.vmulas.s16.xacc  q0, q1    ; += q0 × q1
esp.vmulas.s16.xacc  q2, q3    ; += q2 × q3
esp.srs.s.xacc  a0, 15         ; a0 = (XACC >> 15), signed
```

#### Load/Store QACC (256-bit)

```asm
esp.ld.qacc.h.h.128.ip  xs, imm  ; Load upper half into QACC_H
esp.ld.qacc.l.h.128.ip  xs, imm
esp.ld.qacc.h.l.128.ip  xs, imm
esp.ld.qacc.l.l.128.ip  xs, imm

esp.st.qacc.h.h.128.ip  xs, imm  ; Store QACC_H upper half
esp.st.qacc.l.h.128.ip  xs, imm
esp.st.qacc.h.l.128.ip  xs, imm
esp.st.qacc.l.l.128.ip  xs, imm
```

#### Extract from QACC

```asm
esp.srcmb.u8.qacc   qd, xs, imm   ; Extract u8 from QACC with shift/round/saturate
esp.srcmb.s8.qacc   qd, xs, imm   ; Extract s8 from QACC
esp.srcmb.u16.qacc  qd, xs, imm   ; Extract u16 from QACC
esp.srcmb.s16.qacc  qd, xs, imm   ; Extract s16 from QACC
```

xs specifies shift amount and rounding options.

#### Zero Accumulators

```asm
esp.zero.xacc      ; Clear 40-bit scalar accumulator
esp.zero.qacc      ; Clear 256-bit vector accumulator
esp.zero.q qd      ; Clear vector register qd to all zeros
```

#### Move Between QR and QACC

```asm
esp.mov.u8.qacc  qd   ; Extract u8×16 from QACC_L (lower 128 bits)
esp.mov.s8.qacc  qd
esp.mov.u16.qacc qd   ; Extract u16×8 from QACC_L
esp.mov.s16.qacc qd
```

---

### Bitwise Operations

```asm
esp.andq  qd, qs1, qs2   ; AND
esp.orq   qd, qs1, qs2   ; OR
esp.xorq  qd, qs1, qs2   ; XOR
esp.notq  qd, qs         ; NOT (bitwise complement)
```

---

### Absolute Value

```asm
esp.vabs.8   qd, qs     ; Absolute value of 16×s8
esp.vabs.16  qd, qs     ; Absolute value of 8×s16
esp.vabs.32  qd, qs     ; Absolute value of 4×s32
```

---

### Hardware Loop (Zero-Overhead)

```asm
esp.lp.setup    loop_id, count_reg, loop_label
...
loop_label:
...
; Last instruction in loop body
```

**Critical for performance:** Saves branch overhead on tight kernels.

```asm
; Loop 1024 times
li  a0, 1024
esp.lp.setup  0, a0, .my_loop
    esp.vld.128.ip  q0, x8, 16
    esp.vadd.u8     q0, q0, q1
.my_loop:
    esp.vst.128.ip  q0, x9, 16
```

**Note:** Only the last instruction executes at loop start. Plan accordingly.

---

## What PIE Does NOT Support

### Critical Limitations

#### 1. No Floating-Point SIMD

PIE is **integer-only**. ESP32-P4 has a separate scalar FPU (float unit) that handles flw, fsw, fadd.s, etc., but **NO** float SIMD.

**Consequence:** Functions like `dsps_mul_f32` (element-wise float multiply) are NOT PIE-accelerated on P4. They fall back to ANSI C scalar loops.

```c
// WRONG: This is NOT accelerated by PIE on P4
void process_floats(float *in, float *out, int len) {
    for (int i = 0; i < len; i++) {
        out[i] = in[i] * 2.5f;  // Scalar FPU, NOT SIMD
    }
}

// RIGHT: Use int16/int32 fixed-point instead
void process_fixed(int16_t *in, int16_t *out, int len) {
    // PIE accelerates this
    for (int i = 0; i < len; i += 8) {
        // Load 8×int16, multiply by Q15 scale, store
    }
}
```

#### 2. Float Load/Store Are Scalar Only

You can load/store single float values, but they don't pack into PIE registers:

```asm
flw   f0, 0(x8)     ; Load float into FPU scalar register (valid)
esp.vld.128.ip q0, x8, 16  ; Load 128b into PIE vector (separate)
; f0 and q0 are different units!
```

#### 3. No Mixed Float/Int Operations

PIE and FPU are completely separate execution units. You cannot:
- Mix float results into PIE registers
- Use FPU instructions on QR registers

#### 4. Limited Register Pressure

Only 8 QR registers total (vs 16 NEON, 16+ x86). Spills to stack are expensive.

```asm
; All 8 QR registers in use:
q0, q1, q2, q3, q4, q5, q6, q7

; If you need a 9th: spill to stack (PSRAM roundtrip = slow)
esp.vst.128.ip  q0, sp, 0      ; Save q0 to stack
```

---

## Programming Patterns

Real-world code patterns for common DSP tasks.

### Pattern 1: Processing uint8 Arrays (Audio/SDR Data)

**Scenario:** Add a constant to every byte in a 1024-byte buffer.

**Approach:**
1. Load constant into broadcast register
2. Loop 1024/16 times (16 bytes per QR)
3. Add + store

```asm
.global add_uint8_constant
add_uint8_constant:
    ; a0 = in_buffer
    ; a1 = out_buffer
    ; a2 = len (bytes)
    ; a3 = constant (u8)

    ; Broadcast constant to all 16 bytes
    esp.vldbc.8.ip   q0, sp, 0     ; Load (constant) broadcast
    # Assume constant is at (sp+0)

    li  t0, 16
    esp.movx.w.sar   t0             ; Set shift amount

    srli t1, a2, 4                   ; t1 = len / 16 (loop count)

    esp.lp.setup  0, t1, .add_loop
        esp.vld.128.ip   q1, a0, 16  ; Load 16 bytes
        esp.vadd.u8      q1, q1, q0  ; Add constant
    .add_loop:
        esp.vst.128.ip   q1, a1, 16  ; Store result

    ret
```

---

### Pattern 2: IQ Deinterleave (SDR/Radio)

**Scenario:** Split [I0, Q0, I1, Q1, I2, Q2, ...] into separate I and Q buffers.

```asm
.global deinterleave_iq
deinterleave_iq:
    ; a0 = input (interleaved I/Q as u8)
    ; a1 = output_i
    ; a2 = output_q
    ; a3 = num_samples (IQ pairs)

    srli t1, a3, 3                   ; t1 = num_pairs / 8 (8 pairs per 128b)

    esp.lp.setup  0, t1, .deint_loop
        esp.vld.128.ip   q0, a0, 16  ; Load [I0,Q0, I1,Q1, ...]
        esp.vunzip.8     q0, q0      ; Deinterleave: [I0..I7, Q0..Q7]
        ; q0[0..7] = I values
        ; q0[8..15] = Q values
        esp.vst.h.64.ip  q0, a1, 8   ; Store upper (I values)
    .deint_loop:
        esp.vst.l.64.ip  q0, a2, 8   ; Store lower (Q values)

    ret
```

---

### Pattern 3: Complex Multiply (SDR NCO/Mixer)

**Scenario:** Multiply IQ samples by complex LO (numerically-controlled oscillator).

```asm
.global cmul_s8
cmul_s8:
    ; a0 = input IQ (s8 pairs: [I, Q, I, Q, ...])
    ; a1 = LO (s8 pairs: [I_lo, Q_lo, ...])
    ; a2 = output
    ; a3 = num_samples (IQ pairs, multiple of 8)

    srli t1, a3, 3                   ; t1 = pairs / 8

    esp.lp.setup  0, t1, .cmul_loop
        esp.vld.128.ip   q0, a0, 16  ; Load input
        esp.vld.128.ip   q1, a1, 16  ; Load LO
        esp.cmul.s8      q2, q0, q1, 0  ; Complex multiply (rmode=0)
    .cmul_loop:
        esp.vst.128.ip   q2, a2, 16  ; Store result

    ret
```

**rmode=0:** `Real = R*R - I*I, Imag = R*I + I*R`

---

### Pattern 4: Dot Product (FIR Filter)

**Scenario:** Compute dot product of two int16 vectors (e.g., FIR taps × samples).

```asm
.global dotprod_s16
dotprod_s16:
    ; a0 = vector_a (s16)
    ; a1 = vector_b (s16)
    ; a2 = length (elements, multiple of 8)
    ; Returns: a0 = dot product (s32)

    esp.zero.xacc                    ; Clear accumulator

    srli t0, a2, 3                   ; t0 = len / 8 (8 s16 per QR)

    esp.lp.setup  0, t0, .dot_loop
        esp.vld.128.ip   q0, a0, 16  ; Load 8×s16
        esp.vmulas.s16.xacc  q0, q1  ; XACC += q0 × q1
        esp.vld.128.ip   q1, a1, 16  ; Load next
    .dot_loop:
        nop

    ; Extract result with shift-round-saturate
    li  t1, 0                        ; No shift
    esp.srs.s.xacc  a0, t1           ; a0 = XACC

    ret
```

---

### Pattern 5: Type Widening Pipeline (u8 → s16 → s32)

**Scenario:** Convert uint8 samples to int16 (for headroom), then accumulate as int32.

```asm
.global widen_and_sum
widen_and_sum:
    ; a0 = input u8 (128 bytes = 128 samples)
    ; Returns: a0 = sum (s32)

    esp.zero.xacc

    li  t0, 8                        ; 8 iterations (128/16)
    esp.lp.setup  0, t0, .widen_loop
        esp.vld.128.ip   q0, a0, 16  ; Load 16×u8
        esp.vext.u8      q1, q0, q2, 0  ; Widen to 8×u16 (lower half)
        esp.vext.u8      q2, q0, q2, 1  ; Widen to 8×u16 (upper half)
        esp.vmulas.u16.xacc  q1, q1  ; Accumulate as u16×u16
    .widen_loop:
        esp.vmulas.u16.xacc  q2, q2

    esp.srs.u.xacc  a0, 0            ; Extract unsigned result

    ret
```

---

### Pattern 6: Saturation/Clamp

**Scenario:** Clamp output to [0, 255] after filtering.

```asm
; q0 contains possibly out-of-range u8 values
esp.vsat.u8  q0, q0, 0, 255        ; Saturate to [0, 255]
esp.vst.128.ip  q0, a0, 16         ; Store clamped result
```

---

### Pattern 7: Fast Memcpy (128-bit aligned)

**Scenario:** Copy 1 MB block using PIE, ~74% faster than standard memcpy.

```asm
.global memcpy_pie
memcpy_pie:
    ; a0 = dest
    ; a1 = src
    ; a2 = length (bytes, must be multiple of 16)

    srli t0, a2, 4                   ; t0 = length / 16

    esp.lp.setup  0, t0, .memcpy_loop
        esp.vld.128.ip   q0, a1, 16
    .memcpy_loop:
        esp.vst.128.ip   q0, a0, 16

    ret
```

**Performance:** ~4 cycles per 16 bytes = 400 MHz / 4 = 100 MB/s peak.
(Actual: ~74 MB/s due to pipeline and memory contention, still 2-3× standard memcpy.)

---

## esp-dsp Function Matrix

esp-dsp is Espressif's DSP library. Many functions have P4-specific assembly (arp4 suffix = ARP4 processor = PIE). Others fall back to ANSI C.

### Key Insight

**arp4 suffix = PIE assembly available.** No arp4 = scalar C fallback.

| Function | arp4 available | Data type | What it does | PIE used? |
|----------|----------------|-----------|--------------|-----------|
| dsps_fft2r_fc32_arp4 | **Yes** | complex float | FFT via hardware loops | Partial (hw loop, not SIMD) |
| dsps_fft2r_sc16_arp4 | **Yes** | complex int16 | FFT via PIE SIMD | **Full SIMD** |
| dsps_fird_f32_arp4 | **Yes** | float | FIR decimator | Partial (hw loop, not SIMD) |
| dsps_fird_s16_arp4 | **Yes** | int16 | FIR decimator | **Full SIMD** |
| dspi_dotprod_u8_arp4 | **Yes** | uint8 | Dot product 2D | **Full SIMD** |
| dspi_dotprod_s16_arp4 | **Yes** | int16 | Dot product 2D | **Full SIMD** |
| dspm_mult_f32_arp4 | **Yes** | float | Matrix multiply | Partial (hw loop) |
| dspm_mult_s16_arp4 | **Yes** | int16 | Matrix multiply | **Full SIMD** |
| **dsps_mul_f32** | **NO** | float | Element-wise multiply | Scalar C fallback |
| **dsps_add_f32** | **NO** | float | Element-wise add | Scalar C fallback |
| **dsps_mulc_f32** | **NO** | complex float | Complex multiply | Scalar C fallback |
| **dsps_addc_f32** | **NO** | complex float | Complex add | Scalar C fallback |
| **dsps_sub_f32** | **NO** | float | Element-wise subtract | Scalar C fallback |

### Recommendation

**Use int16/int32 fixed-point whenever possible on P4.** The float functions do NOT benefit from PIE acceleration—you're limited to scalar FPU speed (much slower than SIMD).

Example: If you need to multiply 1024 float samples by a gain:

**Bad (scalar):**
```c
float buf[1024];
dsps_mulc_f32(buf, 2.5f, 1024);  // Calls scalar C loop, NOT PIE
```

**Good (fixed-point SIMD):**
```c
int16_t buf[1024];
// Scale by Q15 fixed point: multiply by 0.305 ≈ 10000 in Q15
// Then use PIE to accelerate:
// esp.vmul.s16 q_result, q_input, q_scale
```

---

## Performance Guidelines

### Register & Alignment

1. **Always 16-byte align vector data:**
   ```c
   uint8_t buffer[1024] __attribute__((aligned(16)));  // Good
   uint8_t buffer[1024];  // Potentially bad (may not align)
   ```

2. **Use 16-byte or 32-byte cache line sizes for optimal throughput:**
   - Minimize cache conflicts by aligning working sets to 32 bytes

3. **Avoid QR register spills:**
   - Only 8 QR registers (0–7)
   - Spilling to stack = ~4 cycles per 128 bits
   - Plan code to reuse registers aggressively

### Load/Compute Pipeline

Overlap loads and computation:

```asm
; Bad: load, wait for result, compute, wait, store
esp.vld.128.ip  q0, x8, 16
esp.vadd.u8     q0, q0, q1      ; Stall: q0 not ready
esp.vst.128.ip  q0, x9, 16

; Good: load next while computing current
esp.vld.128.ip  q0, x8, 16
esp.lp.setup  0, count, .loop
    esp.vadd.u8     q0, q0, q1
    .loop:
        esp.vld.128.ip  q0, x8, 16  ; Load next overlaps with store
        esp.vst.128.ip  q0, x9, 16
```

### Type Selection

- **uint8/int8:** Use for audio/SDR raw samples (8-bit ADC). 16 elements per QR.
- **uint16/int16:** Use for intermediate results with headroom. 8 elements per QR. Widening multiply available (s8→s16, s16→s32).
- **uint32/int32:** Use only for final accumulators or results. 4 elements per QR.

### Hardware Loop Timing

`esp.lp.setup` is zero-overhead: no branch penalty. Critical for tight inner loops.

```asm
; Loop overhead: 0 cycles (hardware)
esp.lp.setup  0, 1000, .loop
    esp.vld.128.ip  q0, x8, 16
    .loop:
        esp.vst.128.ip  q0, x9, 16
; Total: ~1000 iterations × 2 cycles (load + store) = 2000 cycles @ 400 MHz
```

### Avoid Cache Misses

On 400 MHz P4:
- L1 data cache: 16 KB per core, 4-way, 32-byte lines
- Miss penalty: ~10 cycles
- Prefetcher: helps with sequential access

Strategy: Process data in cache-friendly chunks (4–8 KB at a time).

---

## Common Pitfalls

### 1. Assuming Float Functions Are PIE-Accelerated

**Wrong:**
```c
void gain(float *sig, int len) {
    dsps_mulc_f32(sig, 2.5f, len);  // SCALAR on P4, not SIMD!
}
```

**Right:**
```c
void gain_fixed(int16_t *sig, int len) {
    // Use PIE-accelerated fixed-point multiply
    // dsps_mul_s16_arp4 or inline assembly
}
```

### 2. Misaligned Vector Access

**Wrong:**
```c
uint8_t buf[10];
asm("esp.vld.128.ip %0, %1, 16" : "=w"(q0) : "r"(&buf[5]));  // Fault!
```

**Right:**
```c
uint8_t buf[16] __attribute__((aligned(16)));
asm("esp.vld.128.ip %0, %1, 16" : "=w"(q0) : "r"(buf));
```

### 3. Forgetting to Zero Accumulators

```asm
; Wrong: XACC may contain garbage
esp.vmulas.s16.xacc  q0, q1
esp.srs.s.xacc  a0, 0

; Right: Clear first
esp.zero.xacc
esp.vmulas.s16.xacc  q0, q1
esp.srs.s.xacc  a0, 0
```

### 4. QR Register Pressure (Spilling)

```asm
; All 8 registers used, loop needs another:
; q0–q7 = in use
q8 = ...  ; SPILL! Must save/restore q0 to stack each iteration
```

Solution: Reduce working set, loop over smaller batches, or refactor to use fewer intermediate values.

### 5. Complex Rounding Modes Confusion

`esp.cmul.s8` has 4 rounding modes (rmode 0–3). Each produces different results:

```asm
esp.cmul.s8  q2, q0, q1, 0    ; rmode=0: standard (R*R - I*I, R*I + I*R)
esp.cmul.s8  q2, q0, q1, 1    ; rmode=1: magnitude variant
esp.cmul.s8  q2, q0, q1, 2    ; rmode=2: conjugate variant
esp.cmul.s8  q2, q0, q1, 3    ; rmode=3: alt variant
```

For standard complex multiply, use **rmode=0**.

### 6. Mixing PIE and FPU Results

FPU (scalar floats) and PIE (int SIMD) are separate units:

```asm
flw   f0, 0(x8)        ; FPU scalar
esp.vld.128.ip q0, x8, 16  ; PIE vector
; f0 and q0 are independent! Cannot mix.
```

---

## Comparison with VOLK

VOLK (Vector-Optimized Library of Kernels, GNU Radio) is a portable SIMD library. How do VOLK kernels map to PIE?

| VOLK Kernel | Input Type | PIE Equivalent | Notes |
|-------------|-----------|------------------|-------|
| volk_8ic_deinterleave_real_q | complex int8 | esp.vunzip.8 | Separate I from Q (8-bit) |
| volk_8ic_x2_multiply_conjugate | int8 × int8 | esp.cmul.s8 | Single-cycle complex mul |
| volk_16ic_x2_multiply_conjugate | int16 × int16 | esp.cmul.s16 | Complex mul for audio |
| volk_32f_magnitude_squared | float | **No SIMD** | Scalar FPU only on P4 |
| volk_16i_s32f_convert_32f | int16 → float | **No SIMD** | Scalar conversion |
| volk_16i_max_s32 | int16 → max | esp.vmax.s16 | Element-wise max, then scalar max |
| volk_16i_multiply_conjugate_s32 | int16 × conj | esp.vcmulas.s16.qacc | Complex MAC into 256-bit acc |

**Key insight:** VOLK is portable across many architectures (x86 SSE, ARM NEON, AVX, etc.). PIE is ESP32-P4-specific and optimized for integer-heavy DSP (radio, imaging). For float signal processing, VOLK may be more suitable; for integer-heavy SDR, PIE is much faster.

---

## Appendix: Register & Instruction Quick Reference

### QR Register Operands

All instructions use QR0–QR7 notation:

```asm
q0, q1, q2, q3, q4, q5, q6, q7
```

### Scalar Register Operands (x registers)

RISC-V standard x0–x31:

```asm
x0 (zero), x1 (ra), x2 (sp), x3 (gp), x4 (tp),
x5–x7 (t0–t2),
x8–x9 (s0–s1),
x10–x17 (a0–a7),
x18–x27 (s2–s11),
x28–x31 (t3–t6)
```

### Instruction Encoding Quick Table

| Mnemonic Pattern | Format | Example |
|-----------------|--------|---------|
| `esp.v*.u8/s8` | 16×8-bit | `esp.vadd.u8 q0, q1, q2` |
| `esp.v*.u16/s16` | 8×16-bit | `esp.vmul.s16 q0, q1, q2` |
| `esp.v*.u32/s32` | 4×32-bit | `esp.vadd.s32 q0, q1, q2` |
| `esp.cmul.*` | Complex | `esp.cmul.s8 q0, q1, q2, rmode` |
| `esp.*xacc` | XACC (40b) | `esp.vmulas.s16.xacc q0, q1` |
| `esp.*qacc` | QACC (256b) | `esp.vmulas.s8.qacc q0, q1` |
| `esp.vld.128.*` | Load | `esp.vld.128.ip q0, x8, 16` |
| `esp.vst.128.*` | Store | `esp.vst.128.ip q0, x8, 16` |
| `esp.lp.setup` | HW loop | `esp.lp.setup 0, count, label` |

---

## Summary

PIE on ESP32-P4 is a powerful integer SIMD engine optimized for DSP, signal processing, and AI inference. Key takeaways:

1. **Integer-only SIMD:** No float SIMD (separate scalar FPU).
2. **8 QR registers, 2 accumulators:** Limited but sufficient for tight kernels.
3. **Complex-native:** Dedicated cmul/vcmulas for IQ processing.
4. **Hardware loops:** Zero-overhead unrolling.
5. **Alignment critical:** Always 16-byte align vector data.
6. **esp-dsp fallback:** Float functions (dsps_mul_f32, etc.) are NOT PIE-accelerated.
7. **Fixed-point preferred:** Use int16/int32 instead of float for DSP on P4.

For detailed instruction encodings, consult:
- ESP32-P4 Technical Reference Manual (RISC-V extension section)
- esp-dsp source assembly (arp4 implementations)
- esp-idf examples in esp-dsp/examples/

---

## References

- ESP32-P4 Technical Reference Manual: esp-idf RISC-V extension chapter
- esp-dsp GitHub: https://github.com/espressif/esp-dsp (arp4 implementations)
- VOLK: https://www.libvolk.org/
- GNU Radio: https://www.gnuradio.org/
