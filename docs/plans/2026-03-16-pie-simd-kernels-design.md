# ESP32-P4 PIE SIMD Kernel Optimization Plan

> Custom assembly kernels for RTL-SDR signal processing pipeline
> Date: 2026-03-16 | Target: ESP32-P4 (rv32imafc_xesppie)

---

## Executive Summary

The ESP32-P4's PIE SIMD extension provides 128-bit vector registers with dedicated
FFT, complex multiply, and MAC instructions — but the current RTL-SDR codebase uses
almost exclusively **float32 scalar paths** for DSP operations, getting zero SIMD
acceleration. The PIE only accelerates integer types (s8/u8/s16/s32).

This plan identifies 12 optimization targets across all 7 project branches, with
estimated total speedups of 3-8x on the core DSP pipeline. The highest-impact change
is converting the DDC from float32 to int16 Q15, which affects every branch.

---

## 1. PIE SIMD Architecture Summary

### Registers
- **q0-q7**: 8 x 128-bit vector registers (16x int8, 8x int16, 4x int32)
- **qacc**: Wide accumulator (4 x 128-bit quadrants) for MAC without overflow
- **xacc**: 40-bit scalar accumulator for dot-product reductions
- **sar**: Shift Amount Register for requantization
- **Two hardware loop units** (id 0, 1) for zero-overhead nested loops

### Key Instruction Categories for SDR

| Category | Instructions | SDR Use Case |
|----------|-------------|--------------|
| Fused MAC+Load | `esp.vmulas.s16.xacc.ld.ip` | Dot products, FIR, correlators |
| Complex Multiply | `esp.cmul.s16`, `esp.fft.cmul.s16` | NCO mixing, DDC |
| FFT Butterfly | `esp.fft.r2bf.s16`, `esp.fft.ams.s16` | FFT (already used by esp-dsp) |
| Bit Reversal | `esp.fft.bitrev` | FFT reordering |
| Pack/Unpack | `esp.vzip.8`, `esp.vunzip.8` | IQ deinterleave |
| Saturate/Clamp | `esp.vsat.s16`, `esp.vclamp.s16` | Output limiting |
| Broadcast Load | `esp.vldbc.16.ip` | Scalar coefficient broadcast |
| Widening Multiply | `esp.vmul.s32.s16xs16` | Q15 multiply without overflow |
| Vector Compare | `esp.vcmp.gt.s16` | Threshold detection |

### Critical Constraint
**PIE SIMD is integer-only.** All `dsps_*_f32` functions in esp-dsp are scalar C
fallbacks. The DDC currently calls 7x float functions per batch = zero SIMD benefit.

### Alignment Requirements
- int8 kernels: buffer length must be multiple of 16
- int16 kernels: buffer length must be multiple of 8
- Buffer addresses should be 16-byte aligned (`heap_caps_aligned_alloc(16, ...)`)

---

## 2. Current Pipeline Analysis

```
USB Bulk IN (2 MB/s, uint8 IQ pairs)
    |
    v
Ring Buffer (256 KB, lock-free)
    |
    +---> FFT Path (all branches)
    |     1. uint8 -> int16 conversion + Hann window [SCALAR - 12 cyc/sample]
    |     2. dsps_fft2r_sc16 [PIE SIMD - 2 cyc/sample, already optimized]
    |     3. Power spectrum: int16 -> float -> float MAC [SCALAR - 10 cyc/sample]
    |     4. dB conversion: log10f + scale + clamp [SCALAR - 25 cyc/sample]
    |     5. FFT shift + uint8 output [SCALAR - 3 cyc/sample]
    |
    +---> DDC Path (WebSDR narrowband, AIS, APRS)
    |     1. uint8 -> float conversion [SCALAR - 4 cyc/sample]
    |     2. NCO mix: 7x dsps_*_f32 calls [ALL SCALAR - 35 cyc/sample]
    |     3. CIC decimation [SCALAR float - 6 cyc/sample]
    |     4. float -> uint8 output [SCALAR - 3 cyc/sample]
    |
    +---> Branch-specific demodulation
          ADS-B: magnitude + preamble scan [PARTIAL SIMD, scalar gaps]
          AIS:   FM discriminator [SCALAR, broken bias subtract]
          APRS:  Goertzel tone detect [SCALAR, fake SIMD labels]
          GSM:   PSS correlator [PARTIAL SIMD, scalar gaps]
          WSPR:  float FFT + Viterbi/LDPC [ALL SCALAR]
```

**Total scalar overhead per sample through FFT path: ~52 cycles**
**Total scalar overhead per sample through DDC path: ~48 cycles**
**Already-SIMD portion: only the FFT butterfly itself (~2 cycles)**

---

## 3. Implementation Plan

### Phase 0: Quick Wins (30 min, all branches)

Fix 4 instances of "fake SIMD" — scalar `for` loops where `dsps_addc_s16` should
be called. These are 1-line fixes that immediately enable the claimed SIMD paths.

| File | Line | Fix |
|------|------|-----|
| `adsb_decode.c:191-192` | Bias subtract | Replace scalar loop with `dsps_addc_s16` |
| `ais_decode.c:482-483` | Bias subtract | Replace scalar loop with `dsps_addc_s16` |
| `cell_decode.c:432-433` | Bias subtract | Replace scalar loop with `dsps_addc_s16` |
| `cell_decode.c:464-466` | Subtract | Replace scalar loop with `dsps_sub_s16` |

**Impact:** Immediate 2-3x on affected inner loops.

---

### Phase 1: Core FFT Pipeline (1 day, all branches)

New component: `components/dsp/pie_kernels.S` + `components/dsp/pie_kernels.h`

#### Kernel 1.1: `pie_u8iq_to_s16_windowed`
Convert uint8 IQ pairs to windowed int16 in a single pass.

```
Input:  uint8 IQ buffer (16-byte aligned, length multiple of 16)
        int16 Hann window table (Q15, pre-computed)
Output: int16 windowed IQ buffer (interleaved re/im)

Algorithm:
  1. esp.vld.128.ip  q0, src, 16      // load 16 uint8 samples
  2. esp.vldext.s8.ip q0, ...          // sign-extend to 16-bit (after bias sub)
  3. esp.vsadds.s16   q0, q0, -128     // subtract DC bias (128)
  4. esp.vsl.32        q0, q0           // shift left 8 (scale to int16 range)
  5. esp.vld.128.ip  q1, win, 16       // load 8 window coefficients
  6. esp.vmul.s32.s16xs16 ...           // Q15 multiply: sample * window
  7. shift right 15, saturate to s16
  8. esp.vst.128.ip  q2, dst, 16       // store 8 windowed int16 values

Performance: 8 IQ pairs per iteration, ~2 cycles/pair vs ~12 current
```

#### Kernel 1.2: `pie_power_spectrum_s16`
Compute power spectrum from int16 FFT output, accumulate into int32.

```
Input:  int16 FFT output (interleaved re/im, fft_n elements)
        int32 accumulator buffer (fft_n/2 bins)
        int accumulation count
Output: int32 accumulated power per bin

Algorithm (per 8 complex samples = 4 bins):
  1. esp.vld.128.ip  q0, src, 16      // load 8 int16 (4 complex pairs)
  2. esp.vunzip.16   q0, q1           // deinterleave: q0=re[0..3], q1=im[0..3]
  3. esp.vmul.s32.s16xs16 q2, q0, q0  // re*re -> int32
  4. esp.vmul.s32.s16xs16 q3, q1, q1  // im*im -> int32
  5. esp.vadd.s32     q2, q2, q3       // power = re^2 + im^2
  6. esp.vld.128.ip  q4, acc, 0        // load existing accumulator
  7. esp.vadd.s32     q4, q4, q2       // accumulate
  8. esp.vst.128.ip  q4, acc, 16      // store back

Performance: 4 bins per iteration, ~3 cycles/bin vs ~10 current
```

#### Kernel 1.3: `pie_db_convert_s32_to_u8`
Integer log2 approximation + scale to uint8 dB.

```
Input:  int32 power spectrum (fft_n/2 bins)
        int16 scale factor, int16 offset (for dB range mapping)
Output: uint8 dB spectrum (0-255)

Algorithm:
  - Use CLZ (count leading zeros) for integer log2 base
  - 4-entry LUT interpolation for fractional part
  - Scale and clamp to [0, 255] using esp.vsat.u8
  - Apply FFT shift (second half first) via output pointer arithmetic

Performance: 4 bins per iteration, ~6 cycles/bin vs ~25 current
```

---

### Phase 2: DDC Integer Pipeline (2-3 days, all DDC users)

This is the **single highest-impact optimization**. Replace the entire DDC float
pipeline with int16 Q15 fixed-point using PIE SIMD.

#### New DDC Design

```
uint8 IQ input
    |
    v
[pie_u8_to_s16_bias]        // uint8 -> int16, subtract 128, scale
    |                         // PIE: esp.vldext + esp.vsadds + esp.vsl
    v
[pie_nco_mix_s16]            // Complex multiply with pre-computed NCO table
    |                         // PIE: esp.cmul.s16 (dedicated complex multiply!)
    v
[pie_cic_decimate_s16]       // CIC accumulate-and-dump in int32
    |                         // PIE: esp.vadd.s32 for accumulation
    v
[pie_s16_to_u8_output]       // int16 -> uint8 with scaling
                              // PIE: esp.vsat.u8 + esp.vst
```

#### Kernel 2.1: `pie_nco_mix_s16`
The DDC mixing kernel — replaces 7x scalar float calls.

```
Input:  int16 IQ buffer (interleaved, N samples)
        int16 NCO table (pre-computed cos/sin Q15, one period)
        uint32 phase accumulator, uint32 phase increment
Output: int16 mixed IQ buffer

Algorithm:
  The NCO table stores one full period of [cos, -sin, sin, cos] tuples
  packed as int16 Q15 for direct use with esp.cmul.s16.

  Per 4 complex samples:
  1. esp.vld.128.ip  q0, src, 16         // load 4 IQ pairs (8 int16)
  2. esp.vld.128.ip  q1, nco_ptr, 16     // load 4 NCO tuples
  3. esp.cmul.s16     q2, q0, q1, 0      // complex multiply mode 0
  4. esp.vst.128.ip  q2, dst, 16         // store mixed output

  Phase tracking: advance nco_ptr by phase_inc, wrap at table end.

Performance: 4 complex samples per iteration, ~2 cycles/sample vs ~35 current
Speedup: ~17x on the mixing stage alone
```

#### Kernel 2.2: `pie_cic_decimate_s16`
CIC decimation with int32 accumulator.

```
Input:  int16 mixed IQ, decimation ratio R
Output: int16 decimated IQ

Algorithm:
  Accumulate R input samples into int32, then output scaled int16.
  Use esp.vmulas.s16.qacc for fused accumulate.
  After R samples, extract via esp.srcmb.s16.qacc with shift = log2(R).

Performance: R samples per output, ~1 cycle/input sample
```

#### NCO Table Pre-computation

```c
// Generate Q15 NCO table at init time
// Table size = sample_rate / gcd(sample_rate, offset_freq) (one full period)
// Each entry: [cos(phi), -sin(phi), sin(phi), cos(phi)] as int16 Q15
// This layout maps directly to esp.cmul.s16 complex multiply format
void pie_nco_table_init(int16_t *table, uint32_t table_len,
                        int32_t offset_hz, uint32_t sample_rate);
```

---

### Phase 3: Branch-Specific Optimizations (1 week total)

#### 3.1 ADS-B Magnitude Kernel (1 day)
`pie_magnitude_approx_u8_to_u16` — single-pass: bias subtract, abs, max/min,
alpha-beta magnitude approximation.

```
Input:  uint8 IQ pairs
Output: uint16 magnitude per sample

Steps:
  1. esp.vldext.s8.ip -> int16 with bias subtract
  2. esp.vabs.16 for |I| and |Q|
  3. esp.vmax.s16 / esp.vmin.s16 for max(|I|,|Q|), min(|I|,|Q|)
  4. Multiply max by alpha (15/16 = 0xF000 Q15), min by beta (15/32 = 0x3C00 Q15)
  5. esp.vadd.s16 for magnitude estimate
  6. esp.vst.128.ip output

8 IQ pairs per iteration, ~3 cycles/pair vs ~7 current
```

#### 3.2 GSM: Kill Double-Precision Trig (2 hours)
Replace `cos()`/`sin()` calls in FCCH detection (`cell_decode.c:258-259`) with
the same pre-computed int16 NCO table from Phase 2. This alone gives 10-20x speedup
on that function since double-precision trig costs ~60 cycles per call.

#### 3.3 GSM/WSPR: Reuse int16 FFT (2 hours each)
Replace custom `fft_float()` in `cell_decode.c:141-225` and
`wspr_ft8_decode.c:320-351` with the shared `dsp_fft_compute()` path that already
uses `dsps_fft2r_sc16` (PIE-accelerated). 4-5x speedup, zero new assembly needed.

#### 3.4 AIS FM Discriminator Vectorization (1 day)
Vectorize the cross-product/dot-product computation in `ais_decode.c:503-507`:
- `cross = I[n]*Q[n-1] - Q[n]*I[n-1]` -> `esp.cmul.s16` (complex multiply)
- `dot = I[n]*I[n-1] + Q[n]*Q[n-1]` -> `esp.cmul.s16` mode 1
- `fast_atan2_q15` remains scalar (data-dependent, can't vectorize)

#### 3.5 APRS Dual-Goertzel Kernel (1 day)
Pack mark (1200 Hz) and space (2200 Hz) Goertzel state into a single 128-bit
register. Process both tones simultaneously on each input sample.

```
q0 = [mark_s1, mark_s2, space_s1, space_s2]  // int32 x 4
q1 = [mark_coeff, mark_coeff, space_coeff, space_coeff]  // int32 x 4

Per sample:
  1. esp.vmul.s32  q2, q0, q1     // coeff * s1 for both tones
  2. Subtract s2, add new sample
  3. Shift state: s2 = old_s1, s1 = new_s0

2 tones per iteration instead of 2 sequential passes
```

#### 3.6 PSS Correlator Scalar Gap Fixes (2 hours)
Replace scalar loops in `cell_decode.c:432-433,464-466` with `dsps_addc_s16`
and `dsps_sub_s16`. No new assembly, just calling existing SIMD functions.

---

### Phase 4: Custom Kernel Infrastructure (ongoing)

#### File Organization

```
components/dsp/
  dsp.c                          # Existing DSP engine (modified to call PIE kernels)
  dsp.h                          # Existing header
  pie_kernels.S                  # All PIE SIMD assembly kernels
  pie_kernels.h                  # C declarations for assembly functions
  pie_kernels_platform.h         # Conditional compilation (#if CONFIG_IDF_TARGET_ESP32P4)
  pie_nco.c                      # NCO table generation (C, runs once at init)
  pie_nco.h                      # NCO table header
```

#### Assembly Template (follow esp-dsp conventions)

```asm
// pie_kernels.S
#include "pie_kernels_platform.h"

#if (pie_kernels_enabled == 1)

    .text
    .align 4

// -------------------------------------------------------
// pie_u8iq_to_s16_windowed(const uint8_t *src, const int16_t *win,
//                           int16_t *dst, int count)
// a0 = src, a1 = win, a2 = dst, a3 = count
// count must be multiple of 16 (8 IQ pairs)
// -------------------------------------------------------
    .global pie_u8iq_to_s16_windowed
    .type   pie_u8iq_to_s16_windowed, @function
pie_u8iq_to_s16_windowed:
    // Alignment check
    andi    t0, a3, 15
    bnez    t0, pie_u8iq_to_s16_windowed_ansi    // fallback if not aligned

    srli    a3, a3, 4              // count / 16 = loop iterations
    li      t1, -128               // bias value

    esp.lp.setup  0, a3, .L_win_end
        esp.vld.128.ip    q0, a0, 16       // load 16 uint8 samples
        esp.vldext.s8.ip  q0, a0, 0        // widen to int16 (TBD exact encoding)
        // ... bias subtract, window multiply, store ...
.L_win_end:
        esp.vst.128.ip    q2, a2, 16

    ret

    // ANSI C fallback (linked from pie_kernels.c)
pie_u8iq_to_s16_windowed_ansi:
    j   pie_u8iq_to_s16_windowed_c

    .size   pie_u8iq_to_s16_windowed, . - pie_u8iq_to_s16_windowed

#endif // pie_kernels_enabled
```

#### Platform Header

```c
// pie_kernels_platform.h
#pragma once

#if CONFIG_IDF_TARGET_ESP32P4
  #ifdef CONFIG_DSP_OPTIMIZED
    #define pie_kernels_enabled 1
  #else
    #define pie_kernels_enabled 0
  #endif
#else
  #define pie_kernels_enabled 0
#endif
```

#### C Header

```c
// pie_kernels.h
#pragma once
#include <stdint.h>
#include "pie_kernels_platform.h"

// Phase 1: FFT pipeline
void pie_u8iq_to_s16_windowed(const uint8_t *src, const int16_t *win,
                               int16_t *dst, int count);
void pie_power_spectrum_s16(const int16_t *fft_out, int32_t *accum,
                             int fft_n, int avg_count);
void pie_db_convert_s32_to_u8(const int32_t *power, uint8_t *db_out,
                               int bins, float db_min, float db_max);

// Phase 2: DDC pipeline
void pie_nco_mix_s16(const int16_t *iq_in, const int16_t *nco_table,
                      int16_t *iq_out, int count,
                      uint32_t *phase, uint32_t phase_inc, uint32_t table_len);
void pie_cic_decimate_s16(const int16_t *in, int16_t *out,
                           int in_count, int decim_ratio);

// Phase 3: Branch-specific
void pie_magnitude_approx_u16(const uint8_t *iq_in, uint16_t *mag_out, int count);
```

---

## 4. Estimated Impact

### Cycle Budget (2.4 MSPS, 4096-pt FFT, ESP32-P4 @ 400 MHz)

| Stage | Current (cyc/sample) | Optimized | Speedup |
|-------|---------------------|-----------|---------|
| FFT window + convert | 12 | 3 | 4x |
| FFT butterfly (already SIMD) | 2 | 2 | 1x |
| Power spectrum | 10 | 2.5 | 4x |
| dB + shift | 25 (@display rate) | 10 | 2.5x |
| DDC mixing | 35 | 2 | 17x |
| DDC decimation | 6 | 1 | 6x |
| DDC format convert | 7 | 1.5 | 4.5x |
| **FFT path total** | **52** | **17.5** | **3x** |
| **DDC path total** | **48** | **4.5** | **10.6x** |

### Branch-Specific Gains

| Branch | Key Optimization | Estimated Gain |
|--------|-----------------|----------------|
| feature/websdr | DDC int16 + dead client eviction | 5-8x DDC throughput |
| feature/adsb | Magnitude kernel + preamble SIMD | 2-3x decode throughput |
| feature/ais | FM discriminator vectorize + DDC | 3-4x combined |
| feature/aprs | Dual Goertzel + DDC int16 | 2-3x combined |
| feature/gsm-lte | Kill double trig + reuse int16 FFT | 4-10x on scan |
| feature/wspr-ft8 | Reuse int16 FFT + power spectrum | 4-5x on spectrum |

### Throughput Implications

At current scalar performance, the FFT path at 2.4 MSPS uses ~52 * 2.4M = 125M
cycles/sec out of 400M available (31% CPU). After optimization: ~42M cycles/sec
(10.5% CPU). This frees ~83M cycles/sec for additional DDC channels, demodulation,
or higher sample rates (up to ~3.2 MSPS becomes feasible).

---

## 5. Implementation Schedule

| Week | Phase | Deliverable |
|------|-------|-------------|
| 1 Mon | Phase 0 | Fix 4 fake-SIMD scalar loops (all branches) |
| 1 Mon-Tue | Phase 1 | pie_kernels.S infrastructure + FFT windowing kernel |
| 1 Wed | Phase 1 | Power spectrum kernel + dB conversion |
| 1 Thu-Fri | Phase 2 | DDC NCO table + pie_nco_mix_s16 kernel |
| 2 Mon | Phase 2 | CIC decimation kernel + DDC integration into dsp.c |
| 2 Tue | Phase 2 | DDC testing + validation against float reference |
| 2 Wed | Phase 3 | GSM double-trig fix + int16 FFT reuse (GSM + WSPR) |
| 2 Thu | Phase 3 | ADS-B magnitude kernel |
| 2 Fri | Phase 3 | AIS FM discriminator + APRS dual-Goertzel |

---

## 6. Testing Strategy

### Bit-Exact Validation
Each PIE kernel gets a C reference implementation. Test harness runs both and
compares output, allowing small rounding differences (int16 Q15 vs float32).

```c
// test_pie_kernels.c
void test_nco_mix_accuracy(void) {
    // Generate known input
    // Run float reference DDC
    // Run PIE int16 DDC
    // Compare: SNR must be > 60 dB (10-bit accuracy)
    // Print max/avg error in LSBs
}
```

### Performance Measurement
Use `DL_LOG_INFER_LATENCY_START/END` macros or `esp_timer_get_time()` around
each kernel call. Compare before/after cycle counts.

### Regression Testing
The existing AFSK1200 loopback test (100% decode at 88-1090 MHz, per last commit)
serves as end-to-end regression. Run after each phase to verify no decode degradation.

---

## 7. Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| Q15 precision loss in DDC | Validate SNR > 60 dB. Use int32 accumulator for CIC. |
| Alignment faults | Always check alignment in kernel preamble, fall back to C. |
| PIE context save overhead | FreeRTOS lazy save — only pays cost if task switch during PIE use. Pin DSP task to one core. |
| NCO table memory | Table size = sample_rate/gcd(sample_rate, offset). Worst case ~32 KB. Use PSRAM. |
| Assembly maintenance burden | Keep C reference for every kernel. Document PIE instructions inline. |

---

## 8. References

- PIE ISA catalog: `esp-idf/components/esp_gdbstub/test_gdbstub_host/rv_decode/xesppie.S` (370 instructions)
- ESP-DSP PIE kernels: `managed_components/espressif__esp-dsp/modules/` (21 .S files)
- Toolchain flags: `-march=rv32imafc_zicsr_zifencei_xesppie -mabi=ilp32f`
- PIE register save: `esp-idf/components/riscv/include/riscv/rvruntime-frames.h:192-208`
- PIE state CSR: `0x7F2` in `esp-idf/components/riscv/include/riscv/csr_pie.h`
- ESP-DL reference: `/home/rax/exp/esp32/p4/wt-examples-ml/ESP_DL_REFERENCE.md`
