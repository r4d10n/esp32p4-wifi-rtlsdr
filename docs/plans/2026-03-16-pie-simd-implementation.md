# PIE SIMD Pipeline Optimization — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the all-float DSP pipeline with integer PIE SIMD kernels, achieving 3-10x throughput improvement on FFT, DDC, and power spectrum paths.

**Architecture:** New `pie_kernels.c` module in `components/dsp/` provides optimized C implementations using PIE-friendly patterns (batch int16 processing, 16-byte aligned buffers). Custom PIE assembly kernels (`.S` files) replace the hottest inner loops. The existing `dsp.c` is modified to call these new functions instead of scalar float esp-dsp calls. A C reference implementation exists for every optimized function to enable bit-exact testing and non-P4 fallback.

**Tech Stack:** ESP-IDF 5.3+, esp-dsp (for FFT butterfly only), custom PIE assembly (`xesppie` ISA), C99

**Key Constraint:** esp-dsp has **no P4 PIE assembly** for basic s16 math (`dsps_add_s16`, `dsps_mul_s16` etc. all fall back to ANSI C on P4). Only dotprod, matmul, FFT, FIR, and biquad have `_arp4` implementations. We must write our own kernels for everything else.

---

## Task 1: Create PIE Kernel Infrastructure

**Files:**
- Create: `components/dsp/pie_kernels.h`
- Create: `components/dsp/pie_kernels.c`
- Modify: `components/dsp/CMakeLists.txt:1-5`

**Step 1: Create the kernel header with all function declarations**

Create `components/dsp/pie_kernels.h`:

```c
/*
 * PIE SIMD Kernel Functions for ESP32-P4
 *
 * Optimized signal processing kernels using PIE 128-bit SIMD.
 * Each function has a C reference implementation and (future) assembly fast path.
 * All buffers must be 16-byte aligned. Lengths must be multiples of 8 (int16).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── FFT Pipeline Kernels ── */

/**
 * Convert uint8 IQ pairs to windowed int16 in a single pass.
 *
 * For each IQ pair: output = ((uint8 - 128) << 8) * window_q15 >> 15
 * Produces interleaved int16 I,Q suitable for dsps_fft2r_sc16.
 *
 * @param src       uint8 interleaved IQ data (I0,Q0,I1,Q1,...) [16-byte aligned]
 * @param window    Q15 Hann window coefficients [16-byte aligned, fft_n entries]
 * @param dst       int16 interleaved IQ output [16-byte aligned, fft_n*2 entries]
 * @param iq_pairs  Number of IQ pairs to process (must be multiple of 8)
 */
void pie_u8iq_to_s16_windowed(const uint8_t *src, const int16_t *window,
                               int16_t *dst, int iq_pairs);

/**
 * Accumulate power spectrum from int16 FFT output into int32 buffer.
 *
 * For each bin k: accum[k] += re[k]*re[k] + im[k]*im[k]
 * Input is interleaved [re0,im0,re1,im1,...] from dsps_fft2r_sc16.
 *
 * @param fft_out   int16 interleaved FFT output [16-byte aligned]
 * @param accum     int32 power accumulator [16-byte aligned, fft_n entries]
 * @param fft_n     FFT size (number of bins, must be multiple of 8)
 */
void pie_power_spectrum_accumulate(const int16_t *fft_out, int32_t *accum, int fft_n);

/**
 * Convert int32 power spectrum to uint8 dB with FFT shift.
 *
 * Uses integer log2 approximation (CLZ + LUT) instead of float log10f.
 * Output is FFT-shifted: second half first, then first half.
 *
 * @param power     int32 accumulated power [16-byte aligned, fft_n entries]
 * @param db_out    uint8 dB output (0-255) [fft_n entries]
 * @param fft_n     FFT size
 * @param avg_count Number of frames averaged (for normalization)
 * @param db_min    Minimum dB value (maps to 0)
 * @param db_max    Maximum dB value (maps to 255)
 */
void pie_power_to_db_u8(const int32_t *power, uint8_t *db_out, int fft_n,
                         int avg_count, float db_min, float db_max);

/* ── DDC Pipeline Kernels ── */

/**
 * NCO (Numerically Controlled Oscillator) table for int16 Q15 DDC.
 * Stores interleaved [cos, -sin] pairs as Q15 int16.
 */
typedef struct {
    int16_t    *table;       /* Interleaved [cos,-sin,cos,-sin,...] Q15 */
    uint32_t    table_len;   /* Number of complex entries */
    uint32_t    phase_pos;   /* Current position in table */
} pie_nco_t;

/**
 * Create NCO table for given offset frequency.
 *
 * @param sample_rate  Input sample rate in Hz
 * @param offset_hz    Frequency offset from center (signed)
 * @return NCO handle, or NULL on failure. Free with pie_nco_free().
 */
pie_nco_t *pie_nco_create(uint32_t sample_rate, int32_t offset_hz);

/**
 * Free NCO table.
 */
void pie_nco_free(pie_nco_t *nco);

/**
 * Convert uint8 IQ to int16 with bias removal.
 *
 * output[i] = ((int16_t)input[i] - 128) << 8
 * Gives full Q15 range [-32768, +32512].
 *
 * @param src       uint8 IQ data [16-byte aligned]
 * @param dst       int16 output [16-byte aligned]
 * @param count     Number of uint8 samples (must be multiple of 16)
 */
void pie_u8_to_s16_bias(const uint8_t *src, int16_t *dst, int count);

/**
 * Complex multiply int16 IQ stream with NCO table (frequency shift).
 *
 * Performs: out_re = in_re*nco_cos - in_im*(-nco_sin)  (= in_re*cos + in_im*sin)
 *          out_im = in_re*(-nco_sin) + in_im*nco_cos   (= -in_re*sin + in_im*cos)
 *
 * All values are Q15 int16. Multiply is (a*b) >> 15.
 *
 * @param iq_in     int16 interleaved IQ input [16-byte aligned]
 * @param nco       NCO table (position auto-advanced)
 * @param iq_out    int16 interleaved IQ output [16-byte aligned]
 * @param iq_pairs  Number of IQ pairs (must be multiple of 8)
 */
void pie_nco_mix_s16(const int16_t *iq_in, pie_nco_t *nco,
                      int16_t *iq_out, int iq_pairs);

/**
 * CIC decimation on int16 interleaved IQ.
 *
 * Accumulates decim_ratio input pairs, outputs averaged pair.
 * Uses int32 accumulator to avoid overflow.
 *
 * @param iq_in       int16 interleaved IQ input
 * @param in_pairs    Number of input IQ pairs
 * @param iq_out      int16 interleaved IQ output
 * @param out_pairs   On entry: max output pairs. On exit: actual output pairs.
 * @param decim_ratio Decimation ratio (must be power of 2)
 * @param accum       Persistent state: int32[4] = {accum_re, accum_im, count, 0}
 */
void pie_cic_decimate_s16(const int16_t *iq_in, int in_pairs,
                           int16_t *iq_out, int *out_pairs,
                           int decim_ratio, int32_t *accum);

/**
 * Convert int16 IQ to uint8 IQ.
 *
 * output[i] = clamp((input[i] >> 8) + 128, 0, 255)
 *
 * @param src       int16 IQ data [16-byte aligned]
 * @param dst       uint8 output
 * @param count     Number of int16 samples (must be multiple of 16)
 */
void pie_s16_to_u8(const int16_t *src, uint8_t *dst, int count);

#ifdef __cplusplus
}
#endif
```

**Step 2: Create C reference implementations**

Create `components/dsp/pie_kernels.c`:

```c
/*
 * PIE SIMD Kernel Functions — C Reference Implementations
 *
 * These are correct-by-construction C implementations that serve as:
 * 1. Fallback for non-P4 targets
 * 2. Reference for validating future PIE assembly versions
 * 3. Working code before assembly optimization
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "pie_kernels.h"

static const char *TAG = "pie";

/* ── FFT Pipeline ── */

void pie_u8iq_to_s16_windowed(const uint8_t *src, const int16_t *window,
                               int16_t *dst, int iq_pairs)
{
    for (int k = 0; k < iq_pairs; k++) {
        /* Convert uint8 to int16: (val - 128) << 8 gives [-32768, +32512] */
        int16_t si = ((int16_t)src[k * 2]     - 128) << 8;
        int16_t sq = ((int16_t)src[k * 2 + 1] - 128) << 8;

        /* Q15 window multiply: (sample * window) >> 15 */
        int32_t wi = window[k];
        dst[k * 2]     = (int16_t)((si * wi) >> 15);
        dst[k * 2 + 1] = (int16_t)((sq * wi) >> 15);
    }
}

void pie_power_spectrum_accumulate(const int16_t *fft_out, int32_t *accum, int fft_n)
{
    for (int k = 0; k < fft_n; k++) {
        int32_t re = (int32_t)fft_out[k * 2];
        int32_t im = (int32_t)fft_out[k * 2 + 1];
        accum[k] += re * re + im * im;
    }
}

void pie_power_to_db_u8(const int32_t *power, uint8_t *db_out, int fft_n,
                         int avg_count, float db_min, float db_max)
{
    float inv = 1.0f / avg_count;
    float db_range = db_max - db_min;
    if (db_range < 1.0f) db_range = 1.0f;
    float scale = 255.0f / db_range;

    int half = fft_n / 2;

    /* FFT shift: second half -> first half of output, first half -> second half */
    for (int k = 0; k < fft_n; k++) {
        float avg_pwr = (float)power[k] * inv;
        float db;
        if (avg_pwr < 1.0f) {
            db = db_min;
        } else {
            /* Integer-friendly log10 approximation using float for now.
             * TODO: Replace with CLZ-based integer log2 in PIE assembly. */
            db = 10.0f * log10f(avg_pwr);
        }
        if (db < db_min) db = db_min;
        if (db > db_max) db = db_max;

        int out_k = (k < half) ? (k + half) : (k - half);
        db_out[out_k] = (uint8_t)((db - db_min) * scale + 0.5f);
    }
}

/* ── DDC Pipeline ── */

static uint32_t gcd(uint32_t a, uint32_t b)
{
    while (b) { uint32_t t = b; b = a % b; a = t; }
    return a;
}

pie_nco_t *pie_nco_create(uint32_t sample_rate, int32_t offset_hz)
{
    pie_nco_t *nco = calloc(1, sizeof(pie_nco_t));
    if (!nco) return NULL;

    /* Table length = one full period of the NCO waveform.
     * For offset_hz that divides sample_rate evenly, this is exact.
     * Otherwise, use LCM-based length capped at reasonable size. */
    uint32_t abs_offset = (offset_hz >= 0) ? (uint32_t)offset_hz : (uint32_t)(-offset_hz);
    if (abs_offset == 0) abs_offset = 1;

    uint32_t g = gcd(sample_rate, abs_offset);
    uint32_t period = sample_rate / g;

    /* Cap table size to avoid excessive memory. 32K entries = 64KB. */
    if (period > 32768) period = 32768;

    /* Round up to multiple of 8 for SIMD alignment */
    period = (period + 7) & ~7u;

    nco->table = heap_caps_aligned_alloc(16, period * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!nco->table) {
        free(nco);
        return NULL;
    }

    nco->table_len = period;
    nco->phase_pos = 0;

    /* Fill table: interleaved [cos, -sin] as Q15 */
    double phase_inc = 2.0 * M_PI * (double)offset_hz / (double)sample_rate;
    for (uint32_t j = 0; j < period; j++) {
        double phase = phase_inc * j;
        nco->table[j * 2]     = (int16_t)(cos(phase) * 32767.0);
        nco->table[j * 2 + 1] = (int16_t)(-sin(phase) * 32767.0);
    }

    ESP_LOGI(TAG, "NCO created: offset=%ldHz period=%lu entries (%lu bytes)",
             (long)offset_hz, (unsigned long)period,
             (unsigned long)(period * 2 * sizeof(int16_t)));
    return nco;
}

void pie_nco_free(pie_nco_t *nco)
{
    if (!nco) return;
    free(nco->table);
    free(nco);
}

void pie_u8_to_s16_bias(const uint8_t *src, int16_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        dst[i] = ((int16_t)src[i] - 128) << 8;
    }
}

void pie_nco_mix_s16(const int16_t *iq_in, pie_nco_t *nco,
                      int16_t *iq_out, int iq_pairs)
{
    const int16_t *tbl = nco->table;
    uint32_t pos = nco->phase_pos;
    uint32_t len = nco->table_len;

    for (int k = 0; k < iq_pairs; k++) {
        int32_t in_re = iq_in[k * 2];
        int32_t in_im = iq_in[k * 2 + 1];
        int32_t nco_cos  = tbl[pos * 2];       /* cos(phase) Q15 */
        int32_t nco_nsin = tbl[pos * 2 + 1];   /* -sin(phase) Q15 */

        /* Complex multiply (Q15): (in_re + j*in_im) * (cos - j*sin) */
        iq_out[k * 2]     = (int16_t)((in_re * nco_cos  - in_im * nco_nsin) >> 15);
        iq_out[k * 2 + 1] = (int16_t)((in_re * nco_nsin + in_im * nco_cos)  >> 15);

        pos++;
        if (pos >= len) pos = 0;
    }

    nco->phase_pos = pos;
}

void pie_cic_decimate_s16(const int16_t *iq_in, int in_pairs,
                           int16_t *iq_out, int *out_pairs,
                           int decim_ratio, int32_t *accum)
{
    int max_out = *out_pairs;
    int out_pos = 0;
    int32_t acc_re  = accum[0];
    int32_t acc_im  = accum[1];
    int32_t acc_cnt = accum[2];

    /* log2(decim_ratio) for bit shift instead of division */
    int shift = 0;
    { int r = decim_ratio; while (r > 1) { shift++; r >>= 1; } }

    for (int k = 0; k < in_pairs && out_pos < max_out; k++) {
        acc_re += (int32_t)iq_in[k * 2];
        acc_im += (int32_t)iq_in[k * 2 + 1];
        acc_cnt++;

        if (acc_cnt >= decim_ratio) {
            iq_out[out_pos * 2]     = (int16_t)(acc_re >> shift);
            iq_out[out_pos * 2 + 1] = (int16_t)(acc_im >> shift);
            out_pos++;
            acc_re = 0;
            acc_im = 0;
            acc_cnt = 0;
        }
    }

    accum[0] = acc_re;
    accum[1] = acc_im;
    accum[2] = acc_cnt;
    *out_pairs = out_pos;
}

void pie_s16_to_u8(const int16_t *src, uint8_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        int val = (src[i] >> 8) + 128;
        if (val < 0) val = 0;
        if (val > 255) val = 255;
        dst[i] = (uint8_t)val;
    }
}
```

**Step 3: Update CMakeLists.txt to include new source**

Modify `components/dsp/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "dsp.c" "pie_kernels.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer espressif__esp-dsp
)
```

**Step 4: Verify it compiles**

Run: `cd /home/rax/exp/esp32/p4/host/esp32p4-wifi-rtlsdr-simd && idf.py build`

Expected: Clean compile with no errors. New functions are compiled but not yet called.

**Step 5: Commit**

```bash
git add components/dsp/pie_kernels.h components/dsp/pie_kernels.c components/dsp/CMakeLists.txt
git commit -m "feat(dsp): add PIE SIMD kernel infrastructure with C reference implementations

Adds pie_kernels.h/c with optimized-for-SIMD function signatures:
- FFT: u8->s16 windowed conversion, power spectrum accumulate, dB convert
- DDC: NCO table create, int16 complex mix, CIC decimate, format convert

All functions are C reference implementations for now. Assembly PIE
kernels will be added incrementally to replace the hot paths."
```

---

## Task 2: Replace FFT Windowing + Conversion Path

**Files:**
- Modify: `components/dsp/dsp.c:192-217` (dsp_fft_compute inner loop)

**Step 1: Replace the scalar per-sample windowing loop with pie_u8iq_to_s16_windowed**

In `dsp.c`, add include at top:
```c
#include "pie_kernels.h"
```

Replace the inner accumulation loop in `dsp_fft_compute` (lines 202-217) with batch processing:

```c
    /* Accumulate IQ samples into int16 interleaved buffer using SIMD-friendly batch conversion */
    uint32_t i = 0;
    while (i + 1 < len) {
        /* How many IQ pairs can we accept before FFT buffer is full? */
        uint32_t pairs_available = (len - i) / 2;
        uint32_t pairs_needed = fft_n - fft_input_pos;
        uint32_t batch = pairs_available < pairs_needed ? pairs_available : pairs_needed;

        /* Round down to multiple of 8 for SIMD, process remainder one-by-one */
        uint32_t batch_simd = batch & ~7u;

        if (batch_simd > 0) {
            pie_u8iq_to_s16_windowed(
                iq_data + i,
                fft_sc16_window + fft_input_pos,
                fft_sc16_input + fft_input_pos * 2,
                batch_simd);
            fft_input_pos += batch_simd;
            i += batch_simd * 2;
        }

        /* Handle remaining 0-7 pairs with scalar fallback */
        uint32_t remainder = batch - batch_simd;
        for (uint32_t r = 0; r < remainder; r++) {
            int16_t si = ((int16_t)iq_data[i] - 128) << 8;
            int16_t sq = ((int16_t)iq_data[i + 1] - 128) << 8;
            int32_t wi = fft_sc16_window[fft_input_pos];
            fft_sc16_input[fft_input_pos * 2]     = (int16_t)((si * wi) >> 15);
            fft_sc16_input[fft_input_pos * 2 + 1] = (int16_t)((sq * wi) >> 15);
            fft_input_pos++;
            i += 2;
        }

        if (fft_input_pos >= fft_n) {
            /* ... (rest of FFT processing unchanged) ... */
```

**Step 2: Verify it compiles and FFT output is unchanged**

Run: `idf.py build`

Expected: Clean compile. Functionally identical — the C reference does the same math.

**Step 3: Commit**

```bash
git add components/dsp/dsp.c
git commit -m "feat(dsp): use pie_u8iq_to_s16_windowed for FFT input conversion

Replaces per-sample scalar windowing loop with batch SIMD-friendly
function. Currently uses C reference; PIE assembly will replace later.
Batch processes in multiples of 8 with scalar tail handling."
```

---

## Task 3: Replace FFT Power Spectrum Accumulation

**Files:**
- Modify: `components/dsp/dsp.c:238-246` (power spectrum loop)
- Modify: `components/dsp/dsp.c:89-90,129` (change fft_power from float* to int32_t*)

**Step 1: Change fft_power buffer type from float to int32_t**

In `dsp.c`, change:
```c
/* Line ~90: change type */
static int32_t *fft_power = NULL;   /* [fft_n] — accumulated power spectrum (int32) */

/* Line ~129: change allocation */
fft_power = heap_caps_aligned_alloc(16, n * sizeof(int32_t), MALLOC_CAP_DEFAULT);

/* Line ~172-174: change memset in dsp_fft_reset */
if (fft_power && fft_n > 0) {
    memset(fft_power, 0, fft_n * sizeof(int32_t));
}

/* Line ~300: change memset at end of output */
memset(fft_power, 0, fft_n * sizeof(int32_t));
```

**Step 2: Replace the power spectrum loop with pie_power_spectrum_accumulate**

Replace lines 238-246:
```c
            /* Power spectrum: accumulate re^2 + im^2 into int32 buffer */
            pie_power_spectrum_accumulate(fft_sc16_work, fft_power, fft_n);
```

**Step 3: Replace the dB conversion loop with pie_power_to_db_u8**

Replace lines 264-279:
```c
                /* Convert int32 power → dB → uint8 with FFT shift */
                pie_power_to_db_u8(fft_power, fft_out, fft_n,
                                   fft_avg_count, s_db_min, s_db_max);
                *fft_out_len = fft_n;
```

**Step 4: Verify it compiles**

Run: `idf.py build`

Expected: Clean compile. dB values may differ slightly due to int32 vs float accumulation — acceptable.

**Step 5: Commit**

```bash
git add components/dsp/dsp.c
git commit -m "feat(dsp): int32 power spectrum accumulation replaces float

Changes fft_power from float to int32_t, uses pie_power_spectrum_accumulate
and pie_power_to_db_u8. Eliminates float conversion in the power spectrum
hot path. The int16->int32 multiply-accumulate is SIMD-friendly and will
be the first target for PIE assembly optimization."
```

---

## Task 4: Replace DDC with Integer Pipeline

**Files:**
- Modify: `components/dsp/dsp.c:307-494` (entire DDC section)
- Modify: `components/dsp/include/dsp.h:55` (dsp_ddc opaque type stays same)

**Step 1: Rewrite dsp_ddc struct to use int16 buffers + pie_nco_t**

Replace the DDC struct and alloc function (dsp.c lines 311-340):

```c
#define DDC_BATCH_SIZE  1024    /* Must be multiple of 8 */

struct dsp_ddc {
    uint32_t    sample_rate;
    int32_t     center_offset;
    uint32_t    output_bw;
    uint32_t    decim_ratio;

    /* NCO for int16 Q15 mixing */
    pie_nco_t  *nco;

    /* Int16 scratch buffers (16-byte aligned) */
    int16_t    *buf_s16;       /* uint8 -> int16 conversion [DDC_BATCH_SIZE * 2] */
    int16_t    *mix_s16;       /* NCO mixed output [DDC_BATCH_SIZE * 2] */

    /* CIC decimator persistent state */
    int32_t     cic_accum[4];  /* {accum_re, accum_im, count, 0} */
};
```

**Step 2: Rewrite dsp_ddc_create to use pie_nco_create**

```c
dsp_ddc_t *dsp_ddc_create(uint32_t sample_rate, uint32_t center_offset_hz,
                           uint32_t output_bw_hz)
{
    dsp_ddc_t *ddc = calloc(1, sizeof(dsp_ddc_t));
    if (!ddc) return NULL;

    ddc->sample_rate = sample_rate;
    ddc->center_offset = (int32_t)center_offset_hz;
    ddc->output_bw = output_bw_hz;

    /* Decimation ratio (nearest power of 2) */
    uint32_t ratio = sample_rate / output_bw_hz;
    if (ratio < 1) ratio = 1;
    uint32_t p2 = 1;
    while (p2 * 2 <= ratio) p2 *= 2;
    ddc->decim_ratio = p2;

    /* Create int16 Q15 NCO table */
    ddc->nco = pie_nco_create(sample_rate, (int32_t)center_offset_hz);
    if (!ddc->nco) {
        free(ddc);
        return NULL;
    }

    /* Allocate int16 scratch buffers (interleaved IQ, so *2) */
    ddc->buf_s16 = heap_caps_aligned_alloc(16, DDC_BATCH_SIZE * 2 * sizeof(int16_t),
                                            MALLOC_CAP_DEFAULT);
    ddc->mix_s16 = heap_caps_aligned_alloc(16, DDC_BATCH_SIZE * 2 * sizeof(int16_t),
                                            MALLOC_CAP_DEFAULT);

    if (!ddc->buf_s16 || !ddc->mix_s16) {
        ESP_LOGE(TAG, "DDC buffer allocation failed");
        dsp_ddc_free(ddc);
        return NULL;
    }

    memset(ddc->cic_accum, 0, sizeof(ddc->cic_accum));

    ESP_LOGI(TAG, "DDC created: offset=%ldHz bw=%luHz decim=%lu (int16 pipeline)",
             (long)ddc->center_offset, (unsigned long)output_bw_hz,
             (unsigned long)ddc->decim_ratio);
    return ddc;
}
```

**Step 3: Rewrite dsp_ddc_free**

```c
void dsp_ddc_free(dsp_ddc_t *ddc)
{
    if (!ddc) return;
    pie_nco_free(ddc->nco);
    free(ddc->buf_s16);
    free(ddc->mix_s16);
    free(ddc);
}
```

**Step 4: Rewrite dsp_ddc_process to use int16 pipeline**

```c
int dsp_ddc_process(dsp_ddc_t *ddc, const uint8_t *iq_in, uint32_t in_len,
                    uint8_t *iq_out, uint32_t *out_len)
{
    uint32_t max_out = *out_len;
    uint32_t out_pos = 0;
    uint32_t i = 0;

    /* Temporary int16 output from CIC before converting to uint8 */
    int16_t *cic_out = ddc->buf_s16;  /* Reuse buf_s16 since we're done with input by then */

    while (i + 1 < in_len && out_pos + 1 < max_out) {
        /* Determine batch size */
        uint32_t remaining_pairs = (in_len - i) / 2;
        uint32_t batch = remaining_pairs;
        if (batch > DDC_BATCH_SIZE) batch = DDC_BATCH_SIZE;

        /* Round down to multiple of 8 for SIMD */
        batch = batch & ~7u;
        if (batch == 0) {
            /* Handle remaining < 8 pairs with scalar fallback */
            break;
        }

        /* Step 1: uint8 IQ -> int16 with bias removal */
        pie_u8_to_s16_bias(iq_in + i, ddc->buf_s16, batch * 2);

        /* Step 2: Complex multiply with NCO (frequency shift) */
        pie_nco_mix_s16(ddc->buf_s16, ddc->nco, ddc->mix_s16, batch);

        /* Step 3: CIC decimation */
        int cic_out_pairs = (int)((max_out - out_pos) / 2);
        pie_cic_decimate_s16(ddc->mix_s16, batch,
                              cic_out, &cic_out_pairs,
                              ddc->decim_ratio, ddc->cic_accum);

        /* Step 4: int16 -> uint8 output conversion */
        if (cic_out_pairs > 0) {
            pie_s16_to_u8(cic_out, iq_out + out_pos, cic_out_pairs * 2);
            out_pos += cic_out_pairs * 2;
        }

        i += batch * 2;
    }

    *out_len = out_pos;
    return 0;
}
```

**Step 5: Remove old float buffer declarations**

Remove the old float DDC buffers and `ddc_alloc()` helper function (lines 97-99, 337-340). Remove includes for `dsps_mul.h`, `dsps_add.h`, `dsps_mulc.h`, `dsps_addc.h` since they are no longer used by the DDC.

Keep the float tmp buffers only if still needed by FFT power path (check — if Task 3 eliminated them, remove too).

**Step 6: Verify compile**

Run: `idf.py build`

Expected: Clean compile. DDC now runs entirely in int16 domain.

**Step 7: Commit**

```bash
git add components/dsp/dsp.c
git commit -m "feat(dsp): replace float DDC pipeline with int16 Q15

Complete rewrite of DDC to use integer pipeline:
- pie_nco_t replaces float cos/sin table (Q15 int16, exact period)
- pie_u8_to_s16_bias replaces float deinterleave + addc + mulc
- pie_nco_mix_s16 replaces 7x dsps_*_f32 calls for complex multiply
- pie_cic_decimate_s16 replaces float accumulate-and-dump
- pie_s16_to_u8 replaces float->uint8 clamp

Memory reduction: 8 x 4KB float buffers -> 2 x 2KB int16 buffers.
All functions are SIMD-ready (batch of 8) for PIE assembly."
```

---

## Task 5: Add PIE Assembly for Power Spectrum (First Assembly Kernel)

**Files:**
- Create: `components/dsp/pie_power_spectrum_arp4.S`
- Modify: `components/dsp/pie_kernels.c` (add conditional call)
- Modify: `components/dsp/CMakeLists.txt` (add .S source)

**Step 1: Write the PIE assembly kernel**

Create `components/dsp/pie_power_spectrum_arp4.S`:

```asm
/*
 * PIE SIMD: Accumulate power spectrum from int16 FFT output to int32.
 *
 * void pie_power_spectrum_accumulate_arp4(const int16_t *fft_out,
 *                                         int32_t *accum, int fft_n);
 *
 * For each bin k: accum[k] += re[k]^2 + im[k]^2
 * Input: interleaved [re0,im0,re1,im1,...] int16
 * Output: int32 power per bin
 *
 * Processes 4 bins per iteration (loads 8 int16 = 4 complex pairs).
 * Uses esp.vmul.s32.s16xs16 for widening multiply.
 *
 * Registers:
 *   a0 = fft_out (const int16_t *)
 *   a1 = accum (int32_t *)
 *   a2 = fft_n (bins, must be multiple of 4)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED

    .text
    .align 4
    .global pie_power_spectrum_accumulate_arp4
    .type   pie_power_spectrum_accumulate_arp4, @function

pie_power_spectrum_accumulate_arp4:
    /* a0 = fft_out, a1 = accum, a2 = fft_n */
    /* fft_n must be >= 4 and multiple of 4 */
    srli    a2, a2, 2               /* loop count = fft_n / 4 */
    beqz    a2, .L_pwr_done

    esp.lp.setup  0, a2, .L_pwr_loop_end
        /* Load 4 complex pairs = 8 int16 values (128 bits) */
        esp.vld.128.ip  q0, a0, 16         /* q0 = [re0,im0,re1,im1,re2,im2,re3,im3] */

        /* Deinterleave: q1 = [re0,re1,re2,re3], q2 = [im0,im1,im2,im3] */
        esp.vunzip.16   q0, q1             /* q0=even(re), q1=odd(im) */

        /* Widening multiply: re*re -> int32 (4 elements) */
        esp.vmul.s32.s16xs16  q2, q0, q0, q0  /* q2 = re^2 as int32 */

        /* Widening multiply: im*im -> int32 (4 elements) */
        esp.vmul.s32.s16xs16  q3, q1, q1, q1  /* q3 = im^2 as int32 */

        /* Add: power = re^2 + im^2 */
        esp.vadd.s32    q2, q2, q3         /* q2 = power[0..3] */

        /* Load existing accumulator */
        esp.vld.128.ip  q4, a1, 0          /* q4 = accum[k..k+3] (don't advance yet) */

        /* Accumulate */
        esp.vadd.s32    q4, q4, q2         /* q4 += power */

        /* Store back */
.L_pwr_loop_end:
        esp.vst.128.ip  q4, a1, 16        /* store and advance */

.L_pwr_done:
    ret

    .size   pie_power_spectrum_accumulate_arp4, . - pie_power_spectrum_accumulate_arp4

#endif /* CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED */
```

**Step 2: Add dispatch in pie_kernels.c**

At top of `pie_kernels.c`, add:

```c
#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED
extern void pie_power_spectrum_accumulate_arp4(const int16_t *fft_out,
                                                int32_t *accum, int fft_n);
#define PIE_ASM_POWER_SPECTRUM 1
#else
#define PIE_ASM_POWER_SPECTRUM 0
#endif
```

Then modify `pie_power_spectrum_accumulate` to dispatch:

```c
void pie_power_spectrum_accumulate(const int16_t *fft_out, int32_t *accum, int fft_n)
{
#if PIE_ASM_POWER_SPECTRUM
    /* Use PIE assembly for aligned, multiple-of-4 sizes */
    if ((fft_n & 3) == 0 && fft_n >= 4) {
        pie_power_spectrum_accumulate_arp4(fft_out, accum, fft_n);
        return;
    }
#endif
    /* C fallback */
    for (int k = 0; k < fft_n; k++) {
        int32_t re = (int32_t)fft_out[k * 2];
        int32_t im = (int32_t)fft_out[k * 2 + 1];
        accum[k] += re * re + im * im;
    }
}
```

**Step 3: Update CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "dsp.c" "pie_kernels.c" "pie_power_spectrum_arp4.S"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer espressif__esp-dsp
)
```

**Step 4: Verify compile and test**

Run: `idf.py build`

Expected: Clean compile. Assembly included on P4 target, C fallback on others.

**Step 5: Commit**

```bash
git add components/dsp/pie_power_spectrum_arp4.S components/dsp/pie_kernels.c components/dsp/CMakeLists.txt
git commit -m "feat(dsp): first PIE assembly kernel — power spectrum accumulate

Uses esp.vunzip.16 for IQ deinterleave, esp.vmul.s32.s16xs16 for
widening multiply, esp.vadd.s32 for accumulation. Processes 4 FFT
bins per iteration. Falls back to C for non-P4 targets or when
fft_n is not a multiple of 4."
```

---

## Task 6: Add PIE Assembly for NCO Complex Multiply

**Files:**
- Create: `components/dsp/pie_nco_mix_arp4.S`
- Modify: `components/dsp/pie_kernels.c` (add dispatch)
- Modify: `components/dsp/CMakeLists.txt`

**Step 1: Write the PIE assembly kernel for NCO mixing**

Create `components/dsp/pie_nco_mix_arp4.S`:

```asm
/*
 * PIE SIMD: Complex multiply int16 IQ with NCO table.
 *
 * void pie_nco_mix_s16_arp4(const int16_t *iq_in, const int16_t *nco_table,
 *                            int16_t *iq_out, int iq_pairs);
 *
 * Uses esp.cmul.s16 for native complex multiply.
 * Input/output: interleaved [re,im,re,im,...] int16 Q15.
 * NCO table: interleaved [cos,-sin,cos,-sin,...] int16 Q15.
 *
 * Processes 4 complex pairs per iteration.
 *
 * a0 = iq_in, a1 = nco_table, a2 = iq_out, a3 = iq_pairs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED

    .text
    .align 4
    .global pie_nco_mix_s16_arp4
    .type   pie_nco_mix_s16_arp4, @function

pie_nco_mix_s16_arp4:
    /* a0=iq_in, a1=nco_table, a2=iq_out, a3=iq_pairs */
    srli    a3, a3, 2               /* loop count = iq_pairs / 4 */
    beqz    a3, .L_mix_done

    esp.lp.setup  0, a3, .L_mix_loop_end
        esp.vld.128.ip  q0, a0, 16         /* load 4 IQ pairs from input */
        esp.vld.128.ip  q1, a1, 16         /* load 4 NCO [cos,-sin] pairs */

        /* Complex multiply: mode 0 = standard complex mul
         * out_re = in_re*nco_re - in_im*nco_im
         * out_im = in_re*nco_im + in_im*nco_re
         * Result is Q15 (automatically scaled by the instruction) */
        esp.cmul.s16    q2, q0, q1, 0

.L_mix_loop_end:
        esp.vst.128.ip  q2, a2, 16         /* store 4 mixed IQ pairs */

.L_mix_done:
    ret

    .size   pie_nco_mix_s16_arp4, . - pie_nco_mix_s16_arp4

#endif /* CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED */
```

**Step 2: Add dispatch in pie_kernels.c**

Add extern and define:
```c
#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED
extern void pie_nco_mix_s16_arp4(const int16_t *iq_in, const int16_t *nco_table,
                                  int16_t *iq_out, int iq_pairs);
#define PIE_ASM_NCO_MIX 1
#else
#define PIE_ASM_NCO_MIX 0
#endif
```

Modify `pie_nco_mix_s16`:
```c
void pie_nco_mix_s16(const int16_t *iq_in, pie_nco_t *nco,
                      int16_t *iq_out, int iq_pairs)
{
    uint32_t pos = nco->phase_pos;
    uint32_t len = nco->table_len;

    /* Process in contiguous chunks within the NCO table */
    int remaining = iq_pairs;
    const int16_t *in_ptr = iq_in;
    int16_t *out_ptr = iq_out;

    while (remaining > 0) {
        uint32_t chunk = len - pos;
        if (chunk > (uint32_t)remaining) chunk = remaining;
        uint32_t chunk_simd = chunk & ~3u;  /* multiple of 4 for PIE */

#if PIE_ASM_NCO_MIX
        if (chunk_simd >= 4) {
            pie_nco_mix_s16_arp4(in_ptr, nco->table + pos * 2, out_ptr, chunk_simd);
            in_ptr  += chunk_simd * 2;
            out_ptr += chunk_simd * 2;
            pos     += chunk_simd;
            remaining -= chunk_simd;
        }
#endif
        /* Scalar tail or full fallback */
        uint32_t tail = chunk - chunk_simd;
        const int16_t *tbl = nco->table;
        for (uint32_t k = 0; k < tail; k++) {
            int32_t in_re = in_ptr[k * 2];
            int32_t in_im = in_ptr[k * 2 + 1];
            int32_t nco_cos  = tbl[pos * 2];
            int32_t nco_nsin = tbl[pos * 2 + 1];
            out_ptr[k * 2]     = (int16_t)((in_re * nco_cos  - in_im * nco_nsin) >> 15);
            out_ptr[k * 2 + 1] = (int16_t)((in_re * nco_nsin + in_im * nco_cos)  >> 15);
            pos++;
            if (pos >= len) pos = 0;
        }
        in_ptr  += tail * 2;
        out_ptr += tail * 2;
        remaining -= tail;
        if (pos >= len) pos = 0;
    }

    nco->phase_pos = pos;
}
```

**Step 3: Update CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "dsp.c" "pie_kernels.c" "pie_power_spectrum_arp4.S" "pie_nco_mix_arp4.S"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer espressif__esp-dsp
)
```

**Step 4: Verify compile**

Run: `idf.py build`

**Step 5: Commit**

```bash
git add components/dsp/pie_nco_mix_arp4.S components/dsp/pie_kernels.c components/dsp/CMakeLists.txt
git commit -m "feat(dsp): PIE assembly NCO complex multiply using esp.cmul.s16

Native complex multiply instruction processes 4 IQ pairs per iteration.
Replaces 7x scalar float esp-dsp calls in the DDC mixing stage.
Handles NCO table wrap-around with chunked processing."
```

---

## Task 7: Enable Benchmarking and Validate

**Files:**
- Modify: `components/dsp/dsp.c:23` (enable DSP_PERF_BENCH)

**Step 1: Enable benchmarking**

Change line 23:
```c
#define DSP_PERF_BENCH 1  /* Set to 1 for benchmarking */
```

**Step 2: Flash and collect measurements**

Run: `idf.py flash monitor`

Expected output (every 200 FFT frames):
```
DSP BENCH (200 frames, 4096-pt): FFT=XXXus/frame Power=XXXus/frame dB+shift=XXXus/frame Total=XXXus/frame
```

Compare against baseline (from before changes). Expected improvements:
- Power spectrum: ~3-4x faster (int32 SIMD vs float scalar)
- Windowing: ~2-3x faster (batch vs per-sample)
- DDC: ~5-10x faster (int16 pipeline vs 7x float calls)

**Step 3: Disable benchmarking for production**

```c
#define DSP_PERF_BENCH 0
```

**Step 4: Commit results**

```bash
git add components/dsp/dsp.c
git commit -m "perf(dsp): benchmark results after PIE SIMD optimization

Measured performance improvements:
- Power spectrum: Xus -> Yus (Z.Zx faster)
- FFT windowing: Xus -> Yus (Z.Zx faster)
- DDC total: Xus -> Yus (Z.Zx faster)

DSP_PERF_BENCH disabled for production."
```

---

## Task 8: Clean Up Unused Float Buffers

**Files:**
- Modify: `components/dsp/dsp.c` (remove fft_tmp_a, fft_tmp_b, old includes)

**Step 1: Remove float scratch buffers**

Check if `fft_tmp_a` and `fft_tmp_b` are still referenced. If DDC no longer uses them (Task 4 replaced float DDC), remove:

```c
/* Remove these declarations (~lines 97-99) */
// static float *fft_tmp_a = NULL;
// static float *fft_tmp_b = NULL;

/* Remove from fft_free_buffers (~lines 111-112) */
// if (fft_tmp_a) { free(fft_tmp_a); fft_tmp_a = NULL; }
// if (fft_tmp_b) { free(fft_tmp_b); fft_tmp_b = NULL; }

/* Remove from allocation (~lines 133-134) */
// fft_tmp_a = heap_caps_aligned_alloc(16, n * sizeof(float), MALLOC_CAP_DEFAULT);
// fft_tmp_b = heap_caps_aligned_alloc(16, n * sizeof(float), MALLOC_CAP_DEFAULT);

/* Remove from null checks (~line 137) */
// !fft_tmp_a || !fft_tmp_b
```

Remove unused float includes if no longer needed:
```c
// #include "dsps_mul.h"
// #include "dsps_add.h"
// #include "dsps_mulc.h"
// #include "dsps_addc.h"
```

**Step 2: Verify compile**

Run: `idf.py build`

**Step 3: Commit**

```bash
git add components/dsp/dsp.c
git commit -m "refactor(dsp): remove unused float DDC buffers and includes

DDC pipeline is now fully int16. The 8 x 4KB float scratch buffers
(fft_tmp_a, fft_tmp_b, buf_re, buf_im, tmp1, tmp2, mix_re, mix_im)
are replaced by 2 x 2KB int16 buffers. Saves 28KB heap per DDC."
```

---

## Summary of All Files Changed

| File | Action | Task |
|------|--------|------|
| `components/dsp/pie_kernels.h` | Create | 1 |
| `components/dsp/pie_kernels.c` | Create, modify | 1, 5, 6 |
| `components/dsp/pie_power_spectrum_arp4.S` | Create | 5 |
| `components/dsp/pie_nco_mix_arp4.S` | Create | 6 |
| `components/dsp/CMakeLists.txt` | Modify | 1, 5, 6 |
| `components/dsp/dsp.c` | Modify | 2, 3, 4, 7, 8 |

## Expected Outcomes

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| FFT windowing | ~12 cyc/sample (scalar) | ~3 cyc/sample (batch C, future ASM) | 4x |
| Power spectrum | ~10 cyc/sample (float) | ~2.5 cyc/sample (PIE s32 ASM) | 4x |
| DDC mixing | ~35 cyc/sample (7x float) | ~2 cyc/sample (PIE cmul.s16 ASM) | 17x |
| DDC total | ~48 cyc/sample | ~5 cyc/sample | 10x |
| DDC memory | 32 KB (8 float bufs) | 4 KB (2 int16 bufs) | 8x reduction |
| FFT power buffer | 4 KB (float) | 4 KB (int32) | same size, no float conv |
