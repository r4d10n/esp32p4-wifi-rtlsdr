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

#include "sdkconfig.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <alloca.h>

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED
/* Power spectrum: scalar loop with hardware zero-overhead loop, int32 temp accum */
extern void pie_power_spectrum_accumulate_arp4(const int16_t *fft_out,
                                                int32_t *accum, int fft_n);
#define PIE_ASM_POWER_SPECTRUM 1

/* Windowing: scalar with hardware zero-overhead loop */
extern void pie_u8iq_to_s16_windowed_arp4(const uint8_t *src,
                                           const int16_t *window,
                                           int16_t *dst, int iq_pairs);
#define PIE_ASM_WINDOWING 1

/* NCO mix: PIE esp.cmul.s16 SIMD complex multiply */
extern void pie_nco_mix_s16_arp4(const int16_t *iq_in, const int16_t *nco_table,
                                  int16_t *iq_out, int iq_pairs);
#define PIE_ASM_NCO_MIX 1
#else
#define PIE_ASM_POWER_SPECTRUM 0
#define PIE_ASM_WINDOWING 0
#define PIE_ASM_NCO_MIX 0
#endif

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
#if PIE_ASM_WINDOWING
    if (iq_pairs > 0) {
        pie_u8iq_to_s16_windowed_arp4(src, window, dst, iq_pairs);
        return;
    }
#endif
    for (int k = 0; k < iq_pairs; k++) {
        int16_t si = ((int16_t)src[k * 2]     - 128) << 8;
        int16_t sq = ((int16_t)src[k * 2 + 1] - 128) << 8;
        int32_t wi = window[k];
        dst[k * 2]     = (int16_t)((si * wi) >> 15);
        dst[k * 2 + 1] = (int16_t)((sq * wi) >> 15);
    }
}

void pie_power_spectrum_accumulate(const int16_t *fft_out, int64_t *accum, int fft_n)
{
#if PIE_ASM_POWER_SPECTRUM
    /* Assembly uses int32 accumulator for hardware loop efficiency.
     * We use a stack-allocated int32 temp buffer, accumulate there,
     * then widen to int64. Safe because one FFT frame's power
     * (max 32767^2 + 32767^2 = ~2.1G) fits in int32. */
    if (fft_n <= 8192) {
        int32_t *tmp = (int32_t *)alloca(fft_n * sizeof(int32_t));
        memset(tmp, 0, fft_n * sizeof(int32_t));
        pie_power_spectrum_accumulate_arp4(fft_out, tmp, fft_n);
        for (int k = 0; k < fft_n; k++) {
            accum[k] += (int64_t)tmp[k];
        }
        return;
    }
#endif
    /* C fallback — direct int64 accumulation */
    for (int k = 0; k < fft_n; k++) {
        int32_t re = (int32_t)fft_out[k * 2];
        int32_t im = (int32_t)fft_out[k * 2 + 1];
        accum[k] += (int64_t)(re * re + im * im);
    }
}

void pie_power_to_db_u8(const int64_t *power, uint8_t *db_out, int fft_n,
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
    heap_caps_free(nco->table);
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
            in_ptr    += chunk_simd * 2;
            out_ptr   += chunk_simd * 2;
            pos       += chunk_simd;
            remaining -= chunk_simd;
        }
#endif
        /* Scalar tail or full fallback */
        uint32_t tail = chunk - chunk_simd;
        const int16_t *tbl = nco->table;
        for (uint32_t k = 0; k < tail; k++) {
            int32_t in_re    = in_ptr[k * 2];
            int32_t in_im    = in_ptr[k * 2 + 1];
            int32_t nco_cos  = tbl[pos * 2];
            int32_t nco_nsin = tbl[pos * 2 + 1];
            out_ptr[k * 2]     = (int16_t)((in_re * nco_cos  - in_im * nco_nsin) >> 15);
            out_ptr[k * 2 + 1] = (int16_t)((in_re * nco_nsin + in_im * nco_cos)  >> 15);
            pos++;
            if (pos >= len) pos = 0;
        }
        in_ptr    += tail * 2;
        out_ptr   += tail * 2;
        remaining -= tail;
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
