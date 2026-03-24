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

#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "pie_kernels.h"

static const char *TAG = "pie";

/* ── Heap-allocated scratch buffer for power spectrum (replaces alloca) ── */
static int32_t *s_pwr_scratch = NULL;
static int      s_pwr_scratch_n = 0;

void pie_pwr_scratch_init(int fft_n)
{
    if (s_pwr_scratch && s_pwr_scratch_n >= fft_n) return;
    heap_caps_free(s_pwr_scratch);
    s_pwr_scratch = heap_caps_aligned_alloc(16, fft_n * sizeof(int32_t), MALLOC_CAP_DEFAULT);
    s_pwr_scratch_n = s_pwr_scratch ? fft_n : 0;
    if (!s_pwr_scratch) {
        ESP_LOGE(TAG, "Failed to allocate power scratch buffer (%d bins)", fft_n);
    }
}

void pie_pwr_scratch_free(void)
{
    heap_caps_free(s_pwr_scratch);
    s_pwr_scratch = NULL;
    s_pwr_scratch_n = 0;
}

/* ── Fast log10 approximation (IEEE 754 bit manipulation) ── */

static inline float fast_log10f(float x)
{
    /* Based on the integer representation of IEEE 754 float:
     * log2(x) ≈ (*(int32_t*)&x) / (1<<23) - 127
     * log10(x) = log2(x) / log2(10) = log2(x) * 0.30103 */
    union { float f; int32_t i; } u = { .f = x };
    float log2_approx = (float)(u.i - 1064866805) * (1.0f / 8388608.0f);
    return log2_approx * 0.301029995663981f;
}

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
     * We use a heap-allocated int32 temp buffer, accumulate there,
     * then widen to int64. Safe because one FFT frame's power
     * (max 32767^2 + 32767^2 = ~2.1G) fits in int32. */
    if (s_pwr_scratch && fft_n <= s_pwr_scratch_n) {
        memset(s_pwr_scratch, 0, fft_n * sizeof(int32_t));
        pie_power_spectrum_accumulate_arp4(fft_out, s_pwr_scratch, fft_n);
        for (int k = 0; k < fft_n; k++) {
            accum[k] += (int64_t)s_pwr_scratch[k];
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
            db = 10.0f * fast_log10f(avg_pwr);
        }
        if (db < db_min) db = db_min;
        if (db > db_max) db = db_max;

        int out_k = (k < half) ? (k + half) : (k - half);
        db_out[out_k] = (uint8_t)((db - db_min) * scale + 0.5f);
    }
}

/* ── DDC Pipeline ── */

/* ── NCO with 32-bit phase accumulator (no table size cap, no phase discontinuity) ── */

#define NCO_TABLE_SIZE   1024    /* Base lookup table entries */
#define NCO_PHASE_BITS   32      /* Phase accumulator width */

pie_nco_t *pie_nco_create(uint32_t sample_rate, int32_t offset_hz)
{
    pie_nco_t *nco = calloc(1, sizeof(pie_nco_t));
    if (!nco) return NULL;

    /* Compute phase increment: phase_inc = (offset_hz / sample_rate) * 2^32
     * Use 64-bit math to avoid overflow */
    int64_t inc64 = ((int64_t)offset_hz * (int64_t)UINT32_MAX) / (int64_t)sample_rate;
    nco->phase_inc = (uint32_t)inc64;
    nco->phase_acc = 0;

    /* Allocate base sin/cos lookup table (one quadrant, interpolated) */
    nco->table = heap_caps_aligned_alloc(16, NCO_TABLE_SIZE * 2 * sizeof(int16_t),
                                          MALLOC_CAP_DEFAULT);
    if (!nco->table) {
        free(nco);
        return NULL;
    }

    nco->table_len = NCO_TABLE_SIZE;

    /* Fill table: one full cycle, interleaved [cos, -sin] as Q15 */
    for (uint32_t j = 0; j < NCO_TABLE_SIZE; j++) {
        double phase = 2.0 * M_PI * (double)j / (double)NCO_TABLE_SIZE;
        nco->table[j * 2]     = (int16_t)(cos(phase) * 32767.0);
        nco->table[j * 2 + 1] = (int16_t)(-sin(phase) * 32767.0);
    }

    ESP_LOGI(TAG, "NCO created: offset=%ldHz phase_inc=0x%08lx table=%lu entries",
             (long)offset_hz, (unsigned long)nco->phase_inc,
             (unsigned long)NCO_TABLE_SIZE);
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

/* NCO mix using phase accumulator with table lookup + linear interpolation */
void pie_nco_mix_s16(const int16_t *iq_in, pie_nco_t *nco,
                      int16_t *iq_out, int iq_pairs)
{
    uint32_t phase = nco->phase_acc;
    uint32_t inc = nco->phase_inc;
    const int16_t *tbl = nco->table;
    uint32_t tbl_mask = nco->table_len - 1; /* table_len must be power of 2 */

    /* Shift to extract table index from top bits of phase accumulator */
    int idx_shift = NCO_PHASE_BITS - 10; /* 32 - log2(NCO_TABLE_SIZE=1024) = 22 */

    for (int k = 0; k < iq_pairs; k++) {
        /* Extract table index from upper bits of phase accumulator */
        uint32_t idx = (phase >> idx_shift) & tbl_mask;

        int32_t nco_cos  = (int32_t)tbl[idx * 2];
        int32_t nco_nsin = (int32_t)tbl[idx * 2 + 1];

        int32_t in_re = (int32_t)iq_in[k * 2];
        int32_t in_im = (int32_t)iq_in[k * 2 + 1];

        /* Complex multiply: out = in * (cos + j*sin) */
        iq_out[k * 2]     = (int16_t)((in_re * nco_cos  - in_im * nco_nsin) >> 15);
        iq_out[k * 2 + 1] = (int16_t)((in_re * nco_nsin + in_im * nco_cos)  >> 15);

        phase += inc;  /* Natural 32-bit wrap = phase continuity */
    }

    nco->phase_acc = phase;
}

/* ── 3rd-order CIC decimator ── */

void pie_cic_decimate_s16(const int16_t *iq_in, int in_pairs,
                           int16_t *iq_out, int *out_pairs,
                           int decim_ratio, int32_t *accum)
{
    /* accum layout (persistent state):
     * [0..5]  = 3 integrator stages re/im: {i1_re, i1_im, i2_re, i2_im, i3_re, i3_im}
     * [6..11] = 3 comb previous values: {c1_re, c1_im, c2_re, c2_im, c3_re, c3_im}
     * [12]    = sample counter
     * [13]    = reserved
     */
    int max_out = *out_pairs;
    int out_pos = 0;

    /* Restore integrator state */
    int64_t i1_re = ((int64_t)accum[1]  << 32) | (uint32_t)accum[0];
    int64_t i1_im = ((int64_t)accum[3]  << 32) | (uint32_t)accum[2];
    int64_t i2_re = ((int64_t)accum[5]  << 32) | (uint32_t)accum[4];
    int64_t i2_im = ((int64_t)accum[7]  << 32) | (uint32_t)accum[6];
    int64_t i3_re = ((int64_t)accum[9]  << 32) | (uint32_t)accum[8];
    int64_t i3_im = ((int64_t)accum[11] << 32) | (uint32_t)accum[10];

    /* Comb previous values (only need last decimated output per stage) */
    int64_t c1_prev_re = ((int64_t)accum[13] << 32) | (uint32_t)accum[12];
    int64_t c1_prev_im = ((int64_t)accum[15] << 32) | (uint32_t)accum[14];
    int64_t c2_prev_re = ((int64_t)accum[17] << 32) | (uint32_t)accum[16];
    int64_t c2_prev_im = ((int64_t)accum[19] << 32) | (uint32_t)accum[18];
    int64_t c3_prev_re = ((int64_t)accum[21] << 32) | (uint32_t)accum[20];
    int64_t c3_prev_im = ((int64_t)accum[23] << 32) | (uint32_t)accum[22];
    int32_t cnt         = accum[24];

    /* Gain normalization: 3rd-order CIC gain = R^3. Use bit shift = 3*log2(R). */
    int shift = 0;
    { int r = decim_ratio; while (r > 1) { shift++; r >>= 1; } }
    shift *= 3;  /* N=3 stages */

    for (int k = 0; k < in_pairs && out_pos < max_out; k++) {
        int64_t x_re = (int64_t)iq_in[k * 2];
        int64_t x_im = (int64_t)iq_in[k * 2 + 1];

        /* 3 integrator stages (run at input rate) */
        i1_re += x_re;
        i1_im += x_im;
        i2_re += i1_re;
        i2_im += i1_im;
        i3_re += i2_re;
        i3_im += i2_im;

        cnt++;
        if (cnt >= decim_ratio) {
            cnt = 0;

            /* 3 comb stages (run at output rate) */
            int64_t d1_re = i3_re - c1_prev_re;
            int64_t d1_im = i3_im - c1_prev_im;
            c1_prev_re = i3_re;
            c1_prev_im = i3_im;

            int64_t d2_re = d1_re - c2_prev_re;
            int64_t d2_im = d1_im - c2_prev_im;
            c2_prev_re = d1_re;
            c2_prev_im = d1_im;

            int64_t d3_re = d2_re - c3_prev_re;
            int64_t d3_im = d2_im - c3_prev_im;
            c3_prev_re = d2_re;
            c3_prev_im = d2_im;

            /* Normalize and output */
            iq_out[out_pos * 2]     = (int16_t)(d3_re >> shift);
            iq_out[out_pos * 2 + 1] = (int16_t)(d3_im >> shift);
            out_pos++;
        }
    }

    /* Save state */
    accum[0]  = (int32_t)(i1_re);       accum[1]  = (int32_t)(i1_re >> 32);
    accum[2]  = (int32_t)(i1_im);       accum[3]  = (int32_t)(i1_im >> 32);
    accum[4]  = (int32_t)(i2_re);       accum[5]  = (int32_t)(i2_re >> 32);
    accum[6]  = (int32_t)(i2_im);       accum[7]  = (int32_t)(i2_im >> 32);
    accum[8]  = (int32_t)(i3_re);       accum[9]  = (int32_t)(i3_re >> 32);
    accum[10] = (int32_t)(i3_im);       accum[11] = (int32_t)(i3_im >> 32);
    accum[12] = (int32_t)(c1_prev_re);  accum[13] = (int32_t)(c1_prev_re >> 32);
    accum[14] = (int32_t)(c1_prev_im);  accum[15] = (int32_t)(c1_prev_im >> 32);
    accum[16] = (int32_t)(c2_prev_re);  accum[17] = (int32_t)(c2_prev_re >> 32);
    accum[18] = (int32_t)(c2_prev_im);  accum[19] = (int32_t)(c2_prev_im >> 32);
    accum[20] = (int32_t)(c3_prev_re);  accum[21] = (int32_t)(c3_prev_re >> 32);
    accum[22] = (int32_t)(c3_prev_im);  accum[23] = (int32_t)(c3_prev_im >> 32);
    accum[24] = cnt;

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
