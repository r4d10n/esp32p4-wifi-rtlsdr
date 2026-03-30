/*
 * LTE Cell Synchronization -- FFT-based PSS/SSS Detection
 *
 * Implements 3GPP TS 36.211 compliant PSS (Zadoff-Chu) and SSS
 * (m-sequence) detection for LTE cell search at 1.92 MSPS.
 *
 * Algorithm:
 *   1. Slide 128-pt FFT across IQ buffer, correlate with 3 PSS templates
 *   2. At PSS peak: estimate CFO via split-phase method
 *   3. Extract SSS from preceding OFDM symbol, channel-equalize via PSS
 *   4. Brute-force SSS over 168 N_ID_1 x 2 subframes -> PCI
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "lte_sync.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_dsp.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "lte_sync";

/* Zadoff-Chu roots per N_ID_2 */
static const int zc_roots[LTE_PSS_ROOTS] = {
    LTE_ZC_ROOT_0, LTE_ZC_ROOT_1, LTE_ZC_ROOT_2
};

/* ──────────────────────── Internal State ──────────────────────── */

struct lte_sync {
    /* PSS templates in frequency domain (62 complex values per root).
     * Stored PRE-CONJUGATED for correlation: im = -sin(phase). */
    float pss_fd_re[3][LTE_N_SC_PSS];
    float pss_fd_im[3][LTE_N_SC_PSS];

    /* SSS base m-sequences (BPSK: +1/-1) */
    int8_t s_tilda[31];
    int8_t c_tilda[31];
    int8_t z_tilda[31];

    /* Work buffers */
    float *fft_buf;         /* 256 floats = 128 complex interleaved */
    float *iq_float_re;     /* Converted IQ re buffer */
    float *iq_float_im;     /* Converted IQ im buffer */

    bool initialized;
};

/* ──────────────────────── PSS Generation ──────────────────────── */

/**
 * Generate 3 Zadoff-Chu PSS sequences of length 62 in frequency domain.
 * DC element at n=31 (original indexing) is SKIPPED per 3GPP TS 36.211.
 * Stored pre-conjugated for efficient correlation.
 */
static void gen_pss_fd(lte_sync_t *ctx)
{
    for (int r = 0; r < LTE_PSS_ROOTS; r++) {
        int u = zc_roots[r];

        /* n = 0..30: d(n) = exp(-j*pi*u*n*(n+1)/63) */
        for (int n = 0; n <= 30; n++) {
            float phase = -(float)M_PI * (float)u * (float)n * (float)(n + 1) / 63.0f;
            ctx->pss_fd_re[r][n] = cosf(phase);
            ctx->pss_fd_im[r][n] = -sinf(phase);  /* pre-conjugated */
        }

        /* n = 31..61: d(n) = exp(-j*pi*u*(n+1)*(n+2)/63), skip DC */
        for (int n = 31; n <= 61; n++) {
            float phase = -(float)M_PI * (float)u * (float)(n + 1) * (float)(n + 2) / 63.0f;
            ctx->pss_fd_re[r][n] = cosf(phase);
            ctx->pss_fd_im[r][n] = -sinf(phase);  /* pre-conjugated */
        }

        ESP_LOGD(TAG, "PSS root u=%d (N_ID_2=%d) generated", u, r);
    }
}

/* ──────────────────────── SSS m-sequence Generation ──────────────────────── */

/**
 * Generate LFSR m-sequences for SSS detection.
 *
 * Three m-sequences of length 31, initial state [0,0,0,0,1]:
 *   s: x^5 + x^2 + 1
 *   c: x^5 + x^3 + 1
 *   z: x^5 + x^4 + x^2 + x + 1
 *
 * Output: BPSK mapped (1 - 2*bit)
 */
static void gen_sss_tables(lte_sync_t *ctx)
{
    uint8_t x_s[31 + 5];
    uint8_t x_c[31 + 5];
    uint8_t x_z[31 + 5];

    /* Initial state: [0,0,0,0,1] for all three */
    memset(x_s, 0, sizeof(x_s));
    memset(x_c, 0, sizeof(x_c));
    memset(x_z, 0, sizeof(x_z));
    x_s[4] = 1;
    x_c[4] = 1;
    x_z[4] = 1;

    /* Generate LFSR bits */
    for (int i = 0; i < 31; i++) {
        /* s: x^5 + x^2 + 1 -> x[i+5] = (x[i+2] + x[i]) % 2 */
        x_s[i + 5] = (x_s[i + 2] + x_s[i]) % 2;

        /* c: x^5 + x^3 + 1 -> x[i+5] = (x[i+3] + x[i]) % 2 */
        x_c[i + 5] = (x_c[i + 3] + x_c[i]) % 2;

        /* z: x^5 + x^4 + x^2 + x + 1 -> x[i+5] = (x[i+4] + x[i+2] + x[i+1] + x[i]) % 2 */
        x_z[i + 5] = (x_z[i + 4] + x_z[i + 2] + x_z[i + 1] + x_z[i]) % 2;
    }

    /* Convert binary to BPSK: tilda[n] = 1 - 2*x[n] */
    for (int n = 0; n < 31; n++) {
        ctx->s_tilda[n] = 1 - 2 * (int8_t)x_s[n];
        ctx->c_tilda[n] = 1 - 2 * (int8_t)x_c[n];
        ctx->z_tilda[n] = 1 - 2 * (int8_t)x_z[n];
    }

    ESP_LOGD(TAG, "SSS m-sequences generated (s/c/z, len=31)");
}

/* ──────────────────────── PSS Detection ──────────────────────── */

typedef struct {
    int   offset;       /* Sample offset of best PSS window */
    int   n_id_2;       /* Best root (0-2) */
    float magnitude;    /* |correlation|^2 */
    float corr_re;      /* Real part of best correlation */
    float corr_im;      /* Imag part of best correlation */
} pss_result_t;

/**
 * Slide 128-pt FFT across IQ, correlate with 3 PSS templates.
 *
 * Input: float IQ (re/im separate) at 1.92 MSPS.
 * Step: LTE_CPN (9) samples per window shift.
 * Output: best peak across all positions and roots.
 */
static void pss_detect(lte_sync_t *ctx, int num_samples, pss_result_t *result)
{
    float best_mag = 0.0f;
    float best_re = 0.0f, best_im = 0.0f;
    int   best_offset = 0;
    int   best_root = 0;
    int   fft_count = 0;

    int max_offset = num_samples - LTE_N_FFT;
    if (max_offset < 0) {
        memset(result, 0, sizeof(*result));
        return;
    }

    for (int pos = 0; pos <= max_offset; pos += LTE_CPN) {
        /* Copy 128 samples into fft_buf as interleaved [re,im,...] */
        for (int k = 0; k < LTE_N_FFT; k++) {
            ctx->fft_buf[2 * k]     = ctx->iq_float_re[pos + k];
            ctx->fft_buf[2 * k + 1] = ctx->iq_float_im[pos + k];
        }

        /* FFT */
        dsps_fft2r_fc32(ctx->fft_buf, LTE_N_FFT);
        dsps_bit_rev_fc32(ctx->fft_buf, LTE_N_FFT);

        /* Extract 62 PSS subcarrier bins.
         * Negative frequencies: bins 97..127 -> pss_rx[0..30]
         * Positive frequencies: bins 1..31   -> pss_rx[31..61] */
        float pss_rx_re[LTE_N_SC_PSS];
        float pss_rx_im[LTE_N_SC_PSS];

        for (int k = 0; k <= 30; k++) {
            int bin = 97 + k;
            pss_rx_re[k] = ctx->fft_buf[bin * 2];
            pss_rx_im[k] = ctx->fft_buf[bin * 2 + 1];
        }
        for (int k = 0; k <= 30; k++) {
            int bin = 1 + k;
            pss_rx_re[31 + k] = ctx->fft_buf[bin * 2];
            pss_rx_im[31 + k] = ctx->fft_buf[bin * 2 + 1];
        }

        /* Correlate with each of 3 pre-conjugated PSS templates */
        for (int r = 0; r < LTE_PSS_ROOTS; r++) {
            float cr = 0.0f, ci = 0.0f;
            for (int k = 0; k < LTE_N_SC_PSS; k++) {
                cr += pss_rx_re[k] * ctx->pss_fd_re[r][k]
                    - pss_rx_im[k] * ctx->pss_fd_im[r][k];
                ci += pss_rx_re[k] * ctx->pss_fd_im[r][k]
                    + pss_rx_im[k] * ctx->pss_fd_re[r][k];
            }
            float mag = cr * cr + ci * ci;

            if (mag > best_mag) {
                best_mag = mag;
                best_re = cr;
                best_im = ci;
                best_offset = pos;
                best_root = r;
            }
        }

        fft_count++;
        /* Yield every 500 FFTs to prevent WDT */
        if (fft_count % 500 == 0) {
            vTaskDelay(1);
        }
    }

    result->offset = best_offset;
    result->n_id_2 = best_root;
    result->magnitude = best_mag;
    result->corr_re = best_re;
    result->corr_im = best_im;

    ESP_LOGD(TAG, "PSS detect: mag=%.0f offset=%d N_ID_2=%d",
             best_mag, best_offset, best_root);
}

/* ──────────────────────── Frequency Estimation with IFO ──────────────────────── */

/**
 * Extract 62 PSS subcarrier bins from FFT buffer with an integer frequency
 * offset (IFO) shift of `ifo_shift` subcarriers.
 */
static void extract_pss_bins_shifted(const float *fft_buf, int ifo_shift,
                                     float *pss_re, float *pss_im)
{
    /* Negative freq bins: nominally 97..127, shifted by ifo_shift */
    for (int k = 0; k <= 30; k++) {
        int bin = ((97 + k + ifo_shift) % LTE_N_FFT + LTE_N_FFT) % LTE_N_FFT;
        pss_re[k] = fft_buf[bin * 2];
        pss_im[k] = fft_buf[bin * 2 + 1];
    }
    /* Positive freq bins: nominally 1..31, shifted by ifo_shift */
    for (int k = 0; k <= 30; k++) {
        int bin = ((1 + k + ifo_shift) % LTE_N_FFT + LTE_N_FFT) % LTE_N_FFT;
        pss_re[31 + k] = fft_buf[bin * 2];
        pss_im[31 + k] = fft_buf[bin * 2 + 1];
    }
}

/**
 * Correlate extracted PSS bins with a pre-conjugated template.
 * Returns squared magnitude.
 */
static float pss_corr_mag2(const float *rx_re, const float *rx_im,
                           const float *ref_re, const float *ref_im)
{
    float cr = 0.0f, ci = 0.0f;
    for (int k = 0; k < LTE_N_SC_PSS; k++) {
        cr += rx_re[k] * ref_re[k] - rx_im[k] * ref_im[k];
        ci += rx_re[k] * ref_im[k] + rx_im[k] * ref_re[k];
    }
    return cr * cr + ci * ci;
}

/**
 * Full frequency estimation with IFO (Integer Frequency Offset) correction.
 *
 * 1. Re-FFT at PSS peak
 * 2. Try IFO shifts -3..+3 subcarriers, find best PSS correlation
 * 3. IFO gives integer Hz offset: ifo * 15000 Hz
 * 4. Split-phase gives fractional Hz offset: ±7500 Hz
 * 5. Total freq_error = ifo * 15000 + fractional_cfo
 *
 * This replaces the old split-phase-only estimator.
 */
static float freq_estimate(lte_sync_t *ctx, int pss_offset, int n_id_2, int *ifo_out)
{
    /* Re-FFT at the exact peak position */
    for (int k = 0; k < LTE_N_FFT; k++) {
        ctx->fft_buf[2 * k]     = ctx->iq_float_re[pss_offset + k];
        ctx->fft_buf[2 * k + 1] = ctx->iq_float_im[pss_offset + k];
    }
    dsps_fft2r_fc32(ctx->fft_buf, LTE_N_FFT);
    dsps_bit_rev_fc32(ctx->fft_buf, LTE_N_FFT);

    const float *ref_re = ctx->pss_fd_re[n_id_2];
    const float *ref_im = ctx->pss_fd_im[n_id_2];

    /* ── IFO detection: try shifts -3..+3 subcarriers ── */
    float pss_rx_re[LTE_N_SC_PSS], pss_rx_im[LTE_N_SC_PSS];
    float best_ifo_mag = 0.0f;
    int   best_ifo = 0;

    for (int ifo = -3; ifo <= 3; ifo++) {
        extract_pss_bins_shifted(ctx->fft_buf, ifo, pss_rx_re, pss_rx_im);
        float mag = pss_corr_mag2(pss_rx_re, pss_rx_im, ref_re, ref_im);
        if (mag > best_ifo_mag) {
            best_ifo_mag = mag;
            best_ifo = ifo;
        }
    }

    /* Re-extract bins at the best IFO shift */
    extract_pss_bins_shifted(ctx->fft_buf, best_ifo, pss_rx_re, pss_rx_im);

    /* ── Fractional CFO via split-phase ── */
    float y0_re = 0.0f, y0_im = 0.0f;
    float y1_re = 0.0f, y1_im = 0.0f;

    for (int k = 0; k < 31; k++) {
        y0_re += pss_rx_re[k] * ref_re[k] - pss_rx_im[k] * ref_im[k];
        y0_im += pss_rx_re[k] * ref_im[k] + pss_rx_im[k] * ref_re[k];
    }
    for (int k = 31; k < LTE_N_SC_PSS; k++) {
        y1_re += pss_rx_re[k] * ref_re[k] - pss_rx_im[k] * ref_im[k];
        y1_im += pss_rx_re[k] * ref_im[k] + pss_rx_im[k] * ref_re[k];
    }

    float cross_re = y0_re * y1_re + y0_im * y1_im;
    float cross_im = y0_re * y1_im - y0_im * y1_re;
    float frac_cfo_norm = atan2f(cross_im, cross_re) / (float)M_PI;
    float frac_cfo_hz = frac_cfo_norm * 15000.0f / 2.0f;  /* ±7500 Hz */

    /* ── Total frequency error: IFO + fractional ── */
    float freq_error_hz = (float)best_ifo * 15000.0f + frac_cfo_hz;

    if (ifo_out) *ifo_out = best_ifo;

    ESP_LOGI(TAG, "CFO: IFO=%+d (%.0f Hz) + frac=%+.1f Hz = total %+.1f Hz",
             best_ifo, (float)best_ifo * 15000.0f, frac_cfo_hz, freq_error_hz);
    return freq_error_hz;
}

/* ──────────────────────── SSS Detection ──────────────────────── */

typedef struct {
    int     n_id_1;     /* 0-167 */
    uint8_t subframe;   /* 0 or 5 */
    float   corr_mag;   /* Correlation magnitude */
    float   second_corr;/* Second-best correlation (for validation) */
    bool    valid;       /* True if SSS was detected above threshold */
    float   sss_ref[LTE_N_SC_PSS];  /* Matched SSS reference (for CFO) */
    float   freq_err_hz;  /* PSS-SSS CFO estimate in Hz (when valid) */
} sss_result_t;

/**
 * Generate 62-element SSS reference for a given (n_id_1, n_id_2, subframe).
 * Output: real-valued BPSK (+1/-1) array of length 62.
 */
static void gen_sss_ref(const lte_sync_t *ctx, int n_id_1, int n_id_2,
                        int subframe, float *sss_ref)
{
    /* Compute m0, m1 from N_ID_1 */
    uint32_t qp = (uint32_t)n_id_1 / 30;
    uint32_t q  = ((uint32_t)n_id_1 + qp * (qp + 1) / 2) / 30;
    uint32_t mp = (uint32_t)n_id_1 + q * (q + 1) / 2;
    uint32_t m0 = mp % 31;
    uint32_t m1 = (m0 + mp / 31 + 1) % 31;

    /* Cyclic shifts of base m-sequences */
    for (int n = 0; n < 31; n++) {
        int8_t s0 = ctx->s_tilda[(n + m0) % 31];
        int8_t s1 = ctx->s_tilda[(n + m1) % 31];
        int8_t c0 = ctx->c_tilda[(n + (uint32_t)n_id_2) % 31];
        int8_t c1 = ctx->c_tilda[(n + (uint32_t)n_id_2 + 3) % 31];
        int8_t z0 = ctx->z_tilda[(n + (m0 % 8)) % 31];
        int8_t z1 = ctx->z_tilda[(n + (m1 % 8)) % 31];

        float even, odd;
        if (subframe == 0) {
            even = (float)(s0 * c0);
            odd  = (float)(s1 * c1 * z0);
        } else {
            /* subframe 5 */
            even = (float)(s1 * c0);
            odd  = (float)(s0 * c1 * z1);
        }

        /* Interleave: sss[2n] = even[n], sss[2n+1] = odd[n] */
        sss_ref[2 * n]     = even;
        sss_ref[2 * n + 1] = odd;
    }
}

/**
 * Detect SSS after PSS has been found.
 *
 * SSS is one OFDM symbol before PSS:
 *   sss_offset = pss_offset - LTE_CPN - LTE_N_FFT
 *
 * Channel equalization via PSS channel estimate.
 */
static void sss_detect(lte_sync_t *ctx, int pss_offset, int n_id_2, int ifo_shift,
                       int num_samples, sss_result_t *result)
{
    result->valid = false;
    result->n_id_1 = 0;
    result->subframe = 0;
    result->corr_mag = 0.0f;

    /* SSS is one symbol before PSS */
    int sss_offset = pss_offset - LTE_CPN - LTE_N_FFT;
    if (sss_offset < 0 || sss_offset + LTE_N_FFT > num_samples) {
        ESP_LOGD(TAG, "SSS offset out of range: %d", sss_offset);
        return;
    }

    /* FFT the PSS symbol (for channel estimate) */
    for (int k = 0; k < LTE_N_FFT; k++) {
        ctx->fft_buf[2 * k]     = ctx->iq_float_re[pss_offset + k];
        ctx->fft_buf[2 * k + 1] = ctx->iq_float_im[pss_offset + k];
    }
    dsps_fft2r_fc32(ctx->fft_buf, LTE_N_FFT);
    dsps_bit_rev_fc32(ctx->fft_buf, LTE_N_FFT);

    /* Extract PSS received bins with IFO correction */
    float pss_rx_re[LTE_N_SC_PSS], pss_rx_im[LTE_N_SC_PSS];
    extract_pss_bins_shifted(ctx->fft_buf, ifo_shift, pss_rx_re, pss_rx_im);

    /* Channel estimate: H[k] = pss_rx[k] * conj(pss_ref[k])
     * pss_ref is pre-conjugated, so conj(pss_ref) = (re, -im) where
     * stored im is already -sin. So conj(stored) = (re, +sin) = (re, -stored_im). */
    float h_re[LTE_N_SC_PSS], h_im[LTE_N_SC_PSS];
    for (int k = 0; k < LTE_N_SC_PSS; k++) {
        /* conj of pre-conjugated = original = (pss_fd_re, -pss_fd_im) */
        float ref_re = ctx->pss_fd_re[n_id_2][k];
        float ref_im_conj = -ctx->pss_fd_im[n_id_2][k];  /* undo pre-conjugation */
        h_re[k] = pss_rx_re[k] * ref_re - pss_rx_im[k] * ref_im_conj;
        h_im[k] = pss_rx_re[k] * ref_im_conj + pss_rx_im[k] * ref_re;
    }

    /* FFT the SSS symbol */
    for (int k = 0; k < LTE_N_FFT; k++) {
        ctx->fft_buf[2 * k]     = ctx->iq_float_re[sss_offset + k];
        ctx->fft_buf[2 * k + 1] = ctx->iq_float_im[sss_offset + k];
    }
    dsps_fft2r_fc32(ctx->fft_buf, LTE_N_FFT);
    dsps_bit_rev_fc32(ctx->fft_buf, LTE_N_FFT);

    /* Extract SSS received bins with same IFO correction as PSS */
    float sss_rx_re[LTE_N_SC_PSS], sss_rx_im[LTE_N_SC_PSS];
    extract_pss_bins_shifted(ctx->fft_buf, ifo_shift, sss_rx_re, sss_rx_im);

    /* Channel equalize SSS: sss_eq[k] = sss_rx[k] * conj(H[k]) */
    float sss_eq_re[LTE_N_SC_PSS];
    for (int k = 0; k < LTE_N_SC_PSS; k++) {
        sss_eq_re[k] = sss_rx_re[k] * h_re[k] + sss_rx_im[k] * h_im[k];
        /* sss_eq_im not needed: SSS ref is real-valued BPSK */
    }

    /* Brute-force search: 168 N_ID_1 values x 2 subframes */
    float best_corr = 0.0f;
    float second_corr = 0.0f;
    int   best_n_id_1 = 0;
    int   best_sf = 0;
    float sss_ref_tmp[LTE_N_SC_PSS];
    int   iter = 0;

    for (int sf = 0; sf <= 5; sf += 5) {
        for (int nid1 = 0; nid1 < 168; nid1++) {
            gen_sss_ref(ctx, nid1, n_id_2, sf, sss_ref_tmp);

            float corr = 0.0f;
            for (int k = 0; k < LTE_N_SC_PSS; k++) {
                corr += sss_eq_re[k] * sss_ref_tmp[k];
            }

            float abs_corr = fabsf(corr);
            if (abs_corr > best_corr) {
                second_corr = best_corr;
                best_corr = abs_corr;
                best_n_id_1 = nid1;
                best_sf = sf;
            } else if (abs_corr > second_corr) {
                second_corr = abs_corr;
            }

            iter++;
            if (iter % 168 == 0) vTaskDelay(1);
        }
    }

    /* SSS is valid if best correlation is significantly above second-best.
     * With 336 hypotheses, a real match should be >1.5x the next best. */
    float ratio = (second_corr > 0.001f) ? best_corr / second_corr : 10.0f;
    if (ratio > 1.05f && best_corr > 0.01f) {
        result->valid = true;
        result->n_id_1 = best_n_id_1;
        result->subframe = (uint8_t)best_sf;
        result->corr_mag = best_corr;
        result->second_corr = second_corr;

        /* Store the winning SSS reference for PSS-SSS CFO estimation */
        gen_sss_ref(ctx, best_n_id_1, n_id_2, best_sf, result->sss_ref);

        ESP_LOGD(TAG, "SSS: N_ID_1=%d sf=%d corr=%.2f ratio=%.2f",
                 best_n_id_1, best_sf, best_corr, ratio);
    } else {
        ESP_LOGD(TAG, "SSS rejected (corr=%.4f ratio=%.2f)", best_corr, ratio);
    }

    /* ── PSS-SSS frequency offset estimation ──
     *
     * Channel estimate from PSS: h_pss[k] = rx_pss[k] * conj(pss_ref[k])
     * Channel estimate from SSS: h_sss[k] = rx_sss[k] * conj(sss_ref[k])
     * Cross-correlation: M = Σ conj(h_sss[k]) × h_pss[k]
     *   = Σ |H[k]|² × exp(j×2π×f_err×Δt)
     * where Δt = (LTE_CPN + LTE_N_FFT) / Fs = 137/1920000 = 71.35 µs
     *
     * f_err = arg(M) / (2π × Δt)
     * Unambiguous range: ±1/(2×Δt) = ±7003 Hz = ±8.8 PPM at 800 MHz
     */
    if (result->valid) {
        /* h_pss already computed above (h_re, h_im) */
        /* Compute h_sss: sss_rx * conj(sss_ref).
         * sss_ref is real BPSK (±1), so conj(sss_ref) = sss_ref.
         * h_sss = sss_rx * sss_ref */
        float m_re = 0.0f, m_im = 0.0f;
        for (int k = 0; k < LTE_N_SC_PSS; k++) {
            float h_sss_re = sss_rx_re[k] * result->sss_ref[k];
            float h_sss_im = sss_rx_im[k] * result->sss_ref[k];

            /* M += conj(h_sss) * h_pss */
            m_re += h_sss_re * h_re[k] + h_sss_im * h_im[k];
            m_im += h_sss_re * h_im[k] - h_sss_im * h_re[k];
        }

        float delta_t = (float)(LTE_CPN + LTE_N_FFT) / (float)LTE_SAMPLE_RATE_HZ;
        float phase = atan2f(m_im, m_re);
        result->freq_err_hz = phase / (2.0f * (float)M_PI * delta_t);

        ESP_LOGD(TAG, "PSS-SSS CFO: %.1f Hz (phase=%.4f, Δt=%.1fµs)",
                 result->freq_err_hz, phase, delta_t * 1e6f);
    }
}

/* ──────────────────────── Public API ──────────────────────── */

lte_sync_t *lte_sync_init(void)
{
    lte_sync_t *ctx = calloc(1, sizeof(lte_sync_t));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate sync context");
        return NULL;
    }

    /* Initialize esp-dsp FFT tables for float path */
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp-dsp fc32 FFT init failed: %s", esp_err_to_name(ret));
        free(ctx);
        return NULL;
    }

    /* Allocate work buffers in PSRAM */
    ctx->fft_buf = heap_caps_malloc(LTE_N_FFT * 2 * sizeof(float),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ctx->iq_float_re = heap_caps_malloc(LTE_FRAME_SAMPLES * sizeof(float),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ctx->iq_float_im = heap_caps_malloc(LTE_FRAME_SAMPLES * sizeof(float),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!ctx->fft_buf || !ctx->iq_float_re || !ctx->iq_float_im) {
        ESP_LOGE(TAG, "Failed to allocate work buffers in PSRAM");
        free(ctx->fft_buf);
        free(ctx->iq_float_re);
        free(ctx->iq_float_im);
        free(ctx);
        return NULL;
    }

    /* Generate PSS and SSS templates */
    gen_pss_fd(ctx);
    gen_sss_tables(ctx);

    ctx->initialized = true;
    ESP_LOGI(TAG, "LTE sync engine initialized (128-pt FFT, PSS+SSS)");
    return ctx;
}

void lte_sync_deinit(lte_sync_t *ctx)
{
    if (!ctx) return;

    free(ctx->fft_buf);
    free(ctx->iq_float_re);
    free(ctx->iq_float_im);
    free(ctx);

    ESP_LOGI(TAG, "LTE sync engine deinitialized");
}

int lte_sync_detect(lte_sync_t *ctx,
                    const uint8_t *iq_data, uint32_t iq_len,
                    uint32_t carrier_hz,
                    lte_cell_t *cells, int max_cells)
{
    if (!ctx || !ctx->initialized || !iq_data || !cells || max_cells < 1) {
        return 0;
    }

    /* Convert uint8 IQ to float.
     * Limit to one frame worth of samples to fit work buffers. */
    int num_samples = (int)(iq_len / 2);
    if (num_samples > LTE_FRAME_SAMPLES) {
        num_samples = LTE_FRAME_SAMPLES;
    }
    if (num_samples < LTE_N_FFT) {
        ESP_LOGW(TAG, "IQ buffer too short: %d samples (need >= %d)", num_samples, LTE_N_FFT);
        return 0;
    }

    for (int i = 0; i < num_samples; i++) {
        ctx->iq_float_re[i] = ((float)iq_data[2 * i] - 127.5f) / 127.5f;
        ctx->iq_float_im[i] = ((float)iq_data[2 * i + 1] - 127.5f) / 127.5f;
    }

    int cell_count = 0;

    /* PSS detection: find strongest peak */
    pss_result_t pss;
    pss_detect(ctx, num_samples, &pss);

    /* Noise threshold: require significant correlation */
    if (pss.magnitude < 1.0f) {
        ESP_LOGD(TAG, "No PSS peak (mag=%.2f)", pss.magnitude);
        return 0;
    }

    /* IFO + fractional CFO estimation (must come before SSS to get IFO) */
    int ifo = 0;
    float freq_err = 0.0f;
    if (pss.offset + LTE_N_FFT <= num_samples) {
        freq_err = freq_estimate(ctx, pss.offset, pss.n_id_2, &ifo);
    }

    /* SSS detection at PSS-indicated timing with IFO correction */
    sss_result_t sss;
    sss_detect(ctx, pss.offset, pss.n_id_2, ifo, num_samples, &sss);

    /* If SSS decoded, prefer PSS-SSS phase rotation CFO (immune to multipath) */
    if (sss.valid) {
        /* PSS-SSS CFO gives fractional part; add IFO for total */
        freq_err = (float)ifo * 15000.0f + sss.freq_err_hz;
    }

    /* Fill first cell result */
    lte_cell_t *cell = &cells[0];
    memset(cell, 0, sizeof(*cell));
    cell->n_id_2 = (uint8_t)pss.n_id_2;
    cell->pss_power = pss.magnitude;
    cell->pss_offset = pss.offset;
    cell->sss_valid = sss.valid;
    cell->freq_error_hz = freq_err;
    cell->ppm = (carrier_hz > 0) ? (freq_err / (float)carrier_hz * 1e6f) : 0.0f;

    if (sss.valid) {
        cell->n_id_1 = (uint8_t)sss.n_id_1;
        cell->subframe = sss.subframe;
        cell->pci = 3 * (uint16_t)sss.n_id_1 + (uint16_t)pss.n_id_2;

        ESP_LOGI(TAG, "Cell found: PCI=%u (N_ID_1=%u, N_ID_2=%u) "
                 "freq_err=%+.1f Hz ppm=%+.2f sf=%d",
                 cell->pci, cell->n_id_1, cell->n_id_2,
                 cell->freq_error_hz, cell->ppm, cell->subframe);
    } else {
        cell->pci = 0xFFFF;  /* Unknown PCI */
        cell->n_id_1 = 0xFF;

        ESP_LOGI(TAG, "PSS found (N_ID_2=%u mag=%.0f) but SSS not decoded "
                 "freq_err=%+.1f Hz ppm=%+.2f",
                 cell->n_id_2, cell->pss_power,
                 cell->freq_error_hz, cell->ppm);
    }

    cell_count = 1;

    /* Search for additional cells at different timing offsets.
     * Mask out a region around the first PSS peak and search again. */
    if (max_cells > 1 && num_samples > LTE_SLOT_SAMPLES * 2) {
        /* Save original IQ around first peak, zero it out, search again */
        int mask_start = pss.offset - LTE_N_FFT;
        int mask_end   = pss.offset + LTE_N_FFT;
        if (mask_start < 0) mask_start = 0;
        if (mask_end > num_samples) mask_end = num_samples;

        /* Temporarily zero out the first peak region */
        float *saved_re = NULL;
        float *saved_im = NULL;
        int mask_len = mask_end - mask_start;

        saved_re = malloc((uint32_t)mask_len * sizeof(float));
        saved_im = malloc((uint32_t)mask_len * sizeof(float));
        if (saved_re && saved_im) {
            memcpy(saved_re, &ctx->iq_float_re[mask_start], (uint32_t)mask_len * sizeof(float));
            memcpy(saved_im, &ctx->iq_float_im[mask_start], (uint32_t)mask_len * sizeof(float));
            memset(&ctx->iq_float_re[mask_start], 0, (uint32_t)mask_len * sizeof(float));
            memset(&ctx->iq_float_im[mask_start], 0, (uint32_t)mask_len * sizeof(float));

            pss_result_t pss2;
            pss_detect(ctx, num_samples, &pss2);

            /* Restore original IQ */
            memcpy(&ctx->iq_float_re[mask_start], saved_re, (uint32_t)mask_len * sizeof(float));
            memcpy(&ctx->iq_float_im[mask_start], saved_im, (uint32_t)mask_len * sizeof(float));

            /* Accept second cell if strong enough (at least 30% of first) */
            if (pss2.magnitude > pss.magnitude * 0.3f && pss2.magnitude > 1.0f) {
                int ifo2 = 0;
                float freq_err2 = 0.0f;
                if (pss2.offset + LTE_N_FFT <= num_samples) {
                    freq_err2 = freq_estimate(ctx, pss2.offset, pss2.n_id_2, &ifo2);
                }

                sss_result_t sss2;
                sss_detect(ctx, pss2.offset, pss2.n_id_2, ifo2, num_samples, &sss2);

                if (sss2.valid) {
                    freq_err2 = (float)ifo2 * 15000.0f + sss2.freq_err_hz;
                }

                lte_cell_t *cell2 = &cells[cell_count];
                memset(cell2, 0, sizeof(*cell2));
                cell2->n_id_2 = (uint8_t)pss2.n_id_2;
                cell2->pss_power = pss2.magnitude;
                cell2->pss_offset = pss2.offset;
                cell2->freq_error_hz = freq_err2;
                cell2->ppm = (carrier_hz > 0) ? (freq_err2 / (float)carrier_hz * 1e6f) : 0.0f;
                cell2->sss_valid = sss2.valid;

                if (sss2.valid) {
                    cell2->n_id_1 = (uint8_t)sss2.n_id_1;
                    cell2->subframe = sss2.subframe;
                    cell2->pci = 3 * (uint16_t)sss2.n_id_1 + (uint16_t)pss2.n_id_2;
                } else {
                    cell2->pci = 0xFFFF;
                    cell2->n_id_1 = 0xFF;
                }

                cell_count++;
                ESP_LOGI(TAG, "Additional cell: N_ID_2=%u mag=%.0f SSS=%s",
                         cell2->n_id_2, cell2->pss_power,
                         cell2->sss_valid ? "OK" : "no");
            }
        }
        free(saved_re);
        free(saved_im);
    }

    return cell_count;
}
