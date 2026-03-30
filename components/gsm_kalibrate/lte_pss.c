#include "lte_pss.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "lte_pss";

static const int pss_roots[LTE_PSS_ROOTS] = {
    LTE_PSS_ROOT_0, LTE_PSS_ROOT_1, LTE_PSS_ROOT_2
};

void lte_pss_generate(lte_pss_templates_t *templates)
{
    memset(templates, 0, sizeof(*templates));

    for (int r = 0; r < LTE_PSS_ROOTS; r++) {
        int u = pss_roots[r];
        for (int n = 0; n < LTE_PSS_LEN; n++) {
            /* x_u(n) = exp(-j * PI * u * n * (n+1) / 63) */
            float phase = -(float)M_PI * (float)u * (float)n * (float)(n + 1) / (float)LTE_PSS_LEN;
            templates->re[r][n] = cosf(phase);
            templates->im[r][n] = sinf(phase);
        }
        ESP_LOGD(TAG, "Generated PSS root u=%d (N_ID_2=%d)", u, r);
    }

    ESP_LOGI(TAG, "PSS templates generated for all %d roots", LTE_PSS_ROOTS);
}

int lte_pss_correlate(const float *iq_re, const float *iq_im, int iq_len,
                      const lte_pss_templates_t *templates,
                      int search_step,
                      lte_pss_peak_t *best_peak)
{
    if (iq_len < LTE_PSS_LEN || !templates || !best_peak) {
        return 0;
    }

    if (search_step < 1) {
        search_step = 1;
    }

    float best_mag = 0.0f;
    float best_phase = 0.0f;
    int   best_offset = 0;
    int   best_root = 0;
    int   peaks_found = 0;

    int max_offset = iq_len - LTE_PSS_LEN;

    for (int offset = 0; offset <= max_offset; offset += search_step) {
        for (int r = 0; r < LTE_PSS_ROOTS; r++) {
            float corr_re = 0.0f;
            float corr_im = 0.0f;

            /* Complex conjugate multiply: corr = signal * conj(template) */
            for (int k = 0; k < LTE_PSS_LEN; k++) {
                float sig_re = iq_re[offset + k];
                float sig_im = iq_im[offset + k];
                float tmpl_re = templates->re[r][k];
                float tmpl_im = templates->im[r][k];

                corr_re += sig_re * tmpl_re + sig_im * tmpl_im;
                corr_im += sig_im * tmpl_re - sig_re * tmpl_im;
            }

            float mag = sqrtf(corr_re * corr_re + corr_im * corr_im);

            if (mag > best_mag) {
                best_mag = mag;
                best_phase = atan2f(corr_im, corr_re);
                best_offset = offset;
                best_root = r;
                peaks_found = 1;
            }
        }
    }

    best_peak->magnitude = best_mag;
    best_peak->phase = best_phase;
    best_peak->offset = best_offset;
    best_peak->n_id_2 = (uint8_t)best_root;

    ESP_LOGD(TAG, "Correlate: best mag=%.2f phase=%.4f offset=%d N_ID_2=%d",
             best_mag, best_phase, best_offset, best_root);

    return peaks_found;
}

float lte_pss_freq_error(const float *iq_re, const float *iq_im, int iq_len,
                         const lte_pss_templates_t *templates,
                         float sample_rate)
{
    /* Step 1: Coarse search with step=16 (fast, avoids WDT) */
    lte_pss_peak_t coarse;
    int found = lte_pss_correlate(iq_re, iq_im, iq_len, templates, 16, &coarse);

    if (!found || coarse.magnitude < 1.0f) {
        ESP_LOGD(TAG, "No PSS peak found (mag=%.2f)", coarse.magnitude);
        return 0.0f;
    }

    /* Step 2: Fine search at step=1 within ±32 samples of coarse peak */
    int fine_start = coarse.offset - 32;
    int fine_end = coarse.offset + 32;
    if (fine_start < 0) fine_start = 0;
    if (fine_end > iq_len - LTE_PSS_LEN) fine_end = iq_len - LTE_PSS_LEN;

    float best_mag = 0;
    float best_phase = 0;
    int best_root = coarse.n_id_2;

    for (int offset = fine_start; offset <= fine_end; offset++) {
        for (int r = 0; r < LTE_PSS_ROOTS; r++) {
            float corr_re = 0.0f, corr_im = 0.0f;
            for (int k = 0; k < LTE_PSS_LEN; k++) {
                corr_re += iq_re[offset + k] * templates->re[r][k]
                         + iq_im[offset + k] * templates->im[r][k];
                corr_im += iq_im[offset + k] * templates->re[r][k]
                         - iq_re[offset + k] * templates->im[r][k];
            }
            float mag = sqrtf(corr_re * corr_re + corr_im * corr_im);
            if (mag > best_mag) {
                best_mag = mag;
                best_phase = atan2f(corr_im, corr_re);
                best_root = r;
            }
        }
    }

    /* Step 3: Look for repeated PSS peaks at 5ms intervals (half-frame).
     * PSS occurs twice per 10ms LTE frame → every 5ms = sample_rate/200.
     * Average phase across multiple peaks for better accuracy. */
    int pss_period = (int)(sample_rate / 200.0f);  /* 5ms in samples */
    float phase_sum = best_phase;
    int peak_count = 1;
    float threshold = best_mag * 0.3f;

    /* Search forward from best peak at 5ms intervals */
    for (int rep_offset = coarse.offset + pss_period;
         rep_offset + LTE_PSS_LEN < iq_len && peak_count < 40;
         rep_offset += pss_period) {
        /* Fine search ±16 around expected position */
        int rs = rep_offset - 16;
        int re = rep_offset + 16;
        if (rs < 0) rs = 0;
        if (re > iq_len - LTE_PSS_LEN) re = iq_len - LTE_PSS_LEN;

        float rep_best_mag = 0, rep_phase = 0;
        for (int off = rs; off <= re; off++) {
            float cr = 0, ci = 0;
            for (int k = 0; k < LTE_PSS_LEN; k++) {
                cr += iq_re[off + k] * templates->re[best_root][k]
                    + iq_im[off + k] * templates->im[best_root][k];
                ci += iq_im[off + k] * templates->re[best_root][k]
                    - iq_re[off + k] * templates->im[best_root][k];
            }
            float m = sqrtf(cr * cr + ci * ci);
            if (m > rep_best_mag) {
                rep_best_mag = m;
                rep_phase = atan2f(ci, cr);
            }
        }
        if (rep_best_mag >= threshold) {
            phase_sum += rep_phase;
            peak_count++;
        }
    }

    /* Step 4: Convert averaged phase to frequency error */
    float avg_phase = phase_sum / (float)peak_count;
    float pss_duration = (float)LTE_PSS_LEN / sample_rate;
    float freq_error_hz = avg_phase / (2.0f * (float)M_PI * pss_duration);

    ESP_LOGI(TAG, "Freq error: %.1f Hz (from %d PSS peaks, N_ID_2=%d, mag=%.0f)",
             freq_error_hz, peak_count, best_root, best_mag);

    return freq_error_hz;
}
