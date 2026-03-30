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
    /* Step 1: Find the best peak using step=4 */
    lte_pss_peak_t best;
    int found = lte_pss_correlate(iq_re, iq_im, iq_len, templates, 4, &best);

    if (!found || best.magnitude < 1.0f) {
        ESP_LOGD(TAG, "No PSS peak found (mag=%.2f)", best.magnitude);
        return 0.0f;
    }

    ESP_LOGD(TAG, "Best peak: mag=%.2f at offset=%d, N_ID_2=%d",
             best.magnitude, best.offset, best.n_id_2);

    /* Step 2: Search at all offsets to find ALL peaks above 40% of best */
    float threshold = best.magnitude * 0.4f;
    float phase_sum = 0.0f;
    int peak_count = 0;
    int max_offset = iq_len - LTE_PSS_LEN;

    for (int offset = 0; offset <= max_offset; offset++) {
        for (int r = 0; r < LTE_PSS_ROOTS; r++) {
            float corr_re = 0.0f;
            float corr_im = 0.0f;

            for (int k = 0; k < LTE_PSS_LEN; k++) {
                float sig_re = iq_re[offset + k];
                float sig_im = iq_im[offset + k];
                float tmpl_re = templates->re[r][k];
                float tmpl_im = templates->im[r][k];

                corr_re += sig_re * tmpl_re + sig_im * tmpl_im;
                corr_im += sig_im * tmpl_re - sig_re * tmpl_im;
            }

            float mag = sqrtf(corr_re * corr_re + corr_im * corr_im);

            if (mag >= threshold) {
                float phase = atan2f(corr_im, corr_re);
                phase_sum += phase;
                peak_count++;
            }
        }
    }

    if (peak_count == 0) {
        ESP_LOGD(TAG, "No peaks above threshold");
        return 0.0f;
    }

    /* Step 3: Average phase and convert to frequency error */
    float avg_phase = phase_sum / (float)peak_count;
    float pss_duration = (float)LTE_PSS_LEN / sample_rate;
    float freq_error_hz = avg_phase / (2.0f * (float)M_PI * pss_duration);

    ESP_LOGI(TAG, "Freq error: %.1f Hz (from %d peaks, avg_phase=%.4f)",
             freq_error_hz, peak_count, avg_phase);

    return freq_error_hz;
}
