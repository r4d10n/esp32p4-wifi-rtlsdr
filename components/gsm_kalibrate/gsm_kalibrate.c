/*
 * GSM Kalibrate — RTL-SDR PPM Calibration via FCCH Detection
 *
 * Two-stage SOTA energy detection:
 *   Stage 1: Wideband FFT power scan (1 MHz steps, 5 sub-channels each)
 *   Stage 2: Narrowband FCCH validation (FM discriminator variance)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "gsm_kalibrate.h"
#include "gsm_decode.h"
#include "lte_sync.h"

static const char *TAG = "kal";

/* ──────────────────────── Constants ──────────────────────── */

#define GSM_SAMPLE_RATE     1024000     /* 1.024 MSPS for GSM */
/* LTE sample rate now defined in lte_sync.h as LTE_SAMPLE_RATE_HZ (1920000) */
#define LTE_SAMPLE_RATE     LTE_SAMPLE_RATE_HZ
#define FFT_SIZE            1024
#define FCCH_FREQ_HZ        67708.3f    /* GSM FCCH tone = (1625000/6)/4 Hz */
#define CAPTURE_BUF_SIZE    (LTE_SAMPLE_RATE * 2 / 5)  /* 200ms at max rate = 819200 bytes */
#define SCAN_TASK_STACK     32768
#define SCAN_TASK_PRIO      5
#define SETTLE_MS           80          /* Tuner settle time after retune */
#define TWO_PI              6.283185307f
#define MAX_STEPS           200

/* Sub-channel offsets within 1 MHz bandwidth (Hz) */
static const int32_t subchan_offsets[] = { -400000, -200000, 0, 200000, 400000 };
#define NUM_SUBCHANS        5

/* ──────────────────────── Band Definitions ──────────────────────── */

static const kal_band_info_t bands[KAL_BAND_COUNT] = {
    [KAL_BAND_GSM850] = {
        .name = "GSM850",
        .arfcn_start = 128, .arfcn_end = 251,
        .dl_freq_start = 869200000, .dl_freq_end = 893800000,
    },
    [KAL_BAND_GSM900] = {
        .name = "GSM900",
        .arfcn_start = 1, .arfcn_end = 124,
        .dl_freq_start = 935200000, .dl_freq_end = 959800000,
    },
    [KAL_BAND_DCS1800] = {
        .name = "DCS1800",
        .arfcn_start = 512, .arfcn_end = 885,
        .dl_freq_start = 1805200000, .dl_freq_end = 1879800000,
    },
    [KAL_BAND_PCS1900] = {
        .name = "PCS1900",
        .arfcn_start = 512, .arfcn_end = 810,
        .dl_freq_start = 1930200000, .dl_freq_end = 1989800000,
    },
    [KAL_BAND_LTE_B1] = {
        .name = "LTE-B1",
        .arfcn_start = 0, .arfcn_end = 0,  /* EARFCN handled separately */
        .dl_freq_start = 2110000000, .dl_freq_end = 2170000000,
    },
    [KAL_BAND_LTE_B3] = {
        .name = "LTE-B3",
        .arfcn_start = 0, .arfcn_end = 0,
        .dl_freq_start = 1805000000, .dl_freq_end = 1880000000,
    },
    [KAL_BAND_LTE_B7] = {
        .name = "LTE-B7",
        .arfcn_start = 0, .arfcn_end = 0,
        .dl_freq_start = 2620000000, .dl_freq_end = 2690000000,
    },
    [KAL_BAND_LTE_B5] = {
        .name = "LTE-B5",
        .arfcn_start = 0, .arfcn_end = 0,
        .dl_freq_start = 869000000, .dl_freq_end = 894000000,
    },
    [KAL_BAND_LTE_B20] = {
        .name = "LTE-B20",
        .arfcn_start = 0, .arfcn_end = 0,
        .dl_freq_start = 791000000, .dl_freq_end = 821000000,
    },
    [KAL_BAND_LTE_B28] = {
        .name = "LTE-B28",
        .arfcn_start = 0, .arfcn_end = 0,
        .dl_freq_start = 758000000, .dl_freq_end = 803000000,
    },
};

/* ──────────────────────── Module State ──────────────────────── */

static struct {
    kal_config_t    config;
    kal_result_t    result;
    TaskHandle_t    scan_task;
    volatile bool   scan_abort;

    /* IQ capture buffer */
    uint8_t        *capture_buf;
    volatile uint32_t capture_pos;
    volatile bool   capture_active;
    SemaphoreHandle_t capture_ready;

    /* Pre-allocated Goertzel work buffers (avoid per-step malloc) */
    float          *goertzel_re;
    float          *goertzel_im;

    /* LTE sync engine (PSS+SSS, initialized once) */
    lte_sync_t *lte_ctx;

    /* HTTP server (created internally if none provided) */
    httpd_handle_t  httpd;
    bool            httpd_own;     /* true if we created it */

    bool            initialized;
} s_kal;

/* ──────────────────────── ARFCN ↔ Frequency ──────────────────────── */

const kal_band_info_t *kal_get_band_info(kal_band_t band)
{
    if (band >= KAL_BAND_COUNT) return NULL;
    return &bands[band];
}

uint32_t kal_arfcn_to_freq_band(uint16_t arfcn, kal_band_t band)
{
    switch (band) {
    case KAL_BAND_GSM850:
        if (arfcn >= 128 && arfcn <= 251)
            return 869200000u + 200000u * (arfcn - 128);
        break;
    case KAL_BAND_GSM900:
        if (arfcn >= 1 && arfcn <= 124)
            return 935200000u + 200000u * (arfcn - 1);
        if (arfcn == 0)
            return 935000000u;
        if (arfcn >= 975 && arfcn <= 1023)
            return 925200000u + 200000u * (arfcn - 975);
        break;
    case KAL_BAND_DCS1800:
        if (arfcn >= 512 && arfcn <= 885)
            return 1805200000u + 200000u * (arfcn - 512);
        break;
    case KAL_BAND_PCS1900:
        if (arfcn >= 512 && arfcn <= 810)
            return 1930200000u + 200000u * (arfcn - 512);
        break;
    default:
        break;
    }
    return 0;
}

uint32_t kal_arfcn_to_freq(uint16_t arfcn)
{
    /* Auto-detect band from ARFCN range */
    if (arfcn >= 128 && arfcn <= 251)
        return kal_arfcn_to_freq_band(arfcn, KAL_BAND_GSM850);
    if (arfcn <= 124)
        return kal_arfcn_to_freq_band(arfcn, KAL_BAND_GSM900);
    if (arfcn >= 975 && arfcn <= 1023)
        return kal_arfcn_to_freq_band(arfcn, KAL_BAND_GSM900);
    if (arfcn >= 512 && arfcn <= 885)
        return kal_arfcn_to_freq_band(arfcn, KAL_BAND_DCS1800);
    return 0;
}

uint16_t kal_freq_to_arfcn(uint32_t freq_hz, kal_band_t band)
{
    /* GSM900 needs special handling for E-GSM extension (925.2–934.8 MHz)
     * and ARFCN 0 (935.0 MHz), which lie outside the P-GSM dl_freq range. */
    if (band == KAL_BAND_GSM900) {
        if (freq_hz >= 935200000u && freq_hz <= 959800000u)
            return 1 + (uint16_t)((freq_hz - 935200000u) / 200000u);
        if (freq_hz == 935000000u)
            return 0;
        if (freq_hz >= 925200000u && freq_hz <= 934800000u)
            return 975 + (uint16_t)((freq_hz - 925200000u) / 200000u);
        return 0xFFFF;
    }

    const kal_band_info_t *bi = &bands[band];
    if (freq_hz < bi->dl_freq_start || freq_hz > bi->dl_freq_end)
        return 0xFFFF;

    uint32_t offset = freq_hz - bi->dl_freq_start;
    uint16_t arfcn = bi->arfcn_start + (uint16_t)(offset / 200000);
    return arfcn;
}

int kal_band_channel_count(kal_band_t band)
{
    if (band >= KAL_BAND_COUNT) return 0;
    if (kal_band_is_lte(band)) {
        /* LTE: number of 1 MHz steps across the band */
        const kal_band_info_t *bi = &bands[band];
        return (int)((bi->dl_freq_end - bi->dl_freq_start) / 1000000);
    }
    return bands[band].arfcn_end - bands[band].arfcn_start + 1;
}

bool kal_band_is_lte(kal_band_t band)
{
    return band >= KAL_BAND_LTE_B1 && band < KAL_BAND_COUNT;
}

uint32_t kal_earfcn_to_freq(uint32_t earfcn)
{
    /* 3GPP TS 36.101 Table 5.7.3-1: F_DL = F_DL_low + 0.1 * (N_DL - N_offs_DL) */
    if (earfcn <= 599)                        /* Band 1 */
        return 2110000000u + earfcn * 100000u;
    if (earfcn >= 600 && earfcn <= 1199)      /* Band 5 (DL: 869-894 MHz) */
        return 869000000u + (earfcn - 600) * 100000u;
    if (earfcn >= 1200 && earfcn <= 1949)     /* Band 3 */
        return 1805000000u + (earfcn - 1200) * 100000u;
    if (earfcn >= 2750 && earfcn <= 3449)     /* Band 7 */
        return 2620000000u + (earfcn - 2750) * 100000u;
    if (earfcn >= 6150 && earfcn <= 6449)     /* Band 20 */
        return 791000000u + (earfcn - 6150) * 100000u;
    if (earfcn >= 9210 && earfcn <= 9659)     /* Band 28 */
        return 758000000u + (earfcn - 9210) * 100000u;
    return 0;
}

/* ──────────────────────── IQ Capture ──────────────────────── */

void kal_push_samples(const uint8_t *iq_data, uint32_t len)
{
    if (!s_kal.capture_active || !s_kal.capture_buf) return;

    uint32_t pos = s_kal.capture_pos;
    uint32_t remaining = CAPTURE_BUF_SIZE - pos;
    if (remaining == 0) return;

    uint32_t copy = (len < remaining) ? len : remaining;
    memcpy(s_kal.capture_buf + pos, iq_data, copy);
    s_kal.capture_pos = pos + copy;

    if (s_kal.capture_pos >= CAPTURE_BUF_SIZE) {
        s_kal.capture_active = false;
        xSemaphoreGive(s_kal.capture_ready);
    }
}

/* Start capturing IQ into buffer, returns when full or timeout */
static esp_err_t capture_iq(uint32_t timeout_ms)
{
    /* Drain any stale semaphore token from a prior timeout race */
    xSemaphoreTake(s_kal.capture_ready, 0);

    s_kal.capture_pos = 0;
    __asm__ volatile("" ::: "memory");  /* Compiler barrier: pos=0 visible before active=true */
    s_kal.capture_active = true;

    if (xSemaphoreTake(s_kal.capture_ready, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        s_kal.capture_active = false;
        ESP_LOGW(TAG, "IQ capture timeout (%lu ms)", (unsigned long)timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/* ──────────────────────── DSP: Wideband Power Scan ──────────────────────── */

/*
 * Compute power spectrum from uint8 IQ data using a simple FFT.
 * Output: float power per bin in dB (relative to full scale).
 *
 * We use a straightforward DFT approach: convert to float, apply Hann
 * window, compute magnitude^2 per bin. For the initial version we use
 * a brute-force approach (Goertzel at target bins) rather than a full
 * FFT library call, since we only need power at 5 specific frequencies.
 */

/* Goertzel single-bin DFT — returns magnitude^2 at the target bin */
static float goertzel_mag2(const float *re, const float *im, int len,
                           float target_bin_frac)
{
    float w = TWO_PI * target_bin_frac / (float)len;
    float coeff = 2.0f * cosf(w);
    float s0 = 0, s1 = 0, s2 = 0;
    float s0i = 0, s1i = 0, s2i = 0;

    for (int i = 0; i < len; i++) {
        s0 = re[i] + coeff * s1 - s2;
        s2 = s1; s1 = s0;
        s0i = im[i] + coeff * s1i - s2i;
        s2i = s1i; s1i = s0i;
    }

    /* Magnitude^2 of complex result */
    float re_out = s1 - s2 * cosf(w);
    float im_out = s2 * sinf(w);
    float re_outi = s1i - s2i * cosf(w);
    float im_outi = s2i * sinf(w);

    /* Combined complex Goertzel: real_total = re_out - im_outi, im_total = im_out + re_outi */
    float rr = re_out - im_outi;
    float ii = im_out + re_outi;
    return rr * rr + ii * ii;
}

/*
 * Estimate power at 5 sub-channel offsets within 1 MHz capture.
 * Uses Goertzel at each sub-channel center ± averaging over a 200 kHz window.
 *
 * Returns power_db[5] — relative power in dB per sub-channel.
 * Also computes noise_floor_db as the minimum sub-channel power.
 */
static void power_scan_step(const uint8_t *iq_buf, uint32_t iq_len,
                            float *power_db, float *noise_floor_db)
{
    /* Convert first FFT_SIZE samples to float IQ using pre-allocated buffers */
    int n = FFT_SIZE;
    if ((int)(iq_len / 2) < n) n = iq_len / 2;

    float *re = s_kal.goertzel_re;
    float *im = s_kal.goertzel_im;
    if (!re || !im) {
        for (int i = 0; i < NUM_SUBCHANS; i++) power_db[i] = -120.0f;
        *noise_floor_db = -120.0f;
        return;
    }

    /* Convert uint8 IQ to float [-1, +1] with Hann window */
    for (int i = 0; i < n; i++) {
        float win = 0.5f - 0.5f * cosf(TWO_PI * (float)i / (float)n);
        re[i] = ((float)iq_buf[2 * i] - 127.5f) / 127.5f * win;
        im[i] = ((float)iq_buf[2 * i + 1] - 127.5f) / 127.5f * win;
    }

    /* Compute power at each sub-channel center using Goertzel.
     * FFT bin for offset f_offset: bin = f_offset / (sample_rate / N) + N/2
     * Since Goertzel works with "bin fraction", we need:
     *   target_bin = (f_offset / sample_rate) * N    (for positive frequencies)
     * But for negative frequencies, add N to wrap.
     */
    float min_power = 999.0f;
    for (int ch = 0; ch < NUM_SUBCHANS; ch++) {
        float f_offset = (float)subchan_offsets[ch];
        float bin_frac = f_offset / (float)GSM_SAMPLE_RATE * (float)n;
        if (bin_frac < 0) bin_frac += (float)n;

        /* Average Goertzel over 3 adjacent bins for robustness */
        float mag2_sum = 0;
        for (int db = -1; db <= 1; db++) {
            float b = bin_frac + (float)db;
            if (b < 0) b += (float)n;
            if (b >= (float)n) b -= (float)n;
            mag2_sum += goertzel_mag2(re, im, n, b);
        }
        mag2_sum /= 3.0f;

        /* Convert to dB (relative to full-scale) */
        float db_val = (mag2_sum > 1e-20f) ? 10.0f * log10f(mag2_sum) : -120.0f;
        power_db[ch] = db_val;
        if (db_val < min_power) min_power = db_val;
    }

    *noise_floor_db = min_power;
}

/* ──────────────────────── DSP: Narrowband FCCH Detection ──────────────────────── */

/*
 * Extract narrowband IQ from wideband capture at given offset.
 * Simple DDC: NCO mix → CIC decimate → float IQ output.
 *
 * Input:  uint8 IQ at 1.024 MSPS
 * Output: float IQ at sample_rate/decim_ratio
 *
 * decim_ratio=5 → 204.8 kSPS (for FCCH detection)
 * decim_ratio=3 → 341.3 kSPS (for GMSK demod at ~1.26 samples/symbol)
 */
static int ddc_extract_r(const uint8_t *iq_in, uint32_t in_samples,
                         int32_t offset_hz, int decim,
                         float *out_re, float *out_im, int max_out)
{
    float phase = 0;
    float phase_inc = TWO_PI * (float)offset_hz / (float)GSM_SAMPLE_RATE;
    float acc_re = 0, acc_im = 0;
    int acc_cnt = 0;
    int out_idx = 0;

    for (uint32_t i = 0; i < in_samples && out_idx < max_out; i++) {
        /* Convert to float */
        float iq_re = ((float)iq_in[2 * i] - 127.5f) / 127.5f;
        float iq_im = ((float)iq_in[2 * i + 1] - 127.5f) / 127.5f;

        /* NCO mix (frequency shift) */
        float nco_re = cosf(phase);
        float nco_im = -sinf(phase);  /* Negative for downconversion */
        float mix_re = iq_re * nco_re - iq_im * nco_im;
        float mix_im = iq_re * nco_im + iq_im * nco_re;

        phase += phase_inc;
        if (phase > TWO_PI) phase -= TWO_PI;
        if (phase < -TWO_PI) phase += TWO_PI;

        /* CIC accumulate */
        acc_re += mix_re;
        acc_im += mix_im;
        acc_cnt++;

        if (acc_cnt >= decim) {
            out_re[out_idx] = acc_re / (float)decim;
            out_im[out_idx] = acc_im / (float)decim;
            out_idx++;
            acc_re = 0; acc_im = 0; acc_cnt = 0;
        }
    }

    return out_idx;
}

/* Backward-compatible wrapper: decim=5 for FCCH (204.8 kSPS) */
static int ddc_extract(const uint8_t *iq_in, uint32_t in_samples,
                       int32_t offset_hz,
                       float *out_re, float *out_im, int max_out)
{
    return ddc_extract_r(iq_in, in_samples, offset_hz, 5, out_re, out_im, max_out);
}

/*
 * Detect FCCH bursts in narrowband IQ using FM discriminator variance.
 *
 * FCCH = pure sine at +67.708 kHz → FM demod shows constant instantaneous
 * frequency → very low variance in a sliding window.
 *
 * Returns: number of FCCH bursts detected.
 * Outputs: average measured FCCH frequency (Hz) across detected bursts.
 */
static int fcch_detect(const float *nb_re, const float *nb_im, int nb_len,
                       float nb_rate, float *avg_fcch_freq_hz)
{
    if (nb_len < 60) return 0;

    /* FM discriminator: instantaneous frequency from phase difference */
    int phase_len = nb_len - 1;
    float *inst_freq = malloc(phase_len * sizeof(float));
    if (!inst_freq) return 0;

    for (int i = 0; i < phase_len; i++) {
        /* Phase difference: arg(z[n] * conj(z[n-1])) */
        float dr = nb_re[i + 1] * nb_re[i] + nb_im[i + 1] * nb_im[i];
        float di = nb_im[i + 1] * nb_re[i] - nb_re[i + 1] * nb_im[i];
        float phase = atan2f(di, dr);
        /* Convert to Hz: freq = phase * sample_rate / (2*pi) */
        inst_freq[i] = phase * nb_rate / TWO_PI;
    }

    /* Noise-adaptive two-pass FCCH detection.
     *
     * Pass 1: Compute overall variance of FM discriminator output.
     *         This captures the noise floor — AWGN produces high variance,
     *         while FCCH (pure tone) produces very low variance.
     *
     * Pass 2: Find windows where variance < 10% of overall variance
     *         AND mean frequency is near 67708 Hz.
     *
     * This is noise-adaptive: works at any SNR because FCCH windows
     * always have much lower variance than noise/modulated windows. */
    int window = 80;  /* ~0.39 ms at 204.8 kSPS — captures most of FCCH burst */
    float expected_freq = FCCH_FREQ_HZ;
    float freq_tolerance = expected_freq * 0.6f;  /* 60% tolerance for freq check */

    if (phase_len < window) {
        free(inst_freq);
        return 0;
    }

    /* Pass 1: Compute overall variance across the entire capture */
    float global_sum = 0, global_sum2 = 0;
    for (int i = 0; i < phase_len; i++) {
        global_sum += inst_freq[i];
        global_sum2 += inst_freq[i] * inst_freq[i];
    }
    float global_var = global_sum2 / (float)phase_len -
                       (global_sum / (float)phase_len) * (global_sum / (float)phase_len);

    /* Adaptive threshold: FCCH windows should have < 10% of overall variance.
     * Also enforce a minimum floor to avoid false positives on dead air. */
    float var_threshold = global_var * 0.10f;
    float var_floor = 1e4f;  /* Minimum variance (100 Hz stddev) — below this is DC/dead */
    if (var_threshold < var_floor) var_threshold = var_floor;

    /* Pass 2: Find FCCH bursts — low variance + correct mean frequency */
    int burst_count = 0;
    float freq_sum = 0;
    int cooldown = 0;

    /* Diagnostics: track best candidate window */
    float best_var = 1e30f;
    float best_mean = 0;
    float best_var_near = 1e30f;  /* Best variance among windows with mean near FCCH freq */
    float best_mean_near = 0;
    int windows_checked = 0;

    for (int i = 0; i <= phase_len - window; i++) {
        if (cooldown > 0) { cooldown--; continue; }

        float sum = 0, sum2 = 0;
        for (int j = 0; j < window; j++) {
            float f = inst_freq[i + j];
            sum += f;
            sum2 += f * f;
        }
        float mean = sum / (float)window;
        float variance = sum2 / (float)window - mean * mean;
        windows_checked++;

        /* Track best overall window */
        if (variance < best_var) { best_var = variance; best_mean = mean; }

        /* Track best window near expected FCCH frequency */
        if (fabsf(mean - expected_freq) < freq_tolerance && variance < best_var_near) {
            best_var_near = variance;
            best_mean_near = mean;
        }

        if (variance > var_floor && variance < var_threshold &&
            fabsf(mean - expected_freq) < freq_tolerance) {
            burst_count++;
            freq_sum += mean;
            cooldown = window * 3;  /* Skip past this burst + guard */
        }
    }

    ESP_LOGD(TAG, "FCCH: gvar=%.0f thr=%.0f best=%.0f@%.0fHz near=%.0f@%.0fHz w=%d",
             global_var, var_threshold, best_var, best_mean,
             best_var_near, best_mean_near, windows_checked);

    free(inst_freq);

    if (burst_count > 0) {
        *avg_fcch_freq_hz = freq_sum / (float)burst_count;
    } else {
        *avg_fcch_freq_hz = 0;
    }

    return burst_count;
}

/* ──────────────────────── Scan Task ──────────────────────── */

static void scan_task(void *arg)
{
    kal_result_t *res = &s_kal.result;
    const kal_config_t *cfg = &s_kal.config;
    const kal_band_info_t *bi = &bands[res->band];

    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  GSM Kalibrate — Scanning %s             ║", bi->name);
    ESP_LOGI(TAG, "║  %lu – %lu MHz (%d channels)          ║",
             (unsigned long)(bi->dl_freq_start / 1000000),
             (unsigned long)(bi->dl_freq_end / 1000000),
             kal_band_channel_count(res->band));
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");

    /* Configure RTL-SDR for scanning */
    bool is_lte = kal_band_is_lte(res->band);
    uint32_t sample_rate = is_lte ? LTE_SAMPLE_RATE : GSM_SAMPLE_RATE;
    rtlsdr_set_sample_rate(cfg->dev, sample_rate);
    if (cfg->gain == 0) {
        rtlsdr_set_tuner_gain_mode(cfg->dev, 0);  /* AGC */
    } else {
        rtlsdr_set_tuner_gain_mode(cfg->dev, 1);
        rtlsdr_set_tuner_gain(cfg->dev, cfg->gain);
    }

    /* Calculate scan steps: 1 MHz per step, centered in band */
    uint32_t band_start = bi->dl_freq_start;
    uint32_t band_end = bi->dl_freq_end;
    uint32_t step_bw = 1000000;  /* 1 MHz per step */

    /* First step center: align to include first channel */
    uint32_t step_center = band_start - 100000 + step_bw / 2;
    int total_steps = 0;
    uint32_t step_freqs[MAX_STEPS];

    while (step_center - step_bw / 2 < band_end && total_steps < MAX_STEPS) {
        step_freqs[total_steps++] = step_center;
        step_center += step_bw;
    }

    res->total_steps = total_steps;
    res->channel_count = 0;
    res->fcch_count = 0;
    res->state = KAL_STATE_SCANNING;

    ESP_LOGI(TAG, "Scan plan: %d steps of %lu kHz across %.1f MHz band",
             total_steps, (unsigned long)(step_bw / 1000),
             (float)(band_end - band_start) / 1e6f);

    /* ── Stage 1: Wideband power scan ── */
    ESP_LOGI(TAG, "── Stage 1: Wideband power scan ──");

    typedef struct {
        uint32_t freq_hz;
        uint16_t arfcn;
        float    power_dbfs;
        int32_t  ddc_offset_hz;
        uint32_t step_center;
    } candidate_t;

    candidate_t *candidates = malloc(total_steps * NUM_SUBCHANS * sizeof(candidate_t));
    int cand_count = 0;

    if (!candidates) {
        ESP_LOGE(TAG, "Failed to allocate candidate buffer");
        res->state = KAL_STATE_ERROR;
        snprintf(res->status_msg, sizeof(res->status_msg), "Out of memory");
        goto done;
    }

    for (int step = 0; step < total_steps && !s_kal.scan_abort; step++) {
        res->current_step = step + 1;
        uint32_t center = step_freqs[step];

        /* Tune SDR */
        rtlsdr_set_center_freq(cfg->dev, center);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

        /* Capture IQ */
        esp_err_t ret = capture_iq(cfg->dwell_ms + 500);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Step %d/%d: capture failed, skipping", step + 1, total_steps);
            continue;
        }

        /* Compute power at 5 sub-channels */
        float power_db[NUM_SUBCHANS];
        float noise_floor;
        power_scan_step(s_kal.capture_buf, s_kal.capture_pos, power_db, &noise_floor);

        /* Check each sub-channel */
        for (int ch = 0; ch < NUM_SUBCHANS; ch++) {
            float excess = power_db[ch] - noise_floor;
            if (excess < cfg->threshold_db) continue;

            uint32_t chan_freq = center + subchan_offsets[ch];
            uint16_t arfcn = 0;

            if (is_lte) {
                /* LTE: use direct frequency, no ARFCN grid snapping */
                /* Check for duplicates by frequency (within 100 kHz) */
                bool dup = false;
                for (int c = 0; c < cand_count; c++) {
                    int32_t diff = (int32_t)candidates[c].freq_hz - (int32_t)chan_freq;
                    if (abs(diff) < 100000) { dup = true; break; }
                }
                if (dup) continue;
            } else {
                /* GSM: map to ARFCN grid */
                arfcn = kal_freq_to_arfcn(chan_freq, res->band);
                if (arfcn == 0xFFFF) continue;
                chan_freq = kal_arfcn_to_freq_band(arfcn, res->band);

                bool dup = false;
                for (int c = 0; c < cand_count; c++) {
                    if (candidates[c].arfcn == arfcn) { dup = true; break; }
                }
                if (dup) continue;
            }

            candidates[cand_count++] = (candidate_t){
                .freq_hz = chan_freq,
                .arfcn = arfcn,
                .power_dbfs = power_db[ch],
                .ddc_offset_hz = subchan_offsets[ch],
                .step_center = center,
            };

            if (is_lte) {
                ESP_LOGI(TAG, "  [%d/%d] %lu.%01lu MHz  power: %.1f dBFS (+%.1f dB)",
                         step + 1, total_steps,
                         (unsigned long)(chan_freq / 1000000),
                         (unsigned long)((chan_freq % 1000000) / 100000),
                         power_db[ch], excess);
            } else {
                ESP_LOGI(TAG, "  [%d/%d] ARFCN %d (%lu.%01lu MHz)  power: %.1f dBFS (+%.1f dB)",
                         step + 1, total_steps, arfcn,
                         (unsigned long)(chan_freq / 1000000),
                         (unsigned long)((chan_freq % 1000000) / 100000),
                         power_db[ch], excess);
            }
        }
    }

    if (s_kal.scan_abort) {
        ESP_LOGW(TAG, "Scan aborted by user");
        res->state = KAL_STATE_ERROR;
        snprintf(res->status_msg, sizeof(res->status_msg), "Scan aborted");
        goto cleanup;
    }

    ESP_LOGI(TAG, "Stage 1 complete: %d candidate channels found", cand_count);

    if (cand_count == 0) {
        ESP_LOGW(TAG, "No channels above threshold (%.1f dB). Try lower threshold or different band.",
                 cfg->threshold_db);
        res->state = KAL_STATE_COMPLETE;
        snprintf(res->status_msg, sizeof(res->status_msg),
                 "No channels found in %s", bi->name);
        goto cleanup;
    }

    /* Sort candidates by power (strongest first) */
    for (int i = 0; i < cand_count - 1; i++) {
        for (int j = i + 1; j < cand_count; j++) {
            if (candidates[j].power_dbfs > candidates[i].power_dbfs) {
                candidate_t tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }
        }
    }

    /* ── Stage 2: Signal validation (FCCH for GSM, PSS for LTE) ── */
    float ppm_sum = 0, ppm_sum2 = 0;

    if (is_lte) {
        /* ── LTE: FFT-domain PSS/SSS cell detection per candidate ── */
        ESP_LOGI(TAG, "── Stage 2: LTE PSS/SSS detection (%d candidates) ──", cand_count);

        if (!s_kal.lte_ctx) {
            ESP_LOGE(TAG, "LTE sync engine not initialized");
            res->state = KAL_STATE_ERROR;
            snprintf(res->status_msg, sizeof(res->status_msg), "LTE sync not init");
            goto cleanup;
        }

        for (int c = 0; c < cand_count && !s_kal.scan_abort; c++) {
            candidate_t *cand = &candidates[c];

            /* Tune to candidate frequency directly (LTE uses wideband PSS) */
            rtlsdr_set_center_freq(cfg->dev, cand->freq_hz);
            vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

            /* Capture IQ */
            esp_err_t ret = capture_iq(cfg->dwell_ms + 500);
            if (ret != ESP_OK) continue;

            /* Run FFT-domain PSS+SSS cell search */
            lte_cell_t cells[2];
            int n_cells = lte_sync_detect(s_kal.lte_ctx,
                                          s_kal.capture_buf, s_kal.capture_pos,
                                          cand->freq_hz, cells, 2);

            /* Yield to prevent WDT on long scans */
            vTaskDelay(1);

            if (n_cells == 0) {
                /* No cell found — still record the candidate */
                if (res->channel_count < KAL_MAX_CHANNELS) {
                    kal_channel_t *ch = &res->channels[res->channel_count];
                    ch->arfcn = 0;
                    ch->freq_hz = cand->freq_hz;
                    ch->power_dbfs = cand->power_dbfs;
                    ch->n_id_2 = 0;
                    ch->pci = 0xFFFF;
                    ch->confidence = 0;
                    ch->fcch_detected = false;
                    ch->burst_count = 0;
                    ch->freq_error_hz = 0;
                    ch->ppm = 0;

                    ESP_LOGI(TAG, "  %lu.%01lu MHz  PSS: no   power: %.1f dBFS",
                             (unsigned long)(ch->freq_hz / 1000000),
                             (unsigned long)((ch->freq_hz % 1000000) / 100000),
                             ch->power_dbfs);

                    res->channel_count++;
                }
                continue;
            }

            /* Store detected cells */
            for (int ci = 0; ci < n_cells && res->channel_count < KAL_MAX_CHANNELS; ci++) {
                lte_cell_t *cell = &cells[ci];
                kal_channel_t *ch = &res->channels[res->channel_count];
                ch->arfcn = 0;
                ch->freq_hz = cand->freq_hz;
                ch->power_dbfs = cand->power_dbfs;
                ch->n_id_2 = cell->n_id_2;
                ch->pci = cell->pci;
                ch->confidence = cell->pss_power;

                bool pss_found = (cell->pss_power > 1.0f);
                ch->fcch_detected = pss_found;
                ch->burst_count = pss_found ? 1 : 0;

                if (pss_found) {
                    ch->freq_error_hz = cell->freq_error_hz;
                    ch->ppm = cell->ppm;

                    ppm_sum += ch->ppm;
                    ppm_sum2 += ch->ppm * ch->ppm;
                    res->fcch_count++;

                    if (cell->sss_valid) {
                        ESP_LOGI(TAG, "  %lu.%01lu MHz  PCI=%u (N_ID_1=%u N_ID_2=%u)  "
                                 "freq_err: %+.1f Hz  ppm: %+.2f  mag: %.0f  sf=%d",
                                 (unsigned long)(ch->freq_hz / 1000000),
                                 (unsigned long)((ch->freq_hz % 1000000) / 100000),
                                 cell->pci, cell->n_id_1, cell->n_id_2,
                                 ch->freq_error_hz, ch->ppm,
                                 cell->pss_power, cell->subframe);
                    } else {
                        ESP_LOGI(TAG, "  %lu.%01lu MHz  PSS: YES  N_ID_2=%d  "
                                 "freq_err: %+.1f Hz  ppm: %+.2f  mag: %.0f  (no SSS)",
                                 (unsigned long)(ch->freq_hz / 1000000),
                                 (unsigned long)((ch->freq_hz % 1000000) / 100000),
                                 ch->n_id_2, ch->freq_error_hz, ch->ppm,
                                 cell->pss_power);
                    }
                } else {
                    ch->freq_error_hz = 0;
                    ch->ppm = 0;

                    ESP_LOGI(TAG, "  %lu.%01lu MHz  PSS: weak  power: %.1f dBFS  mag: %.0f",
                             (unsigned long)(ch->freq_hz / 1000000),
                             (unsigned long)((ch->freq_hz % 1000000) / 100000),
                             ch->power_dbfs, cell->pss_power);
                }

                res->channel_count++;
            }
        }

    } else {
        /* ── GSM: FCCH validation on candidates ── */
        ESP_LOGI(TAG, "── Stage 2: FCCH validation (%d candidates) ──", cand_count);

        int nb_buf_size = GSM_SAMPLE_RATE / 5;  /* 204800 samples at 204.8 kSPS */
        float *nb_re = malloc(nb_buf_size * sizeof(float));
        float *nb_im = malloc(nb_buf_size * sizeof(float));

        if (!nb_re || !nb_im) {
            ESP_LOGE(TAG, "Failed to allocate narrowband buffers");
            free(nb_re); free(nb_im);
            res->state = KAL_STATE_ERROR;
            snprintf(res->status_msg, sizeof(res->status_msg), "Out of memory");
            goto cleanup;
        }

        for (int c = 0; c < cand_count && !s_kal.scan_abort; c++) {
            candidate_t *cand = &candidates[c];

            rtlsdr_set_center_freq(cfg->dev, cand->step_center);
            vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

            int fcch_total_bursts = 0;
            float fcch_freq_sum = 0;

            int dwells = (cfg->fcch_bursts + 3) / 4;
            if (dwells < 1) dwells = 1;
            if (dwells > 4) dwells = 4;

            for (int d = 0; d < dwells && !s_kal.scan_abort; d++) {
                esp_err_t ret = capture_iq(cfg->dwell_ms + 500);
                if (ret != ESP_OK) continue;

                int nb_len = ddc_extract(s_kal.capture_buf,
                                         s_kal.capture_pos / 2,
                                         cand->ddc_offset_hz,
                                         nb_re, nb_im, nb_buf_size);

                if (nb_len < 100) continue;

                float avg_freq;
                float nb_rate = (float)GSM_SAMPLE_RATE / 5.0f;
                int bursts = fcch_detect(nb_re, nb_im, nb_len, nb_rate, &avg_freq);

                if (bursts > 0) {
                    fcch_total_bursts += bursts;
                    fcch_freq_sum += avg_freq * (float)bursts;
                }
            }

            if (res->channel_count < KAL_MAX_CHANNELS) {
                kal_channel_t *ch = &res->channels[res->channel_count];
                ch->arfcn = cand->arfcn;
                ch->freq_hz = cand->freq_hz;
                ch->power_dbfs = cand->power_dbfs;
                ch->fcch_detected = (fcch_total_bursts > 0);
                ch->burst_count = fcch_total_bursts;

                if (fcch_total_bursts > 0) {
                    float measured_freq = fcch_freq_sum / (float)fcch_total_bursts;
                    ch->freq_error_hz = measured_freq - FCCH_FREQ_HZ;
                    ch->ppm = ch->freq_error_hz / (float)cand->freq_hz * 1e6f;

                    ppm_sum += ch->ppm;
                    ppm_sum2 += ch->ppm * ch->ppm;
                    res->fcch_count++;

                    ESP_LOGI(TAG, "  ARFCN %3d (%lu.%01lu MHz)  FCCH: YES  "
                             "bursts: %d  freq_err: %+.1f Hz  ppm: %+.2f",
                             ch->arfcn,
                             (unsigned long)(ch->freq_hz / 1000000),
                             (unsigned long)((ch->freq_hz % 1000000) / 100000),
                             ch->burst_count, ch->freq_error_hz, ch->ppm);
                } else {
                    ch->freq_error_hz = 0;
                    ch->ppm = 0;

                    ESP_LOGI(TAG, "  ARFCN %3d (%lu.%01lu MHz)  FCCH: no   power: %.1f dBFS",
                             ch->arfcn,
                             (unsigned long)(ch->freq_hz / 1000000),
                             (unsigned long)((ch->freq_hz % 1000000) / 100000),
                             ch->power_dbfs);
                }

                res->channel_count++;
            }
        }

        /* ── Stage 3: Cell Identity Decode (GSM only) ── */
        ESP_LOGI(TAG, "── Stage 3: Cell identity decode (%d FCCH channels) ──", res->fcch_count);

        /* DDC at decim=3 → 341.3 kSPS (1.26 samples per GSM symbol at 270.833 kbaud).
         * This is close enough for differential GMSK demod with soft decisions.
         * The GSM symbol rate is 13e6/48 = 270833.33 Hz. */
        #define GSM_DDC_DECIM    3
        #define GSM_DDC_RATE     (GSM_SAMPLE_RATE / GSM_DDC_DECIM)  /* 341333 Hz */
        #define GSM_SYMBOL_RATE  270833

        int s3_buf_size = GSM_DDC_RATE / 3;  /* ~113k samples for ~330ms */
        float *s3_re = malloc(s3_buf_size * sizeof(float));
        float *s3_im = malloc(s3_buf_size * sizeof(float));
        if (!s3_re || !s3_im) {
            ESP_LOGW(TAG, "Stage 3: alloc failed, skipping cell ID decode");
            free(s3_re); free(s3_im);
            goto skip_stage3;
        }

        /* SCH training sequence for burst timing correlation */
        static const int8_t sch_train_bipolar[64] = {
            1,-1,1,1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,1,-1,-1,
            1,-1,1,-1,-1,-1,1,-1,1,-1,-1,-1,1,1,-1,1,-1,-1,1,1,-1,1,-1,-1,1,-1,1,-1,-1,1,-1,-1
        };

        for (int i = 0; i < res->channel_count && !s_kal.scan_abort; i++) {
            kal_channel_t *ch = &res->channels[i];
            if (!ch->fcch_detected || ch->burst_count == 0) continue;

            candidate_t *cand = NULL;
            for (int c = 0; c < cand_count; c++) {
                if (candidates[c].arfcn == ch->arfcn) { cand = &candidates[c]; break; }
            }
            if (!cand) continue;

            ESP_LOGI(TAG, "  Decoding ARFCN %d (%lu.%01lu MHz)...",
                     ch->arfcn,
                     (unsigned long)(ch->freq_hz / 1000000),
                     (unsigned long)((ch->freq_hz % 1000000) / 100000));

            /* Retune and capture 200ms */
            rtlsdr_set_center_freq(cfg->dev, cand->step_center);
            vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

            esp_err_t ret = capture_iq(cfg->dwell_ms + 500);
            if (ret != ESP_OK) { ESP_LOGW(TAG, "  Capture timeout"); continue; }

            /* DDC with decim=3 for GMSK-rate output (~341.3 kSPS) */
            int nb_len = ddc_extract_r(s_kal.capture_buf, s_kal.capture_pos / 2,
                                       cand->ddc_offset_hz, GSM_DDC_DECIM,
                                       s3_re, s3_im, s3_buf_size);
            if (nb_len < 1000) { ESP_LOGW(TAG, "  DDC too short"); continue; }

            /* GMSK differential demod → soft bits */
            int8_t *soft_bits = malloc(nb_len * sizeof(int8_t));
            if (!soft_bits) continue;
            int n_bits = gsm_gmsk_demod(s3_re, s3_im, nb_len, soft_bits, nb_len);

            /* At 341.3 kSPS and 270.833 kbaud, we get ~1.26 samples per symbol.
             * The GMSK demod produces one soft bit per sample, so n_bits ≈ nb_len.
             * One GSM burst = 148 symbols ≈ 187 samples at 1.26 samp/sym.
             * For burst searching, use the oversampled bit stream directly. */
            float samp_per_sym = (float)GSM_DDC_RATE / (float)GSM_SYMBOL_RATE;  /* ~1.26 */
            int burst_samples = (int)(148.0f * samp_per_sym + 0.5f);  /* ~187 */

            ESP_LOGI(TAG, "  DDC: %d samp → %d bits, %.2f samp/sym, burst=%d samp",
                     nb_len, n_bits, samp_per_sym, burst_samples);

            /* ── SCH burst timing: correlate training sequence ──
             * The SCH training sequence is at offset 42 symbols into the burst.
             * At ~1.26 samp/sym, that's sample offset ~53.
             * Search the entire capture for the best match. */
            int train_offset_sym = 42;  /* Training seq starts at symbol 42 in SCH burst */
            int train_len = 64;         /* Training sequence length in symbols */
            float best_corr = 0;
            int best_pos = -1;

            for (int pos = 0; pos + (int)(train_len * samp_per_sym) < n_bits; pos += 10) {
                float corr = 0;
                for (int k = 0; k < train_len; k++) {
                    int samp_idx = pos + (int)(k * samp_per_sym + 0.5f);
                    if (samp_idx >= n_bits) break;
                    corr += (float)soft_bits[samp_idx] * (float)sch_train_bipolar[k];
                }
                float abs_corr = (corr > 0) ? corr : -corr;
                if (abs_corr > best_corr) {
                    best_corr = abs_corr;
                    best_pos = pos;
                }
            }

            /* The training starts at symbol 42, so burst start is pos - 42*samp_per_sym */
            gsm_sch_info_t sch_info;
            memset(&sch_info, 0, sizeof(sch_info));
            bool sch_found = false;

            if (best_pos >= 0 && best_corr > 500.0f) {
                int burst_start = best_pos - (int)(train_offset_sym * samp_per_sym + 0.5f);
                if (burst_start < 0) burst_start = 0;

                ESP_LOGI(TAG, "  SCH train corr: %.0f at pos %d (burst_start=%d)",
                         best_corr, best_pos, burst_start);

                /* Resample burst to exactly 148 symbols */
                int8_t burst_148[148];
                for (int s = 0; s < 148; s++) {
                    int idx = burst_start + (int)(s * samp_per_sym + 0.5f);
                    burst_148[s] = (idx < n_bits) ? soft_bits[idx] : 0;
                }

                /* Try SCH decode on the resampled burst */
                if (gsm_sch_decode(burst_148, &sch_info)) {
                    ch->cell_id.bsic = sch_info.bsic;
                    sch_found = true;
                    ESP_LOGI(TAG, "  SCH: BSIC=%d (NCC=%d BCC=%d) FN=%lu",
                             sch_info.bsic, sch_info.ncc, sch_info.bcc,
                             (unsigned long)sch_info.fn);
                }
            } else {
                ESP_LOGD(TAG, "  SCH training corr too low (%.0f)", best_corr);
            }

            /* ── BCCH normal burst decode ──
             * Normal bursts repeat every 156.25 symbols ≈ 197 samples.
             * Search from each detected burst timing. */
            bool si3_found = false;
            int norm_burst_period = (int)(156.25f * samp_per_sym + 0.5f);  /* ~197 samp */

            /* Try at multiple starting offsets around the first strong burst */
            for (int start = 0; start + norm_burst_period * 4 < n_bits && !si3_found; start += norm_burst_period) {
                int8_t bcch_bursts[4][116];
                bool valid = true;

                for (int b = 0; b < 4 && valid; b++) {
                    int bstart = start + b * norm_burst_period;

                    /* Resample 148 symbols from this burst position */
                    int8_t burst_148[148];
                    for (int s = 0; s < 148; s++) {
                        int idx = bstart + (int)(s * samp_per_sym + 0.5f);
                        if (idx >= n_bits) { valid = false; break; }
                        burst_148[s] = soft_bits[idx];
                    }
                    if (!valid) break;

                    /* Extract 116 data bits: [3 tail][57 data][26 train][57 data][3 tail][guard] */
                    memcpy(&bcch_bursts[b][0], &burst_148[3], 57);
                    memcpy(&bcch_bursts[b][57], &burst_148[3 + 57 + 26], 57);
                }
                if (!valid) continue;

                uint8_t l2_frame[GSM_BCCH_BLOCK_LEN];
                if (gsm_bcch_decode(bcch_bursts, l2_frame)) {
                    gsm_cell_id_t cid;
                    if (gsm_parse_si3(l2_frame, &cid)) {
                        ch->cell_id = cid;
                        ch->cell_id.bsic = sch_info.bsic;
                        ch->cell_id.valid = true;
                        si3_found = true;
                        ESP_LOGI(TAG, "  *** CELL: MCC=%d MNC=%d LAC=%d CID=%d BSIC=%d ***",
                                 cid.mcc, cid.mnc, cid.lac, cid.cell_id, sch_info.bsic);
                    }
                }
            }

            if (!sch_found && !si3_found) {
                ESP_LOGI(TAG, "  No SCH/BCCH decoded");
            }

            free(soft_bits);
            vTaskDelay(1);
        }

        free(s3_re);
        free(s3_im);
skip_stage3:

        free(nb_re);
        free(nb_im);
    } /* end GSM/LTE branch */

    /* ── Compute summary statistics ── */
    if (res->fcch_count > 0) {
        res->avg_ppm = ppm_sum / (float)res->fcch_count;
        float var = ppm_sum2 / (float)res->fcch_count - res->avg_ppm * res->avg_ppm;
        res->stddev_ppm = (var > 0) ? sqrtf(var) : 0;
    }

    res->state = KAL_STATE_COMPLETE;
    snprintf(res->status_msg, sizeof(res->status_msg), "Scan complete");

    /* Print summary */
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Scan Complete — %s                      ║", bi->name);
    ESP_LOGI(TAG, "╠══════════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║  Channels found:  %d                         ║", res->channel_count);
    ESP_LOGI(TAG, "║  FCCH confirmed:  %d                         ║", res->fcch_count);
    if (res->fcch_count > 0) {
        ESP_LOGI(TAG, "║  Average PPM:     %+.2f                    ║", res->avg_ppm);
        ESP_LOGI(TAG, "║  Std deviation:   %.2f                     ║", res->stddev_ppm);
        ESP_LOGI(TAG, "╠══════════════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Use: rtlsdr_set_freq_correction(dev, %d)  ║",
                 (int)roundf(res->avg_ppm));
    } else {
        ESP_LOGW(TAG, "║  No FCCH bursts detected — cannot calibrate ║");
    }
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");

    /* Print decoded cell identities */
    for (int i = 0; i < res->channel_count; i++) {
        const kal_channel_t *ch = &res->channels[i];
        if (ch->cell_id.valid) {
            ESP_LOGI(TAG, "  ARFCN %3d: MCC=%d MNC=%d LAC=%d CellID=%d BSIC=%d",
                     ch->arfcn, ch->cell_id.mcc, ch->cell_id.mnc,
                     ch->cell_id.lac, ch->cell_id.cell_id, ch->cell_id.bsic);
        }
    }

cleanup:
    free(candidates);
done:
    s_kal.scan_task = NULL;
    vTaskDelete(NULL);
}

/* ──────────────────────── Public API ──────────────────────── */

esp_err_t kal_init(const kal_config_t *config)
{
    if (s_kal.initialized) return ESP_ERR_INVALID_STATE;
    if (!config || !config->dev) return ESP_ERR_INVALID_ARG;

    memset(&s_kal, 0, sizeof(s_kal));
    s_kal.config = *config;

    /* Allocate capture buffer in PSRAM if available */
    s_kal.capture_buf = heap_caps_malloc(CAPTURE_BUF_SIZE,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_kal.capture_buf) {
        s_kal.capture_buf = malloc(CAPTURE_BUF_SIZE);
    }
    if (!s_kal.capture_buf) {
        ESP_LOGE(TAG, "Failed to allocate capture buffer (%d bytes)", CAPTURE_BUF_SIZE);
        return ESP_ERR_NO_MEM;
    }

    /* Pre-allocate Goertzel work buffers (avoids per-step malloc in scan loop) */
    s_kal.goertzel_re = malloc(FFT_SIZE * sizeof(float));
    s_kal.goertzel_im = malloc(FFT_SIZE * sizeof(float));
    if (!s_kal.goertzel_re || !s_kal.goertzel_im) {
        ESP_LOGE(TAG, "Failed to allocate Goertzel buffers");
        free(s_kal.capture_buf);
        free(s_kal.goertzel_re);
        free(s_kal.goertzel_im);
        return ESP_ERR_NO_MEM;
    }

    s_kal.capture_ready = xSemaphoreCreateBinary();
    if (!s_kal.capture_ready) {
        free(s_kal.capture_buf);
        return ESP_ERR_NO_MEM;
    }

    /* Start HTTP server for API */
    httpd_config_t httpd_config = HTTPD_DEFAULT_CONFIG();
    httpd_config.server_port = 8085;
    httpd_config.max_uri_handlers = 8;
    httpd_config.stack_size = 8192;
    httpd_config.lru_purge_enable = true;

    esp_err_t ret = httpd_start(&s_kal.httpd, &httpd_config);
    if (ret == ESP_OK) {
        s_kal.httpd_own = true;
        kal_register_http_handlers(s_kal.httpd);
        ESP_LOGI(TAG, "Kalibrate HTTP server on port %d", httpd_config.server_port);
    } else {
        ESP_LOGW(TAG, "HTTP server start failed: %s (API unavailable)", esp_err_to_name(ret));
    }

    /* Initialize LTE sync engine (PSS+SSS, FFT tables) */
    s_kal.lte_ctx = lte_sync_init();
    if (!s_kal.lte_ctx) {
        ESP_LOGW(TAG, "LTE sync init failed — LTE bands will not work");
    }

    s_kal.result.state = KAL_STATE_IDLE;
    s_kal.initialized = true;

    ESP_LOGI(TAG, "Kalibrate initialized (capture buf: %d KB, GSM+LTE)",
             CAPTURE_BUF_SIZE / 1024);
    return ESP_OK;
}

esp_err_t kal_scan_start(kal_band_t band)
{
    if (!s_kal.initialized) return ESP_ERR_INVALID_STATE;
    if (s_kal.scan_task) {
        ESP_LOGW(TAG, "Scan already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    if (band >= KAL_BAND_COUNT) return ESP_ERR_INVALID_ARG;

    /* Reset results */
    memset(&s_kal.result, 0, sizeof(s_kal.result));
    s_kal.result.band = band;
    s_kal.result.state = KAL_STATE_IDLE;
    s_kal.scan_abort = false;

    /* Launch scan task on Core 1 (leave Core 0 for USB) */
    BaseType_t ret = xTaskCreatePinnedToCore(
        scan_task, "kal_scan", SCAN_TASK_STACK, NULL, SCAN_TASK_PRIO, &s_kal.scan_task, 1);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create scan task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t kal_scan_stop(void)
{
    if (!s_kal.scan_task) return ESP_OK;
    s_kal.scan_abort = true;
    /* Task will delete itself */
    return ESP_OK;
}

const kal_result_t *kal_get_result(void)
{
    return &s_kal.result;
}

/* ──────────────────────── HTTP API Handlers ──────────────────────── */

static esp_err_t http_get_results(httpd_req_t *req)
{
    const kal_result_t *res = &s_kal.result;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "band", bands[res->band].name);
    cJSON_AddStringToObject(root, "state",
        res->state == KAL_STATE_IDLE ? "idle" :
        res->state == KAL_STATE_SCANNING ? "scanning" :
        res->state == KAL_STATE_COMPLETE ? "complete" : "error");

    if (res->fcch_count > 0) {
        cJSON_AddNumberToObject(root, "avg_ppm", res->avg_ppm);
        cJSON_AddNumberToObject(root, "stddev_ppm", res->stddev_ppm);
        cJSON_AddNumberToObject(root, "ppm_correction", (int)roundf(res->avg_ppm));
    }

    cJSON *channels = cJSON_AddArrayToObject(root, "channels");
    for (int i = 0; i < res->channel_count; i++) {
        const kal_channel_t *ch = &res->channels[i];
        cJSON *jch = cJSON_CreateObject();
        cJSON_AddNumberToObject(jch, "arfcn", ch->arfcn);
        cJSON_AddNumberToObject(jch, "freq_mhz", (double)ch->freq_hz / 1e6);
        cJSON_AddNumberToObject(jch, "power_dbfs", ch->power_dbfs);
        cJSON_AddBoolToObject(jch, "fcch", ch->fcch_detected);
        if (ch->pci != 0 && ch->pci != 0xFFFF) {
            cJSON_AddNumberToObject(jch, "pci", ch->pci);
        }
        if (ch->fcch_detected) {
            cJSON_AddNumberToObject(jch, "freq_error_hz", ch->freq_error_hz);
            cJSON_AddNumberToObject(jch, "ppm", ch->ppm);
            cJSON_AddNumberToObject(jch, "bursts", ch->burst_count);
        }
        if (ch->cell_id.valid) {
            cJSON_AddNumberToObject(jch, "mcc", ch->cell_id.mcc);
            cJSON_AddNumberToObject(jch, "mnc", ch->cell_id.mnc);
            cJSON_AddNumberToObject(jch, "lac", ch->cell_id.lac);
            cJSON_AddNumberToObject(jch, "cell_id", ch->cell_id.cell_id);
            cJSON_AddNumberToObject(jch, "bsic", ch->cell_id.bsic);
        }
        cJSON_AddItemToArray(channels, jch);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json);
    free(json);
    return ESP_OK;
}

static esp_err_t http_get_status(httpd_req_t *req)
{
    const kal_result_t *res = &s_kal.result;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state",
        res->state == KAL_STATE_IDLE ? "idle" :
        res->state == KAL_STATE_SCANNING ? "scanning" :
        res->state == KAL_STATE_COMPLETE ? "complete" : "error");
    cJSON_AddNumberToObject(root, "current_step", res->current_step);
    cJSON_AddNumberToObject(root, "total_steps", res->total_steps);
    cJSON_AddNumberToObject(root, "channels_found", res->channel_count);
    cJSON_AddNumberToObject(root, "fcch_confirmed", res->fcch_count);
    cJSON_AddStringToObject(root, "status", res->status_msg);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json);
    free(json);
    return ESP_OK;
}

static esp_err_t http_post_scan(httpd_req_t *req)
{
    char buf[128] = {0};
    int received = httpd_req_recv(req, buf, sizeof(buf) - 1);

    kal_band_t band = s_kal.config.band;

    if (received > 0) {
        cJSON *root = cJSON_Parse(buf);
        if (root) {
            cJSON *jband = cJSON_GetObjectItem(root, "band");
            if (cJSON_IsString(jband)) {
                const char *name = jband->valuestring;
                for (int b = 0; b < KAL_BAND_COUNT; b++) {
                    if (strcasecmp(bands[b].name, name) == 0) {
                        band = (kal_band_t)b;
                        break;
                    }
                }
            }
            cJSON_Delete(root);
        }
    }

    esp_err_t ret = kal_scan_start(band);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", ret == ESP_OK);
    cJSON_AddStringToObject(root, "band", bands[band].name);
    if (ret != ESP_OK) {
        cJSON_AddStringToObject(root, "error", esp_err_to_name(ret));
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json);
    free(json);
    return ESP_OK;
}

esp_err_t kal_register_http_handlers(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/api/kalibrate",        .method = HTTP_GET,  .handler = http_get_results },
        { .uri = "/api/kalibrate/status",  .method = HTTP_GET,  .handler = http_get_status },
        { .uri = "/api/kalibrate/scan",    .method = HTTP_POST, .handler = http_post_scan },
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        esp_err_t ret = httpd_register_uri_handler(server, &uris[i]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register %s: %s", uris[i].uri, esp_err_to_name(ret));
        }
    }

    return ESP_OK;
}
