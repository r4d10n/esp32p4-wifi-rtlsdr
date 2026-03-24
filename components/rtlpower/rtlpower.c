#include "rtlpower.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*
 * rtl_power spectrum sweep engine.
 *
 * CSV output format (rtl_power compatible):
 *   date, time, Hz_low, Hz_high, Hz_step, samples, dBm, dBm, dBm, ...
 *
 * Spectrogram and cloud upload are left as future extensions.
 */

static const char *TAG = "rtlpower";

/* Default RTL-SDR sample rate used for FFT size calculation */
#define RTLPOWER_SAMPLE_RATE 250000U

typedef struct {
    rtlpower_config_t config;
    SemaphoreHandle_t mutex;
    bool running;
    bool has_sweep;
    rtlpower_sweep_t last_sweep;
} rtlpower_ctx_t;

static rtlpower_ctx_t *s_ctx = NULL;

/* ── Window + DFT helpers ─────────────────────────────────────────────────── */

static void apply_window(const uint8_t *iq, float *windowed_i, float *windowed_q,
                          int N, const char *window_type) {
    for (int i = 0; i < N; i++) {
        float w;
        if (strcmp(window_type, "hamming") == 0)
            w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / (N - 1));
        else if (strcmp(window_type, "blackman") == 0)
            w = 0.42f - 0.5f * cosf(2.0f * (float)M_PI * i / (N - 1))
                      + 0.08f * cosf(4.0f * (float)M_PI * i / (N - 1));
        else  /* rectangular */
            w = 1.0f;
        windowed_i[i] = ((float)iq[i*2]   - 128.0f) * w;
        windowed_q[i] = ((float)iq[i*2+1] - 128.0f) * w;
    }
}

/* Simple DFT (for small N; replace with FFT for performance) */
static void compute_power_spectrum(const float *win_i, const float *win_q, int N,
                                    float *power_dbm) {
    for (int k = 0; k < N; k++) {
        float re = 0, im = 0;
        for (int n = 0; n < N; n++) {
            float angle = 2.0f * (float)M_PI * k * n / N;
            re += win_i[n] * cosf(angle) + win_q[n] * sinf(angle);
            im += -win_i[n] * sinf(angle) + win_q[n] * cosf(angle);
        }
        float mag = (re*re + im*im) / ((float)N * (float)N);
        power_dbm[k] = 10.0f * log10f(mag + 1e-10f);
    }
}

/* ── Sweep processing ─────────────────────────────────────────────────────── */

static void rtlpower_process_sweep(rtlpower_ctx_t *c, const uint8_t *iq, uint32_t num_samples) {
    int fft_size = (c->config.bin_size_hz > 0) ? (int)(RTLPOWER_SAMPLE_RATE / c->config.bin_size_hz) : 0;
    if (fft_size > RTLPOWER_MAX_BINS) fft_size = RTLPOWER_MAX_BINS;
    if (fft_size > (int)num_samples) fft_size = (int)num_samples;
    if (fft_size <= 0) return;

    float *win_i = malloc(fft_size * sizeof(float));
    float *win_q = malloc(fft_size * sizeof(float));
    float *power = malloc(fft_size * sizeof(float));
    if (!win_i || !win_q || !power) { free(win_i); free(win_q); free(power); return; }

    apply_window(iq, win_i, win_q, fft_size, c->config.window_func);
    compute_power_spectrum(win_i, win_q, fft_size, power);

    /* Store result */
    xSemaphoreTake(c->mutex, portMAX_DELAY);
    if (c->last_sweep.power_dbm) free(c->last_sweep.power_dbm);
    c->last_sweep.power_dbm    = power;
    c->last_sweep.num_bins     = (uint16_t)fft_size;
    c->last_sweep.freq_start_hz = c->config.freq_start_hz;
    c->last_sweep.freq_stop_hz  = c->config.freq_stop_hz;
    c->last_sweep.bin_size_hz   = c->config.bin_size_hz;
    c->last_sweep.timestamp_ms  = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    c->has_sweep = true;
    xSemaphoreGive(c->mutex);

    free(win_i);
    free(win_q);
    /* power is now owned by last_sweep */
}

/* ── Public API ───────────────────────────────────────────────────────────── */

esp_err_t rtlpower_init(void)
{
    if (s_ctx) return ESP_OK;  /* Already initialised */

    s_ctx = calloc(1, sizeof(rtlpower_ctx_t));
    if (!s_ctx) return ESP_ERR_NO_MEM;

    s_ctx->mutex = xSemaphoreCreateMutex();
    if (!s_ctx->mutex) {
        free(s_ctx);
        s_ctx = NULL;
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    s_ctx->running   = false;
    s_ctx->has_sweep = false;
    memset(&s_ctx->last_sweep, 0, sizeof(s_ctx->last_sweep));
    ESP_LOGI(TAG, "rtl_power spectrum monitor initialized");
    return ESP_OK;
}

esp_err_t rtlpower_start(const rtlpower_config_t *config)
{
    if (!config || !s_ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    s_ctx->config = *config;

    uint32_t span_hz = config->freq_stop_hz - config->freq_start_hz;
    uint32_t num_bins = (config->bin_size_hz > 0) ? (span_hz / config->bin_size_hz) : 0;

    ESP_LOGI(TAG, "rtl_power starting sweep:");
    ESP_LOGI(TAG, "  range : %" PRIu32 " - %" PRIu32 " Hz (span %" PRIu32 " Hz)",
             config->freq_start_hz, config->freq_stop_hz, span_hz);
    ESP_LOGI(TAG, "  bin   : %" PRIu32 " Hz  =>  %" PRIu32 " bins",
             config->bin_size_hz, num_bins);
    ESP_LOGI(TAG, "  interval : %u s, window : %s",
             config->interval_s, config->window_func);
    ESP_LOGI(TAG, "  spectrogram : %s", config->generate_spectrogram ? "yes" : "no");
    if (config->cloud_upload_enable) {
        ESP_LOGI(TAG, "  cloud : %s  path=%s  every %u min",
                 config->cloud_provider, config->cloud_path, config->upload_interval_min);
    } else {
        ESP_LOGI(TAG, "  cloud : disabled");
    }

    s_ctx->running = true;
    return ESP_OK;
}

esp_err_t rtlpower_stop(void)
{
    if (s_ctx) s_ctx->running = false;
    ESP_LOGI(TAG, "rtl_power stopped");
    return ESP_OK;
}

void rtlpower_push_samples(const uint8_t *data, uint32_t len)
{
    if (!s_ctx || !s_ctx->running || !data || len < 2) {
        return;
    }

    uint32_t num_samples = len / 2;
    rtlpower_process_sweep(s_ctx, data, num_samples);
}

esp_err_t rtlpower_get_latest_sweep(rtlpower_sweep_t *sweep)
{
    if (!sweep || !s_ctx || !s_ctx->mutex) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_ctx->has_sweep) return ESP_ERR_NOT_FOUND;

    xSemaphoreTake(s_ctx->mutex, portMAX_DELAY);
    sweep->timestamp_ms  = s_ctx->last_sweep.timestamp_ms;
    sweep->freq_start_hz = s_ctx->last_sweep.freq_start_hz;
    sweep->freq_stop_hz  = s_ctx->last_sweep.freq_stop_hz;
    sweep->bin_size_hz   = s_ctx->last_sweep.bin_size_hz;
    sweep->num_bins      = s_ctx->last_sweep.num_bins;
    sweep->power_dbm     = malloc(sweep->num_bins * sizeof(float));
    if (sweep->power_dbm) {
        memcpy(sweep->power_dbm, s_ctx->last_sweep.power_dbm,
               sweep->num_bins * sizeof(float));
    }
    xSemaphoreGive(s_ctx->mutex);

    return sweep->power_dbm ? ESP_OK : ESP_ERR_NO_MEM;
}
