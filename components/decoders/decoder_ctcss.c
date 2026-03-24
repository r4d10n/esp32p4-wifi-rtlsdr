#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "dec_ctcss";

/* EIA standard CTCSS tones (Hz * 10 for integer storage) */
static const uint16_t CTCSS_TONES[] = {
    670, 693, 719, 744, 770, 797, 825, 854, 885, 915,
    948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273,
    1318, 1365, 1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679,
    1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995,
    2035, 2065, 2107, 2138, 2181, 2217, 2257, 2291, 2336, 2541,
};
#define CTCSS_TONE_COUNT (sizeof(CTCSS_TONES) / sizeof(CTCSS_TONES[0]))

/* Standard PL code names indexed to match CTCSS_TONES[] */
static const char *CTCSS_NAMES[] = {
    "XZ", "WZ", "XA", "WA", "XB", "WB", "YZ", "YA", "YB", "ZZ",
    "ZA", "ZB", "1Z", "1A", "1B", "2Z", "2A", "2B", "3Z", "3A",
    "3B", "4Z", "4A", "4B", "5Z", "5A", "",   "5B", "6Z", "6A",
    "6B", "7Z", "7A", "",   "7B", "M1", "8Z", "M2", "M3", "8A",
    "M4", "8B", "M5", "M6", "M7", "9Z", "",   "",   "",   "0Z",
};

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    uint16_t detected_tone;  /* Hz * 10, 0 = none */
    /* LPF state (2-pole Butterworth, 300 Hz cutoff) */
    double lpf_x1, lpf_x2;  /* Input delay line */
    double lpf_y1, lpf_y2;  /* Output delay line */
    /* Block buffering */
    int16_t *block_buf;
    uint32_t block_pos;
    uint32_t block_size;     /* N = sample_rate / 4 (250ms) */
    /* Detection persistence */
    int prev_tone_idx;       /* Tone index detected in previous block, -1 = none */
    uint8_t consecutive_count;
    uint32_t configured_sample_rate;
} ctcss_ctx_t;

static ctcss_ctx_t s_ctcss_ctx;

static esp_err_t ctcss_init(void *ctx) {
    ctcss_ctx_t *c = (ctcss_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->detected_tone = 0;
    c->lpf_x1 = c->lpf_x2 = 0.0;
    c->lpf_y1 = c->lpf_y2 = 0.0;
    /* 250ms block at 8000 Hz = 2000 samples */
    c->configured_sample_rate = 8000;
    c->block_size = c->configured_sample_rate / 4;
    c->block_buf = (int16_t *)calloc(c->block_size, sizeof(int16_t));
    c->block_pos = 0;
    c->prev_tone_idx = -1;
    c->consecutive_count = 0;
    ESP_LOGI(TAG, "CTCSS decoder initialized (%d standard tones, block=%"PRIu32")",
             (int)CTCSS_TONE_COUNT, c->block_size);
    return ESP_OK;
}

static esp_err_t ctcss_start(void *ctx) { ((ctcss_ctx_t *)ctx)->running = true; return ESP_OK; }
static esp_err_t ctcss_stop(void *ctx) { ((ctcss_ctx_t *)ctx)->running = false; return ESP_OK; }
static void ctcss_destroy(void *ctx) {
    ctcss_ctx_t *c = (ctcss_ctx_t *)ctx;
    free(c->block_buf);
    c->block_buf = NULL;
}

/* ── 2-pole Butterworth LPF coefficients for 300 Hz at given sample rate ── */
/* Pre-computed for 8000 Hz: fc/fs = 300/8000 = 0.0375                     */
/* Using bilinear transform:                                                 */
/*   wc = tan(pi * fc / fs)                                                  */
/*   a0_inv = 1 / (1 + sqrt(2)*wc + wc^2)                                   */
/*   b0 = wc^2 * a0_inv                                                      */
/*   b1 = 2 * b0                                                             */
/*   b2 = b0                                                                 */
/*   a1 = 2*(wc^2 - 1) * a0_inv                                             */
/*   a2 = (1 - sqrt(2)*wc + wc^2) * a0_inv                                  */

typedef struct {
    double b0, b1, b2, a1, a2;
} biquad_coeffs_t;

static void compute_lpf_coeffs(biquad_coeffs_t *co, double cutoff_hz, double sample_rate) {
    double wc = tan(M_PI * cutoff_hz / sample_rate);
    double wc2 = wc * wc;
    double sqrt2_wc = 1.4142135623730951 * wc;  /* sqrt(2) * wc */
    double a0_inv = 1.0 / (1.0 + sqrt2_wc + wc2);
    co->b0 = wc2 * a0_inv;
    co->b1 = 2.0 * co->b0;
    co->b2 = co->b0;
    co->a1 = 2.0 * (wc2 - 1.0) * a0_inv;
    co->a2 = (1.0 - sqrt2_wc + wc2) * a0_inv;
}

/* ── Apply LPF to samples in-place, maintaining filter state ── */
static void ctcss_apply_lpf(ctcss_ctx_t *c, const int16_t *in, int16_t *out,
                              uint32_t count, const biquad_coeffs_t *co) {
    double x1 = c->lpf_x1, x2 = c->lpf_x2;
    double y1 = c->lpf_y1, y2 = c->lpf_y2;

    for (uint32_t i = 0; i < count; i++) {
        double x0 = (double)in[i];
        double y0 = co->b0 * x0 + co->b1 * x1 + co->b2 * x2 - co->a1 * y1 - co->a2 * y2;
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
        /* Clamp to int16 range */
        if (y0 > 32767.0) y0 = 32767.0;
        if (y0 < -32768.0) y0 = -32768.0;
        out[i] = (int16_t)y0;
    }

    c->lpf_x1 = x1; c->lpf_x2 = x2;
    c->lpf_y1 = y1; c->lpf_y2 = y2;
}

/* ── Goertzel magnitude squared for a single frequency ──────── */
static double ctcss_goertzel_mag(const int16_t *samples, int N, double freq, int sample_rate) {
    double k = (double)N * freq / sample_rate;
    double w = 2.0 * M_PI * k / N;
    double coeff = 2.0 * cos(w);
    double s0 = 0.0, s1 = 0.0, s2 = 0.0;

    for (int i = 0; i < N; i++) {
        s0 = (double)samples[i] / 32768.0 + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

/* ── Emit tone detection event ─────────────────────────────── */
static void ctcss_emit_tone(ctcss_ctx_t *c, int tone_idx) {
    double tone_hz = (double)CTCSS_TONES[tone_idx] / 10.0;
    const char *tone_name = (tone_idx < (int)(sizeof(CTCSS_NAMES)/sizeof(CTCSS_NAMES[0])))
                            ? CTCSS_NAMES[tone_idx] : "";

    ESP_LOGI(TAG, "CTCSS tone: %.1f Hz (%s)", tone_hz, tone_name);

    cJSON *data = cJSON_CreateObject();
    if (data) {
        cJSON_AddNumberToObject(data, "tone_hz", tone_hz);
        cJSON_AddStringToObject(data, "tone_name", tone_name);
        decode_event_t ev = {
            .decoder_name = "ctcss",
            .event_type = "tone",
            .timestamp_ms = (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
            .rssi_db = 0,
            .freq_hz = 0,
            .data = data,
        };
        decode_bus_publish(&ev);
    }

    /* Upsert to tracking table */
    tracking_table_t *tt = decoder_get_global_tracking();
    if (tt) {
        char freq_key[16];
        snprintf(freq_key, sizeof(freq_key), "%.1f", tone_hz);
        cJSON *track_data = cJSON_CreateObject();
        if (track_data) {
            cJSON_AddNumberToObject(track_data, "tone_hz", tone_hz);
            cJSON_AddStringToObject(track_data, "tone_name", tone_name);
            tracking_table_upsert(tt, "ctcss", freq_key, track_data, 0);
            cJSON_Delete(track_data);
        }
    }
}

/* ── Process one complete block of filtered samples ────────── */
static void ctcss_process_block(ctcss_ctx_t *c, const int16_t *block,
                                 uint32_t block_size, uint32_t sample_rate) {
    int N = (int)block_size;
    double best_mag = 0.0;
    int best_idx = -1;
    const double threshold = 0.001;  /* Lower threshold for sub-audio tones */

    /* Compute Goertzel for all CTCSS tones */
    for (int t = 0; t < (int)CTCSS_TONE_COUNT; t++) {
        double freq = (double)CTCSS_TONES[t] / 10.0;
        double mag = ctcss_goertzel_mag(block, N, freq, (int)sample_rate);
        if (mag > best_mag) {
            best_mag = mag;
            best_idx = t;
        }
    }

    /* Must exceed threshold */
    if (best_mag < threshold) {
        best_idx = -1;
    }

    /* Persistence check */
    if (best_idx >= 0 && best_idx == c->prev_tone_idx) {
        c->consecutive_count++;
    } else {
        c->consecutive_count = (best_idx >= 0) ? 1 : 0;
    }
    c->prev_tone_idx = best_idx;

    /* Require 2+ consecutive blocks (500ms) for valid detection */
    if (best_idx >= 0 && c->consecutive_count >= 2) {
        uint16_t new_tone = CTCSS_TONES[best_idx];
        if (new_tone != c->detected_tone) {
            xSemaphoreTake(c->mutex, portMAX_DELAY);
            c->detected_tone = new_tone;
            ctcss_emit_tone(c, best_idx);
            xSemaphoreGive(c->mutex);
        }
    } else if (best_idx < 0 && c->consecutive_count == 0) {
        /* No tone for this block; after several silent blocks, clear */
        if (c->detected_tone != 0 && c->prev_tone_idx < 0) {
            xSemaphoreTake(c->mutex, portMAX_DELAY);
            c->detected_tone = 0;
            xSemaphoreGive(c->mutex);
            ESP_LOGD(TAG, "CTCSS tone lost");
        }
    }
}

static void ctcss_process_audio(void *ctx, const int16_t *samples,
                                 uint32_t count, uint32_t sample_rate) {
    ctcss_ctx_t *c = (ctcss_ctx_t *)ctx;
    if (!c->running) return;

    /* Recompute block size if sample rate changed */
    uint32_t needed_block = sample_rate / 4;  /* 250ms window */
    if (needed_block != c->block_size && needed_block > 0) {
        int16_t *new_buf = (int16_t *)realloc(c->block_buf, needed_block * sizeof(int16_t));
        if (new_buf) {
            c->block_buf = new_buf;
            c->block_size = needed_block;
            c->block_pos = 0;
        }
    }
    if (sample_rate != c->configured_sample_rate) {
        c->configured_sample_rate = sample_rate;
        /* Reset LPF state on rate change */
        c->lpf_x1 = c->lpf_x2 = 0.0;
        c->lpf_y1 = c->lpf_y2 = 0.0;
    }

    /* Compute LPF coefficients (300 Hz cutoff) */
    biquad_coeffs_t co;
    compute_lpf_coeffs(&co, 300.0, (double)sample_rate);

    /* Process samples: LPF then accumulate into blocks */
    /* Use a small stack buffer for filtered samples */
    int16_t filt_buf[256];
    uint32_t offset = 0;

    while (offset < count) {
        uint32_t chunk = count - offset;
        if (chunk > 256) chunk = 256;

        /* Apply LPF */
        ctcss_apply_lpf(c, &samples[offset], filt_buf, chunk, &co);

        /* Accumulate filtered samples into block buffer */
        uint32_t filt_offset = 0;
        while (filt_offset < chunk) {
            uint32_t remaining_in_block = c->block_size - c->block_pos;
            uint32_t available = chunk - filt_offset;
            uint32_t to_copy = (available < remaining_in_block) ? available : remaining_in_block;

            memcpy(&c->block_buf[c->block_pos], &filt_buf[filt_offset], to_copy * sizeof(int16_t));
            c->block_pos += to_copy;
            filt_offset += to_copy;

            if (c->block_pos >= c->block_size) {
                ctcss_process_block(c, c->block_buf, c->block_size, sample_rate);
                c->block_pos = 0;
            }
        }

        offset += chunk;
    }
}

static void ctcss_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

static cJSON *ctcss_get_status(void *ctx) {
    ctcss_ctx_t *c = (ctcss_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "detected_tone_hz", c->detected_tone / 10.0);
    }
    return j;
}

static cJSON *ctcss_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "ctcss") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_ctcss_plugin = {
    .name = "ctcss",
    .description = "CTCSS Sub-Audible PL Tone Decoder",
    .category = "tone",
    .demod_type = DEMOD_FM_NARROW,
    .center_freq_hz = 0,
    .bandwidth_hz = 12500,
    .audio_rate_hz = 8000,
    .init = ctcss_init, .start = ctcss_start, .stop = ctcss_stop, .destroy = ctcss_destroy,
    .process_audio = ctcss_process_audio, .process_iq = ctcss_process_iq,
    .get_status = ctcss_get_status, .get_results = ctcss_get_results,
    .ctx = &s_ctcss_ctx,
};

void register_ctcss_decoder(void) {
    decoder_registry_add(&s_ctcss_plugin);
}
