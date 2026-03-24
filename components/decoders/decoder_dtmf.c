#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "dec_dtmf";

/* DTMF frequency pairs
 * Row:    697  770  852  941 Hz
 * Col:   1209 1336 1477 1633 Hz
 *
 *         1209  1336  1477  1633
 *  697:    1     2     3     A
 *  770:    4     5     6     B
 *  852:    7     8     9     C
 *  941:    *     0     #     D
 */
static const uint16_t DTMF_FREQS[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
static const char DTMF_CHARS[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'},
};

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    char last_digits[64];
    int digit_count;
    /* Goertzel block buffering */
    int16_t *block_buf;
    uint32_t block_pos;
    uint32_t block_size;       /* N = sample_rate / 40 */
    /* Debounce state */
    char prev_digit;           /* Digit detected in previous block */
    char confirmed_digit;      /* Last confirmed (debounced) digit */
    uint8_t consecutive_count; /* Consecutive blocks with same digit */
    uint8_t silence_count;     /* Consecutive blocks with no digit */
    bool digit_active;         /* A digit is currently being held */
    /* Silence tracking for sequence termination */
    uint8_t silence_after_digit; /* Silence blocks after last confirmed digit */
} dtmf_ctx_t;

static dtmf_ctx_t s_dtmf_ctx;

static esp_err_t dtmf_init(void *ctx) {
    dtmf_ctx_t *c = (dtmf_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->last_digits[0] = '\0';
    c->digit_count = 0;
    /* 25ms block at 8000 Hz = 200 samples */
    c->block_size = 200;
    c->block_buf = (int16_t *)calloc(c->block_size, sizeof(int16_t));
    c->block_pos = 0;
    c->prev_digit = '\0';
    c->confirmed_digit = '\0';
    c->consecutive_count = 0;
    c->silence_count = 0;
    c->digit_active = false;
    c->silence_after_digit = 0;
    ESP_LOGI(TAG, "DTMF decoder initialized (Goertzel, 8 freq, block=%"PRIu32")", c->block_size);
    return ESP_OK;
}

static esp_err_t dtmf_start(void *ctx) { ((dtmf_ctx_t *)ctx)->running = true; return ESP_OK; }
static esp_err_t dtmf_stop(void *ctx) { ((dtmf_ctx_t *)ctx)->running = false; return ESP_OK; }
static void dtmf_destroy(void *ctx) {
    dtmf_ctx_t *c = (dtmf_ctx_t *)ctx;
    free(c->block_buf);
    c->block_buf = NULL;
}

/* ── Goertzel magnitude squared for a single frequency ──────── */
static double goertzel_mag(const int16_t *samples, int N, double freq, int sample_rate) {
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

/* ── Detect DTMF digit from one block of N samples ─────────── */
static char dtmf_detect_block(const int16_t *samples, int N, int sample_rate) {
    double row_mag[4], col_mag[4];
    const double threshold = 0.01;

    /* Compute Goertzel for all 8 frequencies */
    for (int i = 0; i < 4; i++) {
        row_mag[i] = goertzel_mag(samples, N, (double)DTMF_FREQS[i], sample_rate);
    }
    for (int i = 0; i < 4; i++) {
        col_mag[i] = goertzel_mag(samples, N, (double)DTMF_FREQS[i + 4], sample_rate);
    }

    /* Find strongest row */
    int best_row = 0;
    for (int i = 1; i < 4; i++) {
        if (row_mag[i] > row_mag[best_row]) best_row = i;
    }

    /* Find strongest column */
    int best_col = 0;
    for (int i = 1; i < 4; i++) {
        if (col_mag[i] > col_mag[best_col]) best_col = i;
    }

    /* Threshold check */
    if (row_mag[best_row] < threshold || col_mag[best_col] < threshold) {
        return '\0';
    }

    /* Selectivity: strongest must be >= 2x second-strongest in its group */
    for (int i = 0; i < 4; i++) {
        if (i != best_row && row_mag[i] * 2.0 > row_mag[best_row]) return '\0';
    }
    for (int i = 0; i < 4; i++) {
        if (i != best_col && col_mag[i] * 2.0 > col_mag[best_col]) return '\0';
    }

    /* Twist check: row and column power within 6 dB of each other */
    double ratio = row_mag[best_row] / col_mag[best_col];
    if (ratio < 0.25 || ratio > 4.0) {  /* 10^(6/10) ~ 3.98, use 4.0 */
        return '\0';
    }

    return DTMF_CHARS[best_row][best_col];
}

/* ── Emit a detected digit event ───────────────────────────── */
static void dtmf_emit_digit(dtmf_ctx_t *c, char digit) {
    /* Append to digit buffer */
    if (c->digit_count < (int)(sizeof(c->last_digits) - 1)) {
        c->last_digits[c->digit_count] = digit;
        c->digit_count++;
        c->last_digits[c->digit_count] = '\0';
    }

    ESP_LOGI(TAG, "DTMF digit: %c (seq: %s)", digit, c->last_digits);

    /* Publish event */
    cJSON *data = cJSON_CreateObject();
    if (data) {
        char digit_str[2] = {digit, '\0'};
        cJSON_AddStringToObject(data, "digit", digit_str);
        cJSON_AddStringToObject(data, "digits", c->last_digits);
        decode_event_t ev = {
            .decoder_name = "dtmf",
            .event_type = "tone",
            .timestamp_ms = (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
            .rssi_db = 0,
            .freq_hz = 0,
            .data = data,
        };
        decode_bus_publish(&ev);
    }
}

/* ── Process one complete Goertzel block ───────────────────── */
static void dtmf_process_block(dtmf_ctx_t *c, const int16_t *block,
                                uint32_t block_size, uint32_t sample_rate) {
    char digit = dtmf_detect_block(block, (int)block_size, (int)sample_rate);

    if (digit != '\0') {
        /* Tone present */
        c->silence_count = 0;
        c->silence_after_digit = 0;

        if (digit == c->prev_digit) {
            c->consecutive_count++;
        } else {
            c->consecutive_count = 1;
        }
        c->prev_digit = digit;

        /* Require 2 consecutive blocks with same digit, and digit not already active */
        if (c->consecutive_count >= 2 && !c->digit_active) {
            c->digit_active = true;
            c->confirmed_digit = digit;
            xSemaphoreTake(c->mutex, portMAX_DELAY);
            dtmf_emit_digit(c, digit);
            xSemaphoreGive(c->mutex);
        }
    } else {
        /* Silence */
        c->prev_digit = '\0';
        c->consecutive_count = 0;
        c->silence_count++;

        /* Require 1 block of silence between digits */
        if (c->silence_count >= 1 && c->digit_active) {
            c->digit_active = false;
        }

        /* Track silence after last digit for sequence termination */
        if (c->digit_count > 0) {
            c->silence_after_digit++;
            /* 200ms silence = 8 blocks at 25ms each */
            if (c->silence_after_digit >= 8) {
                /* Sequence terminated, upsert to tracking table */
                tracking_table_t *tt = decoder_get_global_tracking();
                if (tt) {
                    char freq_key[32];
                    snprintf(freq_key, sizeof(freq_key), "dtmf_seq_%d", c->digit_count);
                    cJSON *data_json = cJSON_CreateObject();
                    if (data_json) {
                        cJSON_AddStringToObject(data_json, "digits", c->last_digits);
                        cJSON_AddNumberToObject(data_json, "length", c->digit_count);
                        tracking_table_upsert(tt, "dtmf", freq_key, data_json, 0);
                        cJSON_Delete(data_json);
                    }
                }
                /* Reset digit buffer for next sequence */
                c->last_digits[0] = '\0';
                c->digit_count = 0;
                c->silence_after_digit = 0;
            }
        }
    }
}

static void dtmf_process_audio(void *ctx, const int16_t *samples,
                                uint32_t count, uint32_t sample_rate) {
    dtmf_ctx_t *c = (dtmf_ctx_t *)ctx;
    if (!c->running) return;

    /* Recompute block size if sample rate changed */
    uint32_t needed_block = sample_rate / 40;  /* 25ms window */
    if (needed_block != c->block_size && needed_block > 0) {
        int16_t *new_buf = (int16_t *)realloc(c->block_buf, needed_block * sizeof(int16_t));
        if (new_buf) {
            c->block_buf = new_buf;
            c->block_size = needed_block;
            c->block_pos = 0;
        }
    }

    /* Accumulate samples into blocks and process complete blocks */
    uint32_t offset = 0;
    while (offset < count) {
        uint32_t remaining_in_block = c->block_size - c->block_pos;
        uint32_t available = count - offset;
        uint32_t to_copy = (available < remaining_in_block) ? available : remaining_in_block;

        memcpy(&c->block_buf[c->block_pos], &samples[offset], to_copy * sizeof(int16_t));
        c->block_pos += to_copy;
        offset += to_copy;

        if (c->block_pos >= c->block_size) {
            dtmf_process_block(c, c->block_buf, c->block_size, sample_rate);
            c->block_pos = 0;
        }
    }
}

static void dtmf_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

static cJSON *dtmf_get_status(void *ctx) {
    dtmf_ctx_t *c = (dtmf_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "digit_count", c->digit_count);
    }
    return j;
}

static cJSON *dtmf_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "dtmf") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_dtmf_plugin = {
    .name = "dtmf",
    .description = "DTMF Tone Decoder (Goertzel)",
    .category = "tone",
    .demod_type = DEMOD_FM_NARROW,
    .center_freq_hz = 0,  /* User-configurable */
    .bandwidth_hz = 12500,
    .audio_rate_hz = 8000,
    .init = dtmf_init, .start = dtmf_start, .stop = dtmf_stop, .destroy = dtmf_destroy,
    .process_audio = dtmf_process_audio, .process_iq = dtmf_process_iq,
    .get_status = dtmf_get_status, .get_results = dtmf_get_results,
    .ctx = &s_dtmf_ctx,
};

void register_dtmf_decoder(void) {
    decoder_registry_add(&s_dtmf_plugin);
}
