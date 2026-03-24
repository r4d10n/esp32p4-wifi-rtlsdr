#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_cw";

/* ITU Morse code lookup (index by ASCII - 32) */
static const char *MORSE_TABLE[] __attribute__((used)) = {
    /* SP */ NULL, /* ! */ NULL, /* " */ NULL, /* # */ NULL,
    /* $ */ NULL, /* % */ NULL, /* & */ NULL, /* ' */ NULL,
    /* ( */ "-.--.", /* ) */ "-.--.-", /* * */ NULL, /* + */ ".-.-.",
    /* , */ "--..--", /* - */ "-....-", /* . */ ".-.-.-", /* / */ "-..-.",
    /* 0 */ "-----", /* 1 */ ".----", /* 2 */ "..---", /* 3 */ "...--",
    /* 4 */ "....-", /* 5 */ ".....", /* 6 */ "-....", /* 7 */ "--...",
    /* 8 */ "---..", /* 9 */ "----.",
    /* : */ "---...", /* ; */ "-.-.-.", /* < */ NULL, /* = */ "-...-",
    /* > */ NULL, /* ? */ "..--..",  /* @ */ ".--.-.",
    /* A */ ".-", /* B */ "-...", /* C */ "-.-.", /* D */ "-..",
    /* E */ ".", /* F */ "..-.", /* G */ "--.", /* H */ "....",
    /* I */ "..", /* J */ ".---", /* K */ "-.-", /* L */ ".-..",
    /* M */ "--", /* N */ "-.", /* O */ "---", /* P */ ".--.",
    /* Q */ "--.-", /* R */ ".-.", /* S */ "...", /* T */ "-",
    /* U */ "..-", /* V */ "...-", /* W */ ".--", /* X */ "-..-",
    /* Y */ "-.--", /* Z */ "--..",
};

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    char decoded_text[256];
    int text_len;
    float estimated_wpm;

    /* DSP state */
    double tone_freq;
    double avg_power;
    double noise_floor;
    bool tone_active;
    int tone_start_ms;
    int tone_end_ms;
    int silence_start_ms;
    int sample_counter;

    /* Timing */
    double avg_dit_ms;
    double avg_dah_ms;

    /* Element buffer */
    char elements[16];
    int element_count;
} cw_ctx_t;

static cw_ctx_t s_cw_ctx;

static esp_err_t cw_init(void *ctx) {
    cw_ctx_t *c = (cw_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->decoded_text[0] = '\0';
    c->text_len = 0;
    c->estimated_wpm = 15.0f;

    /* DSP defaults: 700 Hz tone, 15 WPM initial estimate */
    c->tone_freq = 700.0;
    c->avg_power = 0.0;
    c->noise_floor = 0.001;
    c->tone_active = false;
    c->tone_start_ms = 0;
    c->tone_end_ms = 0;
    c->silence_start_ms = 0;
    c->sample_counter = 0;

    c->avg_dit_ms = 80.0;   /* 1200 / 15 WPM */
    c->avg_dah_ms = 240.0;

    c->elements[0] = '\0';
    c->element_count = 0;

    ESP_LOGI(TAG, "CW/Morse decoder initialized (tone=%.0f Hz, initial WPM=%.0f)",
             c->tone_freq, (double)c->estimated_wpm);
    return ESP_OK;
}

static esp_err_t cw_start(void *ctx) { ((cw_ctx_t *)ctx)->running = true; return ESP_OK; }
static esp_err_t cw_stop(void *ctx) { ((cw_ctx_t *)ctx)->running = false; return ESP_OK; }
static void cw_destroy(void *ctx) { (void)ctx; }

/* ═══════════════════════════════════════════════════════════════
 *  Goertzel single-frequency power estimator
 * ═══════════════════════════════════════════════════════════════ */

static double goertzel_power(const int16_t *samples, int N, double freq, int sr) {
    double k = (double)N * freq / sr;
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

/* ═══════════════════════════════════════════════════════════════
 *  Morse lookup: element string -> ASCII character
 * ═══════════════════════════════════════════════════════════════ */

static char morse_lookup(const char *elements) {
    /* MORSE_TABLE is indexed by (ASCII - 32), covering '(' through 'Z' */
    for (int ch = '('; ch <= 'Z'; ch++) {
        int idx = ch - 32;
        if (idx < 0 || idx >= (int)(sizeof(MORSE_TABLE) / sizeof(MORSE_TABLE[0])))
            continue;
        if (MORSE_TABLE[idx] && strcmp(MORSE_TABLE[idx], elements) == 0)
            return (char)ch;
    }
    return '\0';
}

/* ═══════════════════════════════════════════════════════════════
 *  Decode accumulated elements and append to output text
 * ═══════════════════════════════════════════════════════════════ */

static void cw_decode_element_buffer(cw_ctx_t *c) {
    if (c->element_count == 0)
        return;

    c->elements[c->element_count] = '\0';
    char ch = morse_lookup(c->elements);
    if (ch != '\0' && c->text_len < (int)sizeof(c->decoded_text) - 1) {
        c->decoded_text[c->text_len++] = ch;
        c->decoded_text[c->text_len] = '\0';
    }
    c->element_count = 0;
    c->elements[0] = '\0';
}

/* ═══════════════════════════════════════════════════════════════
 *  Publish decoded text event on the decode bus
 * ═══════════════════════════════════════════════════════════════ */

static void cw_publish_event(cw_ctx_t *c) {
    cJSON *data = cJSON_CreateObject();
    if (!data) return;

    cJSON_AddStringToObject(data, "text", c->decoded_text);
    cJSON_AddNumberToObject(data, "wpm", c->estimated_wpm);
    cJSON_AddNumberToObject(data, "frequency_hz", c->tone_freq);

    decode_event_t event = {
        .decoder_name = "cw",
        .event_type = "morse",
        .timestamp_ms = (int64_t)(esp_timer_get_time() / 1000),
        .rssi_db = 0,
        .freq_hz = (uint32_t)c->tone_freq,
        .data = data,
    };
    decode_bus_publish(&event);
}

/* ═══════════════════════════════════════════════════════════════
 *  Main audio processing: Goertzel -> envelope -> timing -> decode
 * ═══════════════════════════════════════════════════════════════ */

#define CW_BLOCK_MS  10

static void cw_process_audio(void *ctx, const int16_t *samples,
                              uint32_t count, uint32_t sample_rate) {
    cw_ctx_t *c = (cw_ctx_t *)ctx;
    if (!c->running) return;

    int block_size = (int)(sample_rate * CW_BLOCK_MS / 1000);
    if (block_size <= 0) return;

    xSemaphoreTake(c->mutex, portMAX_DELAY);

    for (uint32_t offset = 0; offset + (uint32_t)block_size <= count; offset += (uint32_t)block_size) {
        /* Goertzel tone power for this block */
        double power = goertzel_power(&samples[offset], block_size, c->tone_freq, (int)sample_rate);

        /* Smoothed power (EMA, alpha=0.3) */
        c->avg_power = 0.7 * c->avg_power + 0.3 * power;

        /* Current time in ms based on sample position */
        int now_ms = (c->sample_counter * 1000) / (int)sample_rate;
        c->sample_counter += block_size;

        /* Hysteresis thresholds */
        double thresh_on  = c->noise_floor * 3.0;
        double thresh_off = c->noise_floor * 1.5;

        if (!c->tone_active) {
            if (c->avg_power > thresh_on) {
                /* --- Tone ON transition --- */
                c->tone_active = true;
                c->tone_start_ms = now_ms;

                /* Process gap duration if we had a previous tone */
                if (c->tone_end_ms > 0) {
                    int gap_ms = now_ms - c->tone_end_ms;
                    double dit_thresh = 2.0 * c->avg_dit_ms;
                    double word_thresh = 5.0 * c->avg_dit_ms;

                    if (gap_ms >= word_thresh) {
                        /* Inter-word gap: decode + space */
                        cw_decode_element_buffer(c);
                        if (c->text_len < (int)sizeof(c->decoded_text) - 1) {
                            c->decoded_text[c->text_len++] = ' ';
                            c->decoded_text[c->text_len] = '\0';
                        }
                        cw_publish_event(c);
                    } else if (gap_ms >= dit_thresh) {
                        /* Inter-character gap: decode current buffer */
                        cw_decode_element_buffer(c);
                        cw_publish_event(c);
                    }
                    /* else: intra-character gap, continue accumulating elements */
                }
            } else {
                /* Still silent -- update noise floor slowly */
                c->noise_floor = 0.95 * c->noise_floor + 0.05 * c->avg_power;
                if (c->noise_floor < 1e-10) c->noise_floor = 1e-10;
            }
        } else {
            if (c->avg_power < thresh_off) {
                /* --- Tone OFF transition --- */
                c->tone_active = false;
                c->tone_end_ms = now_ms;

                int duration_ms = now_ms - c->tone_start_ms;
                if (duration_ms < 1) duration_ms = 1;

                double dit_thresh = 2.0 * c->avg_dit_ms;

                if (duration_ms < dit_thresh) {
                    /* Dit */
                    if (c->element_count < (int)sizeof(c->elements) - 1)
                        c->elements[c->element_count++] = '.';
                    /* Adapt dit average */
                    c->avg_dit_ms = 0.9 * c->avg_dit_ms + 0.1 * duration_ms;
                } else {
                    /* Dah */
                    if (c->element_count < (int)sizeof(c->elements) - 1)
                        c->elements[c->element_count++] = '-';
                    /* Adapt dah average */
                    c->avg_dah_ms = 0.9 * c->avg_dah_ms + 0.1 * duration_ms;
                }
                c->elements[c->element_count] = '\0';

                /* Update WPM estimate from dit average */
                if (c->avg_dit_ms > 1.0) {
                    c->estimated_wpm = (float)(1200.0 / c->avg_dit_ms);
                }

                c->silence_start_ms = now_ms;
            }
        }
    }

    xSemaphoreGive(c->mutex);
}

static void cw_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

static cJSON *cw_get_status(void *ctx) {
    cw_ctx_t *c = (cw_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "estimated_wpm", c->estimated_wpm);
        cJSON_AddNumberToObject(j, "text_length", c->text_len);
    }
    return j;
}

static cJSON *cw_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "cw") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_cw_plugin = {
    .name = "cw",
    .description = "CW/Morse Code Decoder (Adaptive WPM)",
    .category = "digital",
    .demod_type = DEMOD_USB,
    .center_freq_hz = 7030000,  /* 40m CW segment */
    .bandwidth_hz = 500,
    .audio_rate_hz = 8000,
    .demod_params.tone = { .tone_hz = 700.0f, .threshold = 0.3f },
    .init = cw_init, .start = cw_start, .stop = cw_stop, .destroy = cw_destroy,
    .process_audio = cw_process_audio, .process_iq = cw_process_iq,
    .get_status = cw_get_status, .get_results = cw_get_results,
    .ctx = &s_cw_ctx,
};

void register_cw_decoder(void) {
    decoder_registry_add(&s_cw_plugin);
}
