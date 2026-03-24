#include <string.h>
#include "esp_log.h"
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
} cw_ctx_t;

static cw_ctx_t s_cw_ctx;

static esp_err_t cw_init(void *ctx) {
    cw_ctx_t *c = (cw_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->decoded_text[0] = '\0';
    c->text_len = 0;
    c->estimated_wpm = 15.0f;
    ESP_LOGI(TAG, "CW/Morse decoder initialized");
    return ESP_OK;
}

static esp_err_t cw_start(void *ctx) { ((cw_ctx_t *)ctx)->running = true; return ESP_OK; }
static esp_err_t cw_stop(void *ctx) { ((cw_ctx_t *)ctx)->running = false; return ESP_OK; }
static void cw_destroy(void *ctx) { (void)ctx; }

static void cw_process_audio(void *ctx, const int16_t *samples,
                              uint32_t count, uint32_t sample_rate) {
    (void)ctx; (void)samples; (void)count; (void)sample_rate;
    /* TODO: CW/Morse demodulation
     * Reference: fldigi/src/cw/cw.cxx
     *
     * 1. Bandpass filter around CW tone frequency (default 600-800 Hz)
     * 2. Envelope detection (rectify + LPF or Goertzel magnitude)
     * 3. Threshold with hysteresis (tone present / absent)
     * 4. Timing measurement:
     *    dit_ms = 1200 / wpm  (PARIS standard)
     *    dah = 3 * dit
     *    inter-element gap = 1 dit
     *    inter-character gap = 3 dit
     *    inter-word gap = 7 dit
     * 5. Adaptive WPM: measure actual dit/dah durations, update estimate
     * 6. Morse-to-ASCII: match element sequence against MORSE_TABLE
     * 7. Publish decode_event_t with {text, wpm, freq_hz}
     */
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
