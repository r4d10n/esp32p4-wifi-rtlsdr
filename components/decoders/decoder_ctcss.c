#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_ctcss";

/* EIA standard CTCSS tones (Hz * 10 for integer storage) */
static const uint16_t CTCSS_TONES[] __attribute__((used)) = {
    670, 693, 719, 744, 770, 797, 825, 854, 885, 915,
    948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273,
    1318, 1365, 1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679,
    1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995,
    2035, 2065, 2107, 2138, 2181, 2217, 2257, 2291, 2336, 2541,
};
#define CTCSS_TONE_COUNT (sizeof(CTCSS_TONES) / sizeof(CTCSS_TONES[0]))

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    uint16_t detected_tone;  /* Hz * 10, 0 = none */
} ctcss_ctx_t;

static ctcss_ctx_t s_ctcss_ctx;

static esp_err_t ctcss_init(void *ctx) {
    ctcss_ctx_t *c = (ctcss_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->detected_tone = 0;
    ESP_LOGI(TAG, "CTCSS decoder initialized (%d standard tones)", (int)CTCSS_TONE_COUNT);
    return ESP_OK;
}

static esp_err_t ctcss_start(void *ctx) { ((ctcss_ctx_t *)ctx)->running = true; return ESP_OK; }
static esp_err_t ctcss_stop(void *ctx) { ((ctcss_ctx_t *)ctx)->running = false; return ESP_OK; }
static void ctcss_destroy(void *ctx) { (void)ctx; }

static void ctcss_process_audio(void *ctx, const int16_t *samples,
                                 uint32_t count, uint32_t sample_rate) {
    (void)ctx; (void)samples; (void)count; (void)sample_rate;
    /* TODO: CTCSS tone detection
     * 1. Low-pass filter audio below 300 Hz (sub-audible range)
     * 2. Run Goertzel filter for each of 50 CTCSS tones
     * 3. Detect strongest tone above threshold
     * 4. Require persistence over ~300ms for valid detection
     * 5. Publish decode_event_t with {tone_hz, tone_name}
     */
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
