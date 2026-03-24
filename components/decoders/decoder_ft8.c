#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_ft8";

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    int decode_count;
} ft8_ctx_t;

static ft8_ctx_t s_ft8_ctx;
static ft8_ctx_t s_wspr_ctx;

static esp_err_t ft8_init(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->decode_count = 0;
    ESP_LOGI(TAG, "FT8/WSPR decoder initialized");
    return ESP_OK;
}

static esp_err_t ft8_start(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    c->running = true;
    ESP_LOGI(TAG, "FT8/WSPR decoder started");
    return ESP_OK;
}

static esp_err_t ft8_stop(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    c->running = false;
    ESP_LOGI(TAG, "FT8/WSPR decoder stopped");
    return ESP_OK;
}

static void ft8_destroy(void *ctx) { (void)ctx; }

static void ft8_process_audio(void *ctx, const int16_t *samples,
                               uint32_t count, uint32_t sample_rate) {
    (void)ctx; (void)samples; (void)count; (void)sample_rate;
    /* TODO: FT8 demodulation
     * Reference: ft8_lib (lightweight FT8 decoder), wsjt-x source
     * 1. Collect 15-second audio windows (12.5 Hz FFT resolution)
     * 2. Detect 8-FSK tones (6.25 Hz spacing, 79 symbols per message)
     * 3. Costas array synchronisation, LDPC(174,91) decode
     * 4. Unpack 77-bit message: callsign, Maidenhead grid, signal report
     * 5. Publish via decode_bus_publish()
     */
}

static void ft8_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

static void wspr_process_audio(void *ctx, const int16_t *samples,
                                uint32_t count, uint32_t sample_rate) {
    (void)ctx; (void)samples; (void)count; (void)sample_rate;
    /* TODO: WSPR demodulation
     * Reference: wsjt-x source, wsprd
     * 1. Collect 2-minute windows at 375 Hz sample rate, 1500 Hz center
     * 2. 4-FSK, 1.4648 baud (4 tones, ~2.7 Hz spacing)
     * 3. Convolutional decode (K=32, rate 1/2), interleaver
     * 4. Unpack 50-bit message: callsign, Maidenhead grid (4-char), power (dBm)
     * 5. Publish via decode_bus_publish()
     */
}

static cJSON *ft8_get_status(void *ctx) {
    ft8_ctx_t *c = (ft8_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "decode_count", c->decode_count);
    }
    return j;
}

static cJSON *ft8_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "ft8") :
        cJSON_CreateArray();
}

static cJSON *wspr_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "wspr") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_ft8_plugin = {
    .name = "ft8",
    .description = "FT8 HF Digital Mode (WSJT-X, 15s cycles, 8-FSK)",
    .category = "digital",
    .demod_type = DEMOD_USB,
    .center_freq_hz = 14074000,
    .bandwidth_hz = 3000,
    .audio_rate_hz = 8000,
    .init = ft8_init,
    .start = ft8_start,
    .stop = ft8_stop,
    .destroy = ft8_destroy,
    .process_iq = ft8_process_iq,
    .process_audio = ft8_process_audio,
    .get_status = ft8_get_status,
    .get_results = ft8_get_results,
    .ctx = &s_ft8_ctx,
};

static decoder_plugin_t s_wspr_plugin = {
    .name = "wspr",
    .description = "WSPR HF Weak Signal (2-min cycles, 4-FSK, 1.4648 baud)",
    .category = "digital",
    .demod_type = DEMOD_USB,
    .center_freq_hz = 14095600,
    .bandwidth_hz = 3000,
    .audio_rate_hz = 375,
    .init = ft8_init,
    .start = ft8_start,
    .stop = ft8_stop,
    .destroy = ft8_destroy,
    .process_iq = ft8_process_iq,
    .process_audio = wspr_process_audio,
    .get_status = ft8_get_status,
    .get_results = wspr_get_results,
    .ctx = &s_wspr_ctx,
};

void register_ft8_decoder(void) {
    decoder_registry_add(&s_ft8_plugin);
    decoder_registry_add(&s_wspr_plugin);
}
