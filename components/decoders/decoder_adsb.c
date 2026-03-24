#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_adsb";

#define MAX_AIRCRAFT 64

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    int aircraft_count;
    /* Aircraft tracking data would go here */
} adsb_ctx_t;

static adsb_ctx_t s_adsb_ctx;

static esp_err_t adsb_init(void *ctx) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->aircraft_count = 0;
    ESP_LOGI(TAG, "ADS-B decoder initialized");
    return ESP_OK;
}

static esp_err_t adsb_start(void *ctx) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    c->running = true;
    ESP_LOGI(TAG, "ADS-B decoder started (1090 MHz, 2 MSPS)");
    return ESP_OK;
}

static esp_err_t adsb_stop(void *ctx) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    c->running = false;
    ESP_LOGI(TAG, "ADS-B decoder stopped");
    return ESP_OK;
}

static void adsb_destroy(void *ctx) { (void)ctx; }

static void adsb_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
    /* TODO: Mode S demodulation
     * Reference: esp32p4-adsb/components/adsb/adsb_decode.c
     * 1. IQ -> magnitude via lookup table (128KB in PSRAM)
     * 2. Detect 8us preamble (1010000101000000)
     * 3. Extract 112-bit Mode S message (PPM: 1us per bit)
     * 4. CRC-24 validation
     * 5. Decode DF17: position (CPR), velocity, identification
     * 6. Publish via decode_bus_publish()
     */
}

static cJSON *adsb_get_status(void *ctx) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "aircraft_count", c->aircraft_count);
    }
    return j;
}

static cJSON *adsb_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "adsb") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_adsb_plugin = {
    .name = "adsb",
    .description = "ADS-B 1090 MHz Mode S Aircraft Tracker",
    .category = "aviation",
    .demod_type = DEMOD_RAW_IQ,
    .center_freq_hz = 1090000000,
    .bandwidth_hz = 2000000,
    .audio_rate_hz = 0,
    .init = adsb_init,
    .start = adsb_start,
    .stop = adsb_stop,
    .destroy = adsb_destroy,
    .process_iq = adsb_process_iq,
    .process_audio = NULL,
    .get_status = adsb_get_status,
    .get_results = adsb_get_results,
    .ctx = &s_adsb_ctx,
};

void register_adsb_decoder(void) {
    decoder_registry_add(&s_adsb_plugin);
}
