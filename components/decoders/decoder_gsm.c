#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_gsm";

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    int cell_count;
} gsm_ctx_t;

static gsm_ctx_t s_gsm_ctx;

static esp_err_t gsm_init(void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->cell_count = 0;
    ESP_LOGI(TAG, "GSM scanner initialized");
    return ESP_OK;
}

static esp_err_t gsm_start(void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    c->running = true;
    ESP_LOGI(TAG, "GSM scanner started (935 MHz, 1 MHz span)");
    return ESP_OK;
}

static esp_err_t gsm_stop(void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    c->running = false;
    ESP_LOGI(TAG, "GSM scanner stopped");
    return ESP_OK;
}

static void gsm_destroy(void *ctx) { (void)ctx; }

static void gsm_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
    /* TODO: GSM cell scanner
     * Reference: gr-gsm GNU Radio module, Osmocom GSM stack
     * 1. FFT power scan across 1 MHz span to find active ARFCN channels
     * 2. Goertzel filter for FCCH (frequency correction channel) burst detection
     * 3. SCH (synchronisation channel) burst decode: BSIC, frame number
     * 4. BCCH (broadcast control channel) System Information decode
     * 5. Extract MCC, MNC, LAC, CID from SI3/SI4 messages
     * 6. Measure RxLev per cell and publish via decode_bus_publish()
     */
}

static cJSON *gsm_get_status(void *ctx) {
    gsm_ctx_t *c = (gsm_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "cell_count", c->cell_count);
    }
    return j;
}

static cJSON *gsm_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "gsm_scanner") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_gsm_plugin = {
    .name = "gsm_scanner",
    .description = "GSM 900 Cell Scanner (FCCH/SCH/BCCH)",
    .category = "cellular",
    .demod_type = DEMOD_RAW_IQ,
    .center_freq_hz = 935000000,
    .bandwidth_hz = 1000000,
    .audio_rate_hz = 0,
    .init = gsm_init,
    .start = gsm_start,
    .stop = gsm_stop,
    .destroy = gsm_destroy,
    .process_iq = gsm_process_iq,
    .process_audio = NULL,
    .get_status = gsm_get_status,
    .get_results = gsm_get_results,
    .ctx = &s_gsm_ctx,
};

void register_gsm_decoder(void) {
    decoder_registry_add(&s_gsm_plugin);
}
