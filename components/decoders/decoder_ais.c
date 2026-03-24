#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_ais";

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    int vessel_count;
} ais_ctx_t;

static ais_ctx_t s_ais_ctx;

static esp_err_t ais_init(void *ctx) {
    ais_ctx_t *c = (ais_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->vessel_count = 0;
    ESP_LOGI(TAG, "AIS decoder initialized");
    return ESP_OK;
}

static esp_err_t ais_start(void *ctx) {
    ais_ctx_t *c = (ais_ctx_t *)ctx;
    c->running = true;
    ESP_LOGI(TAG, "AIS decoder started (162 MHz, GMSK 9600 baud)");
    return ESP_OK;
}

static esp_err_t ais_stop(void *ctx) {
    ais_ctx_t *c = (ais_ctx_t *)ctx;
    c->running = false;
    ESP_LOGI(TAG, "AIS decoder stopped");
    return ESP_OK;
}

static void ais_destroy(void *ctx) { (void)ctx; }

static void ais_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
    /* TODO: AIS dual-channel demodulation
     * Reference: libais, gr-ais GNU Radio module
     * 1. Dual-channel DDC: offset ±25 kHz for ch A (161.975) and ch B (162.025)
     * 2. GMSK demodulation (BT=0.4, 9600 baud) on each channel
     * 3. HDLC framing, NRZI decode, bit unstuffing
     * 4. 6-bit ASCII unpack for payload
     * 5. Decode ITU-R M.1371: types 1/2/3 (position), 5 (voyage), 18/19 (class B)
     * 6. Publish via decode_bus_publish()
     */
}

static void ais_process_audio(void *ctx, const int16_t *samples,
                               uint32_t count, uint32_t sample_rate) {
    (void)ctx; (void)samples; (void)count; (void)sample_rate;
    /* TODO: GMSK demod on FM-discriminated audio at 48 kHz */
}

static cJSON *ais_get_status(void *ctx) {
    ais_ctx_t *c = (ais_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "vessel_count", c->vessel_count);
    }
    return j;
}

static cJSON *ais_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "ais") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_ais_plugin = {
    .name = "ais",
    .description = "AIS Marine VHF Vessel Tracker (ITU-R M.1371)",
    .category = "marine",
    .demod_type = DEMOD_GFSK,
    .center_freq_hz = 162000000,
    .bandwidth_hz = 50000,
    .audio_rate_hz = 48000,
    .demod_params.fsk = { .shift_hz = 4800, .baud = 9600 },
    .init = ais_init,
    .start = ais_start,
    .stop = ais_stop,
    .destroy = ais_destroy,
    .process_iq = ais_process_iq,
    .process_audio = ais_process_audio,
    .get_status = ais_get_status,
    .get_results = ais_get_results,
    .ctx = &s_ais_ctx,
};

void register_ais_decoder(void) {
    decoder_registry_add(&s_ais_plugin);
}
