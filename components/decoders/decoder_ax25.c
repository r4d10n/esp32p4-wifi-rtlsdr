#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_ax25";

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    uint32_t baud_rate;
    int packet_count;
} ax25_ctx_t;

static ax25_ctx_t s_ax25_300_ctx  = { .baud_rate = 300 };
static ax25_ctx_t s_ax25_1200_ctx = { .baud_rate = 1200 };
static ax25_ctx_t s_ax25_9600_ctx = { .baud_rate = 9600 };

static esp_err_t ax25_init(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->packet_count = 0;
    ESP_LOGI(TAG, "AX.25 %lu baud decoder initialized", (unsigned long)c->baud_rate);
    return ESP_OK;
}

static esp_err_t ax25_start(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    c->running = true;
    return ESP_OK;
}

static esp_err_t ax25_stop(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    c->running = false;
    return ESP_OK;
}

static void ax25_destroy(void *ctx) { (void)ctx; }

static void ax25_process_audio(void *ctx, const int16_t *samples,
                                uint32_t count, uint32_t sample_rate) {
    (void)ctx; (void)samples; (void)count; (void)sample_rate;
    /* TODO: AX.25 AFSK demodulation
     * Reference: direwolf/src/demod_afsk.c, multimon-ng/demod_afsk12.c
     *
     * For 1200 baud (Bell 202):
     *   mark=1200Hz, space=2200Hz
     *   Correlator or PLL-based demodulation
     *   NRZI decode -> bit unstuffing -> AX.25 HDLC frame
     *
     * For 300 baud (Bell 103):
     *   mark=1270Hz, space=1070Hz
     *   Same HDLC framing
     *
     * For 9600 baud (G3RUH):
     *   Direct FSK, no AFSK tones
     *   Scrambler/descrambler (x^12 + x^5 + 1)
     *   Clock recovery via zero-crossing
     *
     * After frame decode:
     * 1. Parse AX.25: src/dst callsign, digipeater path, info field
     * 2. Check if info field is APRS (starts with !, /, @, =, etc.)
     * 3. If APRS: parse position, weather, message, telemetry
     * 4. Publish decode_event_t via decode_bus_publish()
     * 5. Upsert to global tracking table (key = callsign-SSID)
     */
}

/* Also accept IQ for decoders where DDC isn't ready yet */
static void ax25_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

static cJSON *ax25_get_status(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "baud_rate", c->baud_rate);
        cJSON_AddNumberToObject(j, "packet_count", c->packet_count);
    }
    return j;
}

static cJSON *ax25_get_results(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    char name[16];
    snprintf(name, sizeof(name), "ax25_%lu", (unsigned long)c->baud_rate);
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), name) :
        cJSON_CreateArray();
}

/* Three plugin instances */
static decoder_plugin_t s_ax25_300 = {
    .name = "ax25_300",
    .description = "AX.25 300 baud HF Packet (Bell 103 AFSK)",
    .category = "packet",
    .demod_type = DEMOD_FM_NARROW,
    .center_freq_hz = 7101000,
    .bandwidth_hz = 3000,
    .audio_rate_hz = 11025,
    .demod_params.afsk = { .mark_hz = 1270, .space_hz = 1070 },
    .init = ax25_init, .start = ax25_start, .stop = ax25_stop, .destroy = ax25_destroy,
    .process_audio = ax25_process_audio, .process_iq = ax25_process_iq,
    .get_status = ax25_get_status, .get_results = ax25_get_results,
    .ctx = &s_ax25_300_ctx,
};

static decoder_plugin_t s_ax25_1200 = {
    .name = "ax25_1200",
    .description = "AX.25 1200 baud VHF Packet / APRS (Bell 202 AFSK)",
    .category = "packet",
    .demod_type = DEMOD_FM_NARROW,
    .center_freq_hz = 144390000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 22050,
    .demod_params.afsk = { .mark_hz = 1200, .space_hz = 2200 },
    .init = ax25_init, .start = ax25_start, .stop = ax25_stop, .destroy = ax25_destroy,
    .process_audio = ax25_process_audio, .process_iq = ax25_process_iq,
    .get_status = ax25_get_status, .get_results = ax25_get_results,
    .ctx = &s_ax25_1200_ctx,
};

static decoder_plugin_t s_ax25_9600 = {
    .name = "ax25_9600",
    .description = "AX.25 9600 baud VHF/UHF Packet (G3RUH FSK)",
    .category = "packet",
    .demod_type = DEMOD_FSK,
    .center_freq_hz = 144390000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 48000,
    .demod_params.fsk = { .shift_hz = 3000, .baud = 9600 },
    .init = ax25_init, .start = ax25_start, .stop = ax25_stop, .destroy = ax25_destroy,
    .process_audio = ax25_process_audio, .process_iq = ax25_process_iq,
    .get_status = ax25_get_status, .get_results = ax25_get_results,
    .ctx = &s_ax25_9600_ctx,
};

void register_ax25_decoders(void) {
    decoder_registry_add(&s_ax25_300);
    decoder_registry_add(&s_ax25_1200);
    decoder_registry_add(&s_ax25_9600);
}
