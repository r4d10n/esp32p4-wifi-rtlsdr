#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_pocsag";

#define POCSAG_SYNC_WORD   0x7CD215D8
#define POCSAG_IDLE_WORD   0x7A89C197

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    uint32_t baud_rate;
    int page_count;
} pocsag_ctx_t;

static pocsag_ctx_t s_pocsag_512_ctx = { .baud_rate = 512 };
static pocsag_ctx_t s_pocsag_1200_ctx = { .baud_rate = 1200 };
static pocsag_ctx_t s_pocsag_2400_ctx = { .baud_rate = 2400 };

static esp_err_t pocsag_init(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->page_count = 0;
    ESP_LOGI(TAG, "POCSAG %lu baud decoder initialized", (unsigned long)c->baud_rate);
    return ESP_OK;
}

static esp_err_t pocsag_start(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    c->running = true;
    return ESP_OK;
}

static esp_err_t pocsag_stop(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    c->running = false;
    return ESP_OK;
}

static void pocsag_destroy(void *ctx) { (void)ctx; }

static void pocsag_process_audio(void *ctx, const int16_t *samples,
                                  uint32_t count, uint32_t sample_rate) {
    (void)ctx; (void)samples; (void)count; (void)sample_rate;
    /* TODO: POCSAG demodulation
     * Reference: multimon-ng/demod_poc12.c, multimon-ng/demod_poc24.c
     *
     * 1. FSK demodulation: ±4500 Hz shift
     * 2. Clock recovery: PLL locked to baud rate
     * 3. Bit slicing: comparator threshold
     * 4. Sync word detection: correlate against 0x7CD215D8 (32-bit)
     * 5. Frame parsing: 16 codewords per batch (2 batches per frame)
     *    - Each codeword: 32 bits (1 flag + 20 data + 10 BCH + 1 parity)
     *    - Flag bit 0 = address, 1 = message
     * 6. BCH(31,21) error correction (up to 2-bit correction)
     * 7. Address extraction: 18-bit RIC + 2-bit function
     * 8. Message decode:
     *    - Numeric: 4-bit BCD digits (0-9, *, U, space, -, ], [)
     *    - Alpha: 7-bit ASCII characters (LSB first)
     * 9. Publish via decode_bus_publish()
     */
}

static void pocsag_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

static cJSON *pocsag_get_status(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "baud_rate", c->baud_rate);
        cJSON_AddNumberToObject(j, "page_count", c->page_count);
    }
    return j;
}

static cJSON *pocsag_get_results(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    char name[16];
    snprintf(name, sizeof(name), "pocsag_%lu", (unsigned long)c->baud_rate);
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), name) :
        cJSON_CreateArray();
}

static decoder_plugin_t s_pocsag_512 = {
    .name = "pocsag_512",
    .description = "POCSAG 512 baud Pager Decoder",
    .category = "pager",
    .demod_type = DEMOD_FSK,
    .center_freq_hz = 152000000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 22050,
    .demod_params.fsk = { .shift_hz = 4500, .baud = 512 },
    .init = pocsag_init, .start = pocsag_start, .stop = pocsag_stop, .destroy = pocsag_destroy,
    .process_audio = pocsag_process_audio, .process_iq = pocsag_process_iq,
    .get_status = pocsag_get_status, .get_results = pocsag_get_results,
    .ctx = &s_pocsag_512_ctx,
};

static decoder_plugin_t s_pocsag_1200 = {
    .name = "pocsag_1200",
    .description = "POCSAG 1200 baud Pager Decoder",
    .category = "pager",
    .demod_type = DEMOD_FSK,
    .center_freq_hz = 152000000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 22050,
    .demod_params.fsk = { .shift_hz = 4500, .baud = 1200 },
    .init = pocsag_init, .start = pocsag_start, .stop = pocsag_stop, .destroy = pocsag_destroy,
    .process_audio = pocsag_process_audio, .process_iq = pocsag_process_iq,
    .get_status = pocsag_get_status, .get_results = pocsag_get_results,
    .ctx = &s_pocsag_1200_ctx,
};

static decoder_plugin_t s_pocsag_2400 = {
    .name = "pocsag_2400",
    .description = "POCSAG 2400 baud Pager Decoder",
    .category = "pager",
    .demod_type = DEMOD_FSK,
    .center_freq_hz = 152000000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 22050,
    .demod_params.fsk = { .shift_hz = 4500, .baud = 2400 },
    .init = pocsag_init, .start = pocsag_start, .stop = pocsag_stop, .destroy = pocsag_destroy,
    .process_audio = pocsag_process_audio, .process_iq = pocsag_process_iq,
    .get_status = pocsag_get_status, .get_results = pocsag_get_results,
    .ctx = &s_pocsag_2400_ctx,
};

void register_pocsag_decoders(void) {
    decoder_registry_add(&s_pocsag_512);
    decoder_registry_add(&s_pocsag_1200);
    decoder_registry_add(&s_pocsag_2400);
}
