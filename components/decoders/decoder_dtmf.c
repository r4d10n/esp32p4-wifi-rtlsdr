#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

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
/* Used by process_audio when implemented */
static const uint16_t DTMF_FREQS[8] __attribute__((used)) = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
static const char DTMF_CHARS[4][4] __attribute__((used)) = {
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
} dtmf_ctx_t;

static dtmf_ctx_t s_dtmf_ctx;

static esp_err_t dtmf_init(void *ctx) {
    dtmf_ctx_t *c = (dtmf_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->last_digits[0] = '\0';
    c->digit_count = 0;
    ESP_LOGI(TAG, "DTMF decoder initialized (Goertzel, 8 freq)");
    return ESP_OK;
}

static esp_err_t dtmf_start(void *ctx) { ((dtmf_ctx_t *)ctx)->running = true; return ESP_OK; }
static esp_err_t dtmf_stop(void *ctx) { ((dtmf_ctx_t *)ctx)->running = false; return ESP_OK; }
static void dtmf_destroy(void *ctx) { (void)ctx; }

static void dtmf_process_audio(void *ctx, const int16_t *samples,
                                uint32_t count, uint32_t sample_rate) {
    (void)ctx; (void)samples; (void)count; (void)sample_rate;
    /* TODO: DTMF detection via Goertzel algorithm
     * Reference: multimon-ng/demod_dtmf.c
     *
     * 1. Run Goertzel filter for each of 8 DTMF frequencies
     *    Block size N = sample_rate / baud_rate (~20ms window)
     *    coeff = 2 * cos(2 * PI * freq / sample_rate)
     *    For each sample: s = sample + coeff*s1 - s2; s2=s1; s1=s
     *    Power = s1*s1 + s2*s2 - coeff*s1*s2
     *
     * 2. Find strongest row freq and strongest col freq
     * 3. Check power thresholds (relative and absolute)
     * 4. Debounce: require consistent detection over multiple blocks
     * 5. Decode: map (row, col) to DTMF_CHARS
     * 6. Publish decode_event_t with {digits: "123#"}
     */
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
