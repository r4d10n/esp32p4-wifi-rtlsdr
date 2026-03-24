#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_common.h"

static const char *TAG = "decoder_ft8";

#define MAX_DECODES 64

static ft8_decode_t s_decodes[MAX_DECODES];
static int s_decode_count = 0;
static SemaphoreHandle_t s_mutex;
static bool s_running = false;
static bool s_push_warned = false;

esp_err_t decoder_ft8_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_decodes, 0, sizeof(s_decodes));
    s_decode_count = 0;
    ESP_LOGI(TAG, "FT8/WSPR decoder initialized (stub)");
    return ESP_OK;
}

esp_err_t decoder_ft8_start(void) {
    s_running = true;
    s_push_warned = false;
    ESP_LOGI(TAG, "FT8/WSPR decoder started (HF bands, 15s/2min cycles)");
    return ESP_OK;
}

esp_err_t decoder_ft8_stop(void) {
    s_running = false;
    ESP_LOGI(TAG, "FT8/WSPR decoder stopped");
    return ESP_OK;
}

void decoder_ft8_push_samples(const uint8_t *data, uint32_t len) {
    (void)data; (void)len;
    if (!s_push_warned) {
        s_push_warned = true;
        ESP_LOGI(TAG, "push_samples: not implemented");
    }
    /* TODO: Implement FT8/WSPR demodulation from IQ samples
     * Reference: ft8_lib (lightweight FT8 decoder), wsjt-x source
     * FT8:
     * 1. Collect 15-second audio windows (12.5 Hz resolution FFT)
     * 2. Detect 8-FSK tones (6.25 Hz spacing, 79 symbol messages)
     * 3. Costas array sync, LDPC(174,91) decode
     * 4. Unpack 77-bit message: callsign, grid, report
     * WSPR:
     * 1. Collect 2-minute windows at 1500 Hz center (4-FSK, 1.4648 baud)
     * 2. Convolutional decode, interleaver, 50-bit message unpack
     * 3. Extract callsign, Maidenhead grid, power
     */
}

int decoder_ft8_get_decodes(ft8_decode_t *out, int max_count) {
    if (!out || !s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = s_decode_count < max_count ? s_decode_count : max_count;
    memcpy(out, s_decodes, n * sizeof(ft8_decode_t));
    xSemaphoreGive(s_mutex);
    return n;
}
