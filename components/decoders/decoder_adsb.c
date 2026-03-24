#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_common.h"

static const char *TAG = "decoder_adsb";

#define MAX_AIRCRAFT 64

static adsb_aircraft_t s_aircraft[MAX_AIRCRAFT];
static int s_aircraft_count = 0;
static SemaphoreHandle_t s_mutex;
static bool s_running = false;
static bool s_push_warned = false;

esp_err_t decoder_adsb_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_aircraft, 0, sizeof(s_aircraft));
    s_aircraft_count = 0;
    ESP_LOGI(TAG, "ADS-B decoder initialized (stub)");
    return ESP_OK;
}

esp_err_t decoder_adsb_start(void) {
    s_running = true;
    s_push_warned = false;
    ESP_LOGI(TAG, "ADS-B decoder started (1090 MHz)");
    return ESP_OK;
}

esp_err_t decoder_adsb_stop(void) {
    s_running = false;
    ESP_LOGI(TAG, "ADS-B decoder stopped");
    return ESP_OK;
}

void decoder_adsb_push_samples(const uint8_t *data, uint32_t len) {
    (void)data; (void)len;
    if (!s_push_warned) {
        s_push_warned = true;
        ESP_LOGI(TAG, "push_samples: not implemented");
    }
    /* TODO: Implement Mode S demodulation from IQ samples
     * Reference: esp32p4-adsb/components/adsb/adsb_decode.c
     * 1. Detect 1090 MHz Mode S preamble (8 us)
     * 2. Demodulate 112-bit message (PPM encoding)
     * 3. CRC check (24-bit)
     * 4. Decode DF17 (ADS-B) messages: position, velocity, identification
     */
}

int decoder_adsb_get_aircraft(adsb_aircraft_t *out, int max_count) {
    if (!out || !s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = s_aircraft_count < max_count ? s_aircraft_count : max_count;
    memcpy(out, s_aircraft, n * sizeof(adsb_aircraft_t));
    xSemaphoreGive(s_mutex);
    return n;
}
