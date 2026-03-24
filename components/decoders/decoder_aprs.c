#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_common.h"

static const char *TAG = "decoder_aprs";

#define MAX_STATIONS 64

static aprs_station_t s_stations[MAX_STATIONS];
static int s_station_count = 0;
static SemaphoreHandle_t s_mutex;
static bool s_running = false;
static bool s_push_warned = false;

esp_err_t decoder_aprs_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_stations, 0, sizeof(s_stations));
    s_station_count = 0;
    ESP_LOGI(TAG, "APRS decoder initialized (stub)");
    return ESP_OK;
}

esp_err_t decoder_aprs_start(void) {
    s_running = true;
    s_push_warned = false;
    ESP_LOGI(TAG, "APRS decoder started (144.390 MHz AFSK)");
    return ESP_OK;
}

esp_err_t decoder_aprs_stop(void) {
    s_running = false;
    ESP_LOGI(TAG, "APRS decoder stopped");
    return ESP_OK;
}

void decoder_aprs_push_samples(const uint8_t *data, uint32_t len) {
    (void)data; (void)len;
    if (!s_push_warned) {
        s_push_warned = true;
        ESP_LOGI(TAG, "push_samples: not implemented");
    }
    /* TODO: Implement APRS AFSK demodulation from IQ samples
     * Reference: direwolf TNC software, multimon-ng
     * 1. FM discriminator on 144.390 MHz
     * 2. Bell 202 AFSK demodulation (1200/2200 Hz, 1200 baud)
     * 3. HDLC framing and NRZI decoding
     * 4. AX.25 packet decode
     * 5. Parse APRS position, weather, telemetry, message packets
     */
}

int decoder_aprs_get_stations(aprs_station_t *out, int max_count) {
    if (!out || !s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = s_station_count < max_count ? s_station_count : max_count;
    memcpy(out, s_stations, n * sizeof(aprs_station_t));
    xSemaphoreGive(s_mutex);
    return n;
}
