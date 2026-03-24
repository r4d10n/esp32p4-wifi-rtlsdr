#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_common.h"

static const char *TAG = "decoder_ais";

#define MAX_VESSELS 64

static ais_vessel_t s_vessels[MAX_VESSELS];
static int s_vessel_count = 0;
static SemaphoreHandle_t s_mutex;
static bool s_running = false;
static bool s_push_warned = false;

esp_err_t decoder_ais_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_vessels, 0, sizeof(s_vessels));
    s_vessel_count = 0;
    ESP_LOGI(TAG, "AIS decoder initialized (stub)");
    return ESP_OK;
}

esp_err_t decoder_ais_start(void) {
    s_running = true;
    s_push_warned = false;
    ESP_LOGI(TAG, "AIS decoder started (161.975/162.025 MHz GMSK)");
    return ESP_OK;
}

esp_err_t decoder_ais_stop(void) {
    s_running = false;
    ESP_LOGI(TAG, "AIS decoder stopped");
    return ESP_OK;
}

void decoder_ais_push_samples(const uint8_t *data, uint32_t len) {
    (void)data; (void)len;
    if (!s_push_warned) {
        s_push_warned = true;
        ESP_LOGI(TAG, "push_samples: not implemented");
    }
    /* TODO: Implement AIS GMSK demodulation from IQ samples
     * Reference: libais or gr-ais GNU Radio out-of-tree module
     * 1. FM discriminator on 161.975 MHz and 162.025 MHz channels
     * 2. GMSK demodulation (BT=0.4, 9600 baud)
     * 3. HDLC framing and NRZI decoding
     * 4. Decode ITU-R M.1371 message types 1/2/3 (position), 5 (voyage), 18/19 (class B)
     */
}

int decoder_ais_get_vessels(ais_vessel_t *out, int max_count) {
    if (!out || !s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = s_vessel_count < max_count ? s_vessel_count : max_count;
    memcpy(out, s_vessels, n * sizeof(ais_vessel_t));
    xSemaphoreGive(s_mutex);
    return n;
}
