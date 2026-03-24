#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_common.h"

static const char *TAG = "decoder_gsm";

#define MAX_CELLS 32

static gsm_cell_t s_cells[MAX_CELLS];
static int s_cell_count = 0;
static SemaphoreHandle_t s_mutex;
static bool s_running = false;
static bool s_push_warned = false;

esp_err_t decoder_gsm_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_cells, 0, sizeof(s_cells));
    s_cell_count = 0;
    ESP_LOGI(TAG, "GSM scanner initialized (stub)");
    return ESP_OK;
}

esp_err_t decoder_gsm_start(void) {
    s_running = true;
    s_push_warned = false;
    ESP_LOGI(TAG, "GSM scanner started (900/1800 MHz)");
    return ESP_OK;
}

esp_err_t decoder_gsm_stop(void) {
    s_running = false;
    ESP_LOGI(TAG, "GSM scanner stopped");
    return ESP_OK;
}

void decoder_gsm_push_samples(const uint8_t *data, uint32_t len) {
    (void)data; (void)len;
    if (!s_push_warned) {
        s_push_warned = true;
        ESP_LOGI(TAG, "push_samples: not implemented");
    }
    /* TODO: Implement GSM cell scanner from IQ samples
     * Reference: gr-gsm GNU Radio out-of-tree module, Osmocom GSM stack
     * 1. Scan ARFCN channels in GSM-900 (935-960 MHz) and GSM-1800 (1805-1880 MHz)
     * 2. Detect FCCH (frequency correction) and SCH (synchronization) bursts
     * 3. Decode BCCH (broadcast control channel) System Information messages
     * 4. Extract MCC, MNC, LAC, CID from SI3/SI4 messages
     * 5. Measure signal strength (RxLev) per cell
     */
}

int decoder_gsm_get_cells(gsm_cell_t *out, int max_count) {
    if (!out || !s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = s_cell_count < max_count ? s_cell_count : max_count;
    memcpy(out, s_cells, n * sizeof(gsm_cell_t));
    xSemaphoreGive(s_mutex);
    return n;
}
