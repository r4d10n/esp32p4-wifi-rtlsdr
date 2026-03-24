#include "rtl433.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>

/*
 * Supported protocol stubs (to be implemented via rtl_433 pulse detection):
 *   1.  Acurite         - 433 MHz, PWM/PPM OOK, temperature/humidity/wind
 *   2.  LaCrosse        - 433/868 MHz, FSK/OOK, temperature/humidity
 *   3.  Oregon Scientific - 433 MHz, Manchester OOK, temp/humidity/UV/rain/wind
 *   4.  Bresser         - 433/868 MHz, FSK, weather station sensors
 *   5.  Fine Offset     - 433 MHz, OOK PWM, temperature/humidity/rain
 *   6.  Hideki          - 433 MHz, Manchester OOK, temperature/humidity
 *   7.  Nexus           - 433 MHz, OOK PWM, temperature/humidity
 *   8.  Rubicson        - 433 MHz, OOK PWM, temperature
 *   9.  TFA Dostmann    - 433/868 MHz, FSK, weather sensors
 *  10.  Auriol          - 433 MHz, OOK PWM, temperature/humidity (Lidl brand)
 */

static const char *TAG = "rtl433";

static rtl433_device_t s_devices[RTL433_MAX_DEVICES];
static int s_device_count = 0;
static SemaphoreHandle_t s_mutex = NULL;
static bool s_running = false;

esp_err_t rtl433_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    memset(s_devices, 0, sizeof(s_devices));
    s_device_count = 0;
    s_running = false;
    ESP_LOGI(TAG, "rtl_433 decoder initialized (stub)");
    return ESP_OK;
}

esp_err_t rtl433_start(const rtl433_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "rtl_433 starting: sample_rate=%" PRIu32 " Hz, freq_count=%u, hop_interval=%us",
             config->sample_rate, config->freq_count, config->hop_interval_s);

    for (int i = 0; i < config->freq_count; i++) {
        ESP_LOGI(TAG, "  freq[%d] = %" PRIu32 " Hz", i, config->frequency[i]);
    }

    s_running = true;
    return ESP_OK;
}

esp_err_t rtl433_stop(void)
{
    s_running = false;
    ESP_LOGI(TAG, "rtl_433 stopped");
    return ESP_OK;
}

void rtl433_push_samples(const uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;

    if (!s_running) {
        return;
    }

    /*
     * TODO: Implement rtl_433 pulse detection pipeline:
     *
     *  1. AM envelope detection on 8-bit IQ samples (magnitude = sqrt(I^2 + Q^2),
     *     approximated as max(|I|,|Q|) + 0.4*min(|I|,|Q|) for speed).
     *  2. Pulse/gap measurement: track rising/falling edges against a threshold,
     *     record pulse_width and gap_width in microseconds.
     *  3. OOK demodulation: convert pulse/gap sequences to bit streams using
     *     PWM (short=0, long=1), PPM (short gap=0, long gap=1), or Manchester
     *     (edge mid-bit=1, no edge=0) encoding depending on protocol candidate.
     *  4. FSK demodulation: track instantaneous frequency deviation from carrier;
     *     positive deviation = 1, negative = 0 (for FSK protocols like LaCrosse TX141).
     *  5. Protocol matching: try each registered protocol decoder against the
     *     pulse stream; a match populates an rtl433_device_t entry.
     *  6. CRC/checksum verification per protocol spec before accepting decode.
     *  7. Store matched device under mutex, evict oldest if RTL433_MAX_DEVICES exceeded.
     *
     * Reference: https://github.com/merbanan/rtl_433
     */
}

int rtl433_get_devices(rtl433_device_t *out, int max_count)
{
    if (!out || max_count <= 0 || !s_mutex) {
        return 0;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int count = s_device_count < max_count ? s_device_count : max_count;
    memcpy(out, s_devices, count * sizeof(rtl433_device_t));
    xSemaphoreGive(s_mutex);

    return count;
}
