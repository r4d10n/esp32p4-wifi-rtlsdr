#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define RTL433_MAX_DEVICES 32

typedef struct {
    uint16_t protocol_id;
    char model[32];
    uint32_t device_id;
    char data_json[256];    /* JSON string with decoded fields (temp, humidity, etc.) */
    int8_t rssi_db;
    int64_t timestamp_ms;
} rtl433_device_t;

typedef struct {
    bool enable;
    uint32_t frequency[4];
    uint8_t freq_count;
    uint16_t hop_interval_s;
    uint32_t sample_rate;
} rtl433_config_t;

#define RTL433_CONFIG_DEFAULT() { \
    .enable = false, \
    .frequency = {433920000, 315000000, 0, 0}, \
    .freq_count = 2, \
    .hop_interval_s = 600, \
    .sample_rate = 250000, \
}

esp_err_t rtl433_init(void);
esp_err_t rtl433_start(const rtl433_config_t *config);
esp_err_t rtl433_stop(void);
void rtl433_push_samples(const uint8_t *data, uint32_t len);
int rtl433_get_devices(rtl433_device_t *out, int max_count);
