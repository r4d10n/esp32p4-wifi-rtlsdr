#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define RTLPOWER_MAX_BINS 4096

typedef struct {
    int64_t timestamp_ms;
    uint32_t freq_start_hz;
    uint32_t freq_stop_hz;
    uint32_t bin_size_hz;
    uint16_t num_bins;
    float *power_dbm;          /* Array of dBm values, heap-allocated */
} rtlpower_sweep_t;

typedef struct {
    bool enable;
    uint32_t freq_start_hz;
    uint32_t freq_stop_hz;
    uint32_t bin_size_hz;
    uint16_t interval_s;
    char window_func[16];       /* "hamming", "blackman", etc. */
    bool cloud_upload_enable;
    char cloud_provider[16];
    char cloud_path[64];
    uint16_t upload_interval_min;
    bool generate_spectrogram;
} rtlpower_config_t;

#define RTLPOWER_CONFIG_DEFAULT() { \
    .enable = false, \
    .freq_start_hz = 88000000, \
    .freq_stop_hz = 108000000, \
    .bin_size_hz = 10000, \
    .interval_s = 60, \
    .window_func = "hamming", \
    .cloud_upload_enable = false, \
    .cloud_provider = "google_drive", \
    .cloud_path = "/rtl_power/", \
    .upload_interval_min = 60, \
    .generate_spectrogram = true, \
}

esp_err_t rtlpower_init(void);
esp_err_t rtlpower_start(const rtlpower_config_t *config);
esp_err_t rtlpower_stop(void);
void rtlpower_push_samples(const uint8_t *data, uint32_t len);

/* Get most recent sweep. Caller must free sweep->power_dbm. Returns ESP_ERR_NOT_FOUND if no data. */
esp_err_t rtlpower_get_latest_sweep(rtlpower_sweep_t *sweep);
