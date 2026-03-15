/*
 * ESP32-P4 RTL-SDR USB Host Driver
 *
 * Drives RTL2832U-based DVB-T dongles via USB Host (High-Speed).
 * Ported from librtlsdr (https://github.com/steve-m/librtlsdr)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RTL2832U vendor control transfer constants */
#define RTLSDR_USB_VID          0x0BDA
#define RTLSDR_USB_PID_GENERIC  0x2832
#define RTLSDR_USB_PID_BLOG     0x2838

#define RTLSDR_CTRL_TIMEOUT_MS  300
#define RTLSDR_BULK_EP          0x81

/* Register blocks for vendor control transfers */
#define RTLSDR_BLOCK_DEMOD      0x000
#define RTLSDR_BLOCK_USB        0x100
#define RTLSDR_BLOCK_SYS        0x200
#define RTLSDR_BLOCK_TUNER      0x300
#define RTLSDR_BLOCK_ROM        0x400
#define RTLSDR_BLOCK_IR         0x500
#define RTLSDR_BLOCK_IIC        0x600

/* Tuner types */
typedef enum {
    RTLSDR_TUNER_UNKNOWN = 0,
    RTLSDR_TUNER_E4000,
    RTLSDR_TUNER_FC0012,
    RTLSDR_TUNER_FC0013,
    RTLSDR_TUNER_FC2580,
    RTLSDR_TUNER_R820T,
    RTLSDR_TUNER_R828D,
} rtlsdr_tuner_t;

/* Async read callback */
typedef void (*rtlsdr_read_cb_t)(uint8_t *buf, uint32_t len, void *ctx);

/* Device handle */
typedef struct rtlsdr_dev rtlsdr_dev_t;

/* Device configuration */
typedef struct {
    uint32_t sample_rate;       /* Samples per second (225001-300000 or 900001-3200000) */
    uint32_t center_freq;       /* Center frequency in Hz */
    int      gain;              /* Gain in tenths of dB, or 0 for auto */
    bool     agc_mode;          /* RTL2832U internal AGC */
    bool     bias_tee;          /* Bias-T power on antenna */
    int      freq_correction;   /* Frequency correction in PPM */
    int      direct_sampling;   /* 0=off, 1=I-ADC, 2=Q-ADC */
    bool     offset_tuning;     /* Offset tuning mode */
} rtlsdr_config_t;

#define RTLSDR_CONFIG_DEFAULT() { \
    .sample_rate = 1024000, \
    .center_freq = 100000000, \
    .gain = 0, \
    .agc_mode = true, \
    .bias_tee = false, \
    .freq_correction = 0, \
    .direct_sampling = 0, \
    .offset_tuning = false, \
}

/*
 * Initialize the RTL-SDR USB host driver.
 * Must be called after USB host library is installed.
 * Blocks until an RTL-SDR device is connected and initialized.
 */
esp_err_t rtlsdr_init(rtlsdr_dev_t **dev);

/*
 * Deinitialize and release the RTL-SDR device.
 */
esp_err_t rtlsdr_deinit(rtlsdr_dev_t *dev);

/*
 * Apply configuration to the device.
 */
esp_err_t rtlsdr_configure(rtlsdr_dev_t *dev, const rtlsdr_config_t *config);

/*
 * Set individual parameters.
 */
esp_err_t rtlsdr_set_center_freq(rtlsdr_dev_t *dev, uint32_t freq);
esp_err_t rtlsdr_set_sample_rate(rtlsdr_dev_t *dev, uint32_t rate);
esp_err_t rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t *dev, int manual);
esp_err_t rtlsdr_set_tuner_gain(rtlsdr_dev_t *dev, int gain);
esp_err_t rtlsdr_set_agc_mode(rtlsdr_dev_t *dev, bool on);
esp_err_t rtlsdr_set_freq_correction(rtlsdr_dev_t *dev, int ppm);
esp_err_t rtlsdr_set_direct_sampling(rtlsdr_dev_t *dev, int on);
esp_err_t rtlsdr_set_offset_tuning(rtlsdr_dev_t *dev, bool on);
esp_err_t rtlsdr_set_bias_tee(rtlsdr_dev_t *dev, bool on);
esp_err_t rtlsdr_set_if_gain(rtlsdr_dev_t *dev, int stage, int gain);
esp_err_t rtlsdr_set_test_mode(rtlsdr_dev_t *dev, bool on);

/*
 * Query device state.
 */
uint32_t        rtlsdr_get_center_freq(rtlsdr_dev_t *dev);
uint32_t        rtlsdr_get_sample_rate(rtlsdr_dev_t *dev);
int             rtlsdr_get_tuner_gain(rtlsdr_dev_t *dev);
rtlsdr_tuner_t  rtlsdr_get_tuner_type(rtlsdr_dev_t *dev);
const int      *rtlsdr_get_tuner_gains(rtlsdr_dev_t *dev, int *count);

/*
 * Start async bulk reading. Calls cb for each filled buffer.
 * This function blocks until rtlsdr_stop_async() is called.
 */
esp_err_t rtlsdr_read_async(rtlsdr_dev_t *dev, rtlsdr_read_cb_t cb,
                            void *ctx, uint32_t buf_num, uint32_t buf_len);

/*
 * Stop async bulk reading (thread-safe).
 */
esp_err_t rtlsdr_stop_async(rtlsdr_dev_t *dev);

/*
 * Reset the bulk endpoint buffer.
 */
esp_err_t rtlsdr_reset_buffer(rtlsdr_dev_t *dev);

#ifdef __cplusplus
}
#endif
