/*
 * Simple I2S Audio Output for ESP32-P4 (Waveshare ESP32-P4-WIFI6)
 *
 * Drives the onboard ES8311 codec via I2S + I2C.
 * 48 kHz 16-bit mono output to speaker and headphone jack.
 *
 * Hardware: ES8311 codec (I2C 7-bit addr 0x18) + NS4150B speaker amp (GPIO53)
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

/* Waveshare ESP32-P4-WIFI6 audio GPIO assignments */
#define AUDIO_I2S_MCLK_GPIO    13
#define AUDIO_I2S_BCLK_GPIO    12
#define AUDIO_I2S_WS_GPIO      10
#define AUDIO_I2S_DOUT_GPIO    9
#define AUDIO_I2S_DIN_GPIO     11
#define AUDIO_I2C_SDA_GPIO     7
#define AUDIO_I2C_SCL_GPIO     8
#define AUDIO_PA_CTRL_GPIO     53
#define AUDIO_ES8311_I2C_ADDR  0x30  /* 8-bit addr: esp_codec_dev right-shifts by 1 -> 0x18 */

/* Audio output configuration */
typedef struct {
    uint32_t    sample_rate;        /* Audio sample rate (default 48000) */
    uint8_t     volume;             /* Initial volume 0-100 */
    bool        speaker_enable;     /* Enable onboard speaker amp */
} audio_out_simple_config_t;

#define AUDIO_OUT_SIMPLE_CONFIG_DEFAULT() { \
    .sample_rate = 48000, \
    .volume = 90, \
    .speaker_enable = true, \
}

esp_err_t audio_out_simple_init(const audio_out_simple_config_t *config);
esp_err_t audio_out_simple_deinit(void);
esp_err_t audio_out_simple_write(const int16_t *samples, int count, int timeout_ms);
esp_err_t audio_out_simple_set_volume(uint8_t volume);
uint8_t   audio_out_simple_get_volume(void);
esp_err_t audio_out_simple_set_speaker(bool enable);
esp_err_t audio_out_simple_set_mute(bool mute);

#ifdef __cplusplus
}
#endif
