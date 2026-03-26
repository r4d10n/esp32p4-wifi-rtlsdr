/*
 * I2S Audio Output for ESP32-P4 (Waveshare ESP32-P4-WIFI6)
 *
 * Drives the onboard ES8311 codec via I2S + I2C.
 * Supports 48 kHz 16-bit mono output to speaker and headphone jack.
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
#define AUDIO_ES8311_I2C_ADDR  0x30  /* 8-bit addr: esp_codec_dev right-shifts by 1 → 0x18 */

/* Audio output configuration */
typedef struct {
    uint32_t    sample_rate;        /* Audio sample rate (default 48000) */
    uint8_t     volume;             /* Initial volume 0-100 */
    bool        speaker_enable;     /* Enable onboard speaker amp */
} audio_out_config_t;

#define AUDIO_OUT_CONFIG_DEFAULT() { \
    .sample_rate = 48000, \
    .volume = 90, \
    .speaker_enable = true, \
}

/**
 * Initialize I2S TX channel and ES8311 codec.
 * Sets up I2C bus, configures ES8311 registers, enables I2S output.
 */
esp_err_t audio_out_init(const audio_out_config_t *config);

/**
 * Deinitialize audio output, disable speaker amp.
 */
esp_err_t audio_out_deinit(void);

/**
 * Write PCM audio samples to I2S DMA.
 *
 * @param samples   int16 mono PCM audio data
 * @param count     Number of samples
 * @param timeout_ms Maximum time to wait for DMA space (0 = non-blocking)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if DMA full
 */
esp_err_t audio_out_write(const int16_t *samples, int count, int timeout_ms);

/**
 * Write interleaved stereo PCM audio samples to I2S DMA.
 *
 * @param samples   int16 interleaved L/R stereo PCM data
 * @param frames    Number of stereo frames (each frame = L + R sample)
 * @param timeout_ms Maximum time to wait for DMA space (0 = non-blocking)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if DMA full
 */
esp_err_t audio_out_write_stereo(const int16_t *samples, int frames, int timeout_ms);

/**
 * Set output volume (0-100).
 * Controls ES8311 DAC digital volume register.
 */
esp_err_t audio_out_set_volume(uint8_t volume);

/**
 * Get current volume setting.
 */
uint8_t audio_out_get_volume(void);

/**
 * Enable/disable speaker amplifier (NS4150B via GPIO53).
 */
esp_err_t audio_out_set_speaker(bool enable);

/**
 * Mute/unmute audio output (preserves volume setting).
 */
esp_err_t audio_out_set_mute(bool mute);

/**
 * Switch I2S between mono and stereo slot modes.
 * When stereo=true, expects interleaved L/R int16 pairs.
 * When stereo=false (default), expects mono int16 samples.
 */
esp_err_t audio_out_set_stereo(bool stereo);

/**
 * Get the I2C master bus handle (for sharing with other I2C devices like OLED).
 * Returns NULL if audio_out_init() hasn't been called yet.
 */
void *audio_out_get_i2c_bus(void);

#ifdef __cplusplus
}
#endif
