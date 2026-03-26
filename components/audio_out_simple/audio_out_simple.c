/*
 * Simple I2S Audio Output — ES8311 Codec via esp_codec_dev
 *
 * Waveshare ESP32-P4-WIFI6 board:
 *   ES8311 codec on I2C (0x18) + I2S0
 *   NS4150B Class-D amp on GPIO53
 *
 * I2S always runs in STEREO mode (ES8311 expects stereo I2S frames).
 * Mono audio is duplicated to both L/R channels before writing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_vol.h"
#include "audio_out_simple.h"

static const char *TAG = "audio_out";

/* ── State ── */

static i2s_chan_handle_t        s_i2s_tx = NULL;
static i2c_master_bus_handle_t  s_i2c_bus = NULL;
static esp_codec_dev_handle_t   s_codec = NULL;
static uint8_t                  s_volume = 70;
static bool                     s_muted = false;
static bool                     s_speaker_on = false;

/* Stereo interleave buffer for mono->stereo conversion */
#define STEREO_BUF_FRAMES  512
static int16_t s_stereo_buf[STEREO_BUF_FRAMES * 2];

/* ── Speaker amp control ── */

static void pa_ctrl_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AUDIO_PA_CTRL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(AUDIO_PA_CTRL_GPIO, 0);
}

/* ── Public API ── */

esp_err_t audio_out_simple_init(const audio_out_simple_config_t *config)
{
    esp_err_t ret;
    s_volume = config->volume;

    ESP_LOGI(TAG, "Initializing audio output: %lu Hz, volume=%d%%",
             (unsigned long)config->sample_rate, config->volume);

    /* 1. Initialize I2C bus */
    i2c_master_bus_config_t i2c_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = AUDIO_I2C_SDA_GPIO,
        .scl_io_num = AUDIO_I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&i2c_bus_cfg, &s_i2c_bus);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2C bus init failed");

    /* Scan I2C bus */
    ESP_LOGI(TAG, "Scanning I2C bus (SDA=%d SCL=%d)...", AUDIO_I2C_SDA_GPIO, AUDIO_I2C_SCL_GPIO);
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        esp_err_t probe_ret = i2c_master_probe(s_i2c_bus, addr, 50);
        if (probe_ret == ESP_OK) {
            ESP_LOGI(TAG, "  I2C device found at 0x%02x", addr);
        }
    }

    /* 2. Initialize I2S TX channel — always STEREO */
    i2s_chan_config_t i2s_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_cfg.auto_clear = true;
    i2s_cfg.dma_desc_num = 6;
    i2s_cfg.dma_frame_num = 240;
    ret = i2s_new_channel(&i2s_cfg, &s_i2s_tx, NULL);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2S channel create failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = AUDIO_I2S_MCLK_GPIO,
            .bclk = AUDIO_I2S_BCLK_GPIO,
            .ws   = AUDIO_I2S_WS_GPIO,
            .dout = AUDIO_I2S_DOUT_GPIO,
            .din  = AUDIO_I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    ret = i2s_channel_init_std_mode(s_i2s_tx, &std_cfg);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2S std mode init failed");

    ret = i2s_channel_enable(s_i2s_tx);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2S channel enable failed");

    /* 3. Initialize ES8311 codec via esp_codec_dev */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM_0,
        .addr = AUDIO_ES8311_I2C_ADDR,
        .bus_handle = s_i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(ctrl_if, ESP_FAIL, TAG, "I2C ctrl interface failed");

    audio_codec_i2s_cfg_t i2s_data_cfg = {
        .port = I2S_NUM_0,
        .tx_handle = s_i2s_tx,
        .rx_handle = NULL,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_data_cfg);
    ESP_RETURN_ON_FALSE(data_if, ESP_FAIL, TAG, "I2S data interface failed");

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if, ESP_FAIL, TAG, "GPIO interface failed");

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,
        .use_mclk = true,
        .pa_pin = AUDIO_PA_CTRL_GPIO,
        .pa_reverted = false,
        .hw_gain = {
            .pa_voltage = 5.0,
            .codec_dac_voltage = 3.3,
        },
        .mclk_div = I2S_MCLK_MULTIPLE_256,
    };
    const audio_codec_if_t *es8311_if = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(es8311_if, ESP_FAIL, TAG, "ES8311 codec create failed");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_if,
        .data_if = data_if,
    };
    s_codec = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_codec, ESP_FAIL, TAG, "Codec device create failed");

    esp_codec_dev_sample_info_t sample_cfg = {
        .bits_per_sample = 16,
        .channel = 2,
        .channel_mask = 0x03,
        .sample_rate = config->sample_rate,
    };
    ret = esp_codec_dev_open(s_codec, &sample_cfg);
    ESP_RETURN_ON_ERROR(ret, TAG, "Codec device open failed");

    ret = esp_codec_dev_set_out_vol(s_codec, config->volume);
    ESP_RETURN_ON_ERROR(ret, TAG, "Set volume failed");

    /* 4. Speaker amp GPIO */
    pa_ctrl_init();
    if (config->speaker_enable) {
        gpio_set_level(AUDIO_PA_CTRL_GPIO, 1);
        s_speaker_on = true;
        ESP_LOGI(TAG, "Speaker amplifier enabled (GPIO%d)", AUDIO_PA_CTRL_GPIO);
    }

    ESP_LOGI(TAG, "Audio output ready: I2S0 -> ES8311 -> %s",
             s_speaker_on ? "Speaker+Headphone" : "Headphone only");
    return ESP_OK;
}

esp_err_t audio_out_simple_deinit(void)
{
    gpio_set_level(AUDIO_PA_CTRL_GPIO, 0);
    s_speaker_on = false;

    if (s_codec) {
        esp_codec_dev_close(s_codec);
        s_codec = NULL;
    }

    if (s_i2s_tx) {
        i2s_channel_disable(s_i2s_tx);
        i2s_del_channel(s_i2s_tx);
        s_i2s_tx = NULL;
    }

    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }

    ESP_LOGI(TAG, "Audio output deinitialized");
    return ESP_OK;
}

esp_err_t audio_out_simple_write(const int16_t *samples, int count, int timeout_ms)
{
    if (!s_i2s_tx || !samples || count <= 0) return ESP_ERR_INVALID_STATE;

    /* Convert mono to stereo (duplicate L->R) and write in chunks */
    const int16_t *src = samples;
    int remaining = count;

    while (remaining > 0) {
        int chunk = remaining > STEREO_BUF_FRAMES ? STEREO_BUF_FRAMES : remaining;

        if (s_muted) {
            memset(s_stereo_buf, 0, chunk * 2 * sizeof(int16_t));
        } else {
            for (int i = 0; i < chunk; i++) {
                s_stereo_buf[i * 2]     = src[i];  /* Left */
                s_stereo_buf[i * 2 + 1] = src[i];  /* Right = same */
            }
        }

        size_t bytes_written = 0;
        esp_err_t ret = i2s_channel_write(s_i2s_tx, s_stereo_buf,
                                           chunk * 2 * sizeof(int16_t),
                                           &bytes_written, timeout_ms);
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) return ret;

        src += chunk;
        remaining -= chunk;
    }

    return ESP_OK;
}

esp_err_t audio_out_simple_set_volume(uint8_t volume)
{
    if (volume > 100) volume = 100;
    s_volume = volume;

    if (s_codec) {
        return esp_codec_dev_set_out_vol(s_codec, volume);
    }
    return ESP_OK;
}

uint8_t audio_out_simple_get_volume(void)
{
    return s_volume;
}

esp_err_t audio_out_simple_set_speaker(bool enable)
{
    gpio_set_level(AUDIO_PA_CTRL_GPIO, enable ? 1 : 0);
    s_speaker_on = enable;
    ESP_LOGI(TAG, "Speaker amp %s", enable ? "ON" : "OFF");
    return ESP_OK;
}

esp_err_t audio_out_simple_set_mute(bool mute)
{
    s_muted = mute;
    if (s_codec) {
        esp_codec_dev_set_out_mute(s_codec, mute);
    }
    ESP_LOGI(TAG, "Audio %s", mute ? "MUTED" : "unmuted");
    return ESP_OK;
}
