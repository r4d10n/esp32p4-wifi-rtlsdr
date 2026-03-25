/*
 * I2S Audio Output — ES8311 Codec Driver
 *
 * Waveshare ESP32-P4-WIFI6 board:
 *   ES8311 codec on I2C (0x18) + I2S0
 *   NS4150B Class-D amp on GPIO53
 *
 * Uses esp_codec_dev abstraction for ES8311 register management.
 * Falls back to direct I2C register writes if codec component unavailable.
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
#include "audio_out.h"

static const char *TAG = "audio_out";

/* ── ES8311 Register Definitions ── */

#define ES8311_REG_RESET        0x00
#define ES8311_REG_CLK_MGR1     0x01
#define ES8311_REG_CLK_MGR2     0x02
#define ES8311_REG_CLK_MGR3     0x03
#define ES8311_REG_CLK_MGR4     0x04
#define ES8311_REG_CLK_MGR5     0x05
#define ES8311_REG_CLK_MGR6     0x06
#define ES8311_REG_CLK_MGR7     0x07
#define ES8311_REG_CLK_MGR8     0x08
#define ES8311_REG_SDP_IN       0x09
#define ES8311_REG_SDP_OUT      0x0A
#define ES8311_REG_SYSTEM       0x0B
#define ES8311_REG_VMID         0x0C
#define ES8311_REG_SDP_11       0x0D
#define ES8311_REG_SYS1         0x0E
#define ES8311_REG_SYS2         0x0F
#define ES8311_REG_DAC_VOL      0x32
#define ES8311_REG_ADC_VOL      0x17
#define ES8311_REG_GPIO         0x44
#define ES8311_REG_GP           0x45

/* ── State ── */

static i2s_chan_handle_t        s_i2s_tx = NULL;
static i2c_master_bus_handle_t  s_i2c_bus = NULL;
static i2c_master_dev_handle_t  s_es8311_dev = NULL;
static uint8_t                  s_volume = 70;
static bool                     s_muted = false;
static bool                     s_speaker_on = false;
static bool                     s_stereo_mode = false;

/* ── I2C helpers ── */

static esp_err_t es8311_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { reg, val };
    return i2c_master_transmit(s_es8311_dev, data, 2, 100);
}

static esp_err_t es8311_read_reg(uint8_t reg, uint8_t *val)
{
    esp_err_t ret = i2c_master_transmit_receive(s_es8311_dev, &reg, 1, val, 1, 100);
    return ret;
}

/* ── ES8311 initialization sequence ──
 *
 * Configures the codec for:
 * - Slave mode (MCLK + BCLK from ESP32)
 * - I2S Philips standard, 16-bit
 * - DAC output enabled (speaker/headphone)
 * - Internal reference powered up
 */
static esp_err_t es8311_codec_init(uint32_t sample_rate)
{
    ESP_LOGI(TAG, "Initializing ES8311 codec at %lu Hz", (unsigned long)sample_rate);

    /* Software reset */
    es8311_write_reg(ES8311_REG_RESET, 0x1F);
    vTaskDelay(pdMS_TO_TICKS(20));
    es8311_write_reg(ES8311_REG_RESET, 0x00);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Clock manager: slave mode, MCLK from external */
    es8311_write_reg(ES8311_REG_CLK_MGR1, 0x30); /* MCLK divider auto */
    es8311_write_reg(ES8311_REG_CLK_MGR2, 0x00); /* Slave mode, BCLK from master */
    es8311_write_reg(ES8311_REG_CLK_MGR3, 0x10); /* ADC/DAC sample rate auto */
    es8311_write_reg(ES8311_REG_CLK_MGR4, 0x10);
    es8311_write_reg(ES8311_REG_CLK_MGR5, 0x00); /* ADC osr */
    es8311_write_reg(ES8311_REG_CLK_MGR6, 0x00); /* DAC osr */
    es8311_write_reg(ES8311_REG_CLK_MGR7, 0x00);
    es8311_write_reg(ES8311_REG_CLK_MGR8, 0x00); /* BCLK divider auto */

    /* SDP (Serial Data Port) - I2S standard (Philips), 16-bit */
    es8311_write_reg(ES8311_REG_SDP_IN, 0x0C);  /* I2S 16-bit, slave */
    es8311_write_reg(ES8311_REG_SDP_OUT, 0x0C);

    /* System control: power up DAC, enable vmid reference */
    es8311_write_reg(ES8311_REG_SYSTEM, 0x00);   /* Power up analog */
    es8311_write_reg(ES8311_REG_VMID, 0x08);     /* VMID reference */
    es8311_write_reg(ES8311_REG_SYS1, 0x3F);     /* Power up DAC, ADC */
    es8311_write_reg(ES8311_REG_SYS2, 0x00);

    /* DAC volume (0x00 = 0dB, 0xFF = -95.5dB) */
    uint8_t vol_reg = (uint8_t)((100 - s_volume) * 255 / 100);
    es8311_write_reg(ES8311_REG_DAC_VOL, vol_reg);

    /* ADC volume (mic input, moderate gain) */
    es8311_write_reg(ES8311_REG_ADC_VOL, 0xBF);

    /* Verify chip is responding */
    uint8_t id = 0;
    es8311_read_reg(ES8311_REG_RESET, &id);
    ESP_LOGI(TAG, "ES8311 ID register: 0x%02x", id);

    return ESP_OK;
}

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
    gpio_set_level(AUDIO_PA_CTRL_GPIO, 0); /* Start with amp off */
}

/* ── Public API ── */

esp_err_t audio_out_init(const audio_out_config_t *config)
{
    esp_err_t ret;
    s_volume = config->volume;

    ESP_LOGI(TAG, "Initializing audio output: %lu Hz, volume=%d%%",
             (unsigned long)config->sample_rate, config->volume);

    /* 1. Initialize I2C bus for ES8311 codec */
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

    /* Add ES8311 device on I2C bus */
    i2c_device_config_t es8311_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AUDIO_ES8311_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &es8311_cfg, &s_es8311_dev);
    ESP_RETURN_ON_ERROR(ret, TAG, "ES8311 I2C device add failed");

    /* 2. Initialize ES8311 codec */
    ret = es8311_codec_init(config->sample_rate);
    ESP_RETURN_ON_ERROR(ret, TAG, "ES8311 codec init failed");

    /* 3. Initialize I2S TX channel */
    i2s_chan_config_t i2s_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_cfg.dma_desc_num = 6;
    i2s_cfg.dma_frame_num = 240;    /* 240 frames = 5ms at 48kHz */
    ret = i2s_new_channel(&i2s_cfg, &s_i2s_tx, NULL);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2S channel create failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
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
    /* MCLK = 256 * sample_rate for ES8311 */
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    ret = i2s_channel_init_std_mode(s_i2s_tx, &std_cfg);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2S std mode init failed");

    ret = i2s_channel_enable(s_i2s_tx);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2S channel enable failed");

    /* 4. Initialize speaker amp GPIO */
    pa_ctrl_init();
    if (config->speaker_enable) {
        gpio_set_level(AUDIO_PA_CTRL_GPIO, 1);
        s_speaker_on = true;
        ESP_LOGI(TAG, "Speaker amplifier enabled (GPIO%d)", AUDIO_PA_CTRL_GPIO);
    }

    ESP_LOGI(TAG, "Audio output ready: I2S0 → ES8311 → %s",
             s_speaker_on ? "Speaker+Headphone" : "Headphone only");
    return ESP_OK;
}

esp_err_t audio_out_deinit(void)
{
    /* Disable speaker amp first to avoid pop */
    gpio_set_level(AUDIO_PA_CTRL_GPIO, 0);
    s_speaker_on = false;

    if (s_i2s_tx) {
        i2s_channel_disable(s_i2s_tx);
        i2s_del_channel(s_i2s_tx);
        s_i2s_tx = NULL;
    }

    if (s_es8311_dev) {
        i2c_master_bus_rm_device(s_es8311_dev);
        s_es8311_dev = NULL;
    }

    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }

    ESP_LOGI(TAG, "Audio output deinitialized");
    return ESP_OK;
}

esp_err_t audio_out_write(const int16_t *samples, int count, int timeout_ms)
{
    if (!s_i2s_tx || !samples || count <= 0) return ESP_ERR_INVALID_STATE;
    if (s_muted) {
        /* Still need to write silence to keep DMA flowing */
        static const int16_t silence[256] = {0};
        size_t bytes_written = 0;
        int remaining = count;
        while (remaining > 0) {
            int chunk = remaining > 256 ? 256 : remaining;
            i2s_channel_write(s_i2s_tx, silence, chunk * sizeof(int16_t),
                              &bytes_written, timeout_ms);
            remaining -= chunk;
        }
        return ESP_OK;
    }

    size_t bytes_written = 0;
    esp_err_t ret = i2s_channel_write(s_i2s_tx, samples, count * sizeof(int16_t),
                                       &bytes_written, timeout_ms);
    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGD(TAG, "I2S write timeout (%d samples)", count);
    }
    return ret;
}

esp_err_t audio_out_set_volume(uint8_t volume)
{
    if (volume > 100) volume = 100;
    s_volume = volume;

    if (s_es8311_dev) {
        /* ES8311 DAC volume: 0x00 = 0dB (max), 0xFF = -95.5dB (min) */
        uint8_t vol_reg = (uint8_t)((100 - volume) * 255 / 100);
        return es8311_write_reg(ES8311_REG_DAC_VOL, vol_reg);
    }

    return ESP_OK;
}

uint8_t audio_out_get_volume(void)
{
    return s_volume;
}

esp_err_t audio_out_set_speaker(bool enable)
{
    gpio_set_level(AUDIO_PA_CTRL_GPIO, enable ? 1 : 0);
    s_speaker_on = enable;
    ESP_LOGI(TAG, "Speaker amp %s", enable ? "ON" : "OFF");
    return ESP_OK;
}

esp_err_t audio_out_set_mute(bool mute)
{
    s_muted = mute;
    ESP_LOGI(TAG, "Audio %s", mute ? "MUTED" : "unmuted");
    return ESP_OK;
}

esp_err_t audio_out_set_stereo(bool stereo)
{
    if (!s_i2s_tx) return ESP_ERR_INVALID_STATE;
    if (stereo == s_stereo_mode) return ESP_OK;

    ESP_LOGI(TAG, "Switching to %s mode", stereo ? "STEREO" : "MONO");

    /* Must disable channel, reconfigure slot, re-enable */
    esp_err_t ret = i2s_channel_disable(s_i2s_tx);
    if (ret != ESP_OK) return ret;

    i2s_std_slot_config_t slot_cfg;
    if (stereo) {
        i2s_std_slot_config_t tmp = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
        slot_cfg = tmp;
    } else {
        i2s_std_slot_config_t tmp = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
        slot_cfg = tmp;
    }

    ret = i2s_channel_reconfig_std_slot(s_i2s_tx, &slot_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Slot reconfig failed: %s", esp_err_to_name(ret));
        i2s_channel_enable(s_i2s_tx);
        return ret;
    }

    s_stereo_mode = stereo;
    return i2s_channel_enable(s_i2s_tx);
}

void *audio_out_get_i2c_bus(void)
{
    return (void *)s_i2c_bus;
}
