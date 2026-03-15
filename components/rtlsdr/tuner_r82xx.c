/*
 * R820T/R820T2/R828D Tuner Driver for ESP32-P4
 *
 * Ported from librtlsdr tuner_r82xx.c
 * Controls the tuner via I2C through RTL2832U's I2C repeater.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "rtlsdr.h"

static const char *TAG = "r82xx";

/* R820T I2C address (7-bit) */
#define R820T_ADDR      0x34
#define R82XX_IF_FREQ   3570000  /* 3.57 MHz IF */

/* Shadow registers */
#define R82XX_NUM_REGS  32
static uint8_t r82xx_regs[R82XX_NUM_REGS];

/* R820T initial register values (from librtlsdr) */
static const uint8_t r82xx_init_regs[] = {
    0x83, 0x32, 0x75,           /* regs 0x05-0x07 */
    0xC0, 0x40, 0xD6, 0x6C,    /* regs 0x08-0x0B */
    0xF5, 0x63, 0x75, 0x68,    /* regs 0x0C-0x0F */
    0x6C, 0x83, 0x80, 0x00,    /* regs 0x10-0x13 */
    0x0F, 0x00, 0xC0, 0x30,    /* regs 0x14-0x17 */
    0x48, 0xCC, 0x60, 0x00,    /* regs 0x18-0x1B */
    0x54, 0xAE, 0x4A, 0xC0,    /* regs 0x1C-0x1F */
};

/* Forward declarations for internal rtlsdr register access */
extern esp_err_t rtlsdr_read_reg(void *dev, uint16_t block,
                                  uint16_t addr, uint8_t *val, uint16_t len);
extern esp_err_t rtlsdr_write_reg(void *dev, uint16_t block,
                                   uint16_t addr, const uint8_t *val, uint16_t len);

/* I2C read/write via RTL2832U IIC block */
static esp_err_t r82xx_write(rtlsdr_dev_t *dev, uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    esp_err_t ret = rtlsdr_write_reg(dev, 0x0600, R820T_ADDR, buf, len + 1);
    if (ret == ESP_OK) {
        /* Update shadow regs */
        for (uint8_t i = 0; i < len && (reg + i) < R82XX_NUM_REGS; i++) {
            r82xx_regs[reg + i] = data[i];
        }
    }
    return ret;
}

static esp_err_t r82xx_write_reg_mask(rtlsdr_dev_t *dev, uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t old = r82xx_regs[reg];
    uint8_t new_val = (old & ~mask) | (val & mask);
    return r82xx_write(dev, reg, &new_val, 1);
}

static esp_err_t r82xx_read(rtlsdr_dev_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    /* RTL2832U requires writing the register address first, then reading */
    esp_err_t ret = rtlsdr_write_reg(dev, 0x0600, R820T_ADDR, &reg, 1);
    if (ret != ESP_OK) return ret;
    return rtlsdr_read_reg(dev, 0x0600, R820T_ADDR, data, len);
}

/* ──────────────────────── PLL ──────────────────────── */

static esp_err_t r82xx_set_pll(rtlsdr_dev_t *dev, uint32_t freq)
{
    esp_err_t ret;
    uint32_t vco_freq;
    uint8_t mix_div = 2;
    uint8_t div_num = 0;

    /* Calculate VCO frequency and divider */
    while (mix_div <= 64) {
        vco_freq = freq * mix_div;
        if (vco_freq >= 1770000000UL) break;
        mix_div *= 2;
        div_num++;
    }

    /* Set mixer divider */
    ret = r82xx_write_reg_mask(dev, 0x10, (div_num << 5), 0xE0);
    ESP_RETURN_ON_ERROR(ret, TAG, "PLL div_num write failed");

    /* Calculate PLL integer and fractional parts */
    uint32_t pll_ref = 28800000 / 2;  /* Reference = crystal/2 */
    uint32_t n_sdm = (vco_freq / pll_ref) - 2;
    uint32_t sdm = 0;

    uint32_t vco_fra = vco_freq - (n_sdm + 2) * pll_ref;
    if (vco_fra > 0) {
        sdm = (uint32_t)((uint64_t)vco_fra * 65536ULL / pll_ref);
    }

    /* Write N integer */
    uint8_t ni = (uint8_t)(n_sdm & 0xFF);
    ret = r82xx_write(dev, 0x14, &ni, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "PLL ni write failed");

    /* Write SDM fractional */
    uint8_t sdm_buf[2] = { (sdm >> 8) & 0xFF, sdm & 0xFF };
    ret = r82xx_write(dev, 0x12, sdm_buf, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "PLL sdm write failed");

    /* TODO: check PLL lock */
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGD(TAG, "PLL: freq=%lu mix_div=%d n=%lu sdm=%lu",
             (unsigned long)freq, mix_div, (unsigned long)n_sdm, (unsigned long)sdm);
    return ESP_OK;
}

/* ──────────────────────── Public API ──────────────────────── */

esp_err_t r82xx_init(rtlsdr_dev_t *dev)
{
    ESP_LOGI(TAG, "Initializing R82xx tuner");

    /* Initialize shadow register array */
    memset(r82xx_regs, 0, sizeof(r82xx_regs));
    memcpy(&r82xx_regs[0x05], r82xx_init_regs, sizeof(r82xx_init_regs));

    /* Write initial register values */
    esp_err_t ret = r82xx_write(dev, 0x05, r82xx_init_regs, sizeof(r82xx_init_regs));
    ESP_RETURN_ON_ERROR(ret, TAG, "Initial register write failed");

    /* Set IF frequency */
    /* TODO: proper IF freq register configuration */

    ESP_LOGI(TAG, "R82xx tuner initialized");
    return ESP_OK;
}

esp_err_t r82xx_set_freq(rtlsdr_dev_t *dev, uint32_t freq)
{
    /* RF frequency = desired freq + IF offset */
    uint32_t lo_freq = freq + R82XX_IF_FREQ;

    ESP_LOGI(TAG, "Setting freq=%lu Hz (LO=%lu Hz)", (unsigned long)freq, (unsigned long)lo_freq);
    return r82xx_set_pll(dev, lo_freq);
}

esp_err_t r82xx_set_gain(rtlsdr_dev_t *dev, int gain)
{
    /* R820T gain is set via LNA gain (reg 0x05[3:0]) + mixer gain (reg 0x07[3:0]) */
    /* Simplified: map gain index to LNA+mixer combination */
    uint8_t lna_gain, mixer_gain;

    if (gain <= 0) {
        lna_gain = 0;
        mixer_gain = 0;
    } else if (gain < 200) {
        lna_gain = gain / 30;
        mixer_gain = 0;
    } else {
        lna_gain = 15;
        mixer_gain = (gain - 200) / 30;
        if (mixer_gain > 15) mixer_gain = 15;
    }

    esp_err_t ret;
    ret = r82xx_write_reg_mask(dev, 0x05, lna_gain, 0x0F);
    ESP_RETURN_ON_ERROR(ret, TAG, "LNA gain write failed");

    ret = r82xx_write_reg_mask(dev, 0x07, mixer_gain, 0x0F);
    ESP_RETURN_ON_ERROR(ret, TAG, "Mixer gain write failed");

    ESP_LOGI(TAG, "Gain: %d/10 dB (LNA=%d mixer=%d)", gain, lna_gain, mixer_gain);
    return ESP_OK;
}

esp_err_t r82xx_set_gain_mode(rtlsdr_dev_t *dev, int manual)
{
    /* LNA gain mode: reg 0x05[4] = 0 for auto, 1 for manual */
    esp_err_t ret;
    ret = r82xx_write_reg_mask(dev, 0x05, manual ? 0x10 : 0x00, 0x10);
    ESP_RETURN_ON_ERROR(ret, TAG, "LNA gain mode failed");

    /* Mixer gain mode: reg 0x07[4] = 0 for auto, 1 for manual */
    ret = r82xx_write_reg_mask(dev, 0x07, manual ? 0x10 : 0x00, 0x10);
    ESP_RETURN_ON_ERROR(ret, TAG, "Mixer gain mode failed");

    ESP_LOGI(TAG, "Gain mode: %s", manual ? "manual" : "auto");
    return ESP_OK;
}

esp_err_t r82xx_set_bandwidth(rtlsdr_dev_t *dev, uint32_t bw)
{
    /* Simplified bandwidth setting via IF filter register */
    /* reg 0x0A[3:0] controls the IF filter bandwidth */
    uint8_t hp_cor;

    if (bw > 2000000) {
        hp_cor = 0x00;  /* Widest */
    } else if (bw > 1000000) {
        hp_cor = 0x04;
    } else if (bw > 500000) {
        hp_cor = 0x08;
    } else {
        hp_cor = 0x0C;  /* Narrowest */
    }

    esp_err_t ret = r82xx_write_reg_mask(dev, 0x0A, hp_cor, 0x0F);
    ESP_RETURN_ON_ERROR(ret, TAG, "Bandwidth write failed");

    ESP_LOGI(TAG, "Bandwidth: %lu Hz (hp_cor=0x%02x)", (unsigned long)bw, hp_cor);
    return ESP_OK;
}
