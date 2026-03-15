/*
 * R820T/R820T2/R828D Tuner Driver for ESP32-P4
 *
 * Ported from librtlsdr tuner_r82xx.c (steve-m + rtl-sdr-blog fork)
 * Controls the tuner via I2C through RTL2832U's I2C repeater.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "rtlsdr.h"
#include "rtlsdr_internal.h"

static const char *TAG = "r82xx";

/* I2C addresses */
#define R820T_I2C_ADDR  0x34
#define R828D_I2C_ADDR  0x74

/* Detection register */
#define R82XX_CHECK_ADDR    0x00
#define R82XX_CHECK_VAL     0x69

/* Crystal frequencies */
#define R828D_XTAL_FREQ     16000000
#define RTL_XTAL_FREQ       28800000

/* IF frequency */
#define R82XX_IF_FREQ       3570000

/* Register range */
#define REG_SHADOW_START    5
#define NUM_REGS            30

#define MHZ(x) ((uint32_t)(x) * 1000000UL)

/* Shadow registers */
static uint8_t r82xx_regs[NUM_REGS];

/* R82xx initial register values (regs 0x05 to 0x1f) */
static const uint8_t r82xx_init_array[NUM_REGS - REG_SHADOW_START] = {
    0x83, 0x32, 0x75,           /* 05 to 07 */
    0xc0, 0x40, 0xd6, 0x6c,    /* 08 to 0b */
    0xf5, 0x63, 0x75, 0x68,    /* 0c to 0f */
    0x6c, 0x83, 0x80, 0x00,    /* 10 to 13 */
    0x0f, 0x00, 0xc0, 0x30,    /* 14 to 17 */
    0x48, 0xcc, 0x60, 0x00,    /* 18 to 1b */
    0x54, 0xae, 0x4a, 0xc0,    /* 1c to 1f */
};

/* Gain step tables (cumulative, in tenths of dB) */
static const int r82xx_lna_gain_steps[] = {
    0, 9, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13
};
static const int r82xx_mixer_gain_steps[] = {
    0, 5, 10, 10, 19, 9, 10, 25, 17, 10, 8, 16, 13, 6, 3, -8
};

/* Total gain table (tenths of dB) for RTL-TCP gain-by-index */
static const int r82xx_gains[] = {
    0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
    166, 197, 207, 229, 254, 280, 297, 328,
    338, 364, 372, 386, 402, 421, 434, 439,
    445, 480, 496
};

/* Tuner state */
static enum {
    CHIP_R820T = 0,
    CHIP_R828D,
} r82xx_chip_type;
static uint8_t r82xx_i2c_addr = R820T_I2C_ADDR;
static uint32_t r82xx_xtal_freq = RTL_XTAL_FREQ;
static uint32_t r82xx_int_freq = R82XX_IF_FREQ;
static uint8_t r82xx_input = 0;

/* ──────────────────────── I2C via RTL2832U IIC Block ──────────────────────── */

static esp_err_t r82xx_write(rtlsdr_dev_t *dev, uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[32];
    if (len + 1 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;

    buf[0] = reg;
    memcpy(&buf[1], data, len);

    esp_err_t ret = rtlsdr_write_reg(dev, 0x0600, r82xx_i2c_addr, buf, len + 1);
    if (ret == ESP_OK) {
        for (uint8_t i = 0; i < len && (reg - REG_SHADOW_START + i) < NUM_REGS; i++) {
            if (reg + i >= REG_SHADOW_START) {
                r82xx_regs[reg - REG_SHADOW_START + i] = data[i];
            }
        }
    }
    return ret;
}

static esp_err_t r82xx_write_reg(rtlsdr_dev_t *dev, uint8_t reg, uint8_t val)
{
    return r82xx_write(dev, reg, &val, 1);
}

static esp_err_t r82xx_write_reg_mask(rtlsdr_dev_t *dev, uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t old = r82xx_regs[reg - REG_SHADOW_START];
    uint8_t new_val = (old & ~mask) | (val & mask);
    return r82xx_write_reg(dev, reg, new_val);
}

static esp_err_t r82xx_read(rtlsdr_dev_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    /* R82xx uses read-from-zero, bits are MSB-reversed per byte */
    (void)reg; /* R82xx always reads from register 0 */
    return rtlsdr_read_reg(dev, 0x0600, r82xx_i2c_addr, data, len);
}

/* ──────────────────────── PLL ──────────────────────── */

static esp_err_t r82xx_set_pll(rtlsdr_dev_t *dev, uint32_t freq)
{
    esp_err_t ret;
    uint32_t vco_freq;
    uint8_t mix_div = 2;
    uint8_t div_num = 0;
    uint32_t vco_min = 1770000000UL;
    uint32_t pll_ref;
    int vco_power_ref = (r82xx_chip_type == CHIP_R828D) ? 1 : 2;

    /* Calculate VCO frequency and divider */
    while (mix_div <= 64) {
        vco_freq = (uint64_t)freq * mix_div;
        if (vco_freq >= vco_min) break;
        mix_div *= 2;
        div_num++;
    }

    /* Set mixer divider — reg 0x10[7:5] */
    ret = r82xx_write_reg_mask(dev, 0x10, (div_num << 5), 0xe0);
    ESP_RETURN_ON_ERROR(ret, TAG, "PLL div_num write failed");

    /* Calculate reference divider */
    pll_ref = r82xx_xtal_freq;

    /* VCO current: reg 0x12 = 0x80 (auto) */
    ret = r82xx_write_reg(dev, 0x12, 0x80);
    ESP_RETURN_ON_ERROR(ret, TAG, "VCO current write failed");

    /* PLL autotune: reg 0x1a[3] = 0 (128kHz step) */
    ret = r82xx_write_reg_mask(dev, 0x1a, 0x00, 0x0c);
    ESP_RETURN_ON_ERROR(ret, TAG, "PLL autotune write failed");

    /* Set PLL integer and fractional parts */
    uint32_t n_sdm = (uint32_t)(vco_freq / (2 * pll_ref));
    uint32_t nint = n_sdm - 13;

    /* Reg 0x14[6:0] = nint */
    ret = r82xx_write_reg_mask(dev, 0x14, (uint8_t)(nint & 0x7f), 0x7f);
    ESP_RETURN_ON_ERROR(ret, TAG, "PLL nint write failed");

    /* Calculate SDM (sigma-delta modulator fractional) */
    uint32_t vco_fra = vco_freq - 2 * pll_ref * n_sdm;
    uint32_t sdm = 0;
    if (vco_fra > 0) {
        sdm = (uint32_t)(((uint64_t)vco_fra * 65536ULL) / (2 * pll_ref));
    }

    /* Write SDM: reg 0x16 (hi), reg 0x15 (lo) */
    ret = r82xx_write_reg(dev, 0x16, (sdm >> 8) & 0xff);
    ESP_RETURN_ON_ERROR(ret, TAG, "SDM hi write failed");
    ret = r82xx_write_reg(dev, 0x15, sdm & 0xff);
    ESP_RETURN_ON_ERROR(ret, TAG, "SDM lo write failed");

    /* Wait for PLL lock */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Check PLL lock: read reg 0x02[6] */
    uint8_t data[3];
    ret = r82xx_read(dev, 0x00, data, 3);
    if (ret == ESP_OK) {
        if (!(data[2] & 0x40)) {
            ESP_LOGW(TAG, "PLL not locked (reg2=0x%02x), retrying with higher VCO current", data[2]);
            /* Increase VCO current */
            r82xx_write_reg(dev, 0x12, 0x60);
            vTaskDelay(pdMS_TO_TICKS(10));
            ret = r82xx_read(dev, 0x00, data, 3);
            if (ret == ESP_OK && !(data[2] & 0x40)) {
                ESP_LOGE(TAG, "PLL still not locked");
            }
        }
    }

    /* Set VCO power ref */
    (void)vco_power_ref;

    ESP_LOGI(TAG, "PLL: freq=%lu vco=%lu mix_div=%d n=%lu sdm=%lu",
             (unsigned long)freq, (unsigned long)vco_freq,
             mix_div, (unsigned long)n_sdm, (unsigned long)sdm);
    return ESP_OK;
}

/* ──────────────────────── System Frequency Selection ──────────────────────── */

static esp_err_t r82xx_sysfreq_sel(rtlsdr_dev_t *dev, uint32_t freq)
{
    esp_err_t ret;

    /* LNA top current: reg 0x06[7:6] = 11 */
    ret = r82xx_write_reg_mask(dev, 0x06, 0xc0, 0xc0);
    ESP_RETURN_ON_ERROR(ret, TAG, "LNA top failed");

    /* Mixer top current: reg 0x07[7:6] = 00 */
    ret = r82xx_write_reg_mask(dev, 0x07, 0x00, 0xc0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Mixer top failed");

    /* LNA discharge current: reg 0x1e[7:4] = 0100 */
    ret = r82xx_write_reg_mask(dev, 0x1e, 0x40, 0xf0);
    ESP_RETURN_ON_ERROR(ret, TAG, "LNA discharge failed");

    /* AGC clk: reg 0x1a[5:4] = 00 (60Hz) */
    ret = r82xx_write_reg_mask(dev, 0x1a, 0x00, 0x30);
    ESP_RETURN_ON_ERROR(ret, TAG, "AGC clk failed");

    return ESP_OK;
}

/* ──────────────────────── TV Standard (SDR mode) ──────────────────────── */

static esp_err_t r82xx_set_tv_standard(rtlsdr_dev_t *dev)
{
    esp_err_t ret;

    /* Set IF filter: reg 0x0a[3:0], reg 0x0b */
    /* Default 6MHz bandwidth for SDR */
    ret = r82xx_write_reg_mask(dev, 0x0a, 0x10, 0x1f);
    ESP_RETURN_ON_ERROR(ret, TAG, "IF filter 0x0a failed");

    ret = r82xx_write_reg(dev, 0x0b, 0x6b);
    ESP_RETURN_ON_ERROR(ret, TAG, "IF filter 0x0b failed");

    r82xx_int_freq = R82XX_IF_FREQ;

    /* Filter bandwidth: reg 0x0a[4] */
    ret = r82xx_write_reg_mask(dev, 0x0a, 0x10, 0x10);
    ESP_RETURN_ON_ERROR(ret, TAG, "Filter BW failed");

    /* Pre-detect: reg 0x06[5:4] = 01 */
    ret = r82xx_write_reg_mask(dev, 0x06, 0x10, 0x30);
    ESP_RETURN_ON_ERROR(ret, TAG, "Pre-detect failed");

    /* Channel filter extension: reg 0x1e[0] = 1 */
    ret = r82xx_write_reg_mask(dev, 0x1e, 0x01, 0x01);
    ESP_RETURN_ON_ERROR(ret, TAG, "Chan filter ext failed");

    /* Loop through: reg 0x05[7] = 0 */
    ret = r82xx_write_reg_mask(dev, 0x05, 0x00, 0x80);
    ESP_RETURN_ON_ERROR(ret, TAG, "Loop through failed");

    /* LT gain: reg 0x05[6:5] = 00 */
    ret = r82xx_write_reg_mask(dev, 0x05, 0x00, 0x60);
    ESP_RETURN_ON_ERROR(ret, TAG, "LT gain failed");

    return ESP_OK;
}

/* ──────────────────────── Public API ──────────────────────── */

esp_err_t r82xx_init(rtlsdr_dev_t *dev)
{
    esp_err_t ret;
    rtlsdr_tuner_t tuner = rtlsdr_get_tuner_type(dev);

    if (tuner == RTLSDR_TUNER_R828D) {
        r82xx_chip_type = CHIP_R828D;
        r82xx_i2c_addr = R828D_I2C_ADDR;
        /* RTL-SDR Blog V4 uses 28.8 MHz xtal, standard R828D uses 16 MHz.
         * Since we can't reliably detect Blog V4 from USB descriptors at
         * this stage, default to 28.8 MHz (works for Blog V4). */
        r82xx_xtal_freq = RTL_XTAL_FREQ;
        ESP_LOGI(TAG, "Initializing R828D tuner (addr=0x%02x xtal=%luHz)",
                 r82xx_i2c_addr, (unsigned long)r82xx_xtal_freq);
    } else {
        r82xx_chip_type = CHIP_R820T;
        r82xx_i2c_addr = R820T_I2C_ADDR;
        r82xx_xtal_freq = RTL_XTAL_FREQ;
        ESP_LOGI(TAG, "Initializing R820T tuner (addr=0x%02x)", r82xx_i2c_addr);
    }

    /* Initialize shadow register array */
    memset(r82xx_regs, 0, sizeof(r82xx_regs));
    memcpy(r82xx_regs, r82xx_init_array, sizeof(r82xx_init_array));

    /* Write initial register values (regs 0x05 to 0x1f) */
    ret = r82xx_write(dev, 0x05, r82xx_init_array, sizeof(r82xx_init_array));
    ESP_RETURN_ON_ERROR(ret, TAG, "Initial register write failed");

    /* Set TV standard for SDR mode */
    ret = r82xx_set_tv_standard(dev);
    ESP_RETURN_ON_ERROR(ret, TAG, "TV standard set failed");

    /* System frequency selection */
    ret = r82xx_sysfreq_sel(dev, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sysfreq sel failed");

    ESP_LOGI(TAG, "R82xx tuner initialized successfully");
    return ESP_OK;
}

esp_err_t r82xx_set_freq(rtlsdr_dev_t *dev, uint32_t freq)
{
    esp_err_t ret;

    /* Handle R828D input switching */
    if (r82xx_chip_type == CHIP_R828D) {
        uint8_t air_cable1_in;

        /* Standard R828D: switch at 345 MHz */
        if (freq > MHZ(345)) {
            air_cable1_in = 0x00;  /* Air input */
        } else {
            air_cable1_in = 0x60;  /* Cable1 input */
        }

        if (air_cable1_in != r82xx_input) {
            r82xx_input = air_cable1_in;
            ret = r82xx_write_reg_mask(dev, 0x05, air_cable1_in, 0x60);
            ESP_RETURN_ON_ERROR(ret, TAG, "Input switch failed");
        }
    }

    /* RF frequency = desired freq + IF offset */
    uint32_t lo_freq = freq + r82xx_int_freq;

    /* Set open drain: reg 0x17[3] */
    uint8_t open_d = 0x08; /* Normal */
    ret = r82xx_write_reg_mask(dev, 0x17, open_d, 0x08);
    ESP_RETURN_ON_ERROR(ret, TAG, "Open drain failed");

    /* Set RF MUX based on frequency */
    /* RF_MUX: reg 0x1a[7:6] */
    uint8_t rf_mux;
    if (freq <= MHZ(50)) {
        rf_mux = 0x80;
    } else if (freq <= MHZ(340)) {
        rf_mux = 0x40;
    } else {
        rf_mux = 0x00;
    }
    ret = r82xx_write_reg_mask(dev, 0x1a, rf_mux, 0xc0);
    ESP_RETURN_ON_ERROR(ret, TAG, "RF MUX failed");

    /* Set PLL */
    ret = r82xx_set_pll(dev, lo_freq);
    ESP_RETURN_ON_ERROR(ret, TAG, "PLL set failed");

    ESP_LOGI(TAG, "Freq: %lu Hz (LO=%lu Hz, IF=%lu Hz)",
             (unsigned long)freq, (unsigned long)lo_freq, (unsigned long)r82xx_int_freq);
    return ESP_OK;
}

esp_err_t r82xx_set_gain(rtlsdr_dev_t *dev, int gain)
{
    esp_err_t ret;

    /* Find the nearest gain entry in the gain table */
    int total_gain = 0;
    int lna_index = 0;
    int mix_index = 0;

    /* Find best LNA + mixer combination */
    for (int i = 0; i < 16; i++) {
        int lna_sum = 0;
        for (int j = 0; j <= i && j < 16; j++) lna_sum += r82xx_lna_gain_steps[j];

        for (int k = 0; k < 16; k++) {
            int mix_sum = 0;
            for (int j = 0; j <= k && j < 16; j++) mix_sum += r82xx_mixer_gain_steps[j];

            int this_gain = lna_sum + mix_sum;
            if (abs(this_gain - gain) < abs(total_gain - gain)) {
                total_gain = this_gain;
                lna_index = i;
                mix_index = k;
            }
        }
    }

    /* LNA gain: reg 0x05[3:0] */
    ret = r82xx_write_reg_mask(dev, 0x05, lna_index, 0x0f);
    ESP_RETURN_ON_ERROR(ret, TAG, "LNA gain write failed");

    /* Mixer gain: reg 0x07[3:0] */
    ret = r82xx_write_reg_mask(dev, 0x07, mix_index, 0x0f);
    ESP_RETURN_ON_ERROR(ret, TAG, "Mixer gain write failed");

    ESP_LOGI(TAG, "Gain: requested=%d/10dB actual=%d/10dB (LNA=%d mixer=%d)",
             gain, total_gain, lna_index, mix_index);
    return ESP_OK;
}

esp_err_t r82xx_set_gain_mode(rtlsdr_dev_t *dev, int manual)
{
    esp_err_t ret;

    /* LNA auto/manual: reg 0x05[4] */
    ret = r82xx_write_reg_mask(dev, 0x05, manual ? 0x10 : 0x00, 0x10);
    ESP_RETURN_ON_ERROR(ret, TAG, "LNA gain mode failed");

    /* Mixer auto/manual: reg 0x07[4] */
    ret = r82xx_write_reg_mask(dev, 0x07, manual ? 0x10 : 0x00, 0x10);
    ESP_RETURN_ON_ERROR(ret, TAG, "Mixer gain mode failed");

    ESP_LOGI(TAG, "Gain mode: %s", manual ? "manual" : "auto");
    return ESP_OK;
}

esp_err_t r82xx_set_bandwidth(rtlsdr_dev_t *dev, uint32_t bw)
{
    esp_err_t ret;
    uint8_t reg_0a, reg_0b;
    uint32_t if_freq;

    /* Bandwidth selection from librtlsdr */
    if (bw > 7000000) {
        reg_0a = 0x10; reg_0b = 0x0b;
        if_freq = 4570000;
    } else if (bw > 6000000) {
        reg_0a = 0x10; reg_0b = 0x2a;
        if_freq = 4570000;
    } else if (bw > 2430000) {  /* 1700000 + 350000 + 380000 */
        reg_0a = 0x10; reg_0b = 0x6b;
        if_freq = 3570000;
    } else if (bw > 1530000) {
        reg_0a = 0x00; reg_0b = 0x2a;
        if_freq = 3570000;
    } else if (bw > 1230000) {
        reg_0a = 0x00; reg_0b = 0x4a;
        if_freq = 3570000;
    } else if (bw > 930000) {
        reg_0a = 0x00; reg_0b = 0x6a;
        if_freq = 3570000;
    } else if (bw > 730000) {
        reg_0a = 0x00; reg_0b = 0x8a;
        if_freq = 3570000;
    } else {
        reg_0a = 0x00; reg_0b = 0xab;
        if_freq = 3570000;
    }

    ret = r82xx_write_reg_mask(dev, 0x0a, reg_0a, 0x1f);
    ESP_RETURN_ON_ERROR(ret, TAG, "BW 0x0a failed");

    ret = r82xx_write_reg(dev, 0x0b, reg_0b);
    ESP_RETURN_ON_ERROR(ret, TAG, "BW 0x0b failed");

    r82xx_int_freq = if_freq;

    ESP_LOGI(TAG, "Bandwidth: %lu Hz (IF=%lu Hz)", (unsigned long)bw, (unsigned long)if_freq);
    return ESP_OK;
}

/* ──────────────────────── Gain Table Access ──────────────────────── */

const int *r82xx_get_gains(int *count)
{
    *count = sizeof(r82xx_gains) / sizeof(r82xx_gains[0]);
    return r82xx_gains;
}
