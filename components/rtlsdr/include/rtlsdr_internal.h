/*
 * Internal API for RTL-SDR driver — shared between rtlsdr.c and tuner drivers.
 * Not part of the public API.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "rtlsdr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────── RTL2832U Register Access ──────────────────────── */

/* Low-level register access via USB vendor control transfers */
esp_err_t rtlsdr_read_reg(rtlsdr_dev_t *dev, uint16_t block,
                           uint16_t addr, uint8_t *val, uint16_t len);

esp_err_t rtlsdr_write_reg(rtlsdr_dev_t *dev, uint16_t block,
                            uint16_t addr, const uint8_t *val, uint16_t len);

/* I2C repeater control */
esp_err_t rtlsdr_set_i2c_repeater(rtlsdr_dev_t *dev, bool on);

/* Demod IF frequency (written to demod page 1, regs 0x19-0x1b) */
esp_err_t rtlsdr_set_if_freq(rtlsdr_dev_t *dev, uint32_t freq);

/* GPIO control (SYS block: GPO=0x3001, GPOE=0x3003, GPD=0x3004) */
esp_err_t rtlsdr_set_gpio_output(rtlsdr_dev_t *dev, uint8_t gpio);
esp_err_t rtlsdr_set_gpio_bit(rtlsdr_dev_t *dev, uint8_t gpio, int val);
esp_err_t rtlsdr_set_bias_tee_gpio(rtlsdr_dev_t *dev, int gpio, int on);

/* Blog V4 detection */
int rtlsdr_is_blog_v4(rtlsdr_dev_t *dev);

/* Demod write (for tuner use during calibration) */
esp_err_t rtlsdr_demod_write_reg_ext(rtlsdr_dev_t *dev, uint8_t page,
                                      uint16_t addr, uint16_t val, uint8_t len);

/* ──────────────────────── R82xx Tuner API ──────────────────────── */

esp_err_t r82xx_init(rtlsdr_dev_t *dev);
esp_err_t r82xx_standby(rtlsdr_dev_t *dev);
esp_err_t r82xx_set_freq(rtlsdr_dev_t *dev, uint32_t freq);
int       r82xx_set_bandwidth(rtlsdr_dev_t *dev, int bw, uint32_t rate);
esp_err_t r82xx_set_gain(rtlsdr_dev_t *dev, int gain);
esp_err_t r82xx_set_gain_mode(rtlsdr_dev_t *dev, int manual);
const int *r82xx_get_gains(int *count);

#ifdef __cplusplus
}
#endif
