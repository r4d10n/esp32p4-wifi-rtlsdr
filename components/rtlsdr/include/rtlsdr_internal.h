/*
 * Internal API for RTL-SDR driver — shared between rtlsdr.c and tuner drivers.
 * Not part of the public API.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "rtlsdr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Low-level register access via USB vendor control transfers */
esp_err_t rtlsdr_read_reg(rtlsdr_dev_t *dev, uint16_t block,
                           uint16_t addr, uint8_t *val, uint16_t len);

esp_err_t rtlsdr_write_reg(rtlsdr_dev_t *dev, uint16_t block,
                            uint16_t addr, const uint8_t *val, uint16_t len);

/* I2C repeater control */
esp_err_t rtlsdr_set_i2c_repeater(rtlsdr_dev_t *dev, bool on);

#ifdef __cplusplus
}
#endif
