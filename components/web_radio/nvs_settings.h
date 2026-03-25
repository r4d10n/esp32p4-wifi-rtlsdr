/*
 * NVS Settings Persistence for FM Radio
 * Save/load radio parameters to Non-Volatile Storage.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t    frequency;
    int         gain;
    uint8_t     volume;
    int         mode;       /* 0=WBFM, 1=NBFM */
    uint32_t    filter_bw;
    uint8_t     squelch;
    bool        nb_enabled;
    uint8_t     nb_threshold;
} nvs_radio_settings_t;

esp_err_t nvs_settings_init(void);
esp_err_t nvs_settings_load(nvs_radio_settings_t *settings);
esp_err_t nvs_settings_save(const nvs_radio_settings_t *settings);
esp_err_t nvs_settings_reset(void);

#ifdef __cplusplus
}
#endif
