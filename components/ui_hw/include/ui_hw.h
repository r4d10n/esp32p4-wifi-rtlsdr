/*
 * Hardware UI: SSD1306 OLED + Rotary Encoder
 *
 * Optional hardware interface for FM radio control.
 * Guarded by CONFIG_FM_OLED_ENABLE.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include "sdkconfig.h"

#ifdef CONFIG_FM_OLED_ENABLE

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_HW_EVT_NONE = 0,
    UI_HW_EVT_CW,
    UI_HW_EVT_CCW,
    UI_HW_EVT_PRESS,
    UI_HW_EVT_LONG_PRESS,
} ui_hw_event_t;

typedef struct {
    uint32_t    frequency;
    int         mode;           /* 0=WBFM, 1=NBFM */
    uint8_t     volume;
    int16_t     signal_strength;
    bool        muted;
    int         gain;
} ui_hw_state_t;

typedef void (*ui_hw_encoder_cb_t)(ui_hw_event_t event, void *ctx);

typedef struct {
    ui_hw_encoder_cb_t  encoder_cb;
    void               *cb_ctx;
} ui_hw_config_t;

esp_err_t ui_hw_init(const ui_hw_config_t *config);
esp_err_t ui_hw_deinit(void);
void ui_hw_update(const ui_hw_state_t *state);
ui_hw_event_t ui_hw_poll_encoder(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_FM_OLED_ENABLE */
