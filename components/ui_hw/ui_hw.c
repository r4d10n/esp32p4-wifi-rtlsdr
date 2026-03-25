/*
 * Hardware UI: SSD1306 OLED + Rotary Encoder
 *
 * Minimal SSD1306 driver (no LVGL dependency) + PCNT-based encoder.
 * Entire file compiles to empty when CONFIG_FM_OLED_ENABLE is not set.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#ifdef CONFIG_FM_OLED_ENABLE

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "audio_out.h"
#include "ui_hw.h"

static const char *TAG = "ui_hw";

/* ── SSD1306 Constants ── */

#define OLED_W          128
#define OLED_H          64
#define OLED_PAGES      (OLED_H / 8)
#define OLED_BUFSIZE    (OLED_W * OLED_PAGES)

#define SSD1306_CMD     0x00
#define SSD1306_DATA    0x40

/* ── State ── */

static i2c_master_dev_handle_t s_oled_dev = NULL;
static uint8_t s_fb[OLED_BUFSIZE];

static pcnt_unit_handle_t   s_pcnt_unit = NULL;
static ui_hw_encoder_cb_t   s_encoder_cb = NULL;
static void                *s_encoder_ctx = NULL;

/* Button state */
static int64_t  s_btn_press_time = 0;
static bool     s_btn_was_pressed = false;

/* ── SSD1306 I2C helpers ── */

static esp_err_t ssd1306_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CMD, cmd };
    return i2c_master_transmit(s_oled_dev, buf, 2, 50);
}

static esp_err_t ssd1306_init_display(void)
{
    static const uint8_t init_cmds[] = {
        0xAE,       /* Display off */
        0xD5, 0x80, /* Clock divide ratio */
        0xA8, 0x3F, /* MUX ratio = 63 (64 lines) */
        0xD3, 0x00, /* Display offset = 0 */
        0x40,       /* Start line = 0 */
        0x8D, 0x14, /* Charge pump enable */
        0x20, 0x00, /* Horizontal addressing mode */
        0xA1,       /* Segment remap (col 127 = SEG0) */
        0xC8,       /* COM scan direction remapped */
        0xDA, 0x12, /* COM pins: alternative, no remap */
        0x81, 0x7F, /* Contrast = 127 */
        0xD9, 0xF1, /* Pre-charge period */
        0xDB, 0x40, /* VCOMH deselect level */
        0xA4,       /* Display from RAM */
        0xA6,       /* Normal display (not inverted) */
        0xAF,       /* Display on */
    };

    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        esp_err_t ret = ssd1306_cmd(init_cmds[i]);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static void ssd1306_clear(void)
{
    memset(s_fb, 0, OLED_BUFSIZE);
}

static esp_err_t ssd1306_flush(void)
{
    /* Set column and page address to full screen */
    ssd1306_cmd(0x21); ssd1306_cmd(0); ssd1306_cmd(127); /* Col 0-127 */
    ssd1306_cmd(0x22); ssd1306_cmd(0); ssd1306_cmd(7);   /* Page 0-7 */

    /* Send framebuffer in chunks (I2C max ~128 bytes per transaction on some buses) */
    for (int page = 0; page < OLED_PAGES; page++) {
        uint8_t buf[OLED_W + 1];
        buf[0] = SSD1306_DATA;
        memcpy(&buf[1], &s_fb[page * OLED_W], OLED_W);
        esp_err_t ret = i2c_master_transmit(s_oled_dev, buf, OLED_W + 1, 100);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

/* ── Minimal 5x7 bitmap font ── */

static const uint8_t font_5x7[] = {
    /* ASCII 32-127, 5 bytes per character (column-major, LSB=top) */
    /* Space */ 0x00,0x00,0x00,0x00,0x00,
    /* !     */ 0x00,0x00,0x5F,0x00,0x00,
    /* "     */ 0x00,0x07,0x00,0x07,0x00,
    /* #     */ 0x14,0x7F,0x14,0x7F,0x14,
    /* $     */ 0x24,0x2A,0x7F,0x2A,0x12,
    /* %     */ 0x23,0x13,0x08,0x64,0x62,
    /* &     */ 0x36,0x49,0x55,0x22,0x50,
    /* '     */ 0x00,0x05,0x03,0x00,0x00,
    /* (     */ 0x00,0x1C,0x22,0x41,0x00,
    /* )     */ 0x00,0x41,0x22,0x1C,0x00,
    /* *     */ 0x14,0x08,0x3E,0x08,0x14,
    /* +     */ 0x08,0x08,0x3E,0x08,0x08,
    /* ,     */ 0x00,0x50,0x30,0x00,0x00,
    /* -     */ 0x08,0x08,0x08,0x08,0x08,
    /* .     */ 0x00,0x60,0x60,0x00,0x00,
    /* /     */ 0x20,0x10,0x08,0x04,0x02,
    /* 0     */ 0x3E,0x51,0x49,0x45,0x3E,
    /* 1     */ 0x00,0x42,0x7F,0x40,0x00,
    /* 2     */ 0x42,0x61,0x51,0x49,0x46,
    /* 3     */ 0x21,0x41,0x45,0x4B,0x31,
    /* 4     */ 0x18,0x14,0x12,0x7F,0x10,
    /* 5     */ 0x27,0x45,0x45,0x45,0x39,
    /* 6     */ 0x3C,0x4A,0x49,0x49,0x30,
    /* 7     */ 0x01,0x71,0x09,0x05,0x03,
    /* 8     */ 0x36,0x49,0x49,0x49,0x36,
    /* 9     */ 0x06,0x49,0x49,0x29,0x1E,
    /* :     */ 0x00,0x36,0x36,0x00,0x00,
    /* ;     */ 0x00,0x56,0x36,0x00,0x00,
    /* <     */ 0x08,0x14,0x22,0x41,0x00,
    /* =     */ 0x14,0x14,0x14,0x14,0x14,
    /* >     */ 0x00,0x41,0x22,0x14,0x08,
    /* ?     */ 0x02,0x01,0x51,0x09,0x06,
    /* @     */ 0x32,0x49,0x79,0x41,0x3E,
    /* A     */ 0x7E,0x11,0x11,0x11,0x7E,
    /* B     */ 0x7F,0x49,0x49,0x49,0x36,
    /* C     */ 0x3E,0x41,0x41,0x41,0x22,
    /* D     */ 0x7F,0x41,0x41,0x22,0x1C,
    /* E     */ 0x7F,0x49,0x49,0x49,0x41,
    /* F     */ 0x7F,0x09,0x09,0x09,0x01,
    /* G     */ 0x3E,0x41,0x49,0x49,0x7A,
    /* H     */ 0x7F,0x08,0x08,0x08,0x7F,
    /* I     */ 0x00,0x41,0x7F,0x41,0x00,
    /* J     */ 0x20,0x40,0x41,0x3F,0x01,
    /* K     */ 0x7F,0x08,0x14,0x22,0x41,
    /* L     */ 0x7F,0x40,0x40,0x40,0x40,
    /* M     */ 0x7F,0x02,0x0C,0x02,0x7F,
    /* N     */ 0x7F,0x04,0x08,0x10,0x7F,
    /* O     */ 0x3E,0x41,0x41,0x41,0x3E,
    /* P     */ 0x7F,0x09,0x09,0x09,0x06,
    /* Q     */ 0x3E,0x41,0x51,0x21,0x5E,
    /* R     */ 0x7F,0x09,0x19,0x29,0x46,
    /* S     */ 0x46,0x49,0x49,0x49,0x31,
    /* T     */ 0x01,0x01,0x7F,0x01,0x01,
    /* U     */ 0x3F,0x40,0x40,0x40,0x3F,
    /* V     */ 0x1F,0x20,0x40,0x20,0x1F,
    /* W     */ 0x3F,0x40,0x38,0x40,0x3F,
    /* X     */ 0x63,0x14,0x08,0x14,0x63,
    /* Y     */ 0x07,0x08,0x70,0x08,0x07,
    /* Z     */ 0x61,0x51,0x49,0x45,0x43,
    /* [     */ 0x00,0x7F,0x41,0x41,0x00,
    /* \     */ 0x02,0x04,0x08,0x10,0x20,
    /* ]     */ 0x00,0x41,0x41,0x7F,0x00,
    /* ^     */ 0x04,0x02,0x01,0x02,0x04,
    /* _     */ 0x40,0x40,0x40,0x40,0x40,
    /* `     */ 0x00,0x01,0x02,0x04,0x00,
    /* a     */ 0x20,0x54,0x54,0x54,0x78,
    /* b     */ 0x7F,0x48,0x44,0x44,0x38,
    /* c     */ 0x38,0x44,0x44,0x44,0x20,
    /* d     */ 0x38,0x44,0x44,0x48,0x7F,
    /* e     */ 0x38,0x54,0x54,0x54,0x18,
    /* f     */ 0x08,0x7E,0x09,0x01,0x02,
    /* g     */ 0x0C,0x52,0x52,0x52,0x3E,
    /* h     */ 0x7F,0x08,0x04,0x04,0x78,
    /* i     */ 0x00,0x44,0x7D,0x40,0x00,
    /* j     */ 0x20,0x40,0x44,0x3D,0x00,
    /* k     */ 0x7F,0x10,0x28,0x44,0x00,
    /* l     */ 0x00,0x41,0x7F,0x40,0x00,
    /* m     */ 0x7C,0x04,0x18,0x04,0x78,
    /* n     */ 0x7C,0x08,0x04,0x04,0x78,
    /* o     */ 0x38,0x44,0x44,0x44,0x38,
    /* p     */ 0x7C,0x14,0x14,0x14,0x08,
    /* q     */ 0x08,0x14,0x14,0x18,0x7C,
    /* r     */ 0x7C,0x08,0x04,0x04,0x08,
    /* s     */ 0x48,0x54,0x54,0x54,0x20,
    /* t     */ 0x04,0x3F,0x44,0x40,0x20,
    /* u     */ 0x3C,0x40,0x40,0x20,0x7C,
    /* v     */ 0x1C,0x20,0x40,0x20,0x1C,
    /* w     */ 0x3C,0x40,0x30,0x40,0x3C,
    /* x     */ 0x44,0x28,0x10,0x28,0x44,
    /* y     */ 0x0C,0x50,0x50,0x50,0x3C,
    /* z     */ 0x44,0x64,0x54,0x4C,0x44,
    /* {     */ 0x00,0x08,0x36,0x41,0x00,
    /* |     */ 0x00,0x00,0x7F,0x00,0x00,
    /* }     */ 0x00,0x41,0x36,0x08,0x00,
    /* ~     */ 0x10,0x08,0x08,0x10,0x10,
};

/* Draw a character at pixel (x, page_y) where page_y is 0-7 */
static void draw_char(int x, int page, char c, int scale)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t *glyph = &font_5x7[(c - 32) * 5];

    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        if (scale == 1) {
            int px = x + col;
            if (px >= 0 && px < OLED_W && page < OLED_PAGES) {
                s_fb[page * OLED_W + px] |= bits;
            }
        } else {
            /* Double-size: each pixel becomes 2x2 */
            for (int bit = 0; bit < 7; bit++) {
                if (bits & (1 << bit)) {
                    int py = bit * 2;
                    int p0 = page + (py / 8);
                    int b0 = py % 8;
                    for (int dx = 0; dx < 2; dx++) {
                        int px = x + col * 2 + dx;
                        if (px < 0 || px >= OLED_W) continue;
                        if (p0 < OLED_PAGES)
                            s_fb[p0 * OLED_W + px] |= (0x03 << b0);
                        /* Handle page boundary */
                        if (b0 >= 7 && (p0 + 1) < OLED_PAGES)
                            s_fb[(p0 + 1) * OLED_W + px] |= (0x03 >> (8 - b0));
                    }
                }
            }
        }
    }
}

static void draw_text(int x, int page, const char *text, int scale)
{
    int spacing = (scale == 1) ? 6 : 12;
    for (int i = 0; text[i]; i++) {
        draw_char(x + i * spacing, page, text[i], scale);
        if (x + (i + 1) * spacing >= OLED_W) break;
    }
}

static void draw_hbar(int x, int page, int w, int fill_pct)
{
    if (fill_pct < 0) fill_pct = 0;
    if (fill_pct > 100) fill_pct = 100;
    int fill_w = w * fill_pct / 100;

    for (int i = 0; i < w && (x + i) < OLED_W; i++) {
        if (i < fill_w) {
            s_fb[page * OLED_W + x + i] |= 0x7E; /* Filled bar */
        } else {
            s_fb[page * OLED_W + x + i] |= 0x42; /* Empty outline */
        }
    }
}

/* ── PCNT Encoder ── */

static esp_err_t encoder_init(void)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = 100,
        .low_limit = -100,
    };
    esp_err_t ret = pcnt_new_unit(&unit_cfg, &s_pcnt_unit);
    ESP_RETURN_ON_ERROR(ret, TAG, "PCNT unit create failed");

    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000,
    };
    pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter_cfg);

    /* Channel A: counts on edge of GPIO_A, level of GPIO_B determines direction */
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = CONFIG_FM_ENCODER_GPIO_A,
        .level_gpio_num = CONFIG_FM_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_new_channel(s_pcnt_unit, &chan_a_cfg, &chan_a);
    pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Channel B: counts on edge of GPIO_B, level of GPIO_A determines direction */
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = CONFIG_FM_ENCODER_GPIO_B,
        .level_gpio_num = CONFIG_FM_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    pcnt_new_channel(s_pcnt_unit, &chan_b_cfg, &chan_b);
    pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(s_pcnt_unit);
    pcnt_unit_clear_count(s_pcnt_unit);
    pcnt_unit_start(s_pcnt_unit);

    /* Button GPIO */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_FM_ENCODER_GPIO_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);

    ESP_LOGI(TAG, "Encoder init: A=%d B=%d BTN=%d",
             CONFIG_FM_ENCODER_GPIO_A, CONFIG_FM_ENCODER_GPIO_B,
             CONFIG_FM_ENCODER_GPIO_BTN);
    return ESP_OK;
}

/* ── Public API ── */

esp_err_t ui_hw_init(const ui_hw_config_t *config)
{
    s_encoder_cb = config->encoder_cb;
    s_encoder_ctx = config->cb_ctx;

    /* Get shared I2C bus from audio_out */
    i2c_master_bus_handle_t i2c_bus = (i2c_master_bus_handle_t)audio_out_get_i2c_bus();
    if (!i2c_bus) {
        ESP_LOGE(TAG, "I2C bus not available (audio_out not initialized?)");
        return ESP_ERR_INVALID_STATE;
    }

    /* Add SSD1306 on shared I2C bus */
    i2c_device_config_t oled_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_FM_OLED_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &oled_cfg, &s_oled_dev);
    ESP_RETURN_ON_ERROR(ret, TAG, "OLED I2C device add failed");

    /* Initialize display */
    ret = ssd1306_init_display();
    ESP_RETURN_ON_ERROR(ret, TAG, "SSD1306 init failed");

    ssd1306_clear();
    draw_text(10, 3, "FM Radio", 2);
    draw_text(20, 6, "Initializing...", 1);
    ssd1306_flush();

    /* Initialize encoder */
    ret = encoder_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Encoder init failed (display-only mode): %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Hardware UI initialized (OLED@0x%02x)", CONFIG_FM_OLED_I2C_ADDR);
    return ESP_OK;
}

esp_err_t ui_hw_deinit(void)
{
    if (s_pcnt_unit) {
        pcnt_unit_stop(s_pcnt_unit);
        pcnt_unit_disable(s_pcnt_unit);
        pcnt_del_unit(s_pcnt_unit);
        s_pcnt_unit = NULL;
    }
    if (s_oled_dev) {
        ssd1306_cmd(0xAE); /* Display off */
        i2c_master_bus_rm_device(s_oled_dev);
        s_oled_dev = NULL;
    }
    return ESP_OK;
}

void ui_hw_update(const ui_hw_state_t *state)
{
    if (!s_oled_dev) return;

    ssd1306_clear();

    /* Row 0-1: Frequency (double-size) */
    char freq_str[16];
    snprintf(freq_str, sizeof(freq_str), "%3lu.%03lu",
             (unsigned long)(state->frequency / 1000000),
             (unsigned long)((state->frequency % 1000000) / 1000));
    draw_text(4, 0, freq_str, 2);
    draw_text(100, 1, "MHz", 1);

    /* Row 3: Mode + Volume */
    char info_str[24];
    snprintf(info_str, sizeof(info_str), "%s  Vol:%d%s",
             state->mode == 0 ? "WBFM" : "NBFM",
             state->volume,
             state->muted ? " M" : "");
    draw_text(0, 3, info_str, 1);

    /* Row 4: Signal bar */
    draw_text(0, 4, "Sig:", 1);
    int sig_pct = state->signal_strength * 100 / 32767;
    draw_hbar(30, 4, 90, sig_pct);

    /* Row 5: Gain */
    char gain_str[20];
    if (state->gain == 0) {
        snprintf(gain_str, sizeof(gain_str), "Gain: Auto");
    } else {
        snprintf(gain_str, sizeof(gain_str), "Gain: %d.%d dB",
                 state->gain / 10, state->gain % 10);
    }
    draw_text(0, 5, gain_str, 1);

    /* Row 7: Volume bar */
    draw_text(0, 7, "Vol:", 1);
    draw_hbar(30, 7, 90, state->volume);

    ssd1306_flush();
}

ui_hw_event_t ui_hw_poll_encoder(void)
{
    /* Check rotary encoder */
    if (s_pcnt_unit) {
        int count = 0;
        pcnt_unit_get_count(s_pcnt_unit, &count);
        if (count != 0) {
            pcnt_unit_clear_count(s_pcnt_unit);
            if (count > 0) return UI_HW_EVT_CW;
            if (count < 0) return UI_HW_EVT_CCW;
        }
    }

    /* Check button with debounce */
    bool pressed = (gpio_get_level(CONFIG_FM_ENCODER_GPIO_BTN) == 0); /* Active low */
    int64_t now = esp_timer_get_time();

    if (pressed && !s_btn_was_pressed) {
        s_btn_press_time = now;
        s_btn_was_pressed = true;
    } else if (!pressed && s_btn_was_pressed) {
        s_btn_was_pressed = false;
        int64_t duration = now - s_btn_press_time;
        if (duration > 50000) { /* >50ms debounce */
            if (duration > 1000000) return UI_HW_EVT_LONG_PRESS;
            return UI_HW_EVT_PRESS;
        }
    }

    return UI_HW_EVT_NONE;
}

#endif /* CONFIG_FM_OLED_ENABLE */
