/*
 * USB Audio Class (UAC 2.0) Device Implementation
 *
 * Composite device: UAC 2.0 microphone (48kHz/16-bit/mono) + CDC serial (CI-V).
 * Runs on ESP32-P4 USB Controller 1 (FS, GPIO26/27) via TinyUSB.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "sdkconfig.h"
#ifdef CONFIG_FM_USB_AUDIO_ENABLE

#include <string.h>
#include "tinyusb.h"
#include "tusb.h"
#include "class/audio/audio.h"
#include "class/audio/audio_device.h"
#include "class/cdc/cdc_device.h"
#include "esp_log.h"
#include "esp_check.h"
#include "usb_audio.h"

static const char *TAG = "usb_audio";

/* ── Audio Parameters ── */

#define AUDIO_SAMPLE_RATE       48000
#define AUDIO_BIT_DEPTH         16
#define AUDIO_CHANNELS          1
#define AUDIO_BYTES_PER_SAMPLE  (AUDIO_BIT_DEPTH / 8 * AUDIO_CHANNELS)

/* USB frame: 48 samples per 1ms frame at 48kHz */
#define SAMPLES_PER_FRAME       48
#define BYTES_PER_FRAME         (SAMPLES_PER_FRAME * AUDIO_BYTES_PER_SAMPLE)

/* Ring buffer for audio data: 20 frames = 20ms */
#define USB_AUDIO_BUF_FRAMES    20
#define USB_AUDIO_BUF_SIZE      (BYTES_PER_FRAME * USB_AUDIO_BUF_FRAMES)

static uint8_t  s_audio_buf[USB_AUDIO_BUF_SIZE];
static volatile uint32_t s_buf_wr;
static volatile uint32_t s_buf_rd;
static volatile bool     s_connected;

/* ── USB Descriptor Entity IDs ── */

enum {
    UAC_ENTITY_CLOCK   = 0x04,
    UAC_ENTITY_INPUT   = 0x01,
    UAC_ENTITY_FEATURE = 0x02,
    UAC_ENTITY_OUTPUT  = 0x03,
};

/* ── Interface & Endpoint Numbers ── */

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING,
    ITF_NUM_CDC_CONTROL,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL,
};

#define EPNUM_AUDIO_IN      0x01
#define EPNUM_CDC_NOTIF     0x02
#define EPNUM_CDC_DATA      0x03

/* ── String Descriptors ── */

enum {
    STR_LANGID = 0,
    STR_MANUFACTURER,
    STR_PRODUCT,
    STR_SERIAL,
    STR_CDC_NAME,
};

static const char *s_string_desc[] = {
    [STR_LANGID]       = "\x09\x04",  /* English */
    [STR_MANUFACTURER] = "ESP32-P4 FM Radio",
    [STR_PRODUCT]      = "FM Radio Audio + Control",
    [STR_SERIAL]       = "123456",
    [STR_CDC_NAME]     = "CI-V Control",
};

/* ── Device Descriptor ── */

static const tusb_desc_device_t s_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,       /* Espressif VID */
    .idProduct          = 0x4006,       /* Custom PID */
    .bcdDevice          = 0x0100,
    .iManufacturer      = STR_MANUFACTURER,
    .iProduct           = STR_PRODUCT,
    .iSerialNumber      = STR_SERIAL,
    .bNumConfigurations = 1,
};

/* ── Configuration Descriptor ── */

/*
 * UAC 2.0 microphone one-channel descriptor length + CDC descriptor length.
 * TUD_AUDIO_MIC_ONE_CH_DESC_LEN and TUD_CDC_DESC_LEN are TinyUSB macros.
 * If they are not available, we define approximate lengths.
 */
#ifndef TUD_AUDIO_MIC_ONE_CH_DESC_LEN
#define TUD_AUDIO_MIC_ONE_CH_DESC_LEN  122
#endif

#ifndef TUD_CDC_DESC_LEN
#define TUD_CDC_DESC_LEN  66
#endif

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_AUDIO_MIC_ONE_CH_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t s_config_desc[] = {
    /* Configuration descriptor header */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    /* UAC 2.0 Microphone: mono, 48kHz, 16-bit */
    TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR(
        /* _itfnum */       ITF_NUM_AUDIO_CONTROL,
        /* _stridx */       0,
        /* _nBytesPerSample */ 2,
        /* _nBitsUsedPerSample */ 16,
        /* _epin */         (0x80 | EPNUM_AUDIO_IN),
        /* _epsize */       BYTES_PER_FRAME
    ),

    /* CDC Serial for CI-V protocol */
    TUD_CDC_DESCRIPTOR(
        /* _itfnum */       ITF_NUM_CDC_CONTROL,
        /* _stridx */       STR_CDC_NAME,
        /* _ep_notif */     (0x80 | EPNUM_CDC_NOTIF),
        /* _ep_notif_size */ 8,
        /* _epout */        EPNUM_CDC_DATA,
        /* _epin */         (0x80 | EPNUM_CDC_DATA),
        /* _epsize */       64
    ),
};

/* ── TinyUSB Descriptor Callbacks ── */

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&s_device_desc;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return s_config_desc;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t desc_str[32 + 1];

    if (index == 0) {
        desc_str[1] = 0x0409; /* English */
        desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * 1 + 2));
        return desc_str;
    }

    if (index >= sizeof(s_string_desc) / sizeof(s_string_desc[0])) {
        return NULL;
    }

    const char *str = s_string_desc[index];
    if (!str) return NULL;

    uint8_t len = (uint8_t)strlen(str);
    if (len > 31) len = 31;

    for (uint8_t i = 0; i < len; i++) {
        desc_str[1 + i] = str[i];
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));

    return desc_str;
}

/* ── Ring Buffer Helpers ── */

static inline uint32_t buf_available(void)
{
    return (s_buf_wr - s_buf_rd + USB_AUDIO_BUF_SIZE) % USB_AUDIO_BUF_SIZE;
}

/* ── TinyUSB Audio Callbacks ── */

/* Called by TinyUSB when it needs audio data for the next USB frame */
bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf,
                                    uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void)rhport; (void)itf; (void)ep_in; (void)cur_alt_setting;

    uint8_t frame[BYTES_PER_FRAME];
    uint32_t avail = buf_available();

    if (avail >= BYTES_PER_FRAME) {
        for (int i = 0; i < BYTES_PER_FRAME; i++) {
            frame[i] = s_audio_buf[s_buf_rd % USB_AUDIO_BUF_SIZE];
            s_buf_rd = (s_buf_rd + 1) % USB_AUDIO_BUF_SIZE;
        }
    } else {
        /* Underrun: send silence */
        memset(frame, 0, BYTES_PER_FRAME);
    }

    tud_audio_write(frame, BYTES_PER_FRAME);
    return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied,
                                     uint8_t itf, uint8_t ep_in,
                                     uint8_t cur_alt_setting)
{
    (void)rhport; (void)n_bytes_copied; (void)itf; (void)ep_in;
    (void)cur_alt_setting;
    return true;
}

/* Called when host sets alternate setting (starts/stops streaming) */
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t alt_setting = (uint8_t)p_request->wValue;
    s_connected = (alt_setting > 0);
    ESP_LOGI(TAG, "USB Audio: host %s streaming (alt=%d)",
             s_connected ? "started" : "stopped", alt_setting);
    return true;
}

/* Handle clock source and feature unit control requests */
bool tud_audio_get_req_entity_cb(uint8_t rhport,
                                  tusb_control_request_t const *p_request)
{
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    uint8_t ctrlSel  = TU_U16_HIGH(p_request->wValue);

    if (entityID == UAC_ENTITY_CLOCK) {
        if (ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) {
            /* Report fixed 48kHz sample rate */
            audio_control_cur_4_t freq = { .bCur = AUDIO_SAMPLE_RATE };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, p_request, &freq, sizeof(freq));
        }
        if (ctrlSel == AUDIO_CS_CTRL_CLK_VALID) {
            audio_control_cur_1_t valid = { .bCur = 1 };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, p_request, &valid, sizeof(valid));
        }
    }

    /* Feature Unit: mute/volume -- accept but ignore */
    if (entityID == UAC_ENTITY_FEATURE) {
        if (ctrlSel == AUDIO_FU_CTRL_MUTE) {
            audio_control_cur_1_t mute = { .bCur = 0 };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, p_request, &mute, sizeof(mute));
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME) {
            audio_control_cur_2_t vol = { .bCur = 0x0000 };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, p_request, &vol, sizeof(vol));
        }
    }

    ESP_LOGD(TAG, "Unhandled audio entity request: entity=%d ctrl=%d", entityID, ctrlSel);
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport,
                                  tusb_control_request_t const *p_request,
                                  uint8_t *buf)
{
    (void)rhport; (void)buf;
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    uint8_t ctrlSel  = TU_U16_HIGH(p_request->wValue);

    /* Accept all set requests (clock freq, mute, volume) without error */
    ESP_LOGD(TAG, "Audio set entity: entity=%d ctrl=%d", entityID, ctrlSel);
    return true;
}

/* ── Public API ── */

esp_err_t usb_audio_init(void)
{
    ESP_LOGI(TAG, "Initializing USB Audio (UAC 2.0) + CDC on Controller 1");

    s_buf_wr = 0;
    s_buf_rd = 0;
    s_connected = false;
    memset(s_audio_buf, 0, sizeof(s_audio_buf));

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor        = &s_device_desc,
        .string_descriptor        = s_string_desc,
        .string_descriptor_count  = sizeof(s_string_desc) / sizeof(s_string_desc[0]),
        .external_phy             = false,
        .configuration_descriptor = s_config_desc,
    };

    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG,
                        "TinyUSB install failed");

    ESP_LOGI(TAG, "USB Audio ready (48kHz/16-bit/mono, GPIO26/27)");
    return ESP_OK;
}

esp_err_t usb_audio_deinit(void)
{
    s_connected = false;
    /* TinyUSB doesn't have a clean uninstall in ESP-IDF wrapper */
    ESP_LOGI(TAG, "USB Audio deinitialized");
    return ESP_OK;
}

esp_err_t usb_audio_write(const int16_t *samples, int count)
{
    if (!s_connected) {
        return ESP_OK;  /* No host streaming, silently discard */
    }

    const uint8_t *data = (const uint8_t *)samples;
    int bytes = count * AUDIO_BYTES_PER_SAMPLE;

    for (int i = 0; i < bytes; i++) {
        uint32_t next_wr = (s_buf_wr + 1) % USB_AUDIO_BUF_SIZE;
        if (next_wr == s_buf_rd) {
            break;  /* Buffer full, drop remaining */
        }
        s_audio_buf[s_buf_wr] = data[i];
        s_buf_wr = next_wr;
    }

    return ESP_OK;
}

bool usb_audio_is_connected(void)
{
    return s_connected;
}

#endif /* CONFIG_FM_USB_AUDIO_ENABLE */
