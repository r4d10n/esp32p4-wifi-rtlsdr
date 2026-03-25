/*
 * USB Audio Class (UAC 2.0) Device
 *
 * Streams FM demodulated audio to PC as a virtual USB microphone.
 * Uses ESP32-P4 Controller 1 (FS, GPIO26/27) via TinyUSB.
 * Composite device: UAC + CDC (for CI-V).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include "sdkconfig.h"

#ifdef CONFIG_FM_USB_AUDIO_ENABLE

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t usb_audio_init(void);
esp_err_t usb_audio_deinit(void);

/**
 * Queue audio samples for USB transmission.
 * Called from the FM pipeline after demodulation.
 * Non-blocking: drops samples if USB buffer is full.
 */
esp_err_t usb_audio_write(const int16_t *samples, int count);

/**
 * Check if USB audio host is connected and streaming.
 */
bool usb_audio_is_connected(void);

#ifdef __cplusplus
}
#endif
#endif /* CONFIG_FM_USB_AUDIO_ENABLE */
