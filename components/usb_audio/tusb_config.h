/*
 * TinyUSB configuration for USB Audio (UAC 2.0) + CDC composite device.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once

#include "sdkconfig.h"

/* Use Controller 1 (FS) as device */
#define CFG_TUSB_RHPORT1_MODE   OPT_MODE_DEVICE

/* Control endpoint size */
#define CFG_TUD_ENDPOINT0_SIZE  64

#ifdef CONFIG_FM_USB_AUDIO_ENABLE

/* Enable Audio and CDC classes */
#define CFG_TUD_AUDIO           1
#define CFG_TUD_CDC             1

/* Audio format: 48kHz / 16-bit / mono microphone (TX to host) */
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN              TUD_AUDIO_MIC_ONE_CH_DESC_LEN
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT              1
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ           64
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX         1
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX 2
#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE            48000

#define CFG_TUD_AUDIO_ENABLE_EP_IN                 1

/* EP size: 48 samples * 2 bytes = 96 bytes per 1ms USB frame */
#define CFG_TUD_AUDIO_EP_SZ_IN                     (48 * 2)
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX          CFG_TUD_AUDIO_EP_SZ_IN
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ       (CFG_TUD_AUDIO_EP_SZ_IN * 4)

/* CDC buffer sizes for CI-V serial */
#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  256

#else /* !CONFIG_FM_USB_AUDIO_ENABLE */

/* Disable all classes when feature is off */
#define CFG_TUD_AUDIO           0
#define CFG_TUD_CDC             0

#endif /* CONFIG_FM_USB_AUDIO_ENABLE */
