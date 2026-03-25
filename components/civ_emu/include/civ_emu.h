/*
 * Icom CI-V Protocol Emulator
 *
 * Emulates an Icom IC-R8600 wideband receiver (address 0x96).
 * Processes CI-V commands over USB CDC serial for CAT control
 * from Hamlib, SDR++, HDSDR, and other radio control software.
 *
 * CI-V frame: 0xFE 0xFE <to> <from> <cmd> [<sub>] [<data>...] 0xFD
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CIV_ADDR_RADIO  0x96    /* IC-R8600 address */
#define CIV_ADDR_PC     0xE0    /* Default PC controller address */
#define CIV_PREAMBLE    0xFE
#define CIV_EOM         0xFD

/* CI-V commands */
#define CIV_CMD_SET_FREQ    0x05    /* Set frequency (BCD) */
#define CIV_CMD_READ_FREQ   0x03    /* Read frequency */
#define CIV_CMD_READ_MODE   0x04    /* Read mode */
#define CIV_CMD_SET_MODE    0x06    /* Set mode */
#define CIV_CMD_READ_METER  0x15    /* Read meter */
#define CIV_CMD_SET_LEVEL   0x14    /* Set level */
#define CIV_CMD_READ_LEVEL  0x15    /* Read level */
#define CIV_CMD_SCOPE_DATA  0x27    /* Scope/spectrum data */

/* CI-V mode codes */
#define CIV_MODE_FM     0x05
#define CIV_MODE_WFM    0x06

/* Callback for parameter changes from CI-V */
typedef struct {
    uint32_t    frequency;
    int         mode;       /* 0=WBFM, 1=NBFM */
    uint8_t     volume;
    uint8_t     squelch;
} civ_params_t;

typedef void (*civ_change_cb_t)(const civ_params_t *params, void *ctx);

typedef struct civ_emu civ_emu_t;

/**
 * Create CI-V emulator.
 * @param change_cb  Called when PC changes a parameter via CI-V
 * @param ctx        Callback context
 */
civ_emu_t *civ_emu_create(civ_change_cb_t change_cb, void *ctx);
void civ_emu_free(civ_emu_t *civ);

/**
 * Process incoming bytes from USB CDC serial.
 * Parses CI-V frames and generates responses.
 * @param civ       Emulator handle
 * @param data      Received bytes
 * @param len       Number of bytes
 * @param resp_buf  Buffer for response bytes
 * @param resp_max  Max response buffer size
 * @return Number of response bytes to send back
 */
int civ_emu_process(civ_emu_t *civ, const uint8_t *data, int len,
                     uint8_t *resp_buf, int resp_max);

/**
 * Update emulator's internal state (call from main loop).
 * Allows CI-V to report current frequency, mode, S-meter.
 */
void civ_emu_update_state(civ_emu_t *civ, uint32_t freq, int mode,
                           int16_t signal_strength, uint8_t volume, uint8_t squelch);

/**
 * Generate spectrum data CI-V frame.
 * @param civ       Emulator handle
 * @param fft_data  FFT dB data (uint8, 0-255)
 * @param fft_len   Number of FFT bins
 * @param center_freq Center frequency in Hz
 * @param span      Span in Hz
 * @param out_buf   Output buffer for CI-V frame
 * @param out_max   Max output buffer size
 * @return Number of bytes in output frame
 */
int civ_emu_make_scope_frame(civ_emu_t *civ, const uint8_t *fft_data, int fft_len,
                               uint32_t center_freq, uint32_t span,
                               uint8_t *out_buf, int out_max);

#ifdef __cplusplus
}
#endif
