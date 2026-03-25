/*
 * Icom CI-V Protocol Emulator
 *
 * Emulates an Icom IC-R8600 wideband receiver (address 0x96).
 * Processes CI-V commands over USB CDC serial for CAT control.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "civ_emu.h"

static const char *TAG = "civ_emu";

/* ──────────────────────── Internal State ──────────────────────── */

struct civ_emu {
    civ_change_cb_t change_cb;
    void           *cb_ctx;

    /* Current radio state (updated via civ_emu_update_state) */
    uint32_t        freq;
    int             mode;
    int16_t         signal_strength;
    uint8_t         volume;
    uint8_t         squelch;

    /* Frame parser state */
    uint8_t         rx_buf[64];
    int             rx_pos;
    bool            in_frame;
};

/* ──────────────────────── BCD Helpers ──────────────────────── */

/*
 * CI-V frequency: 5 bytes, little-endian BCD.
 * byte[0] = 10Hz+1Hz digits, byte[1] = 1kHz+100Hz, ... byte[4] = 10GHz+1GHz
 */
static void freq_to_bcd(uint32_t freq_hz, uint8_t *bcd5)
{
    uint64_t f = (uint64_t)freq_hz / 10; /* work in 10 Hz units */
    for (int i = 0; i < 5; i++) {
        uint8_t lo = f % 10; f /= 10;
        uint8_t hi = f % 10; f /= 10;
        bcd5[i] = (hi << 4) | lo;
    }
}

static uint32_t bcd_to_freq(const uint8_t *bcd5)
{
    uint64_t freq = 0;
    uint64_t mul = 1;
    for (int i = 0; i < 5; i++) {
        freq += (bcd5[i] & 0x0F) * mul;        mul *= 10;
        freq += ((bcd5[i] >> 4) & 0x0F) * mul; mul *= 10;
    }
    return (uint32_t)(freq * 10);
}

/* ──────────────────────── Frame Builder ──────────────────────── */

static int build_frame(uint8_t *buf, int max, uint8_t cmd, uint8_t sub,
                       const uint8_t *data, int data_len)
{
    /* sub == 0xFF means no sub-command byte */
    int needed = 2 + 2 + 1 + (sub != 0xFF ? 1 : 0) + data_len + 1;
    if (max < needed) return 0;

    int pos = 0;
    buf[pos++] = CIV_PREAMBLE;
    buf[pos++] = CIV_PREAMBLE;
    buf[pos++] = CIV_ADDR_PC;
    buf[pos++] = CIV_ADDR_RADIO;
    buf[pos++] = cmd;
    if (sub != 0xFF) {
        buf[pos++] = sub;
    }
    for (int i = 0; i < data_len; i++) {
        buf[pos++] = data[i];
    }
    buf[pos++] = CIV_EOM;
    return pos;
}

/* ──────────────────────── Command Handlers ──────────────────────── */

static int handle_read_freq(civ_emu_t *civ, uint8_t *resp, int resp_max)
{
    uint8_t bcd[5];
    freq_to_bcd(civ->freq, bcd);
    return build_frame(resp, resp_max, CIV_CMD_READ_FREQ, 0xFF, bcd, 5);
}

static int handle_set_freq(civ_emu_t *civ, const uint8_t *data, int data_len,
                           uint8_t *resp, int resp_max)
{
    if (data_len < 5) return 0;
    uint32_t new_freq = bcd_to_freq(data);
    civ->freq = new_freq;
    if (civ->change_cb) {
        civ_params_t p = {
            .frequency = new_freq,
            .mode      = civ->mode,
            .volume    = civ->volume,
            .squelch   = civ->squelch,
        };
        civ->change_cb(&p, civ->cb_ctx);
    }
    return build_frame(resp, resp_max, 0xFB, 0xFF, NULL, 0); /* ACK */
}

static int handle_read_mode(civ_emu_t *civ, uint8_t *resp, int resp_max)
{
    uint8_t mode_data[2];
    mode_data[0] = (civ->mode == 0) ? CIV_MODE_WFM : CIV_MODE_FM;
    mode_data[1] = 0x01; /* Filter 1 */
    return build_frame(resp, resp_max, CIV_CMD_READ_MODE, 0xFF, mode_data, 2);
}

static int handle_read_meter(civ_emu_t *civ, uint8_t *resp, int resp_max)
{
    int16_t sig = civ->signal_strength;
    if (sig < 0) sig = 0;
    uint8_t meter[3];
    meter[0] = 0x02; /* S-meter sub-selector */
    uint16_t level = (sig > 0x120) ? 0x120 : (uint16_t)sig;
    meter[1] = (level >> 8) & 0xFF;
    meter[2] = level & 0xFF;
    return build_frame(resp, resp_max, CIV_CMD_READ_METER, 0xFF, meter, 3);
}

/* ──────────────────────── Frame Parser ──────────────────────── */

static int process_frame(civ_emu_t *civ, const uint8_t *frame, int frame_len,
                         uint8_t *resp, int resp_max)
{
    if (frame_len < 6) return 0;
    if (frame[0] != CIV_PREAMBLE || frame[1] != CIV_PREAMBLE) return 0;
    if (frame[frame_len - 1] != CIV_EOM) return 0;

    uint8_t cmd = frame[4];
    const uint8_t *data = &frame[5];
    int data_len = frame_len - 6;

    switch (cmd) {
    case CIV_CMD_READ_FREQ:
        return handle_read_freq(civ, resp, resp_max);
    case CIV_CMD_SET_FREQ:
        return handle_set_freq(civ, data, data_len, resp, resp_max);
    case CIV_CMD_READ_MODE:
        return handle_read_mode(civ, resp, resp_max);
    case CIV_CMD_READ_METER:
        return handle_read_meter(civ, resp, resp_max);
    default:
        ESP_LOGD(TAG, "Unhandled CI-V cmd: 0x%02X", cmd);
        return build_frame(resp, resp_max, 0xFA, 0xFF, NULL, 0); /* NACK */
    }
}

/* ──────────────────────── Public API ──────────────────────── */

civ_emu_t *civ_emu_create(civ_change_cb_t change_cb, void *ctx)
{
    civ_emu_t *civ = calloc(1, sizeof(civ_emu_t));
    if (!civ) return NULL;
    civ->change_cb = change_cb;
    civ->cb_ctx    = ctx;
    civ->freq      = 100000000;
    civ->mode      = 0;
    ESP_LOGI(TAG, "CI-V emulator created (IC-R8600, addr=0x%02X)", CIV_ADDR_RADIO);
    return civ;
}

void civ_emu_free(civ_emu_t *civ)
{
    free(civ);
}

void civ_emu_update_state(civ_emu_t *civ, uint32_t freq, int mode,
                           int16_t signal_strength, uint8_t volume, uint8_t squelch)
{
    if (!civ) return;
    civ->freq            = freq;
    civ->mode            = mode;
    civ->signal_strength = signal_strength;
    civ->volume          = volume;
    civ->squelch         = squelch;
}

int civ_emu_process(civ_emu_t *civ, const uint8_t *data, int len,
                     uint8_t *resp_buf, int resp_max)
{
    if (!civ || !data || len <= 0) return 0;

    int resp_len = 0;

    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];

        if (b == CIV_PREAMBLE && civ->rx_pos == 0) {
            civ->rx_buf[civ->rx_pos++] = b;
            civ->in_frame = true;
        } else if (civ->in_frame) {
            if (civ->rx_pos < (int)sizeof(civ->rx_buf)) {
                civ->rx_buf[civ->rx_pos++] = b;
            }
            if (b == CIV_EOM) {
                int r = process_frame(civ, civ->rx_buf, civ->rx_pos,
                                      resp_buf + resp_len, resp_max - resp_len);
                resp_len += r;
                civ->rx_pos = 0;
                civ->in_frame = false;
            }
        }
    }

    return resp_len;
}

int civ_emu_make_scope_frame(civ_emu_t *civ, const uint8_t *fft_data, int fft_len,
                               uint32_t center_freq, uint32_t span,
                               uint8_t *out_buf, int out_max)
{
    /* CI-V scope frame format:
     * FE FE <to> <from> 0x27 0x00 <center_freq_bcd[5]> <span_bcd[5]> <data[N]> FD
     *
     * Total overhead: 2(preamble) + 2(addr) + 2(cmd+sub) + 5(freq) + 5(span) + 1(EOM) = 17 bytes
     */
    if (!civ || !fft_data || fft_len <= 0) return 0;
    if (out_max < 17 + fft_len) return 0;

    int pos = 0;
    out_buf[pos++] = CIV_PREAMBLE;
    out_buf[pos++] = CIV_PREAMBLE;
    out_buf[pos++] = CIV_ADDR_PC;
    out_buf[pos++] = CIV_ADDR_RADIO;
    out_buf[pos++] = CIV_CMD_SCOPE_DATA;   /* 0x27 */
    out_buf[pos++] = 0x00;                  /* Sub-command: scope data */

    /* Center frequency as 5-byte BCD */
    freq_to_bcd(center_freq, &out_buf[pos]);
    pos += 5;

    /* Span as 5-byte BCD (in Hz) */
    freq_to_bcd(span, &out_buf[pos]);
    pos += 5;

    /* FFT dB data (0-255, already computed) */
    /* Avoid protocol bytes appearing in data stream */
    for (int i = 0; i < fft_len && pos < out_max - 1; i++) {
        uint8_t d = fft_data[i];
        if (d == CIV_PREAMBLE || d == CIV_EOM) d--;
        out_buf[pos++] = d;
    }

    out_buf[pos++] = CIV_EOM;
    return pos;
}
