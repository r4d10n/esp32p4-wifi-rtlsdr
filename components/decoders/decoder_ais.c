/*
 * AIS Maritime Vessel Tracker — Dual-Channel GMSK/FSK Demodulator
 *
 * Decodes AIS (Automatic Identification System) on two VHF channels:
 *   Channel A: 161.975 MHz (AIS1)
 *   Channel B: 162.025 MHz (AIS2)
 *
 * Modulation: GMSK at 9600 baud, BT=0.4 (approximated as FSK per rtl-ais)
 * Framing: HDLC with NRZI encoding, bit stuffing, CRC-16 FCS
 * Payload: 6-bit ASCII packed binary (ITU-R M.1371)
 *
 * Message types decoded:
 *   1-3: Position report (Class A)
 *   4:   Base station report
 *   5:   Static and voyage data
 *   18:  Standard Class B position report
 *   19:  Extended Class B position report
 *   21:  Aid-to-Navigation report
 *   24:  Class B CS static data
 *
 * Architecture: Raw IQ in -> dual DDC (NCO mix to +-25 kHz) -> FM discriminator
 *   -> PLL clock recovery -> NRZI decode -> HDLC deframe -> CRC check -> parse
 *
 * References: rtl-ais, ais-catcher, libais, gr-ais
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/socket.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "decoder_framework.h"

static const char *TAG = "dec_ais";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════
 *  Constants
 * ═══════════════════════════════════════════════════════════════ */

#define AIS_BAUD            9600
#define AIS_CH_A_OFFSET     (-25000.0)   /* Hz from 162.0 MHz center */
#define AIS_CH_B_OFFSET     (+25000.0)
#define AIS_CH_A_FREQ_HZ    161975000
#define AIS_CH_B_FREQ_HZ    162025000
#define AIS_CENTER_FREQ_HZ  162000000
#define AIS_SAMPLE_RATE     250000.0     /* Default IQ sample rate */
#define AIS_MAX_FRAME_LEN   512          /* Max HDLC frame bytes */
#define AIS_HDLC_FLAG       0x7E
#define AIS_CRC_RESIDUAL    0xF0B8       /* Valid CRC-16 residual */

/* PLL tuning constants (rtl-ais style) */
#define PLL_GAIN_PHASE      0.1
#define PLL_GAIN_FREQ       0.01

/* ═══════════════════════════════════════════════════════════════
 *  Per-channel demodulator state
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    /* DDC NCO */
    double nco_phase;

    /* FM discriminator memory */
    double prev_i;
    double prev_q;

    /* PLL clock recovery */
    double pll_phase;
    double pll_freq;           /* Nominal = samples_per_bit */
    int    prev_sign;          /* Previous demod sign for zero-crossing */

    /* HDLC framing */
    uint8_t shift_reg;         /* 8-bit sliding window for flag detection */
    int     ones_count;        /* Consecutive 1s for bit unstuffing */
    uint8_t frame_buf[AIS_MAX_FRAME_LEN];
    int     frame_bits;        /* Total bits accumulated in current frame */
    bool    in_frame;
    int     prev_nrzi_bit;     /* NRZI state: previous raw bit */
} ais_channel_t;

/* ═══════════════════════════════════════════════════════════════
 *  Decoder context
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    /* Per-channel state */
    ais_channel_t ch_a;
    ais_channel_t ch_b;

    /* Common */
    SemaphoreHandle_t mutex;
    bool     running;
    int      vessel_count;
    uint32_t frames_decoded;
    uint32_t frames_crc_fail;

    /* DDC config */
    double sample_rate;
    double center_freq;        /* RTL-SDR center (162.0 MHz) */
} ais_ctx_t;

static ais_ctx_t s_ais_ctx;

/* Forward declarations */
static void ais_process_frame(ais_ctx_t *ctx, const uint8_t *frame,
                               int frame_bytes, char channel);

/* ═══════════════════════════════════════════════════════════════
 *  CRC-16 FCS (CCITT, polynomial 0x8408 reflected)
 *
 *  Same polynomial as AX.25. Feed the entire frame including
 *  the 2-byte FCS; a valid frame yields residual 0xF0B8.
 * ═══════════════════════════════════════════════════════════════ */

static uint16_t ais_crc16(const uint8_t *data, int bytes) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < bytes; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════
 *  FM discriminator (cross-product method)
 *
 *  Approximates GMSK as FM: instantaneous frequency is proportional
 *  to cross-product of consecutive IQ samples normalized by magnitude.
 *  For AIS: +4800 Hz = mark (1), -4800 Hz = space (0)
 * ═══════════════════════════════════════════════════════════════ */

static double fm_demod(double i, double q, double *prev_i, double *prev_q) {
    double cross = i * (*prev_q) - q * (*prev_i);
    double mag_sq = i * i + q * q;
    if (mag_sq > 0.0001)
        cross /= mag_sq;
    *prev_i = i;
    *prev_q = q;
    return cross;
}

/* ═══════════════════════════════════════════════════════════════
 *  PLL-based clock recovery
 *
 *  Track zero crossings in the FM-demodulated signal. Adjust PLL
 *  phase proportionally and frequency integrally at each crossing.
 *  Output a bit at PLL phase wrap (center of bit period).
 *
 *  Returns: 0 or 1 when a bit is ready, -1 otherwise.
 * ═══════════════════════════════════════════════════════════════ */

static int clock_recover_sample(double sample, double *pll_phase,
                                 double *pll_freq, double samples_per_bit,
                                 int *prev_sign) {
    int current_sign = (sample > 0.0) ? 1 : 0;

    /* Detect zero crossing */
    if (current_sign != *prev_sign) {
        /* Phase error: distance from expected crossing at phase=0.5 */
        double phase_error = *pll_phase - 0.5;
        /* Proportional correction */
        *pll_phase -= phase_error * PLL_GAIN_PHASE;
        /* Integral correction (pull frequency toward nominal) */
        *pll_freq = samples_per_bit - phase_error * PLL_GAIN_FREQ;
        *prev_sign = current_sign;
    }

    /* Advance PLL phase */
    *pll_phase += 1.0 / *pll_freq;

    /* Output bit at phase wrap (center of bit) */
    if (*pll_phase >= 1.0) {
        *pll_phase -= 1.0;
        return (sample > 0.0) ? 1 : 0;
    }
    return -1;  /* No bit this sample */
}

/* ═══════════════════════════════════════════════════════════════
 *  HDLC bit processing (NRZI decode + unstuffing + framing)
 *
 *  AIS uses NRZI: same level = 1, level change = 0.
 *  After 5 consecutive 1s, a stuffed 0 is discarded.
 *  Flag byte 0x7E delimits frames.
 *  Bits are accumulated MSB-first (network order) per AIS spec.
 * ═══════════════════════════════════════════════════════════════ */

static void hdlc_process_bit(ais_ctx_t *ctx, ais_channel_t *ch,
                              int bit, char channel) {
    /* NRZI decode: same = 1, change = 0 */
    int data_bit = (bit == ch->prev_nrzi_bit) ? 1 : 0;
    ch->prev_nrzi_bit = bit;

    /* Shift register for flag detection */
    ch->shift_reg = (ch->shift_reg << 1) | (data_bit & 1);

    if (ch->shift_reg == AIS_HDLC_FLAG) {
        /* Flag byte detected */
        if (ch->in_frame && ch->frame_bits >= 40) {
            /* End of frame — need at least 5 bytes (header + FCS) */
            int frame_bytes = ch->frame_bits / 8;
            ais_process_frame(ctx, ch->frame_buf, frame_bytes, channel);
        }
        /* Start/reset frame */
        ch->in_frame = true;
        ch->frame_bits = 0;
        ch->ones_count = 0;
        return;
    }

    if (!ch->in_frame) return;

    /* Abort: 7+ consecutive 1s */
    if (data_bit == 1) {
        ch->ones_count++;
        if (ch->ones_count > 6) {
            ch->in_frame = false;
            return;
        }
    } else {
        if (ch->ones_count == 5) {
            /* Stuffed bit — discard */
            ch->ones_count = 0;
            return;
        }
        ch->ones_count = 0;
    }

    /* Accumulate bit into frame buffer (MSB first, network byte order) */
    int byte_idx = ch->frame_bits / 8;
    int bit_idx  = ch->frame_bits % 8;
    if (bit_idx == 0 && byte_idx < AIS_MAX_FRAME_LEN) {
        ch->frame_buf[byte_idx] = 0;
    }
    if (byte_idx < AIS_MAX_FRAME_LEN) {
        ch->frame_buf[byte_idx] |= (data_bit << (7 - bit_idx));
        ch->frame_bits++;
    } else {
        /* Frame too long — abort */
        ch->in_frame = false;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  AIS payload bit extraction
 *
 *  AIS payloads are packed MSB-first. These helpers extract
 *  unsigned/signed integers and 6-bit ASCII strings.
 * ═══════════════════════════════════════════════════════════════ */

static uint32_t ais_get_uint(const uint8_t *payload, int start_bit, int num_bits) {
    uint32_t result = 0;
    for (int i = 0; i < num_bits; i++) {
        int byte_idx = (start_bit + i) / 8;
        int bit_idx  = 7 - ((start_bit + i) % 8);
        result = (result << 1) | ((payload[byte_idx] >> bit_idx) & 1);
    }
    return result;
}

static int32_t ais_get_int(const uint8_t *payload, int start_bit, int num_bits) {
    uint32_t u = ais_get_uint(payload, start_bit, num_bits);
    /* Sign extend */
    if (u & (1U << (num_bits - 1))) {
        u |= ~((1U << num_bits) - 1);
    }
    return (int32_t)u;
}

/* 6-bit ASCII decode per ITU-R M.1371 */
static void ais_get_string(const uint8_t *payload, int start_bit,
                            int num_chars, char *out) {
    for (int i = 0; i < num_chars; i++) {
        int c = (int)ais_get_uint(payload, start_bit + i * 6, 6);
        if (c < 32)
            c += 64;  /* 6-bit AIS ASCII mapping: 0-31 -> '@'-'_' */
        out[i] = (char)c;
    }
    out[num_chars] = '\0';
    /* Trim trailing '@' (AIS padding character) and spaces */
    for (int i = num_chars - 1; i >= 0; i--) {
        if (out[i] == '@' || out[i] == ' ')
            out[i] = '\0';
        else
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  AIS frame processing and message parsing
 *
 *  Validates CRC, extracts message type + MMSI, parses known
 *  message types into JSON, publishes event and updates tracking.
 * ═══════════════════════════════════════════════════════════════ */

static void ais_process_frame(ais_ctx_t *ctx, const uint8_t *frame,
                               int frame_bytes, char channel) {
    /* Minimum: payload (at least 1 byte) + 2 bytes FCS */
    if (frame_bytes < 5) return;

    /* CRC-16 check: feed entire frame including FCS, expect residual */
    uint16_t crc = ais_crc16(frame, frame_bytes);
    if (crc != AIS_CRC_RESIDUAL) {
        xSemaphoreTake(ctx->mutex, portMAX_DELAY);
        ctx->frames_crc_fail++;
        xSemaphoreGive(ctx->mutex);
        return;
    }

    /* Payload excludes the 2-byte FCS */
    int payload_bytes = frame_bytes - 2;
    int payload_bits  = payload_bytes * 8;
    const uint8_t *payload = frame;

    /* All AIS messages start with: msg_type(6) + repeat(2) + MMSI(30) = 38 bits min */
    if (payload_bits < 38) return;

    int msg_type = (int)ais_get_uint(payload, 0, 6);
    uint32_t mmsi = ais_get_uint(payload, 8, 30);

    char mmsi_str[12];
    snprintf(mmsi_str, sizeof(mmsi_str), "%09" PRIu32, mmsi);

    ESP_LOGI(TAG, "AIS type %d from %s on ch %c (%d bytes)",
             msg_type, mmsi_str, channel, frame_bytes);

    /* Build JSON event data */
    cJSON *data = cJSON_CreateObject();
    if (!data) return;

    cJSON_AddStringToObject(data, "mmsi", mmsi_str);
    cJSON_AddNumberToObject(data, "type", msg_type);
    cJSON_AddStringToObject(data, "channel", channel == 'A' ? "A" : "B");

    if (msg_type >= 1 && msg_type <= 3) {
        /* ── Position Report (Class A) ────────────────────────
         * Bits: type(6) repeat(2) mmsi(30) nav_status(4) rot(8)
         *       sog(10) accuracy(1) lon(28) lat(27) cog(12)
         *       heading(9) second(6) maneuver(2) spare(3) raim(1) radio(19)
         * Total: 168 bits */
        if (payload_bits < 168) goto publish;

        int nav_status = (int)ais_get_uint(payload, 38, 4);
        int rot_raw    = ais_get_int(payload, 42, 8);
        int sog_raw    = (int)ais_get_uint(payload, 50, 10);
        int accuracy   = (int)ais_get_uint(payload, 60, 1);
        int32_t lon_raw = ais_get_int(payload, 61, 28);
        int32_t lat_raw = ais_get_int(payload, 89, 27);
        int cog_raw    = (int)ais_get_uint(payload, 116, 12);
        int heading    = (int)ais_get_uint(payload, 128, 9);
        int second     = (int)ais_get_uint(payload, 137, 6);

        double lat_deg = lat_raw / 600000.0;
        double lon_deg = lon_raw / 600000.0;

        /* Validate: AIS uses 0x3412140 (91 deg) for lat N/A,
         *           0x6791AC0 (181 deg) for lon N/A */
        if (lat_raw != 0x3412140 && lon_raw != 0x6791AC0) {
            cJSON_AddNumberToObject(data, "lat", lat_deg);
            cJSON_AddNumberToObject(data, "lon", lon_deg);
        }
        if (sog_raw != 1023) {  /* 1023 = N/A */
            cJSON_AddNumberToObject(data, "sog", sog_raw / 10.0);
        }
        if (cog_raw != 3600) {  /* 3600 = N/A */
            cJSON_AddNumberToObject(data, "cog", cog_raw / 10.0);
        }
        if (heading != 511) {   /* 511 = N/A */
            cJSON_AddNumberToObject(data, "heading", heading);
        }
        cJSON_AddNumberToObject(data, "nav_status", nav_status);
        cJSON_AddNumberToObject(data, "rot", rot_raw);
        cJSON_AddNumberToObject(data, "accuracy", accuracy);
        cJSON_AddNumberToObject(data, "second", second);
    }
    else if (msg_type == 4) {
        /* ── Base Station Report ──────────────────────────────
         * Contains UTC time and position of a base station.
         * Bits: type(6) repeat(2) mmsi(30) year(14) month(4) day(5)
         *       hour(5) minute(6) second(6) accuracy(1) lon(28) lat(27)
         *       ... Total: 168 bits */
        if (payload_bits < 168) goto publish;

        int year   = (int)ais_get_uint(payload, 38, 14);
        int month  = (int)ais_get_uint(payload, 52, 4);
        int day    = (int)ais_get_uint(payload, 56, 5);
        int hour   = (int)ais_get_uint(payload, 61, 5);
        int minute = (int)ais_get_uint(payload, 66, 6);
        int second = (int)ais_get_uint(payload, 72, 6);
        int32_t lon_raw = ais_get_int(payload, 79, 28);
        int32_t lat_raw = ais_get_int(payload, 107, 27);

        char utc[24];
        snprintf(utc, sizeof(utc), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 year, month, day, hour, minute, second);
        cJSON_AddStringToObject(data, "utc", utc);

        if (lat_raw != 0x3412140 && lon_raw != 0x6791AC0) {
            cJSON_AddNumberToObject(data, "lat", lat_raw / 600000.0);
            cJSON_AddNumberToObject(data, "lon", lon_raw / 600000.0);
        }
    }
    else if (msg_type == 5) {
        /* ── Static and Voyage Data ───────────────────────────
         * Bits: type(6) repeat(2) mmsi(30) ais_version(2) imo(30)
         *       callsign(42) shipname(120) ship_type(8) dim_bow(9)
         *       dim_stern(9) dim_port(6) dim_starboard(6) epfd(4)
         *       eta_month(4) eta_day(5) eta_hour(5) eta_minute(6)
         *       draught(8) destination(120) dte(1) spare(1)
         * Total: 424 bits (usually arrives as 2-slot message) */
        if (payload_bits < 424) goto publish;

        uint32_t imo = ais_get_uint(payload, 40, 30);
        char callsign[8], shipname[21], destination[21];
        ais_get_string(payload, 70, 7, callsign);
        ais_get_string(payload, 112, 20, shipname);
        ais_get_string(payload, 302, 20, destination);
        int ship_type = (int)ais_get_uint(payload, 232, 8);
        int dim_bow   = (int)ais_get_uint(payload, 240, 9);
        int dim_stern = (int)ais_get_uint(payload, 249, 9);
        int dim_port  = (int)ais_get_uint(payload, 258, 6);
        int dim_stbd  = (int)ais_get_uint(payload, 264, 6);
        int draught   = (int)ais_get_uint(payload, 294, 8);
        int eta_month = (int)ais_get_uint(payload, 274, 4);
        int eta_day   = (int)ais_get_uint(payload, 278, 5);
        int eta_hour  = (int)ais_get_uint(payload, 283, 5);
        int eta_min   = (int)ais_get_uint(payload, 288, 6);

        cJSON_AddNumberToObject(data, "imo", imo);
        cJSON_AddStringToObject(data, "callsign", callsign);
        cJSON_AddStringToObject(data, "shipname", shipname);
        cJSON_AddStringToObject(data, "destination", destination);
        cJSON_AddNumberToObject(data, "ship_type", ship_type);
        cJSON_AddNumberToObject(data, "draught", draught / 10.0);
        cJSON_AddNumberToObject(data, "length", dim_bow + dim_stern);
        cJSON_AddNumberToObject(data, "beam", dim_port + dim_stbd);

        char eta[20];
        snprintf(eta, sizeof(eta), "%02d-%02dT%02d:%02dZ",
                 eta_month, eta_day, eta_hour, eta_min);
        cJSON_AddStringToObject(data, "eta", eta);
    }
    else if (msg_type == 18) {
        /* ── Standard Class B Position Report ─────────────────
         * Bits: type(6) repeat(2) mmsi(30) reserved(8) sog(10)
         *       accuracy(1) lon(28) lat(27) cog(12) heading(9)
         *       second(6) ... Total: 168 bits */
        if (payload_bits < 168) goto publish;

        int sog_raw    = (int)ais_get_uint(payload, 46, 10);
        int accuracy   = (int)ais_get_uint(payload, 56, 1);
        int32_t lon_raw = ais_get_int(payload, 57, 28);
        int32_t lat_raw = ais_get_int(payload, 85, 27);
        int cog_raw    = (int)ais_get_uint(payload, 112, 12);
        int heading    = (int)ais_get_uint(payload, 124, 9);

        if (lat_raw != 0x3412140 && lon_raw != 0x6791AC0) {
            cJSON_AddNumberToObject(data, "lat", lat_raw / 600000.0);
            cJSON_AddNumberToObject(data, "lon", lon_raw / 600000.0);
        }
        if (sog_raw != 1023) {
            cJSON_AddNumberToObject(data, "sog", sog_raw / 10.0);
        }
        if (cog_raw != 3600) {
            cJSON_AddNumberToObject(data, "cog", cog_raw / 10.0);
        }
        if (heading != 511) {
            cJSON_AddNumberToObject(data, "heading", heading);
        }
        cJSON_AddNumberToObject(data, "accuracy", accuracy);
    }
    else if (msg_type == 19) {
        /* ── Extended Class B Position Report ─────────────────
         * Same position fields as type 18, plus ship name and type.
         * Bits: ... sog(10) accuracy(1) lon(28) lat(27) cog(12)
         *       heading(9) second(6) reserved(4) shipname(120)
         *       ship_type(8) ... Total: 312 bits */
        if (payload_bits < 312) goto publish;

        int sog_raw    = (int)ais_get_uint(payload, 46, 10);
        int32_t lon_raw = ais_get_int(payload, 57, 28);
        int32_t lat_raw = ais_get_int(payload, 85, 27);
        int cog_raw    = (int)ais_get_uint(payload, 112, 12);
        int heading    = (int)ais_get_uint(payload, 124, 9);

        char shipname[21];
        ais_get_string(payload, 143, 20, shipname);
        int ship_type = (int)ais_get_uint(payload, 263, 8);

        if (lat_raw != 0x3412140 && lon_raw != 0x6791AC0) {
            cJSON_AddNumberToObject(data, "lat", lat_raw / 600000.0);
            cJSON_AddNumberToObject(data, "lon", lon_raw / 600000.0);
        }
        if (sog_raw != 1023) {
            cJSON_AddNumberToObject(data, "sog", sog_raw / 10.0);
        }
        if (cog_raw != 3600) {
            cJSON_AddNumberToObject(data, "cog", cog_raw / 10.0);
        }
        if (heading != 511) {
            cJSON_AddNumberToObject(data, "heading", heading);
        }
        cJSON_AddStringToObject(data, "shipname", shipname);
        cJSON_AddNumberToObject(data, "ship_type", ship_type);
    }
    else if (msg_type == 21) {
        /* ── Aid-to-Navigation Report ─────────────────────────
         * Bits: type(6) repeat(2) mmsi(30) aid_type(5) name(120)
         *       accuracy(1) lon(28) lat(27) dim(30) epfd(4) second(6)
         *       ... Total: 272+ bits */
        if (payload_bits < 272) goto publish;

        int aid_type   = (int)ais_get_uint(payload, 38, 5);
        char name[21];
        ais_get_string(payload, 43, 20, name);
        int32_t lon_raw = ais_get_int(payload, 164, 28);
        int32_t lat_raw = ais_get_int(payload, 192, 27);

        cJSON_AddNumberToObject(data, "aid_type", aid_type);
        cJSON_AddStringToObject(data, "name", name);
        if (lat_raw != 0x3412140 && lon_raw != 0x6791AC0) {
            cJSON_AddNumberToObject(data, "lat", lat_raw / 600000.0);
            cJSON_AddNumberToObject(data, "lon", lon_raw / 600000.0);
        }
    }
    else if (msg_type == 24) {
        /* ── Class B CS Static Data ───────────────────────────
         * Part A (part_num=0): shipname(120) — 168 bits
         * Part B (part_num=1): ship_type, callsign, dimensions — 168 bits */
        if (payload_bits < 168) goto publish;

        int part_num = (int)ais_get_uint(payload, 38, 2);
        cJSON_AddNumberToObject(data, "part_num", part_num);

        if (part_num == 0) {
            char shipname[21];
            ais_get_string(payload, 40, 20, shipname);
            cJSON_AddStringToObject(data, "shipname", shipname);
        } else if (part_num == 1) {
            int ship_type = (int)ais_get_uint(payload, 40, 8);
            char callsign[8];
            ais_get_string(payload, 90, 7, callsign);
            int dim_bow   = (int)ais_get_uint(payload, 132, 9);
            int dim_stern = (int)ais_get_uint(payload, 141, 9);
            int dim_port  = (int)ais_get_uint(payload, 150, 6);
            int dim_stbd  = (int)ais_get_uint(payload, 156, 6);

            cJSON_AddNumberToObject(data, "ship_type", ship_type);
            cJSON_AddStringToObject(data, "callsign", callsign);
            cJSON_AddNumberToObject(data, "length", dim_bow + dim_stern);
            cJSON_AddNumberToObject(data, "beam", dim_port + dim_stbd);
        }
    }

publish:
    /* Update counters */
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    ctx->frames_decoded++;
    ctx->vessel_count++;  /* Approximate; tracking table handles uniqueness */
    xSemaphoreGive(ctx->mutex);

    /* Duplicate data for tracking before publish (bus frees event.data) */
    cJSON *track_data = cJSON_Duplicate(data, true);

    /* Publish event on decode bus */
    int64_t now = (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    decode_event_t event = {
        .decoder_name = "ais",
        .event_type   = "vessel",
        .timestamp_ms = now,
        .rssi_db      = 0,
        .freq_hz      = (channel == 'A') ? AIS_CH_A_FREQ_HZ : AIS_CH_B_FREQ_HZ,
        .data         = data,
    };
    decode_bus_publish(&event);
    /* data is now freed by the bus — do not use */

    /* Upsert into tracking table */
    tracking_table_t *tt = decoder_get_global_tracking();
    if (tt && track_data) {
        tracking_table_upsert(tt, "ais", mmsi_str, track_data, 0);
    }
    if (track_data) {
        cJSON_Delete(track_data);
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  Channel initialization helper
 * ═══════════════════════════════════════════════════════════════ */

static void ais_channel_init(ais_channel_t *ch, double samples_per_bit) {
    memset(ch, 0, sizeof(*ch));
    ch->pll_freq    = samples_per_bit;
    ch->prev_sign   = 0;
    ch->prev_nrzi_bit = 0;
    ch->in_frame    = false;
}

/* ═══════════════════════════════════════════════════════════════
 *  Main IQ processing: dual-channel DDC -> FM demod -> clock
 *  recovery -> HDLC -> frame decode
 *
 *  For each raw IQ sample from RTL-SDR:
 *    1. Mix with NCO at channel offset (+-25 kHz) to baseband
 *    2. FM discriminate (cross-product)
 *    3. PLL clock recovery -> bit output
 *    4. NRZI decode + HDLC deframing
 * ═══════════════════════════════════════════════════════════════ */

static void ais_process_iq(void *ctx_ptr, const uint8_t *iq, uint32_t len) {
    ais_ctx_t *ctx = (ais_ctx_t *)ctx_ptr;
    if (!ctx->running) return;

    uint32_t num_samples = len / 2;
    double sample_rate = ctx->sample_rate;
    double samples_per_bit = sample_rate / (double)AIS_BAUD;

    /* NCO phase increments for each channel offset */
    double phase_inc_a = 2.0 * M_PI * AIS_CH_A_OFFSET / sample_rate;
    double phase_inc_b = 2.0 * M_PI * AIS_CH_B_OFFSET / sample_rate;

    for (uint32_t i = 0; i < num_samples; i++) {
        /* Convert unsigned 8-bit IQ to centered doubles */
        double in_i = (double)iq[i * 2]     - 128.0;
        double in_q = (double)iq[i * 2 + 1] - 128.0;

        /* ── Channel A DDC: mix down by -25 kHz ────────────── */
        double cos_a = cos(ctx->ch_a.nco_phase);
        double sin_a = sin(ctx->ch_a.nco_phase);
        double a_i =  in_i * cos_a + in_q * sin_a;
        double a_q = -in_i * sin_a + in_q * cos_a;
        ctx->ch_a.nco_phase += phase_inc_a;
        if (ctx->ch_a.nco_phase >  M_PI) ctx->ch_a.nco_phase -= 2.0 * M_PI;
        if (ctx->ch_a.nco_phase < -M_PI) ctx->ch_a.nco_phase += 2.0 * M_PI;

        /* ── Channel B DDC: mix down by +25 kHz ────────────── */
        double cos_b = cos(ctx->ch_b.nco_phase);
        double sin_b = sin(ctx->ch_b.nco_phase);
        double b_i =  in_i * cos_b + in_q * sin_b;
        double b_q = -in_i * sin_b + in_q * cos_b;
        ctx->ch_b.nco_phase += phase_inc_b;
        if (ctx->ch_b.nco_phase >  M_PI) ctx->ch_b.nco_phase -= 2.0 * M_PI;
        if (ctx->ch_b.nco_phase < -M_PI) ctx->ch_b.nco_phase += 2.0 * M_PI;

        /* ── FM demodulate both channels ───────────────────── */
        double audio_a = fm_demod(a_i, a_q,
                                   &ctx->ch_a.prev_i, &ctx->ch_a.prev_q);
        double audio_b = fm_demod(b_i, b_q,
                                   &ctx->ch_b.prev_i, &ctx->ch_b.prev_q);

        /* ── Clock recovery + HDLC for channel A ──────────── */
        int bit_a = clock_recover_sample(audio_a,
                        &ctx->ch_a.pll_phase, &ctx->ch_a.pll_freq,
                        samples_per_bit, &ctx->ch_a.prev_sign);
        if (bit_a >= 0) {
            hdlc_process_bit(ctx, &ctx->ch_a, bit_a, 'A');
        }

        /* ── Clock recovery + HDLC for channel B ──────────── */
        int bit_b = clock_recover_sample(audio_b,
                        &ctx->ch_b.pll_phase, &ctx->ch_b.pll_freq,
                        samples_per_bit, &ctx->ch_b.prev_sign);
        if (bit_b >= 0) {
            hdlc_process_bit(ctx, &ctx->ch_b, bit_b, 'B');
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  Audio processing fallback
 *
 *  If the framework routes FM-demodulated audio instead of raw IQ
 *  (single-channel mode), we can still clock-recover and decode.
 *  The audio is already frequency-discriminated, so skip DDC/FM.
 * ═══════════════════════════════════════════════════════════════ */

static void ais_process_audio(void *ctx_ptr, const int16_t *samples,
                               uint32_t count, uint32_t sample_rate) {
    ais_ctx_t *ctx = (ais_ctx_t *)ctx_ptr;
    if (!ctx->running) return;

    double samples_per_bit = (double)sample_rate / (double)AIS_BAUD;

    /* Re-initialize PLL freq if sample rate changed */
    if (ctx->ch_a.pll_freq < 1.0) {
        ctx->ch_a.pll_freq = samples_per_bit;
    }

    for (uint32_t i = 0; i < count; i++) {
        double s = (double)samples[i] / 32768.0;

        int bit = clock_recover_sample(s,
                      &ctx->ch_a.pll_phase, &ctx->ch_a.pll_freq,
                      samples_per_bit, &ctx->ch_a.prev_sign);
        if (bit >= 0) {
            hdlc_process_bit(ctx, &ctx->ch_a, bit, 'A');
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  Lifecycle callbacks
 * ═══════════════════════════════════════════════════════════════ */

static esp_err_t ais_init(void *ctx_ptr) {
    ais_ctx_t *ctx = (ais_ctx_t *)ctx_ptr;
    ctx->mutex = xSemaphoreCreateMutex();
    ctx->running = false;
    ctx->vessel_count = 0;
    ctx->frames_decoded = 0;
    ctx->frames_crc_fail = 0;
    ctx->sample_rate = AIS_SAMPLE_RATE;
    ctx->center_freq = (double)AIS_CENTER_FREQ_HZ;

    double spb = ctx->sample_rate / (double)AIS_BAUD;
    ais_channel_init(&ctx->ch_a, spb);
    ais_channel_init(&ctx->ch_b, spb);

    ESP_LOGI(TAG, "AIS decoder initialized (%.0f Hz, %.1f samp/bit, dual-channel)",
             ctx->sample_rate, spb);
    return ESP_OK;
}

static esp_err_t ais_start(void *ctx_ptr) {
    ais_ctx_t *ctx = (ais_ctx_t *)ctx_ptr;
    ctx->running = true;

    /* Reset channel state on start for clean lock */
    double spb = ctx->sample_rate / (double)AIS_BAUD;
    ais_channel_init(&ctx->ch_a, spb);
    ais_channel_init(&ctx->ch_b, spb);

    ESP_LOGI(TAG, "AIS decoder started (ch A: %d Hz, ch B: %d Hz, GMSK 9600 baud)",
             AIS_CH_A_FREQ_HZ, AIS_CH_B_FREQ_HZ);
    return ESP_OK;
}

static esp_err_t ais_stop(void *ctx_ptr) {
    ais_ctx_t *ctx = (ais_ctx_t *)ctx_ptr;
    ctx->running = false;
    ESP_LOGI(TAG, "AIS decoder stopped (decoded: %" PRIu32 ", CRC fail: %" PRIu32 ")",
             ctx->frames_decoded, ctx->frames_crc_fail);
    return ESP_OK;
}

static void ais_destroy(void *ctx_ptr) {
    (void)ctx_ptr;
}

/* ═══════════════════════════════════════════════════════════════
 *  Status and results (REST API)
 * ═══════════════════════════════════════════════════════════════ */

static cJSON *ais_get_status(void *ctx_ptr) {
    ais_ctx_t *ctx = (ais_ctx_t *)ctx_ptr;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", ctx->running);
        cJSON_AddNumberToObject(j, "vessel_count", ctx->vessel_count);
        cJSON_AddNumberToObject(j, "frames_decoded", ctx->frames_decoded);
        cJSON_AddNumberToObject(j, "frames_crc_fail", ctx->frames_crc_fail);
        cJSON_AddNumberToObject(j, "sample_rate", ctx->sample_rate);
    }
    return j;
}

static cJSON *ais_get_results(void *ctx_ptr) {
    (void)ctx_ptr;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "ais") :
        cJSON_CreateArray();
}

/* ── AIVDM NMEA sentence builder ──────────────────────── */
/* Creates standard !AIVDM sentence from raw AIS payload for forwarding */
static int ais_build_nmea(const uint8_t *payload, int payload_bits, char channel,
                           char *nmea_out, int max_len) {
    /* Convert binary payload to 6-bit ASCII armor */
    int payload_chars = (payload_bits + 5) / 6;
    char armored[64];
    for (int i = 0; i < payload_chars && i < 63; i++) {
        int val = 0;
        int start_bit = i * 6;
        for (int b = 0; b < 6 && (start_bit + b) < payload_bits; b++) {
            int byte_idx = (start_bit + b) / 8;
            int bit_idx = 7 - ((start_bit + b) % 8);
            val = (val << 1) | ((payload[byte_idx] >> bit_idx) & 1);
        }
        /* 6-bit to ASCII armor: add 48, if > 87 add 8 more */
        val += 48;
        if (val > 87) val += 8;
        armored[i] = (char)val;
    }
    armored[payload_chars] = '\0';

    int fill_bits = (payload_chars * 6) - payload_bits;

    /* Build sentence without checksum */
    char body[128];
    int body_len = snprintf(body, sizeof(body), "!AIVDM,1,1,,%c,%s,%d",
                             channel, armored, fill_bits);

    /* Compute XOR checksum (between ! and *) */
    uint8_t cksum = 0;
    for (int i = 1; i < body_len; i++) cksum ^= body[i];

    return snprintf(nmea_out, max_len, "%s*%02X\r\n", body, cksum);
}

/* ── NMEA TCP forwarding ──────────────────────────────── */
static int s_nmea_sock __attribute__((used)) = -1;

static void ais_nmea_forward(const uint8_t *payload, int payload_bits, char channel) {
    if (s_nmea_sock < 0) return;
    char nmea[128];
    int len = ais_build_nmea(payload, payload_bits, channel, nmea, sizeof(nmea));
    if (len > 0) send(s_nmea_sock, nmea, len, MSG_DONTWAIT);
}

/* ═══════════════════════════════════════════════════════════════
 *  Plugin registration
 * ═══════════════════════════════════════════════════════════════ */

static decoder_plugin_t s_ais_plugin = {
    .name            = "ais",
    .description     = "AIS Maritime Vessel Tracker (dual-channel GMSK/FSK)",
    .category        = "marine",
    .demod_type      = DEMOD_RAW_IQ,       /* Internal dual-channel DDC */
    .center_freq_hz  = AIS_CENTER_FREQ_HZ,
    .bandwidth_hz    = 100000,             /* +-50 kHz covers both channels */
    .audio_rate_hz   = 0,                  /* Raw IQ mode */
    .demod_params.fsk = { .shift_hz = 4800, .baud = 9600 },
    .init            = ais_init,
    .start           = ais_start,
    .stop            = ais_stop,
    .destroy         = ais_destroy,
    .process_iq      = ais_process_iq,
    .process_audio   = ais_process_audio,
    .get_status      = ais_get_status,
    .get_results     = ais_get_results,
    .ctx             = &s_ais_ctx,
};

void register_ais_decoder(void) {
    decoder_registry_add(&s_ais_plugin);
}
