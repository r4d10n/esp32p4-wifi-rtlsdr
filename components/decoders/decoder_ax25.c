#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_ax25";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════
 *  AFSK correlator demodulator constants
 * ═══════════════════════════════════════════════════════════════ */

#define AX25_MAX_SAMPLE_RATE    48000
#define AX25_MAX_CORR_LEN       (AX25_MAX_SAMPLE_RATE / 300) /* Worst case: 300 baud */
#define AX25_MAX_FRAME_LEN      512
#define HDLC_FLAG               0x7E

/* ═══════════════════════════════════════════════════════════════
 *  AFSK correlator state
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    float *mark_i_buf;
    float *mark_q_buf;
    float *space_i_buf;
    float *space_q_buf;
    float mark_i_sum, mark_q_sum;
    float space_i_sum, space_q_sum;
    int buf_len;
    int buf_idx;
    float mark_phase, space_phase;
    float mark_phase_inc, space_phase_inc;
    bool initialized;
} afsk_demod_state_t;

/* ═══════════════════════════════════════════════════════════════
 *  HDLC / bit-level state
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    uint8_t shift_reg;          /* Last 8 bits for flag detection */
    int ones_count;             /* Consecutive 1s for unstuffing */
    uint8_t frame_buf[AX25_MAX_FRAME_LEN];
    int frame_bits;             /* Bits accumulated in current byte */
    int frame_bytes;            /* Complete bytes in frame_buf */
    bool in_frame;
    int prev_level;             /* NRZI: previous demod level */
} hdlc_state_t;

/* ═══════════════════════════════════════════════════════════════
 *  Clock recovery (bit sync)
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    float phase;                /* 0.0 to 1.0 within current bit */
    float phase_inc;            /* 1.0 / samples_per_bit */
    float prev_demod;           /* Previous demod output for zero-cross */
} bit_sync_t;

/* ═══════════════════════════════════════════════════════════════
 *  Decoder context (expanded from stub)
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    uint32_t baud_rate;
    int packet_count;

    /* DSP state */
    afsk_demod_state_t afsk;
    hdlc_state_t hdlc;
    bit_sync_t sync;
    uint32_t configured_rate;   /* Sample rate we initialized for */
    uint16_t mark_hz;
    uint16_t space_hz;
} ax25_ctx_t;

static ax25_ctx_t s_ax25_300_ctx  = { .baud_rate = 300,  .mark_hz = 1270, .space_hz = 1070 };
static ax25_ctx_t s_ax25_1200_ctx = { .baud_rate = 1200, .mark_hz = 1200, .space_hz = 2200 };
static ax25_ctx_t s_ax25_9600_ctx = { .baud_rate = 9600 };

/* ═══════════════════════════════════════════════════════════════
 *  CRC-16 CCITT (used for AX.25 FCS)
 * ═══════════════════════════════════════════════════════════════ */

static uint16_t crc_ccitt_update(uint16_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0x8408;
        else
            crc >>= 1;
    }
    return crc;
}

static bool ax25_check_fcs(const uint8_t *frame, int len) {
    if (len < 4) return false; /* Minimum: 2 addresses (14) + control + FCS(2), but be lenient */
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len - 2; i++) {
        crc = crc_ccitt_update(crc, frame[i]);
    }
    crc ^= 0xFFFF;
    uint16_t fcs = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    return crc == fcs;
}

/* ═══════════════════════════════════════════════════════════════
 *  AX.25 address parsing
 * ═══════════════════════════════════════════════════════════════ */

static void parse_ax25_address(const uint8_t *data, char *callsign, int *ssid) {
    for (int i = 0; i < 6; i++) {
        callsign[i] = (data[i] >> 1) & 0x7F;
        if (callsign[i] == ' ') {
            callsign[i] = '\0';
            break;
        }
    }
    callsign[6] = '\0';
    /* Trim trailing spaces */
    for (int i = 5; i >= 0; i--) {
        if (callsign[i] == ' ' || callsign[i] == '\0')
            callsign[i] = '\0';
        else
            break;
    }
    *ssid = (data[6] >> 1) & 0x0F;
}

static void format_callsign_ssid(const uint8_t *data, char *out, size_t out_len) {
    char call[7];
    int ssid;
    parse_ax25_address(data, call, &ssid);
    if (ssid > 0)
        snprintf(out, out_len, "%s-%d", call, ssid);
    else
        snprintf(out, out_len, "%s", call);
}

/* ═══════════════════════════════════════════════════════════════
 *  APRS detection and position parsing
 * ═══════════════════════════════════════════════════════════════ */

static bool is_aprs_data_type(char c) {
    return c == '!' || c == '/' || c == '@' || c == '=' ||
           c == '>' || c == '<' || c == '?' || c == ';' ||
           c == ')' || c == 'T' || c == '#' || c == '*' || c == '_';
}

/* Parse APRS position from formats like:
 *   !DDMM.mmN/DDDMM.mmW
 *   /HHMMSSzDDMM.mmN/DDDMM.mmW
 *   @HHMMSSzDDMM.mmN/DDDMM.mmW
 * Returns true if position was parsed successfully. */
static bool parse_aprs_position(const char *info, double *lat, double *lon) {
    const char *p = info;
    /* Skip data type char and optional timestamp */
    if (!p || !*p) return false;
    char dtype = *p++;

    /* Skip timestamp for / and @ types: HHMMSSz or DDHHMMz (7 chars) */
    if ((dtype == '/' || dtype == '@') && strlen(p) >= 7) {
        p += 7;
    }

    /* Expect: DDMM.mmN/DDDMM.mmW (or similar with symbol table char) */
    if (strlen(p) < 18) return false;

    /* Latitude: DDMM.mm[NS] */
    if (p[4] != '.') return false;
    int lat_deg = (p[0] - '0') * 10 + (p[1] - '0');
    float lat_min = (float)atof(p + 2);
    char lat_ns = p[7];
    if (lat_ns != 'N' && lat_ns != 'S') return false;

    /* Symbol table char at position 8 */

    /* Longitude: DDDMM.mm[EW] */
    const char *lp = p + 9;
    if (lp[5] != '.') return false;
    int lon_deg = (lp[0] - '0') * 100 + (lp[1] - '0') * 10 + (lp[2] - '0');
    float lon_min = (float)atof(lp + 3);
    char lon_ew = lp[8];
    if (lon_ew != 'E' && lon_ew != 'W') return false;

    *lat = (double)lat_deg + (double)lat_min / 60.0;
    if (lat_ns == 'S') *lat = -*lat;

    *lon = (double)lon_deg + (double)lon_min / 60.0;
    if (lon_ew == 'W') *lon = -*lon;

    return true;
}

/* ═══════════════════════════════════════════════════════════════
 *  Frame handling: parse AX.25 and publish event
 * ═══════════════════════════════════════════════════════════════ */

static void ax25_handle_frame(ax25_ctx_t *c, const uint8_t *frame, int len) {
    /* Minimum AX.25 frame: dst(7) + src(7) + control(1) + FCS(2) = 17 */
    if (len < 17) return;

    if (!ax25_check_fcs(frame, len)) {
        ESP_LOGD(TAG, "FCS check failed (len=%d)", len);
        return;
    }

    /* Strip FCS for parsing */
    int payload_len = len - 2;

    /* Parse destination and source */
    char dst[16], src[16];
    format_callsign_ssid(frame, dst, sizeof(dst));
    format_callsign_ssid(frame + 7, src, sizeof(src));

    /* Parse digipeater path */
    char path[128] = {0};
    int path_pos = 0;
    int addr_idx = 14; /* After dst + src */
    while (addr_idx + 7 <= payload_len && !(frame[addr_idx - 1] & 0x01)) {
        /* Extension bit in previous address byte: 0 = more addresses */
        char digi[16];
        format_callsign_ssid(frame + addr_idx, digi, sizeof(digi));
        /* Check H-bit (has-been-digipeated): bit 7 of SSID byte */
        bool h_bit = (frame[addr_idx + 6] & 0x80) != 0;
        if (path_pos > 0) {
            path_pos += snprintf(path + path_pos, sizeof(path) - path_pos, ",");
        }
        path_pos += snprintf(path + path_pos, sizeof(path) - path_pos, "%s%s",
                             digi, h_bit ? "*" : "");
        addr_idx += 7;
        if (addr_idx > 70) break; /* Safety: max 8 digipeaters */
    }

    /* Find start of info field: after all address bytes + control [+ PID] */
    int info_start = addr_idx;
    if (info_start >= payload_len) return;

    /* Control field */
    info_start++; /* Skip control byte */

    /* PID field (present in I-frames and UI-frames) */
    if (info_start < payload_len) {
        info_start++; /* Skip PID byte */
    }

    /* Extract info field as string */
    int info_len = payload_len - info_start;
    if (info_len < 0) info_len = 0;
    if (info_len > 256) info_len = 256;

    char info[257];
    if (info_len > 0) {
        memcpy(info, frame + info_start, info_len);
    }
    info[info_len] = '\0';

    /* Check for APRS */
    bool is_aprs = (info_len > 0 && is_aprs_data_type(info[0]));
    double lat = 0.0, lon = 0.0;
    bool has_position = false;

    if (is_aprs) {
        has_position = parse_aprs_position(info, &lat, &lon);
    }

    xSemaphoreTake(c->mutex, portMAX_DELAY);
    c->packet_count++;
    xSemaphoreGive(c->mutex);

    char decoder_name[16];
    snprintf(decoder_name, sizeof(decoder_name), "ax25_%lu", (unsigned long)c->baud_rate);

    ESP_LOGI(TAG, "[%s] %s>%s%s%s: %s",
             decoder_name, src, dst,
             path[0] ? "," : "", path, info);

    /* Build event JSON */
    cJSON *data = cJSON_CreateObject();
    if (!data) return;

    cJSON_AddStringToObject(data, "src", src);
    cJSON_AddStringToObject(data, "dst", dst);
    if (path[0]) cJSON_AddStringToObject(data, "path", path);
    cJSON_AddStringToObject(data, "info", info);
    cJSON_AddStringToObject(data, "key", src); /* For tracking table */
    cJSON_AddBoolToObject(data, "is_aprs", is_aprs);

    if (has_position) {
        cJSON_AddNumberToObject(data, "lat", lat);
        cJSON_AddNumberToObject(data, "lon", lon);
    }

    /* Publish event */
    decode_event_t ev = {
        .decoder_name = decoder_name,
        .event_type = "packet",
        .timestamp_ms = (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        .rssi_db = 0,
        .freq_hz = 0,
        .data = data,
    };
    decode_bus_publish(&ev);

    /* Upsert to tracking table */
    tracking_table_t *tt = decoder_get_global_tracking();
    if (tt) {
        /* Rebuild data for tracking (bus freed the previous one) */
        cJSON *track = cJSON_CreateObject();
        if (track) {
            cJSON_AddStringToObject(track, "src", src);
            cJSON_AddStringToObject(track, "dst", dst);
            if (path[0]) cJSON_AddStringToObject(track, "path", path);
            cJSON_AddStringToObject(track, "info", info);
            cJSON_AddBoolToObject(track, "is_aprs", is_aprs);
            if (has_position) {
                cJSON_AddNumberToObject(track, "lat", lat);
                cJSON_AddNumberToObject(track, "lon", lon);
            }
            tracking_table_upsert(tt, decoder_name, src, track, 0);
            cJSON_Delete(track);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  AFSK correlator initialization
 * ═══════════════════════════════════════════════════════════════ */

static void afsk_demod_init(afsk_demod_state_t *d, uint32_t sample_rate,
                             uint16_t mark_hz, uint16_t space_hz, uint32_t baud) {
    d->buf_len = sample_rate / baud;
    if (d->buf_len < 1) d->buf_len = 1;
    if (d->buf_len > AX25_MAX_CORR_LEN) d->buf_len = AX25_MAX_CORR_LEN;

    /* Allocate correlation buffers */
    size_t sz = d->buf_len * sizeof(float);
    d->mark_i_buf  = (float *)calloc(1, sz);
    d->mark_q_buf  = (float *)calloc(1, sz);
    d->space_i_buf = (float *)calloc(1, sz);
    d->space_q_buf = (float *)calloc(1, sz);

    if (!d->mark_i_buf || !d->mark_q_buf || !d->space_i_buf || !d->space_q_buf) {
        ESP_LOGE("afsk", "Failed to allocate correlation buffers (%d samples)", d->buf_len);
        d->initialized = false;
        return;
    }

    d->buf_idx = 0;
    d->mark_i_sum = d->mark_q_sum = 0.0f;
    d->space_i_sum = d->space_q_sum = 0.0f;
    d->mark_phase = 0.0f;
    d->space_phase = 0.0f;
    d->mark_phase_inc = 2.0f * (float)M_PI * (float)mark_hz / (float)sample_rate;
    d->space_phase_inc = 2.0f * (float)M_PI * (float)space_hz / (float)sample_rate;
    d->initialized = true;
}

static void afsk_demod_free(afsk_demod_state_t *d) {
    free(d->mark_i_buf);
    free(d->mark_q_buf);
    free(d->space_i_buf);
    free(d->space_q_buf);
    d->mark_i_buf = d->mark_q_buf = NULL;
    d->space_i_buf = d->space_q_buf = NULL;
    d->initialized = false;
}

/* Process one sample through the AFSK correlator.
 * Returns the soft decision: positive = mark, negative = space. */
static float afsk_demod_process(afsk_demod_state_t *d, float sample) {
    int idx = d->buf_idx;

    /* Compute new correlation values for this sample */
    float mi = sample * cosf(d->mark_phase);
    float mq = sample * sinf(d->mark_phase);
    float si = sample * cosf(d->space_phase);
    float sq = sample * sinf(d->space_phase);

    /* Subtract oldest value from running sum, add new */
    d->mark_i_sum  += mi - d->mark_i_buf[idx];
    d->mark_q_sum  += mq - d->mark_q_buf[idx];
    d->space_i_sum += si - d->space_i_buf[idx];
    d->space_q_sum += sq - d->space_q_buf[idx];

    /* Store in circular buffer */
    d->mark_i_buf[idx]  = mi;
    d->mark_q_buf[idx]  = mq;
    d->space_i_buf[idx] = si;
    d->space_q_buf[idx] = sq;

    d->buf_idx = (idx + 1) % d->buf_len;

    /* Advance phase accumulators */
    d->mark_phase += d->mark_phase_inc;
    if (d->mark_phase > 2.0f * (float)M_PI)
        d->mark_phase -= 2.0f * (float)M_PI;

    d->space_phase += d->space_phase_inc;
    if (d->space_phase > 2.0f * (float)M_PI)
        d->space_phase -= 2.0f * (float)M_PI;

    /* Magnitude squared (avoid sqrt for speed) */
    float mark_mag  = d->mark_i_sum * d->mark_i_sum + d->mark_q_sum * d->mark_q_sum;
    float space_mag = d->space_i_sum * d->space_i_sum + d->space_q_sum * d->space_q_sum;

    return mark_mag - space_mag;
}

/* ═══════════════════════════════════════════════════════════════
 *  HDLC bit processing (NRZI + unstuffing + framing)
 * ═══════════════════════════════════════════════════════════════ */

static void hdlc_init(hdlc_state_t *h) {
    memset(h, 0, sizeof(*h));
    h->prev_level = 0;
}

static void hdlc_process_bit(ax25_ctx_t *c, int raw_level) {
    hdlc_state_t *h = &c->hdlc;

    /* NRZI decode: same level = 1, level change = 0 */
    int data_bit = (raw_level == h->prev_level) ? 1 : 0;
    h->prev_level = raw_level;

    /* Shift into flag detection register */
    h->shift_reg = (h->shift_reg << 1) | (data_bit & 1);

    /* Check for HDLC flag (0x7E) */
    if (h->shift_reg == HDLC_FLAG) {
        if (h->in_frame && h->frame_bytes >= 17) {
            /* End of frame - process it */
            ax25_handle_frame(c, h->frame_buf, h->frame_bytes);
        }
        /* Start new frame */
        h->in_frame = true;
        h->frame_bits = 0;
        h->frame_bytes = 0;
        h->ones_count = 0;
        return;
    }

    if (!h->in_frame) return;

    /* Check for abort (7+ consecutive 1s) */
    if (h->ones_count >= 7) {
        h->in_frame = false;
        return;
    }

    /* Bit unstuffing: after 5 consecutive 1s, the next 0 is a stuff bit */
    if (data_bit == 1) {
        h->ones_count++;
    } else {
        if (h->ones_count == 5) {
            /* This is a stuffed 0 - discard it */
            h->ones_count = 0;
            return;
        }
        h->ones_count = 0;
    }

    /* Accumulate data bit (LSB first) */
    if (h->frame_bytes < AX25_MAX_FRAME_LEN) {
        if (h->frame_bits < 8) {
            h->frame_buf[h->frame_bytes] >>= 1;
            if (data_bit)
                h->frame_buf[h->frame_bytes] |= 0x80;
            h->frame_bits++;
            if (h->frame_bits == 8) {
                h->frame_bits = 0;
                h->frame_bytes++;
                if (h->frame_bytes < AX25_MAX_FRAME_LEN)
                    h->frame_buf[h->frame_bytes] = 0;
            }
        }
    } else {
        /* Frame too long, abort */
        h->in_frame = false;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  Lifecycle callbacks
 * ═══════════════════════════════════════════════════════════════ */

static esp_err_t ax25_init(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->packet_count = 0;
    c->configured_rate = 0;
    hdlc_init(&c->hdlc);
    memset(&c->afsk, 0, sizeof(c->afsk));
    memset(&c->sync, 0, sizeof(c->sync));
    ESP_LOGI(TAG, "AX.25 %lu baud decoder initialized", (unsigned long)c->baud_rate);
    return ESP_OK;
}

static esp_err_t ax25_start(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    c->running = true;
    return ESP_OK;
}

static esp_err_t ax25_stop(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    c->running = false;
    if (c->afsk.initialized) {
        afsk_demod_free(&c->afsk);
    }
    c->configured_rate = 0;
    return ESP_OK;
}

static void ax25_destroy(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    if (c->afsk.initialized) {
        afsk_demod_free(&c->afsk);
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  Audio processing: AFSK demod -> bit sync -> HDLC
 * ═══════════════════════════════════════════════════════════════ */

static void ax25_process_audio(void *ctx, const int16_t *samples,
                                uint32_t count, uint32_t sample_rate) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    if (!c->running) return;

    /* Only implement AFSK for 300 and 1200 baud modes */
    if (c->baud_rate != 300 && c->baud_rate != 1200) return;
    if (c->mark_hz == 0 || c->space_hz == 0) return;

    /* Lazy initialization of AFSK demod when sample rate is known */
    if (c->configured_rate != sample_rate) {
        if (c->afsk.initialized) {
            afsk_demod_free(&c->afsk);
        }
        afsk_demod_init(&c->afsk, sample_rate, c->mark_hz, c->space_hz, c->baud_rate);
        if (!c->afsk.initialized) return;

        /* Initialize bit sync */
        c->sync.phase = 0.0f;
        c->sync.phase_inc = (float)c->baud_rate / (float)sample_rate;
        c->sync.prev_demod = 0.0f;

        /* Reset HDLC state */
        hdlc_init(&c->hdlc);
        c->configured_rate = sample_rate;

        ESP_LOGI(TAG, "AFSK demod initialized: rate=%lu, mark=%u, space=%u, corr_len=%d",
                 (unsigned long)sample_rate, c->mark_hz, c->space_hz, c->afsk.buf_len);
    }

    for (uint32_t i = 0; i < count; i++) {
        /* Normalize sample to -1.0 .. +1.0 */
        float s = (float)samples[i] / 32768.0f;

        /* Run correlator */
        float demod = afsk_demod_process(&c->afsk, s);

        /* Clock recovery: detect zero crossings and sample at bit center */
        float prev = c->sync.prev_demod;
        c->sync.prev_demod = demod;

        /* Zero crossing detected - adjust phase toward 0.5 (mid-bit) */
        if ((prev < 0.0f && demod >= 0.0f) || (prev >= 0.0f && demod < 0.0f)) {
            /* Phase error: how far from the ideal mid-bit crossing point (0.0 or 1.0) */
            float phase_error;
            if (c->sync.phase < 0.5f)
                phase_error = c->sync.phase;       /* Early: push phase backward */
            else
                phase_error = c->sync.phase - 1.0f; /* Late: push phase forward */
            /* Adjust phase with moderate gain */
            c->sync.phase -= phase_error * 0.10f;
        }

        /* Advance phase */
        c->sync.phase += c->sync.phase_inc;

        /* Sample the bit when phase crosses 1.0 */
        if (c->sync.phase >= 1.0f) {
            c->sync.phase -= 1.0f;
            int level = (demod > 0.0f) ? 1 : 0;
            hdlc_process_bit(c, level);
        }
    }
}

/* Also accept IQ for decoders where DDC isn't ready yet */
static void ax25_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

static cJSON *ax25_get_status(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "baud_rate", c->baud_rate);
        cJSON_AddNumberToObject(j, "packet_count", c->packet_count);
    }
    return j;
}

static cJSON *ax25_get_results(void *ctx) {
    ax25_ctx_t *c = (ax25_ctx_t *)ctx;
    char name[16];
    snprintf(name, sizeof(name), "ax25_%lu", (unsigned long)c->baud_rate);
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), name) :
        cJSON_CreateArray();
}

/* Three plugin instances */
static decoder_plugin_t s_ax25_300 = {
    .name = "ax25_300",
    .description = "AX.25 300 baud HF Packet (Bell 103 AFSK)",
    .category = "packet",
    .demod_type = DEMOD_FM_NARROW,
    .center_freq_hz = 7101000,
    .bandwidth_hz = 3000,
    .audio_rate_hz = 11025,
    .demod_params.afsk = { .mark_hz = 1270, .space_hz = 1070 },
    .init = ax25_init, .start = ax25_start, .stop = ax25_stop, .destroy = ax25_destroy,
    .process_audio = ax25_process_audio, .process_iq = ax25_process_iq,
    .get_status = ax25_get_status, .get_results = ax25_get_results,
    .ctx = &s_ax25_300_ctx,
};

static decoder_plugin_t s_ax25_1200 = {
    .name = "ax25_1200",
    .description = "AX.25 1200 baud VHF Packet / APRS (Bell 202 AFSK)",
    .category = "packet",
    .demod_type = DEMOD_FM_NARROW,
    .center_freq_hz = 144390000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 22050,
    .demod_params.afsk = { .mark_hz = 1200, .space_hz = 2200 },
    .init = ax25_init, .start = ax25_start, .stop = ax25_stop, .destroy = ax25_destroy,
    .process_audio = ax25_process_audio, .process_iq = ax25_process_iq,
    .get_status = ax25_get_status, .get_results = ax25_get_results,
    .ctx = &s_ax25_1200_ctx,
};

static decoder_plugin_t s_ax25_9600 = {
    .name = "ax25_9600",
    .description = "AX.25 9600 baud VHF/UHF Packet (G3RUH FSK)",
    .category = "packet",
    .demod_type = DEMOD_FSK,
    .center_freq_hz = 144390000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 48000,
    .demod_params.fsk = { .shift_hz = 3000, .baud = 9600 },
    .init = ax25_init, .start = ax25_start, .stop = ax25_stop, .destroy = ax25_destroy,
    .process_audio = ax25_process_audio, .process_iq = ax25_process_iq,
    .get_status = ax25_get_status, .get_results = ax25_get_results,
    .ctx = &s_ax25_9600_ctx,
};

void register_ax25_decoders(void) {
    decoder_registry_add(&s_ax25_300);
    decoder_registry_add(&s_ax25_1200);
    decoder_registry_add(&s_ax25_9600);
}
