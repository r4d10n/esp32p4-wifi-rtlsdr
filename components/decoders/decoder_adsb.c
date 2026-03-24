#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "decoder_framework.h"

static const char *TAG = "dec_adsb";

#define MAX_AIRCRAFT 64
#define MODES_LONG_MSG_BITS  112
#define MODES_SHORT_MSG_BITS 56
#define MODES_PREAMBLE_US    8
#define MODES_FULL_LEN       (MODES_PREAMBLE_US + MODES_LONG_MSG_BITS)

/* ── Magnitude lookup table (256x256 = 64KB) ─────────── */
static uint16_t *s_mag_lut;  /* [i*256+q] = sqrt(i^2+q^2) * 258 */

static void build_mag_lut(void) {
    s_mag_lut = (uint16_t *)malloc(256 * 256 * sizeof(uint16_t));
    if (!s_mag_lut) return;
    for (int i = 0; i < 256; i++) {
        for (int q = 0; q < 256; q++) {
            double fi = (i - 128.0) / 128.0;
            double fq = (q - 128.0) / 128.0;
            s_mag_lut[i * 256 + q] = (uint16_t)(sqrt(fi * fi + fq * fq) * 258);
        }
    }
}

/* ── CRC-24 for Mode S ───────────────────────────────── */
static uint32_t modes_checksum(const uint8_t *msg, int bits) {
    uint32_t crc = 0;
    int bytes = bits / 8;
    int npoly = (bits == 112) ? bytes - 3 : bytes - 3;

    for (int i = 0; i < npoly; i++) {
        crc ^= ((uint32_t)msg[i]) << 16;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000)
                crc = (crc << 1) ^ 0xFFF409;  /* Generator polynomial */
            else
                crc = crc << 1;
        }
    }
    /* XOR with last 3 bytes (the transmitted CRC) */
    uint32_t rem = ((uint32_t)msg[npoly] << 16) |
                   ((uint32_t)msg[npoly + 1] << 8) |
                   (uint32_t)msg[npoly + 2];
    return (crc ^ rem) & 0xFFFFFF;
}

/* ── Aircraft tracking ────────────────────────────────── */
typedef struct {
    uint32_t icao;
    char callsign[9];
    double lat, lon;
    int32_t altitude_ft;
    int16_t heading;
    uint16_t speed_kt;
    int16_t vert_rate;
    uint16_t squawk;
    int64_t last_seen_ms;
    uint32_t msg_count;
    /* CPR position decoding state */
    int cpr_even_lat, cpr_even_lon;
    int cpr_odd_lat, cpr_odd_lon;
    int64_t cpr_even_time, cpr_odd_time;
} adsb_aircraft_t;

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    int aircraft_count;
    adsb_aircraft_t aircraft[MAX_AIRCRAFT];
    uint32_t messages_total;
    uint32_t messages_crc_ok;
} adsb_ctx_t;

static adsb_ctx_t s_adsb_ctx;

static adsb_aircraft_t *find_or_create_aircraft(adsb_ctx_t *c, uint32_t icao) {
    /* Find existing */
    for (int i = 0; i < c->aircraft_count; i++) {
        if (c->aircraft[i].icao == icao) return &c->aircraft[i];
    }
    /* Create new */
    if (c->aircraft_count >= MAX_AIRCRAFT) {
        /* Evict oldest */
        int oldest = 0;
        for (int i = 1; i < MAX_AIRCRAFT; i++) {
            if (c->aircraft[i].last_seen_ms < c->aircraft[oldest].last_seen_ms)
                oldest = i;
        }
        memset(&c->aircraft[oldest], 0, sizeof(adsb_aircraft_t));
        c->aircraft[oldest].icao = icao;
        return &c->aircraft[oldest];
    }
    adsb_aircraft_t *a = &c->aircraft[c->aircraft_count++];
    memset(a, 0, sizeof(adsb_aircraft_t));
    a->icao = icao;
    return a;
}

/* ── DF17 ADS-B message decoding ──────────────────────── */
static void decode_adsb_msg(adsb_ctx_t *c, const uint8_t *msg, int len) {
    if (len < 14) return;  /* Need at least 112 bits = 14 bytes */

    uint8_t df = (msg[0] >> 3) & 0x1F;
    if (df != 17 && df != 18) return;  /* Only DF17 (ADS-B) and DF18 */

    uint32_t icao = ((uint32_t)msg[1] << 16) | ((uint32_t)msg[2] << 8) | msg[3];
    uint8_t tc = (msg[4] >> 3) & 0x1F;  /* Type code */

    int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;

    xSemaphoreTake(c->mutex, portMAX_DELAY);
    adsb_aircraft_t *a = find_or_create_aircraft(c, icao);
    a->last_seen_ms = now;
    a->msg_count++;

    if (tc >= 1 && tc <= 4) {
        /* Aircraft identification */
        static const char charset[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";
        uint64_t payload = 0;
        for (int i = 5; i < 11; i++) payload = (payload << 8) | msg[i];
        for (int i = 0; i < 8; i++) {
            int idx = (payload >> (42 - i * 6)) & 0x3F;
            a->callsign[i] = charset[idx];
        }
        a->callsign[8] = '\0';
        /* Trim trailing spaces */
        for (int i = 7; i >= 0 && a->callsign[i] == ' '; i--) a->callsign[i] = '\0';
    }
    else if (tc >= 9 && tc <= 18) {
        /* Airborne position (Barometric altitude) */
        int alt_code = ((msg[5] & 0xFF) << 4) | ((msg[6] >> 4) & 0x0F);
        /* Remove Q-bit (bit 4) */
        int q_bit = (alt_code >> 4) & 1;
        if (q_bit) {
            int n = ((alt_code >> 5) << 4) | (alt_code & 0x0F);
            a->altitude_ft = n * 25 - 1000;
        }
        /* CPR latitude/longitude */
        int cpr_format = (msg[6] >> 2) & 1;  /* 0=even, 1=odd */
        int cpr_lat = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
        int cpr_lon = ((msg[8] & 1) << 16) | (msg[9] << 8) | msg[10];

        if (cpr_format == 0) {
            a->cpr_even_lat = cpr_lat;
            a->cpr_even_lon = cpr_lon;
            a->cpr_even_time = now;
        } else {
            a->cpr_odd_lat = cpr_lat;
            a->cpr_odd_lon = cpr_lon;
            a->cpr_odd_time = now;
        }

        /* Global CPR decode if both even and odd are recent (< 10s) */
        if (a->cpr_even_time > 0 && a->cpr_odd_time > 0 &&
            labs((long)(a->cpr_even_time - a->cpr_odd_time)) < 10000) {
            double lat_even = (double)a->cpr_even_lat / 131072.0;
            double lat_odd = (double)a->cpr_odd_lat / 131072.0;
            double lon_even = (double)a->cpr_even_lon / 131072.0;
            double lon_odd = (double)a->cpr_odd_lon / 131072.0;

            /* NL computation simplified */
            int j = (int)floor(59.0 * lat_even - 60.0 * lat_odd + 0.5);
            double lat0 = (360.0 / 60.0) * (j % 60 + lat_even);
            double lat1 = (360.0 / 59.0) * (j % 59 + lat_odd);
            if (lat0 >= 270) lat0 -= 360;
            if (lat1 >= 270) lat1 -= 360;

            /* Use most recent CPR frame */
            if (a->cpr_even_time > a->cpr_odd_time) {
                a->lat = lat0;
                /* Simplified longitude — full NL computation needed for accuracy */
                a->lon = (360.0 / 60.0) * (0 + lon_even);
                if (a->lon > 180) a->lon -= 360;
            } else {
                a->lat = lat1;
                a->lon = (360.0 / 59.0) * (0 + lon_odd);
                if (a->lon > 180) a->lon -= 360;
            }
        }
    }
    else if (tc == 19) {
        /* Airborne velocity */
        int subtype = msg[4] & 0x07;
        if (subtype == 1 || subtype == 2) {
            /* Ground speed */
            int ew_sign = (msg[5] >> 2) & 1;
            int ew_vel = ((msg[5] & 3) << 8) | msg[6];
            int ns_sign = (msg[7] >> 7) & 1;
            int ns_vel = ((msg[7] & 0x7F) << 3) | ((msg[8] >> 5) & 7);

            ew_vel -= 1; ns_vel -= 1;
            if (ew_sign) ew_vel = -ew_vel;
            if (ns_sign) ns_vel = -ns_vel;

            a->speed_kt = (uint16_t)sqrt(ew_vel * ew_vel + ns_vel * ns_vel);
            a->heading = (int16_t)(atan2(ew_vel, ns_vel) * 180.0 / M_PI);
            if (a->heading < 0) a->heading += 360;

            /* Vertical rate */
            int vr_sign = (msg[8] >> 3) & 1;
            int vr = ((msg[8] & 7) << 6) | ((msg[9] >> 2) & 0x3F);
            a->vert_rate = (vr - 1) * 64;
            if (vr_sign) a->vert_rate = -a->vert_rate;
        }
    }

    xSemaphoreGive(c->mutex);

    /* Publish event */
    cJSON *data = cJSON_CreateObject();
    if (data) {
        char icao_hex[8];
        snprintf(icao_hex, sizeof(icao_hex), "%06" PRIX32, icao);
        cJSON_AddStringToObject(data, "icao", icao_hex);
        cJSON_AddStringToObject(data, "callsign", a->callsign);
        cJSON_AddNumberToObject(data, "altitude_ft", a->altitude_ft);
        cJSON_AddNumberToObject(data, "speed_kt", a->speed_kt);
        cJSON_AddNumberToObject(data, "heading", a->heading);
        if (a->lat != 0 || a->lon != 0) {
            cJSON_AddNumberToObject(data, "lat", a->lat);
            cJSON_AddNumberToObject(data, "lon", a->lon);
        }

        decode_event_t event = {
            .decoder_name = "adsb",
            .event_type = "aircraft",
            .timestamp_ms = now,
            .freq_hz = 1090000000,
            .data = data,
        };
        decode_bus_publish(&event);

        tracking_table_upsert(decoder_get_global_tracking(), "adsb", icao_hex, data, 0);
    }
}

static esp_err_t adsb_init(void *ctx) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->aircraft_count = 0;
    c->messages_total = 0;
    c->messages_crc_ok = 0;
    build_mag_lut();
    ESP_LOGI(TAG, "ADS-B decoder initialized (mag LUT: %s)", s_mag_lut ? "OK" : "FAILED");
    return ESP_OK;
}

static esp_err_t adsb_start(void *ctx) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    c->running = true;
    ESP_LOGI(TAG, "ADS-B decoder started (1090 MHz, 2 MSPS)");
    return ESP_OK;
}

static esp_err_t adsb_stop(void *ctx) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    c->running = false;
    ESP_LOGI(TAG, "ADS-B decoder stopped");
    return ESP_OK;
}

static void adsb_destroy(void *ctx) { (void)ctx; }

static void adsb_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    if (!c->running || !s_mag_lut) return;

    /* At 2 MSPS, each Mode S bit = 2 samples (1us per sample)
     * Preamble: 8us = 16 samples
     * Long message: 112 bits = 224 samples
     * Total: 240 samples minimum per message
     */
    uint32_t num_samples = len / 2;
    if (num_samples < 240) return;

    /* Convert IQ to magnitude using lookup table */
    uint16_t *mag = (uint16_t *)malloc(num_samples * sizeof(uint16_t));
    if (!mag) return;

    for (uint32_t i = 0; i < num_samples; i++) {
        mag[i] = s_mag_lut[iq[i * 2] * 256 + iq[i * 2 + 1]];
    }

    /* Scan for Mode S preambles */
    for (uint32_t i = 0; i < num_samples - MODES_FULL_LEN * 2; i++) {
        c->messages_total++;

        /* Preamble pattern at 2 MSPS (2 samples per us):
         * Pulse positions (us): 0, 1, 3.5, 4.5
         * At 2x: samples 0-1 high, 2-3 low, 7-8 high, 9-10 low
         * Simplified check: peaks at 0,2,7,9 and valleys at 3-4,5-6,11-13
         */
        uint16_t p0 = mag[i];
        uint16_t p2 = mag[i + 2];
        uint16_t p7 = mag[i + 7];
        uint16_t p9 = mag[i + 9];

        /* Check high pulses */
        uint16_t high_avg = (p0 + p2 + p7 + p9) / 4;
        if (high_avg < 20) continue;  /* Too weak */

        /* Check low valleys */
        uint16_t v3 = mag[i + 3];
        uint16_t v4 = mag[i + 4];
        uint16_t v5 = mag[i + 5];
        uint16_t v6 = mag[i + 6];

        uint16_t low_avg = (v3 + v4 + v5 + v6) / 4;
        if (low_avg * 2 > high_avg) continue;  /* Not enough contrast */

        /* Potential preamble found — extract message bits */
        /* PPM encoding: bit period = 2 samples
         * Bit = 1 if first sample > second sample
         * Bit = 0 if first sample < second sample
         */
        uint8_t msg[14];  /* 112 bits = 14 bytes */
        memset(msg, 0, sizeof(msg));

        int base = i + 16;  /* Skip preamble (16 samples = 8us) */

        for (int bit = 0; bit < MODES_LONG_MSG_BITS; bit++) {
            int pos = base + bit * 2;
            if (pos + 1 >= (int)num_samples) break;

            if (mag[pos] > mag[pos + 1]) {
                msg[bit / 8] |= (1 << (7 - (bit % 8)));
            }
        }

        /* Determine message length from DF (downlink format) */
        uint8_t df = (msg[0] >> 3) & 0x1F;
        int msg_bits = (df >= 16) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS;
        int msg_bytes = msg_bits / 8;

        /* CRC check */
        uint32_t crc = modes_checksum(msg, msg_bits);
        if (crc != 0) continue;  /* CRC failed */

        c->messages_crc_ok++;

        /* Decode ADS-B message */
        decode_adsb_msg(c, msg, msg_bytes);

        /* Skip past this message to avoid re-detecting */
        i += 16 + msg_bits * 2;
    }

    free(mag);
}

static cJSON *adsb_get_status(void *ctx) {
    adsb_ctx_t *c = (adsb_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "aircraft_count", c->aircraft_count);
        cJSON_AddNumberToObject(j, "messages_total", c->messages_total);
        cJSON_AddNumberToObject(j, "messages_crc_ok", c->messages_crc_ok);
        cJSON_AddBoolToObject(j, "mag_lut_loaded", s_mag_lut != NULL);
    }
    return j;
}

static cJSON *adsb_get_results(void *ctx) {
    (void)ctx;
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), "adsb") :
        cJSON_CreateArray();
}

static decoder_plugin_t s_adsb_plugin = {
    .name = "adsb",
    .description = "ADS-B 1090 MHz Mode S Aircraft Tracker",
    .category = "aviation",
    .demod_type = DEMOD_RAW_IQ,
    .center_freq_hz = 1090000000,
    .bandwidth_hz = 2000000,
    .audio_rate_hz = 0,
    .init = adsb_init,
    .start = adsb_start,
    .stop = adsb_stop,
    .destroy = adsb_destroy,
    .process_iq = adsb_process_iq,
    .process_audio = NULL,
    .get_status = adsb_get_status,
    .get_results = adsb_get_results,
    .ctx = &s_adsb_ctx,
};

void register_adsb_decoder(void) {
    decoder_registry_add(&s_adsb_plugin);
}
