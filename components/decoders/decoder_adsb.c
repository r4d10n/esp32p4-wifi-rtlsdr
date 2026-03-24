#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <sys/socket.h>
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

/* ── NL (Number of Longitude zones) table for CPR decode ── */
/* From ICAO Annex 10, Vol IV. NL(lat) = number of longitude zones at latitude */
static int cpr_NL(double lat) {
    if (lat < 0) lat = -lat;
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    return 1;
}

static int cpr_N(double lat, int is_odd) {
    int nl = cpr_NL(lat) - is_odd;
    if (nl < 1) nl = 1;
    return nl;
}

static double cpr_Dlon(double lat, int is_odd) {
    return 360.0 / cpr_N(lat, is_odd);
}

static double cpr_mod(double a, double b) {
    double res = fmod(a, b);
    if (res < 0) res += b;
    return res;
}

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

        /* Global CPR decode (dump1090 algorithm with NL table) */
        if (a->cpr_even_time > 0 && a->cpr_odd_time > 0 &&
            labs((long)(a->cpr_even_time - a->cpr_odd_time)) < 10000) {

            double rlat_even = (double)a->cpr_even_lat / 131072.0;
            double rlat_odd = (double)a->cpr_odd_lat / 131072.0;

            /* Compute latitude index j */
            int j = (int)floor(59.0 * rlat_even - 60.0 * rlat_odd + 0.5);

            /* Compute latitudes for even and odd frames */
            double dlat0 = 360.0 / 60.0;
            double dlat1 = 360.0 / 59.0;
            double lat0 = dlat0 * (cpr_mod(j, 60) + rlat_even);
            double lat1 = dlat1 * (cpr_mod(j, 59) + rlat_odd);

            if (lat0 >= 270) lat0 -= 360;
            if (lat1 >= 270) lat1 -= 360;

            /* Check that both latitudes are in the same NL zone */
            if (cpr_NL(lat0) != cpr_NL(lat1)) {
                /* Ambiguity — cannot decode, wait for next pair */
                xSemaphoreGive(c->mutex);
                return;
            }

            /* Compute longitude */
            double rlon_even = (double)a->cpr_even_lon / 131072.0;
            double rlon_odd = (double)a->cpr_odd_lon / 131072.0;

            if (a->cpr_even_time > a->cpr_odd_time) {
                /* Use even frame (most recent) */
                a->lat = lat0;
                int ni = cpr_N(lat0, 0);
                int m = (int)floor(rlon_even * (cpr_NL(lat0) - 1) -
                                    rlon_odd * cpr_NL(lat0) + 0.5);
                a->lon = cpr_Dlon(lat0, 0) * (cpr_mod(m, ni) + rlon_even);
            } else {
                /* Use odd frame (most recent) */
                a->lat = lat1;
                int ni = cpr_N(lat1, 1);
                int m = (int)floor(rlon_even * (cpr_NL(lat1) - 1) -
                                    rlon_odd * cpr_NL(lat1) + 0.5);
                a->lon = cpr_Dlon(lat1, 1) * (cpr_mod(m, ni) + rlon_odd);
            }
            if (a->lon > 180) a->lon -= 360;
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

/* ── SBS-1 BaseStation format for FlightAware/FR24 ────── */
static int adsb_build_sbs1(const adsb_aircraft_t *a, int msg_type,
                            char *sbs_out, int max_len) {
    /* SBS-1 format: MSG,type,session,aircraft,hex,flight,date,time,date,time,callsign,alt,speed,track,lat,lon,vrate,squawk,alert,emergency,spi,ground */
    return snprintf(sbs_out, max_len,
        "MSG,%d,1,1,%06" PRIX32 ",1,,,,%s,%d,%d,%d,%.5f,%.5f,%d,,,,\r\n",
        msg_type, a->icao, a->callsign,
        (int)a->altitude_ft, (int)a->speed_kt, (int)a->heading,
        a->lat, a->lon, (int)a->vert_rate);
}

/* ── SBS-1 TCP forwarding ─────────────────────────────── */
static int s_sbs1_sock __attribute__((used)) = -1;

static void adsb_sbs1_forward(const adsb_aircraft_t *a, int msg_type) {
    if (s_sbs1_sock < 0) return;
    char sbs[256];
    int len = adsb_build_sbs1(a, msg_type, sbs, sizeof(sbs));
    if (len > 0) send(s_sbs1_sock, sbs, len, MSG_DONTWAIT);
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

        /* dump1090-style preamble check at 2 MSPS:
         * Preamble: 1 0 1 0 0 0 0 1 0 1 0 0 0 0 0 0 (at 1us resolution)
         * At 2 MSPS: each position is 2 samples wide
         * High pulses at: [0-1], [4-5], [14-15], [18-19]
         * Low gaps at: [2-3], [6-13], [16-17], [20-31]
         */
        if (i + MODES_FULL_LEN * 2 + 16 >= num_samples) break;

        /* Check the 4 high pulses */
        uint32_t high = mag[i] + mag[i+1] + mag[i+4] + mag[i+5] +
                        mag[i+14] + mag[i+15] + mag[i+18] + mag[i+19];

        /* Check the low gaps between pulses */
        uint32_t low = mag[i+2] + mag[i+3] + mag[i+6] + mag[i+7] +
                       mag[i+8] + mag[i+9] + mag[i+10] + mag[i+11] +
                       mag[i+12] + mag[i+13] + mag[i+16] + mag[i+17];

        /* High must be significantly above low (dump1090 uses ~2x ratio) */
        if (high < 100) continue;  /* Too weak */
        if (low * 2 > high) continue;  /* Not enough contrast */

        /* Potential preamble found — extract message bits */
        /* PPM encoding: bit period = 2 samples
         * Bit = 1 if first sample > second sample
         * Bit = 0 if first sample < second sample
         */
        uint8_t msg[14];  /* 112 bits = 14 bytes */
        memset(msg, 0, sizeof(msg));

        int base = i + 32;  /* Skip preamble (16 us * 2 samples/us = 32 samples) */

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

        /* CRC check with 1-bit error correction (dump1090 technique) */
        uint32_t crc = modes_checksum(msg, msg_bits);
        if (crc != 0) {
            /* Try 1-bit error correction for DF17 only */
            if (df == 17) {
                int fixed = 0;
                for (int bit = 0; bit < msg_bits && !fixed; bit++) {
                    /* Flip bit and recheck CRC */
                    msg[bit / 8] ^= (1 << (7 - (bit % 8)));
                    if (modes_checksum(msg, msg_bits) == 0) {
                        fixed = 1;
                        ESP_LOGD(TAG, "1-bit error corrected at bit %d", bit);
                    } else {
                        msg[bit / 8] ^= (1 << (7 - (bit % 8))); /* Flip back */
                    }
                }
                if (!fixed) continue;
            } else {
                continue;
            }
        }

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
