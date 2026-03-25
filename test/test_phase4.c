/*
 * Phase 4 Feature Tests
 *
 * Tests for: stereo blend, NVS settings, CI-V parser, BCD frequency, scan
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "civ_emu.h"
#include "nvs_settings.h"
#ifdef CONFIG_FM_STEREO_ENABLE
#include "fm_stereo.h"
#endif

static const char *TAG = "test_p4";
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { ESP_LOGE(TAG, "FAIL: %s (line %d)", msg, __LINE__); tests_failed++; } \
    else { ESP_LOGI(TAG, "PASS: %s", msg); tests_passed++; } \
} while(0)

/* ──────────────────────── BCD helpers (copied from civ_emu.c) ──────────────────────── */

/*
 * CI-V frequency: 5 bytes, little-endian BCD.
 * byte[0] = 10Hz+1Hz digits, byte[4] = 10GHz+1GHz
 */
static void test_freq_to_bcd(uint32_t freq_hz, uint8_t *bcd5)
{
    uint64_t f = (uint64_t)freq_hz / 10; /* work in 10 Hz units */
    for (int i = 0; i < 5; i++) {
        uint8_t lo = f % 10; f /= 10;
        uint8_t hi = f % 10; f /= 10;
        bcd5[i] = (hi << 4) | lo;
    }
}

static uint32_t test_bcd_to_freq(const uint8_t *bcd5)
{
    uint64_t freq = 0;
    uint64_t mul = 1;
    for (int i = 0; i < 5; i++) {
        freq += (bcd5[i] & 0x0F) * mul;        mul *= 10;
        freq += ((bcd5[i] >> 4) & 0x0F) * mul; mul *= 10;
    }
    return (uint32_t)(freq * 10);
}

/* ──────────────────────── Test: BCD frequency encode/decode ──────────────────────── */

static void test_civ_bcd_frequency(void)
{
    ESP_LOGI(TAG, "--- test_civ_bcd_frequency ---");

    uint8_t bcd[5];
    uint32_t decoded;

    /* 100.1 MHz = 100100000 Hz → {0x00, 0x00, 0x10, 0x00, 0x01} */
    test_freq_to_bcd(100100000, bcd);
    TEST_ASSERT(bcd[0] == 0x00, "100.1MHz BCD byte[0]");
    TEST_ASSERT(bcd[1] == 0x00, "100.1MHz BCD byte[1]");
    TEST_ASSERT(bcd[2] == 0x10, "100.1MHz BCD byte[2]");
    TEST_ASSERT(bcd[3] == 0x00, "100.1MHz BCD byte[3]");
    TEST_ASSERT(bcd[4] == 0x01, "100.1MHz BCD byte[4]");
    decoded = test_bcd_to_freq(bcd);
    TEST_ASSERT(decoded == 100100000, "100.1MHz BCD roundtrip");

    /* 145.8 MHz = 145800000 Hz → {0x00, 0x00, 0x80, 0x45, 0x01} */
    test_freq_to_bcd(145800000, bcd);
    TEST_ASSERT(bcd[0] == 0x00, "145.8MHz BCD byte[0]");
    TEST_ASSERT(bcd[1] == 0x00, "145.8MHz BCD byte[1]");
    TEST_ASSERT(bcd[2] == 0x80, "145.8MHz BCD byte[2]");
    TEST_ASSERT(bcd[3] == 0x45, "145.8MHz BCD byte[3]");
    TEST_ASSERT(bcd[4] == 0x01, "145.8MHz BCD byte[4]");
    decoded = test_bcd_to_freq(bcd);
    TEST_ASSERT(decoded == 145800000, "145.8MHz BCD roundtrip");

    /* 88.1 MHz = 88100000 Hz → {0x00, 0x00, 0x10, 0x88, 0x00} */
    test_freq_to_bcd(88100000, bcd);
    TEST_ASSERT(bcd[0] == 0x00, "88.1MHz BCD byte[0]");
    TEST_ASSERT(bcd[1] == 0x00, "88.1MHz BCD byte[1]");
    TEST_ASSERT(bcd[2] == 0x10, "88.1MHz BCD byte[2]");
    TEST_ASSERT(bcd[3] == 0x88, "88.1MHz BCD byte[3]");
    TEST_ASSERT(bcd[4] == 0x00, "88.1MHz BCD byte[4]");
    decoded = test_bcd_to_freq(bcd);
    TEST_ASSERT(decoded == 88100000, "88.1MHz BCD roundtrip");
}

/* ──────────────────────── CI-V parser test helpers ──────────────────────── */

typedef struct {
    uint32_t last_freq;
    int      call_count;
} test_cb_ctx_t;

static void test_change_cb(const civ_params_t *params, void *ctx)
{
    test_cb_ctx_t *c = (test_cb_ctx_t *)ctx;
    c->last_freq = params->frequency;
    c->call_count++;
}

/* Build a CI-V frame into buf; returns frame length.
 * sub == 0xFF means no sub-command byte. */
static int build_test_frame(uint8_t *buf, uint8_t cmd, uint8_t sub,
                             const uint8_t *data, int data_len)
{
    int pos = 0;
    buf[pos++] = 0xFE; /* preamble */
    buf[pos++] = 0xFE;
    buf[pos++] = 0x96; /* to: radio (CIV_ADDR_RADIO) */
    buf[pos++] = 0xE0; /* from: PC  (CIV_ADDR_PC)    */
    buf[pos++] = cmd;
    if (sub != 0xFF) {
        buf[pos++] = sub;
    }
    for (int i = 0; i < data_len; i++) {
        buf[pos++] = data[i];
    }
    buf[pos++] = 0xFD; /* EOM */
    return pos;
}

/* ──────────────────────── Test: set-frequency command ──────────────────────── */

static void test_civ_parser_set_freq(void)
{
    ESP_LOGI(TAG, "--- test_civ_parser_set_freq ---");

    test_cb_ctx_t ctx = {0};
    civ_emu_t *civ = civ_emu_create(test_change_cb, &ctx);
    TEST_ASSERT(civ != NULL, "civ_emu_create succeeds");
    if (!civ) return;

    /* BCD for 100.1 MHz */
    uint8_t bcd[5];
    test_freq_to_bcd(100100000, bcd);

    uint8_t frame[32];
    int flen = build_test_frame(frame, 0x05 /* CIV_CMD_SET_FREQ */, 0xFF, bcd, 5);

    uint8_t resp[64];
    int rlen = civ_emu_process(civ, frame, flen, resp, sizeof(resp));

    /* Callback must have fired with correct frequency */
    TEST_ASSERT(ctx.call_count == 1, "set-freq callback called once");
    TEST_ASSERT(ctx.last_freq == 100100000, "set-freq callback frequency = 100.1MHz");

    /* Response must contain ACK (0xFB) */
    bool found_ack = false;
    for (int i = 0; i < rlen; i++) {
        if (resp[i] == 0xFB) { found_ack = true; break; }
    }
    TEST_ASSERT(found_ack, "set-freq response contains ACK (0xFB)");

    civ_emu_free(civ);
}

/* ──────────────────────── Test: read-frequency command ──────────────────────── */

static void test_civ_parser_read_freq(void)
{
    ESP_LOGI(TAG, "--- test_civ_parser_read_freq ---");

    civ_emu_t *civ = civ_emu_create(NULL, NULL);
    TEST_ASSERT(civ != NULL, "civ_emu_create succeeds");
    if (!civ) return;

    /* Set state to 145.8 MHz */
    civ_emu_update_state(civ, 145800000, 0, 0, 70, 0);

    /* Build read-frequency frame: FE FE 96 E0 03 FD */
    uint8_t frame[16];
    int flen = build_test_frame(frame, 0x03 /* CIV_CMD_READ_FREQ */, 0xFF, NULL, 0);

    uint8_t resp[64];
    int rlen = civ_emu_process(civ, frame, flen, resp, sizeof(resp));

    /* Response must contain cmd 0x03 */
    bool found_cmd = false;
    for (int i = 0; i < rlen; i++) {
        if (resp[i] == 0x03) { found_cmd = true; break; }
    }
    TEST_ASSERT(found_cmd, "read-freq response contains cmd 0x03");

    /* Extract BCD bytes that follow 0x03 in the response and verify frequency */
    uint32_t resp_freq = 0;
    for (int i = 0; i < rlen - 5; i++) {
        if (resp[i] == 0x03) {
            resp_freq = test_bcd_to_freq(&resp[i + 1]);
            break;
        }
    }
    TEST_ASSERT(resp_freq == 145800000, "read-freq response BCD = 145.8MHz");

    civ_emu_free(civ);
}

/* ──────────────────────── Test: read-mode command ──────────────────────── */

static void test_civ_parser_read_mode(void)
{
    ESP_LOGI(TAG, "--- test_civ_parser_read_mode ---");

    civ_emu_t *civ = civ_emu_create(NULL, NULL);
    TEST_ASSERT(civ != NULL, "civ_emu_create succeeds");
    if (!civ) return;

    /* Set mode to WBFM (0) */
    civ_emu_update_state(civ, 100000000, 0 /* WBFM */, 0, 70, 0);

    uint8_t frame[16];
    int flen = build_test_frame(frame, 0x04 /* CIV_CMD_READ_MODE */, 0xFF, NULL, 0);

    uint8_t resp[64];
    int rlen = civ_emu_process(civ, frame, flen, resp, sizeof(resp));

    /* Response must contain cmd 0x04 followed by mode byte 0x06 (WFM) */
    bool found_wfm = false;
    for (int i = 0; i < rlen - 1; i++) {
        if (resp[i] == 0x04 && resp[i + 1] == 0x06) {
            found_wfm = true;
            break;
        }
    }
    TEST_ASSERT(rlen > 0, "read-mode produces a response");
    TEST_ASSERT(found_wfm, "read-mode WBFM response mode byte = 0x06 (WFM)");

    civ_emu_free(civ);
}

/* ──────────────────────── Test: S-meter read command ──────────────────────── */

static void test_civ_parser_smeter(void)
{
    ESP_LOGI(TAG, "--- test_civ_parser_smeter ---");

    civ_emu_t *civ = civ_emu_create(NULL, NULL);
    TEST_ASSERT(civ != NULL, "civ_emu_create succeeds");
    if (!civ) return;

    /* Set signal_strength to 0x00FF (255) — well within range */
    civ_emu_update_state(civ, 100000000, 0, (int16_t)255, 70, 0);

    /* Build read-meter frame: FE FE 96 E0 15 02 FD */
    uint8_t data[1] = { 0x02 };
    uint8_t frame[16];
    int flen = build_test_frame(frame, 0x15 /* CIV_CMD_READ_METER */, 0xFF, data, 1);

    uint8_t resp[64];
    int rlen = civ_emu_process(civ, frame, flen, resp, sizeof(resp));

    TEST_ASSERT(rlen > 0, "s-meter read produces a response");

    /* Response should contain cmd 0x15 */
    bool found_cmd = false;
    for (int i = 0; i < rlen; i++) {
        if (resp[i] == 0x15) { found_cmd = true; break; }
    }
    TEST_ASSERT(found_cmd, "s-meter response contains cmd 0x15");

    civ_emu_free(civ);
}

/* ──────────────────────── Test: NAK for unknown command ──────────────────────── */

static void test_civ_parser_nak(void)
{
    ESP_LOGI(TAG, "--- test_civ_parser_nak ---");

    civ_emu_t *civ = civ_emu_create(NULL, NULL);
    TEST_ASSERT(civ != NULL, "civ_emu_create succeeds");
    if (!civ) return;

    /* Send unknown command 0x7F */
    uint8_t frame[16];
    int flen = build_test_frame(frame, 0x7F, 0xFF, NULL, 0);

    uint8_t resp[64];
    int rlen = civ_emu_process(civ, frame, flen, resp, sizeof(resp));

    bool found_nak = false;
    for (int i = 0; i < rlen; i++) {
        if (resp[i] == 0xFA) { found_nak = true; break; }
    }
    TEST_ASSERT(found_nak, "unknown command returns NAK (0xFA)");

    civ_emu_free(civ);
}

/* ──────────────────────── Test: NVS settings roundtrip ──────────────────────── */

static void test_nvs_roundtrip(void)
{
    ESP_LOGI(TAG, "--- test_nvs_roundtrip ---");

    nvs_radio_settings_t orig = {
        .frequency    = 98500000,
        .gain         = 400,
        .volume       = 85,
        .mode         = 1,  /* NBFM */
        .filter_bw    = 3000,
        .squelch      = 25,
        .nb_enabled   = true,
        .nb_threshold = 7,
    };

    esp_err_t ret = nvs_settings_save(&orig);
    TEST_ASSERT(ret == ESP_OK, "NVS save succeeds");

    nvs_radio_settings_t loaded = {0};
    ret = nvs_settings_load(&loaded);
    TEST_ASSERT(ret == ESP_OK, "NVS load succeeds");
    TEST_ASSERT(loaded.frequency    == orig.frequency,    "NVS freq roundtrip");
    TEST_ASSERT(loaded.gain         == orig.gain,         "NVS gain roundtrip");
    TEST_ASSERT(loaded.volume       == orig.volume,       "NVS volume roundtrip");
    TEST_ASSERT(loaded.mode         == orig.mode,         "NVS mode roundtrip");
    TEST_ASSERT(loaded.filter_bw    == orig.filter_bw,    "NVS filter_bw roundtrip");
    TEST_ASSERT(loaded.squelch      == orig.squelch,      "NVS squelch roundtrip");
    TEST_ASSERT(loaded.nb_enabled   == orig.nb_enabled,   "NVS nb_enabled roundtrip");
    TEST_ASSERT(loaded.nb_threshold == orig.nb_threshold, "NVS nb_threshold roundtrip");
}

/* ──────────────────────── Test: stereo blend ratio ──────────────────────── */

#ifdef CONFIG_FM_STEREO_ENABLE
static void test_stereo_blend_ratio(void)
{
    ESP_LOGI(TAG, "--- test_stereo_blend_ratio ---");

    fm_stereo_config_t cfg = FM_STEREO_CONFIG_DEFAULT();
    fm_stereo_t *st = fm_stereo_create(&cfg);
    TEST_ASSERT(st != NULL, "fm_stereo_create succeeds");
    if (!st) return;

    /* Fresh instance with no input: blend ratio must be in [0, 100] */
    int ratio = fm_stereo_get_blend_ratio(st);
    TEST_ASSERT(ratio >= 0,   "blend ratio >= 0");
    TEST_ASSERT(ratio <= 100, "blend ratio <= 100");

    fm_stereo_free(st);
}
#endif /* CONFIG_FM_STEREO_ENABLE */

/* ──────────────────────── Public entry point ──────────────────────── */

void test_phase4(void)
{
    ESP_LOGI(TAG, "========== Phase 4 Feature Tests ==========");
    tests_passed = 0;
    tests_failed = 0;

    test_civ_bcd_frequency();
    test_civ_parser_set_freq();
    test_civ_parser_read_freq();
    test_civ_parser_read_mode();
    test_civ_parser_smeter();
    test_civ_parser_nak();
    test_nvs_roundtrip();
#ifdef CONFIG_FM_STEREO_ENABLE
    test_stereo_blend_ratio();
#endif

    ESP_LOGI(TAG, "========== Results: %d passed, %d failed ==========",
             tests_passed, tests_failed);
}
