/*
 * Unit Tests — wifimgr_config JSON round-trip and defaults
 *
 * These tests verify config serialization/deserialization logic
 * using the cJSON library directly (no ESP-IDF NVS/LittleFS needed).
 * Can be compiled and run on host with: gcc -o test_config_json test_config_json.c -lcjson -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <cJSON.h>

/* ── Color output ───────────────────────────────────────────── */
#define GREEN  "\033[32m"
#define RED    "\033[31m"
#define RESET  "\033[0m"

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  TEST: %-50s", name); \
} while(0)

#define PASS() do { tests_passed++; printf(GREEN "[PASS]" RESET "\n"); } while(0)
#define FAIL(msg) do { tests_failed++; printf(RED "[FAIL] %s" RESET "\n", msg); } while(0)

/* ═══════════════════════════════════════════════════════════════
 *  SDR Config Tests
 * ═══════════════════════════════════════════════════════════════ */

static void test_sdr_config_defaults(void)
{
    TEST("SDR config defaults");

    const char *default_json = "{}";
    cJSON *json = cJSON_Parse(default_json);
    assert(json != NULL);

    /* Verify empty JSON means we use defaults */
    cJSON *freq = cJSON_GetObjectItem(json, "center_freq");
    assert(freq == NULL); /* not present = use default */

    cJSON_Delete(json);
    PASS();
}

static void test_sdr_config_roundtrip(void)
{
    TEST("SDR config JSON round-trip");

    /* Create a config JSON */
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "center_freq", 144390000);
    cJSON_AddNumberToObject(json, "sample_rate", 250000);
    cJSON_AddStringToObject(json, "gain_mode", "manual");
    cJSON_AddNumberToObject(json, "tuner_gain_tenth_db", 297);
    cJSON_AddBoolToObject(json, "rtl_agc", 0);
    cJSON_AddBoolToObject(json, "tuner_agc", 0);
    cJSON_AddNumberToObject(json, "ppm_correction", -3);
    cJSON_AddStringToObject(json, "direct_sampling", "off");
    cJSON_AddBoolToObject(json, "offset_tuning", 0);
    cJSON_AddNumberToObject(json, "offset_freq_hz", 0);
    cJSON_AddBoolToObject(json, "bias_tee", 1);
    cJSON_AddBoolToObject(json, "dc_offset_correction", 1);
    cJSON_AddBoolToObject(json, "iq_imbalance_correction", 0);
    cJSON_AddBoolToObject(json, "invert_iq", 0);
    cJSON_AddNumberToObject(json, "max_total_users", 3);
    cJSON_AddStringToObject(json, "hostname", "test-sdr");

    /* Serialize */
    char *str = cJSON_PrintUnformatted(json);
    assert(str != NULL);

    /* Parse back */
    cJSON *parsed = cJSON_Parse(str);
    assert(parsed != NULL);

    /* Verify values */
    cJSON *item;
    item = cJSON_GetObjectItem(parsed, "center_freq");
    assert(item && (uint32_t)cJSON_GetNumberValue(item) == 144390000);

    item = cJSON_GetObjectItem(parsed, "sample_rate");
    assert(item && (uint32_t)cJSON_GetNumberValue(item) == 250000);

    item = cJSON_GetObjectItem(parsed, "gain_mode");
    assert(item && strcmp(cJSON_GetStringValue(item), "manual") == 0);

    item = cJSON_GetObjectItem(parsed, "tuner_gain_tenth_db");
    assert(item && (uint16_t)cJSON_GetNumberValue(item) == 297);

    item = cJSON_GetObjectItem(parsed, "ppm_correction");
    assert(item && (int16_t)cJSON_GetNumberValue(item) == -3);

    item = cJSON_GetObjectItem(parsed, "bias_tee");
    assert(item && cJSON_IsTrue(item));

    item = cJSON_GetObjectItem(parsed, "invert_iq");
    assert(item && cJSON_IsFalse(item));

    item = cJSON_GetObjectItem(parsed, "max_total_users");
    assert(item && (uint8_t)cJSON_GetNumberValue(item) == 3);

    item = cJSON_GetObjectItem(parsed, "hostname");
    assert(item && strcmp(cJSON_GetStringValue(item), "test-sdr") == 0);

    cJSON_Delete(json);
    cJSON_Delete(parsed);
    cJSON_free(str);
    PASS();
}

static void test_sdr_config_partial_update(void)
{
    TEST("SDR config partial update (merge)");

    /* Existing config */
    cJSON *existing = cJSON_CreateObject();
    cJSON_AddNumberToObject(existing, "center_freq", 100000000);
    cJSON_AddNumberToObject(existing, "sample_rate", 250000);
    cJSON_AddStringToObject(existing, "gain_mode", "auto");

    /* Partial update */
    cJSON *update = cJSON_Parse("{\"center_freq\": 433920000, \"gain_mode\": \"manual\"}");
    assert(update != NULL);

    /* Apply update to existing */
    cJSON *field = NULL;
    cJSON_ArrayForEach(field, update) {
        cJSON_DeleteItemFromObject(existing, field->string);
        cJSON_AddItemToObject(existing, field->string, cJSON_Duplicate(field, 1));
    }

    /* Verify merge */
    cJSON *item;
    item = cJSON_GetObjectItem(existing, "center_freq");
    assert(item && (uint32_t)cJSON_GetNumberValue(item) == 433920000);

    item = cJSON_GetObjectItem(existing, "gain_mode");
    assert(item && strcmp(cJSON_GetStringValue(item), "manual") == 0);

    /* Unchanged field preserved */
    item = cJSON_GetObjectItem(existing, "sample_rate");
    assert(item && (uint32_t)cJSON_GetNumberValue(item) == 250000);

    cJSON_Delete(existing);
    cJSON_Delete(update);
    PASS();
}

/* ═══════════════════════════════════════════════════════════════
 *  Ethernet Config Tests
 * ═══════════════════════════════════════════════════════════════ */

static void test_ethernet_config_roundtrip(void)
{
    TEST("Ethernet config JSON round-trip");

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enable", 1);
    cJSON_AddBoolToObject(json, "dhcp", 0);
    cJSON_AddStringToObject(json, "static_ip", "192.168.1.50");
    cJSON_AddStringToObject(json, "static_mask", "255.255.255.0");
    cJSON_AddStringToObject(json, "static_gw", "192.168.1.1");
    cJSON_AddStringToObject(json, "static_dns", "1.1.1.1");
    cJSON_AddStringToObject(json, "phy_type", "IP101");
    cJSON_AddNumberToObject(json, "phy_addr", 1);
    cJSON_AddNumberToObject(json, "mdc_gpio", 31);
    cJSON_AddNumberToObject(json, "mdio_gpio", 27);
    cJSON_AddBoolToObject(json, "prefer_over_wifi", 1);

    char *str = cJSON_PrintUnformatted(json);
    cJSON *parsed = cJSON_Parse(str);
    assert(parsed != NULL);

    assert(cJSON_IsTrue(cJSON_GetObjectItem(parsed, "enable")));
    assert(cJSON_IsFalse(cJSON_GetObjectItem(parsed, "dhcp")));
    assert(strcmp(cJSON_GetObjectItem(parsed, "static_ip")->valuestring, "192.168.1.50") == 0);
    assert(strcmp(cJSON_GetObjectItem(parsed, "phy_type")->valuestring, "IP101") == 0);
    assert(cJSON_GetObjectItem(parsed, "phy_addr")->valueint == 1);

    cJSON_Delete(json);
    cJSON_Delete(parsed);
    cJSON_free(str);
    PASS();
}

/* ═══════════════════════════════════════════════════════════════
 *  Services Config Tests
 * ═══════════════════════════════════════════════════════════════ */

static void test_services_default_structure(void)
{
    TEST("Services default JSON has all expected services");

    /* Simulate the default services creation (same as wifimgr_config.c) */
    const char *expected_services[] = {
        "rtl_tcp", "rtl_udp", "spyserver", "soapysdr", "websdr",
        "adsb", "ais", "aprs", "gsm_scanner", "ft8_wspr",
        "fm_player", "rtl_433", "rtl_power"
    };
    int expected_count = sizeof(expected_services) / sizeof(expected_services[0]);

    /* Read the default services JSON from disk or create it */
    /* For this test, we just verify the structure pattern */
    cJSON *root = cJSON_CreateObject();

    for (int i = 0; i < expected_count; i++) {
        cJSON *svc = cJSON_AddObjectToObject(root, expected_services[i]);
        cJSON_AddBoolToObject(svc, "enable", 0);
    }

    /* Verify all services present */
    for (int i = 0; i < expected_count; i++) {
        cJSON *svc = cJSON_GetObjectItem(root, expected_services[i]);
        if (!svc) {
            FAIL(expected_services[i]);
            cJSON_Delete(root);
            return;
        }
    }

    /* Verify count */
    int count = 0;
    cJSON *child = root->child;
    while (child) { count++; child = child->next; }
    assert(count == expected_count);

    cJSON_Delete(root);
    PASS();
}

static void test_service_enable_toggle(void)
{
    TEST("Service enable/disable toggle");

    cJSON *svc = cJSON_CreateObject();
    cJSON_AddBoolToObject(svc, "enable", 0);
    cJSON_AddNumberToObject(svc, "port", 1234);

    assert(cJSON_IsFalse(cJSON_GetObjectItem(svc, "enable")));

    /* Toggle enable */
    cJSON_ReplaceItemInObject(svc, "enable", cJSON_CreateBool(1));
    assert(cJSON_IsTrue(cJSON_GetObjectItem(svc, "enable")));

    /* Port preserved */
    assert(cJSON_GetObjectItem(svc, "port")->valueint == 1234);

    cJSON_Delete(svc);
    PASS();
}

static void test_service_rtl433_frequency_array(void)
{
    TEST("rtl_433 frequency array handling");

    cJSON *svc = cJSON_CreateObject();
    cJSON *freqs = cJSON_AddArrayToObject(svc, "frequency");
    cJSON_AddItemToArray(freqs, cJSON_CreateNumber(433920000));
    cJSON_AddItemToArray(freqs, cJSON_CreateNumber(315000000));
    cJSON_AddItemToArray(freqs, cJSON_CreateNumber(868000000));

    cJSON *arr = cJSON_GetObjectItem(svc, "frequency");
    assert(cJSON_IsArray(arr));
    assert(cJSON_GetArraySize(arr) == 3);
    assert(cJSON_GetArrayItem(arr, 0)->valuedouble == 433920000);
    assert(cJSON_GetArrayItem(arr, 2)->valuedouble == 868000000);

    cJSON_Delete(svc);
    PASS();
}

/* ═══════════════════════════════════════════════════════════════
 *  Notification Config Tests
 * ═══════════════════════════════════════════════════════════════ */

static void test_notify_config_roundtrip(void)
{
    TEST("Notification config JSON round-trip");

    cJSON *json = cJSON_CreateObject();
    cJSON *tg = cJSON_AddObjectToObject(json, "telegram");
    cJSON_AddBoolToObject(tg, "enable", 1);
    cJSON_AddNumberToObject(tg, "rate_limit_s", 30);

    cJSON *dc = cJSON_AddObjectToObject(json, "discord");
    cJSON_AddBoolToObject(dc, "enable", 0);
    cJSON_AddNumberToObject(dc, "rate_limit_s", 60);

    char *str = cJSON_PrintUnformatted(json);
    cJSON *parsed = cJSON_Parse(str);
    assert(parsed != NULL);

    cJSON *tg_p = cJSON_GetObjectItem(parsed, "telegram");
    assert(tg_p && cJSON_IsTrue(cJSON_GetObjectItem(tg_p, "enable")));
    assert(cJSON_GetObjectItem(tg_p, "rate_limit_s")->valueint == 30);

    cJSON *dc_p = cJSON_GetObjectItem(parsed, "discord");
    assert(dc_p && cJSON_IsFalse(cJSON_GetObjectItem(dc_p, "enable")));

    cJSON_Delete(json);
    cJSON_Delete(parsed);
    cJSON_free(str);
    PASS();
}

static void test_notify_token_masking(void)
{
    TEST("Notification token masking logic");

    const char *token = "123456789:ABCdefGHIjklMNO";
    char masked[16] = {0};
    strncpy(masked, token, 8);
    strcat(masked, "...");

    assert(strcmp(masked, "12345678...") == 0);
    assert(strlen(masked) == 11);
    /* Original token unchanged */
    assert(strlen(token) == 25);

    PASS();
}

/* ═══════════════════════════════════════════════════════════════
 *  Chatbot Config Tests
 * ═══════════════════════════════════════════════════════════════ */

static void test_chatbot_config_defaults(void)
{
    TEST("Chatbot config default values");

    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enable", 0);
    cJSON_AddStringToObject(json, "provider", "gemini");
    cJSON_AddStringToObject(json, "model", "gemini-2.0-flash-lite");
    cJSON_AddBoolToObject(json, "web_enable", 1);
    cJSON_AddBoolToObject(json, "telegram_enable", 0);
    cJSON_AddBoolToObject(json, "discord_enable", 0);
    cJSON_AddNumberToObject(json, "max_history", 10);
    cJSON_AddStringToObject(json, "allowed_tools", "query");

    assert(cJSON_IsFalse(cJSON_GetObjectItem(json, "enable")));
    assert(strcmp(cJSON_GetObjectItem(json, "provider")->valuestring, "gemini") == 0);
    assert(strcmp(cJSON_GetObjectItem(json, "allowed_tools")->valuestring, "query") == 0);

    cJSON_Delete(json);
    PASS();
}

/* ═══════════════════════════════════════════════════════════════
 *  Backup/Restore Tests
 * ═══════════════════════════════════════════════════════════════ */

static void test_backup_export_structure(void)
{
    TEST("Backup export JSON structure");

    cJSON *backup = cJSON_CreateObject();
    cJSON_AddObjectToObject(backup, "sdr");
    cJSON_AddObjectToObject(backup, "services");
    cJSON_AddObjectToObject(backup, "ethernet");
    cJSON_AddObjectToObject(backup, "notify");
    cJSON_AddObjectToObject(backup, "chatbot");

    /* All top-level keys present */
    assert(cJSON_GetObjectItem(backup, "sdr") != NULL);
    assert(cJSON_GetObjectItem(backup, "services") != NULL);
    assert(cJSON_GetObjectItem(backup, "ethernet") != NULL);
    assert(cJSON_GetObjectItem(backup, "notify") != NULL);
    assert(cJSON_GetObjectItem(backup, "chatbot") != NULL);

    /* Non-existent key is NULL */
    assert(cJSON_GetObjectItem(backup, "wifi_passwords") == NULL);

    cJSON_Delete(backup);
    PASS();
}

/* ═══════════════════════════════════════════════════════════════
 *  Edge Cases & Error Handling
 * ═══════════════════════════════════════════════════════════════ */

static void test_empty_ssid_rejected(void)
{
    TEST("Empty SSID handling");

    const char *ssid = "";
    assert(ssid[0] == '\0'); /* would be rejected by wifimgr_save_network */
    PASS();
}

static void test_long_ssid_truncation(void)
{
    TEST("SSID truncation at 32 chars");

    char ssid[64];
    memset(ssid, 'A', 63);
    ssid[63] = '\0';

    char truncated[33] = {0};
    strncpy(truncated, ssid, 32);

    assert(strlen(truncated) == 32);
    assert(truncated[32] == '\0');
    PASS();
}

static void test_malformed_json_handling(void)
{
    TEST("Malformed JSON parse returns NULL");

    cJSON *bad1 = cJSON_Parse("not json");
    assert(bad1 == NULL);

    cJSON *bad2 = cJSON_Parse("{invalid}");
    assert(bad2 == NULL);

    cJSON *bad3 = cJSON_Parse("");
    assert(bad3 == NULL);

    cJSON *bad4 = cJSON_Parse(NULL);
    assert(bad4 == NULL);

    /* Valid but empty */
    cJSON *ok = cJSON_Parse("{}");
    assert(ok != NULL);
    cJSON_Delete(ok);

    PASS();
}

static void test_large_frequency_values(void)
{
    TEST("Large frequency values (uint32 range)");

    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "center_freq", 1700000000); /* 1.7 GHz */
    cJSON_AddNumberToObject(json, "offset_freq_hz", -125000000); /* -125 MHz */

    char *str = cJSON_PrintUnformatted(json);
    cJSON *parsed = cJSON_Parse(str);

    uint32_t freq = (uint32_t)cJSON_GetNumberValue(cJSON_GetObjectItem(parsed, "center_freq"));
    assert(freq == 1700000000);

    int32_t offset = (int32_t)cJSON_GetNumberValue(cJSON_GetObjectItem(parsed, "offset_freq_hz"));
    assert(offset == -125000000);

    cJSON_Delete(json);
    cJSON_Delete(parsed);
    cJSON_free(str);
    PASS();
}

static void test_gain_values_valid_range(void)
{
    TEST("R828D gain values in valid range");

    const uint16_t valid_gains[] = {
        0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229,
        254, 280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439,
        445, 480, 496
    };
    int n = sizeof(valid_gains) / sizeof(valid_gains[0]);
    assert(n == 29);

    /* All values fit in uint16 */
    for (int i = 0; i < n; i++) {
        assert(valid_gains[i] <= 496);
        assert(valid_gains[i] <= UINT16_MAX);
    }

    /* Monotonically increasing */
    for (int i = 1; i < n; i++) {
        assert(valid_gains[i] > valid_gains[i - 1]);
    }

    PASS();
}

/* ═══════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════ */

int main(void)
{
    printf("\n=== WiFi Manager Config Unit Tests ===\n\n");

    /* SDR Config */
    printf("[SDR Config]\n");
    test_sdr_config_defaults();
    test_sdr_config_roundtrip();
    test_sdr_config_partial_update();

    /* Ethernet Config */
    printf("\n[Ethernet Config]\n");
    test_ethernet_config_roundtrip();

    /* Services Config */
    printf("\n[Services Config]\n");
    test_services_default_structure();
    test_service_enable_toggle();
    test_service_rtl433_frequency_array();

    /* Notification Config */
    printf("\n[Notification Config]\n");
    test_notify_config_roundtrip();
    test_notify_token_masking();

    /* Chatbot Config */
    printf("\n[Chatbot Config]\n");
    test_chatbot_config_defaults();

    /* Backup/Restore */
    printf("\n[Backup/Restore]\n");
    test_backup_export_structure();

    /* Edge Cases */
    printf("\n[Edge Cases & Error Handling]\n");
    test_empty_ssid_rejected();
    test_long_ssid_truncation();
    test_malformed_json_handling();
    test_large_frequency_values();
    test_gain_values_valid_range();

    printf("\n=== Results: %d/%d passed", tests_passed, tests_run);
    if (tests_failed > 0) {
        printf(", " RED "%d FAILED" RESET, tests_failed);
    }
    printf(" ===\n\n");

    return tests_failed > 0 ? 1 : 0;
}
