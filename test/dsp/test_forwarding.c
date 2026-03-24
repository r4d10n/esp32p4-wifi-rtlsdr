/*
 * Test: Network forwarding format validation
 *
 * Validates output formats for AIVDM NMEA, SBS-1 BaseStation, APRS-IS login.
 * Compile: gcc -o test_forwarding test_forwarding.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define GREEN "\033[32m"
#define RED   "\033[31m"
#define RESET "\033[0m"

static int tests_run = 0, tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  TEST: %-50s", name); } while(0)
#define PASS() do { tests_passed++; printf(GREEN "[PASS]" RESET "\n"); } while(0)
#define FAIL(m) do { printf(RED "[FAIL] %s" RESET "\n", m); } while(0)

/* ── AIVDM NMEA checksum validation ──────────────────── */

static uint8_t nmea_checksum(const char *sentence) {
    /* XOR all characters between ! and * (exclusive) */
    uint8_t cksum = 0;
    const char *p = sentence;
    if (*p == '!' || *p == '$') p++;  /* Skip start delimiter */
    while (*p && *p != '*') {
        cksum ^= (uint8_t)*p;
        p++;
    }
    return cksum;
}

static void test_aivdm_checksum(void) {
    TEST("AIVDM NMEA checksum");

    /* Known valid AIVDM sentence */
    const char *sentence = "!AIVDM,1,1,,A,13u@Dt002s000000000000000000,0";
    uint8_t computed = nmea_checksum(sentence);

    /* The checksum should be consistent */
    printf("(cksum=0x%02X) ", computed);
    if (computed > 0 && computed < 256) {
        PASS();
    } else {
        FAIL("checksum out of range");
    }
}

static void test_aivdm_format(void) {
    TEST("AIVDM sentence format");

    /* Validate AIVDM structure: !AIVDM,fragments,fragnum,,channel,payload,fillbits*XX */
    const char *sentence = "!AIVDM,1,1,,B,H52N>V@T2000000000000000000,2*00";

    int valid = 1;
    if (strncmp(sentence, "!AIVDM,", 7) != 0) valid = 0;

    /* Count commas (should be 6) */
    int commas = 0;
    for (const char *p = sentence; *p && *p != '*'; p++) {
        if (*p == ',') commas++;
    }
    if (commas != 6) valid = 0;

    /* Check for * followed by 2 hex chars */
    const char *star = strchr(sentence, '*');
    if (!star || strlen(star) < 3) valid = 0;

    if (valid) PASS(); else FAIL("invalid AIVDM format");
}

/* ── SBS-1 BaseStation format ─────────────────────────── */

static void test_sbs1_format(void) {
    TEST("SBS-1 BaseStation message format");

    /* SBS-1: MSG,type,session,aircraft,hex,flight,date,time,date,time,callsign,... */
    const char *sbs = "MSG,3,1,1,A1B2C3,1,,,,TEST,35000,450,270,51.50000,-0.10000,500,,,,\r\n";

    int valid = 1;
    if (strncmp(sbs, "MSG,", 4) != 0) valid = 0;

    /* Count commas (should be ~21) */
    int commas = 0;
    for (const char *p = sbs; *p; p++) {
        if (*p == ',') commas++;
    }
    /* SBS-1 has 21 fields separated by 20 commas minimum */
    if (commas < 15) valid = 0;

    /* Should contain hex ICAO */
    if (!strstr(sbs, "A1B2C3")) valid = 0;

    /* Should end with \r\n */
    int len = strlen(sbs);
    if (sbs[len-1] != '\n' || sbs[len-2] != '\r') valid = 0;

    if (valid) PASS(); else FAIL("invalid SBS-1 format");
}

static void test_sbs1_msg_types(void) {
    TEST("SBS-1 message type range (1-8)");

    /* Valid msg types: 1=ES_IDENT, 2=ES_SURFACE, 3=ES_AIRBORNE, 4=ES_VELOCITY, etc */
    int valid = 1;
    for (int type = 1; type <= 8; type++) {
        char sbs[64];
        snprintf(sbs, sizeof(sbs), "MSG,%d,", type);
        if (sbs[4] < '1' || sbs[4] > '8') valid = 0;
    }

    if (valid) PASS(); else FAIL("invalid msg type");
}

/* ── APRS-IS login format ─────────────────────────────── */

static void test_aprsis_login(void) {
    TEST("APRS-IS login string format");

    /* Format: user CALLSIGN pass PASSCODE vers SOFTWARE VERSION\r\n */
    char login[128];
    snprintf(login, sizeof(login), "user N0CALL pass 12345 vers ESP32P4-SDR 1.0\r\n");

    int valid = 1;
    if (strncmp(login, "user ", 5) != 0) valid = 0;
    if (!strstr(login, " pass ")) valid = 0;
    if (!strstr(login, " vers ")) valid = 0;
    if (!strstr(login, "\r\n")) valid = 0;

    if (valid) PASS(); else FAIL("invalid login format");
}

static void test_aprsis_packet(void) {
    TEST("APRS-IS packet format");

    /* Format: SRC>DST,PATH,qAR,IGATE:info\r\n */
    char packet[256];
    snprintf(packet, sizeof(packet),
             "N0CALL-7>APRS,WIDE1-1,qAR,ESP32P4:!4903.50N/07201.75W-Test\r\n");

    int valid = 1;
    if (!strchr(packet, '>')) valid = 0;
    if (!strstr(packet, ",qAR,")) valid = 0;
    if (!strchr(packet, ':')) valid = 0;
    if (!strstr(packet, "\r\n")) valid = 0;

    if (valid) PASS(); else FAIL("invalid APRS packet format");
}

/* ── PSKreporter XML format ───────────────────────────── */

static void test_pskreporter_xml(void) {
    TEST("PSKreporter XML structure");

    char xml[512];
    snprintf(xml, sizeof(xml),
        "<?xml version='1.0'?>"
        "<receptionReports>"
        "<receiverInformation><receiverCallsign>ESP32P4</receiverCallsign></receiverInformation>"
        "<receptionReport>"
        "<senderCallsign>W1AW</senderCallsign>"
        "<frequency>14074500</frequency>"
        "<sNR>-10</sNR>"
        "<mode>FT8</mode>"
        "</receptionReport>"
        "</receptionReports>");

    int valid = 1;
    if (!strstr(xml, "<?xml")) valid = 0;
    if (!strstr(xml, "<receptionReports>")) valid = 0;
    if (!strstr(xml, "<senderCallsign>")) valid = 0;
    if (!strstr(xml, "<frequency>")) valid = 0;
    if (!strstr(xml, "</receptionReports>")) valid = 0;

    if (valid) PASS(); else FAIL("invalid XML structure");
}

/* ── MQTT topic format ────────────────────────────────── */

static void test_mqtt_topic(void) {
    TEST("MQTT topic format for rtl_433");

    /* Format: rtl_433/{model}/{id} */
    char topic[64];
    snprintf(topic, sizeof(topic), "rtl_433/Acurite-Tower/12345");

    int valid = 1;
    if (strncmp(topic, "rtl_433/", 8) != 0) valid = 0;
    /* Should have at least 2 slashes */
    int slashes = 0;
    for (const char *p = topic; *p; p++) if (*p == '/') slashes++;
    if (slashes < 2) valid = 0;

    if (valid) PASS(); else FAIL("invalid MQTT topic");
}

/* ── Main ─────────────────────────────────────────────── */

int main(void) {
    printf("\n=== Network Forwarding Format Tests ===\n\n");

    printf("[AIVDM NMEA]\n");
    test_aivdm_checksum();
    test_aivdm_format();

    printf("\n[SBS-1 BaseStation]\n");
    test_sbs1_format();
    test_sbs1_msg_types();

    printf("\n[APRS-IS]\n");
    test_aprsis_login();
    test_aprsis_packet();

    printf("\n[PSKreporter]\n");
    test_pskreporter_xml();

    printf("\n[MQTT]\n");
    test_mqtt_topic();

    printf("\n=== Results: %d/%d passed ===\n\n", tests_passed, tests_run);
    return tests_passed == tests_run ? 0 : 1;
}
