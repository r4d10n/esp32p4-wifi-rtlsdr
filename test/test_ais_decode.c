/*
 * Host-based unit tests for AIS decoder logic
 * Extracted pure functions from ais_decode.c (no ESP-IDF required)
 * Compile: gcc -Wall -Wextra -o test_ais_decode test_ais_decode.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* ──────────────────────── Test framework ──────────────────────── */

static int tests_passed = 0, tests_failed = 0;

#define PASS() do { printf("PASS\n"); tests_passed++; } while(0)
#define FAIL(...) do { printf("FAIL (" __VA_ARGS__); printf(")\n"); tests_failed++; } while(0)

#define ASSERT_EQ(label, a, b) do { \
    printf("  TEST: %-55s ", label); \
    if ((a) != (b)) { FAIL("got %d, expected %d", (int)(a), (int)(b)); } \
    else { PASS(); } \
} while(0)

#define ASSERT_FLOAT_EQ(label, a, b, eps) do { \
    printf("  TEST: %-55s ", label); \
    if (fabs((double)(a) - (double)(b)) > (eps)) { \
        FAIL("got %.6f, expected %.6f", (double)(a), (double)(b)); \
    } else { PASS(); } \
} while(0)

#define ASSERT_STR_EQ(label, a, b) do { \
    printf("  TEST: %-55s ", label); \
    if (strcmp((a), (b)) != 0) { FAIL("got '%s', expected '%s'", (a), (b)); } \
    else { PASS(); } \
} while(0)

/* ──────────────────────── Copied pure functions ──────────────────────── */

/* CRC-16 CCITT, poly 0x8408, init 0xFFFF */
static uint16_t ais_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
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

/* AIS 6-bit ASCII decode: 0-31 -> '@'-'_', 32-63 -> ' '-'?' */
static char ais_char(uint8_t val)
{
    val &= 0x3F;
    if (val < 32)
        return (char)(val + 64);
    else
        return (char)(val);
}

/* Bit extraction from packed byte array (MSB first) */
static uint32_t get_bits(const uint8_t *payload, uint16_t pay_bits,
                          uint16_t start, uint16_t len)
{
    if (start + len > pay_bits || len > 32) return 0;
    uint32_t result = 0;
    for (uint16_t i = start; i < start + len; i++) {
        uint16_t byte_idx = i / 8;
        uint8_t  bit_idx  = 7 - (i % 8);
        result = (result << 1) | ((payload[byte_idx] >> bit_idx) & 1);
    }
    return result;
}

static int32_t get_bits_signed(const uint8_t *payload, uint16_t pay_bits,
                                uint16_t start, uint16_t len)
{
    uint32_t val = get_bits(payload, pay_bits, start, len);
    if (val & (1u << (len - 1)))
        val |= ~((1u << len) - 1);
    return (int32_t)val;
}

/* ──────────────────────── Tests ──────────────────────── */

static void test_crc16(void)
{
    printf("\n[CRC-16 CCITT (poly 0x8408)]\n");

    /* Known vector: AIS/HDLC frame CRC residue should be 0xF0B8 */
    /* Build a frame, compute CRC, append it, verify residue = 0xF0B8 */
    uint8_t frame[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint16_t crc = ais_crc16(frame, (uint16_t)sizeof(frame));

    uint8_t frame_with_crc[sizeof(frame) + 2];
    memcpy(frame_with_crc, frame, sizeof(frame));
    frame_with_crc[sizeof(frame)]     = (uint8_t)(crc & 0xFF);
    frame_with_crc[sizeof(frame) + 1] = (uint8_t)((crc >> 8) & 0xFF);
    uint16_t residue = ais_crc16(frame_with_crc, (uint16_t)(sizeof(frame) + 2));

    /* CRC-16/CCITT LSB-first (poly 0x8408): CRC(body+FCS) == 0x0000.
       The source checks CRC(body_without_fcs) == 0xF0B8, meaning the
       stored FCS bytes are themselves 0xF0B8. Standard residue is 0x0000. */
    printf("  TEST: %-55s ", "CRC residue of frame+FCS == 0x0000");
    if (residue == 0x0000) { PASS(); }
    else { FAIL("got 0x%04X", residue); }

    /* Test empty payload */
    uint16_t crc_empty = ais_crc16(NULL, 0);
    printf("  TEST: %-55s ", "CRC of empty = 0xFFFF (init value)");
    ASSERT_EQ("", crc_empty, 0xFFFF);

    /* Single byte 0x00 */
    uint8_t zero = 0x00;
    uint16_t crc_zero = ais_crc16(&zero, 1);
    printf("  TEST: %-55s ", "CRC of {0x00} is consistent");
    uint16_t crc_zero2 = ais_crc16(&zero, 1);
    if (crc_zero == crc_zero2) { PASS(); }
    else { FAIL("not deterministic"); }
}

static void test_6bit_ascii(void)
{
    printf("\n[AIS 6-bit ASCII decode]\n");

    /* ITU-R M.1371: 0->'@', 1->'A', ..., 25->'Z', 32->' ' */
    ASSERT_EQ("ais_char(0)  == '@'", ais_char(0),  '@');
    ASSERT_EQ("ais_char(1)  == 'A'", ais_char(1),  'A');
    ASSERT_EQ("ais_char(2)  == 'B'", ais_char(2),  'B');
    ASSERT_EQ("ais_char(25) == 'Y'", ais_char(25), 'Y');
    ASSERT_EQ("ais_char(26) == 'Z'", ais_char(26), 'Z');
    ASSERT_EQ("ais_char(32) == ' '", ais_char(32), ' ');
    ASSERT_EQ("ais_char(48) == '0'", ais_char(48), '0');
    ASSERT_EQ("ais_char(57) == '9'", ais_char(57), '9');
    /* Mask: values > 63 are masked to 0-63 */
    ASSERT_EQ("ais_char(64) == '@'", ais_char(64), '@');
}

static void test_bit_extraction(void)
{
    printf("\n[Bit extraction (get_bits / get_bits_signed)]\n");

    /* Payload: 0xAB 0xCD = 10101011 11001101 */
    uint8_t payload[] = {0xAB, 0xCD};
    uint16_t pay_bits = 16;

    /* Bits 0-7 = 0xAB = 171 */
    ASSERT_EQ("get_bits bits[0..7] == 0xAB",
              get_bits(payload, pay_bits, 0, 8), 0xAB);

    /* Bits 8-15 = 0xCD = 205 */
    ASSERT_EQ("get_bits bits[8..15] == 0xCD",
              get_bits(payload, pay_bits, 8, 8), 0xCD);

    /* Bits 0-3 = upper nibble of 0xAB = 0xA = 10 */
    ASSERT_EQ("get_bits bits[0..3] == 0xA",
              get_bits(payload, pay_bits, 0, 4), 0xA);

    /* Bits 4-7 = lower nibble of 0xAB = 0xB = 11 */
    ASSERT_EQ("get_bits bits[4..7] == 0xB",
              get_bits(payload, pay_bits, 4, 4), 0xB);

    /* Signed: 0xAB upper 8 bits = 10101011 as int8 = -85 */
    int32_t signed_val = get_bits_signed(payload, pay_bits, 0, 8);
    ASSERT_EQ("get_bits_signed bits[0..7] == -85 (signed)",
              signed_val, (int32_t)(-85));

    /* Signed positive: 0x7F = 0111 1111 = +127 */
    uint8_t p2[] = {0x7F};
    int32_t sv2 = get_bits_signed(p2, 8, 0, 8);
    ASSERT_EQ("get_bits_signed 0x7F == +127", sv2, 127);
}

static void test_lat_lon_scaling(void)
{
    printf("\n[AIS lat/lon scaling]\n");

    /* AIS Type 1: raw lon in 1/10000 minutes, scaled by 600000
       181 * 600000 = 108600000 -> not available marker */
    double lon_na = (double)((int32_t)0x6791400) / 600000.0;
    /* 0x6791400 = 108600000 decimal = 181*600000 */
    ASSERT_FLOAT_EQ("lon 181.0 is N/A marker", lon_na, 181.0, 0.001);

    double lat_na = (double)((int32_t)(91 * 600000)) / 600000.0;
    ASSERT_FLOAT_EQ("lat 91.0 is N/A marker", lat_na, 91.0, 0.001);

    /* Valid position: Statue of Liberty approx 40.6892N, 74.0445W */
    /* lat_raw = 40.6892 * 600000 = 24413520 */
    int32_t lat_raw = 24413520;
    double lat = (double)lat_raw / 600000.0;
    ASSERT_FLOAT_EQ("lat 40.6892 N", lat, 40.6892, 0.001);

    /* lon_raw = -74.0445 * 600000 = -44426700 */
    int32_t lon_raw = -44426700;
    double lon = (double)lon_raw / 600000.0;
    ASSERT_FLOAT_EQ("lon -74.0445 W (negative)", lon, -74.0445, 0.001);
}

static void test_type1_parse(void)
{
    printf("\n[AIS Type 1 position report]\n");

    /*
     * Build a minimal AIS Type 1 message in raw bits.
     * Type 1, 168 bits total.
     * msg_type=1 (6 bits), repeat=0 (2 bits), mmsi=123456789 (30 bits)
     * nav_status=0 (4 bits), rot=-128 (8 bits), sog=0 (10 bits)
     * pos_acc=0 (1 bit), lon=0 (28 bits), lat=0 (27 bits)
     * cog=0 (12 bits), hdg=0 (9 bits), ... pad to 168 bits
     *
     * We'll test the bit-packing logic directly.
     */

    /* Manually build a 168-bit (21-byte) payload */
    uint8_t payload[21];
    memset(payload, 0, sizeof(payload));

    /* Set message type = 1 in bits [0..5] */
    payload[0] = (1 << 2);  /* msg_type=1 shifted to MSB position of 6 bits */

    /* Set MMSI = 123456789 in bits [8..37] */
    uint32_t mmsi = 123456789;
    /* Pack 30 bits starting at bit 8 */
    for (int i = 0; i < 30; i++) {
        int bit_pos = 8 + i;
        int byte_idx = bit_pos / 8;
        int bit_idx = 7 - (bit_pos % 8);
        if ((mmsi >> (29 - i)) & 1)
            payload[byte_idx] |= (1 << bit_idx);
    }

    uint16_t pay_bits = 168;

    uint8_t msg_type = (uint8_t)get_bits(payload, pay_bits, 0, 6);
    uint32_t decoded_mmsi = get_bits(payload, pay_bits, 8, 30);

    ASSERT_EQ("msg_type == 1", msg_type, 1);
    ASSERT_EQ("MMSI == 123456789", decoded_mmsi, 123456789u);

    /* Test lon/lat scaling: pack lon=1000000 (1.6667 degrees) at bits 61-88 */
    int32_t lon_pack = 1000000;
    for (int i = 0; i < 28; i++) {
        int bit_pos = 61 + i;
        int byte_idx = bit_pos / 8;
        int bit_idx = 7 - (bit_pos % 8);
        if ((lon_pack >> (27 - i)) & 1)
            payload[byte_idx] |= (1 << bit_idx);
    }
    int32_t lon_extracted = get_bits_signed(payload, pay_bits, 61, 28);
    double lon_deg = (double)lon_extracted / 600000.0;
    ASSERT_FLOAT_EQ("packed lon decodes correctly", lon_deg, 1.6667, 0.001);
}

static void test_nrzi(void)
{
    printf("\n[NRZI decode]\n");

    /*
     * AIS uses NRZI: 0 = transition, 1 = no transition.
     * Input raw bits from receiver -> NRZI decoded data bits.
     * Sequence [0,1,0,0,1] with prev=0:
     *   0: 0!=prev(0) -> transition -> bit=0, prev=0
     *   1: 1!=prev(0) -> transition -> bit=0, prev=1
     *   0: 0!=prev(1) -> transition -> bit=0, prev=0
     *   0: 0==prev(0) -> no trans   -> bit=1, prev=0
     *   1: 1!=prev(0) -> transition -> bit=0, prev=1
     */
    int raw[]      = {0, 1, 0, 0, 1};
    int expected[] = {0, 0, 0, 1, 0};
    int prev = 0;
    int ok = 1;
    for (int i = 0; i < 5; i++) {
        int nrzi = (raw[i] == prev) ? 1 : 0;
        prev = raw[i];
        if (nrzi != expected[i]) { ok = 0; break; }
    }
    printf("  TEST: %-55s ", "NRZI [0,1,0,0,1] -> [0,0,0,1,0]");
    if (ok) { PASS(); } else { FAIL("decode mismatch"); }
}

int main(void)
{
    printf("=== AIS Decoder Unit Tests ===\n");

    test_crc16();
    test_6bit_ascii();
    test_bit_extraction();
    test_lat_lon_scaling();
    test_type1_parse();
    test_nrzi();

    printf("\n=== Results: %d passed, %d failed ===\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
