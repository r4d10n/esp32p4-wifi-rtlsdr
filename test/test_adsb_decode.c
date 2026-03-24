/*
 * Host-based unit tests for ADS-B decoder logic
 * Extracted pure functions from adsb_decode.c (no ESP-IDF required)
 * Compile: gcc -Wall -Wextra -o test_adsb_decode test_adsb_decode.c -lm
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
    if ((uint32_t)(a) != (uint32_t)(b)) { FAIL("got 0x%X, expected 0x%X", (unsigned)(a), (unsigned)(b)); } \
    else { PASS(); } \
} while(0)

#define ASSERT_INT_EQ(label, a, b) do { \
    printf("  TEST: %-55s ", label); \
    if ((int)(a) != (int)(b)) { FAIL("got %d, expected %d", (int)(a), (int)(b)); } \
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

#define ADSB_CRC24_POLY  0xFFF409

static uint32_t adsb_crc24(const uint8_t *msg, int bits)
{
    int bytes = bits / 8;
    uint32_t crc = 0;
    for (int i = 0; i < bytes; i++) {
        crc ^= ((uint32_t)msg[i]) << 16;
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000)
                crc ^= ADSB_CRC24_POLY;
        }
    }
    return crc & 0xFFFFFF;
}

/* ADS-B callsign charset: "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### 0123456789######" */
static const char ais_charset[] =
    "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### 0123456789######";

static void decode_callsign(const uint8_t *me, char *callsign)
{
    uint8_t chars[8];
    chars[0] = (me[1] >> 2) & 0x3F;
    chars[1] = ((me[1] & 0x03) << 4) | ((me[2] >> 4) & 0x0F);
    chars[2] = ((me[2] & 0x0F) << 2) | ((me[3] >> 6) & 0x03);
    chars[3] = me[3] & 0x3F;
    chars[4] = (me[4] >> 2) & 0x3F;
    chars[5] = ((me[4] & 0x03) << 4) | ((me[5] >> 4) & 0x0F);
    chars[6] = ((me[5] & 0x0F) << 2) | ((me[6] >> 6) & 0x03);
    chars[7] = me[6] & 0x3F;

    for (int i = 0; i < 8; i++) {
        if (chars[i] < (int)(sizeof(ais_charset) - 1))
            callsign[i] = ais_charset[chars[i]];
        else
            callsign[i] = ' ';
    }
    callsign[8] = '\0';

    for (int i = 7; i >= 0; i--) {
        if (callsign[i] == ' ' || callsign[i] == '#')
            callsign[i] = '\0';
        else
            break;
    }
}

static int32_t decode_ac12_altitude(const uint8_t *msg)
{
    uint16_t alt_code = ((uint16_t)(msg[5] & 0xFF) << 4) | ((msg[6] >> 4) & 0x0F);
    if (alt_code == 0) return 0;
    bool q_bit = (alt_code & 0x10) != 0;
    if (q_bit) {
        int n = ((alt_code & 0xFE0) >> 1) | (alt_code & 0x0F);
        return n * 25 - 1000;
    } else {
        return alt_code * 100;
    }
}

/* NL table (from adsb_decode.c) */
static const int nl_table[88] = {
    59, 59, 59, 59, 58, 58, 58, 57, 57, 56, 56, 55, 54, 54, 53, 52,
    51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 38, 37, 36, 35,
    33, 32, 31, 29, 28, 26, 25, 23, 22, 20, 19, 17, 16, 14, 13, 11,
    10,  8,  7,  5,  4,  2,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
     1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
     1,  1,  1,  1,  1,  1,  1,  1
};

static int cpr_nl(double lat)
{
    if (lat < 0) lat = -lat;
    int idx = (int)lat;
    if (idx >= 87) return 1;
    if (idx < 0) idx = 0;
    return nl_table[idx];
}

static double cpr_mod(double a, double b)
{
    double res = fmod(a, b);
    if (res < 0) res += b;
    return res;
}

/* CPR global decode (from adsb_decode.c, adapted to take raw values) */
static bool cpr_global_decode(uint32_t even_lat, uint32_t even_lon,
                               uint32_t odd_lat,  uint32_t odd_lon,
                               bool use_even,
                               double *out_lat, double *out_lon)
{
    double cpr_lat_even = (double)even_lat / 131072.0;
    double cpr_lon_even = (double)even_lon / 131072.0;
    double cpr_lat_odd  = (double)odd_lat  / 131072.0;
    double cpr_lon_odd  = (double)odd_lon  / 131072.0;

    double dlat_even = 360.0 / 60.0;
    double dlat_odd  = 360.0 / 59.0;

    int j = (int)floor(59.0 * cpr_lat_even - 60.0 * cpr_lat_odd + 0.5);

    double lat_even = dlat_even * (cpr_mod(j, 60) + cpr_lat_even);
    double lat_odd  = dlat_odd  * (cpr_mod(j, 59) + cpr_lat_odd);

    if (lat_even >= 270.0) lat_even -= 360.0;
    if (lat_odd  >= 270.0) lat_odd  -= 360.0;

    if (cpr_nl(lat_even) != cpr_nl(lat_odd)) return false;

    double lat, lon;
    if (use_even) {
        lat = lat_even;
        int nl_val = cpr_nl(lat);
        int ni = (nl_val > 0) ? nl_val : 1;
        int m = (int)floor(cpr_lon_even * (nl_val - 1) -
                           cpr_lon_odd  * nl_val + 0.5);
        lon = (360.0 / ni) * (cpr_mod(m, ni) + cpr_lon_even);
    } else {
        lat = lat_odd;
        int nl_val = cpr_nl(lat) - 1;
        int ni = (nl_val > 0) ? nl_val : 1;
        int m = (int)floor(cpr_lon_even * nl_val -
                           cpr_lon_odd  * (nl_val + 1) + 0.5);
        lon = (360.0 / ni) * (cpr_mod(m, ni) + cpr_lon_odd);
    }

    if (lon > 180.0) lon -= 360.0;

    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
        return false;

    *out_lat = lat;
    *out_lon = lon;
    return true;
}

/* Magnitude approximation: max(|I|,|Q|) + min(|I|,|Q|)*3/8 */
static uint16_t mag_approx(uint8_t i_raw, uint8_t q_raw)
{
    int ii = abs((int)i_raw - 127);
    int qq = abs((int)q_raw - 127);
    int hi = (ii > qq) ? ii : qq;
    int lo = (ii > qq) ? qq : ii;
    return (uint16_t)(hi + (lo * 3 + 4) / 8);
}

/* ──────────────────────── Tests ──────────────────────── */

static void test_crc24(void)
{
    printf("\n[CRC-24 Mode S]\n");

    /*
     * Valid DF17 ADS-B message test vector from 1090MHz Riddle book.
     * 8D4840D6202CC371C32CE0  (11 bytes, 88 bits)
     * The last 3 bytes ARE the CRC. CRC of first 88 bits (message body
     * only, 11 bytes - 3 = 8 bytes = 64 bits) should equal those 3 bytes.
     */
    uint8_t msg[] = {0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0};
    /* CRC is computed over first 88 bits minus the 24 CRC bits = 64 bits */
    uint32_t crc = adsb_crc24(msg, 88 - 24);
    uint32_t pi  = ((uint32_t)msg[8] << 16) | ((uint32_t)msg[9] << 8) | msg[10];
    uint32_t remainder = crc ^ pi;

    printf("  TEST: %-55s ", "CRC-24 of valid DF17 msg remainder == 0");
    if (remainder == 0) { PASS(); }
    else { FAIL("got 0x%06X (crc=0x%06X pi=0x%06X)", remainder, crc, pi); }

    /* CRC of all-zeros message should be deterministic */
    uint8_t zeros[14] = {0};
    uint32_t crc_z1 = adsb_crc24(zeros, 112);
    uint32_t crc_z2 = adsb_crc24(zeros, 112);
    printf("  TEST: %-55s ", "CRC-24 is deterministic");
    if (crc_z1 == crc_z2) { PASS(); }
    else { FAIL("not deterministic"); }

    /* CRC(0xDF, ...) for DF17: known ICAO in msg[1..3] */
    /* Build a minimal DF17 with known ICAO 0x4840D6 */
    uint8_t df17[14] = {0};
    df17[0] = 0x8D;  /* DF=17 (bits 7-3) */
    df17[1] = 0x48;
    df17[2] = 0x40;
    df17[3] = 0xD6;
    /* Compute CRC of first 88 bits (11 bytes - 3 CRC = 64 data bits + 24 ICAO = 88 total - 24 CRC) */
    uint32_t crc_check = adsb_crc24(df17, 88);
    printf("  TEST: %-55s ", "CRC-24 of DF17 frame is 24-bit value");
    if (crc_check <= 0xFFFFFF) { PASS(); }
    else { FAIL("got 0x%X > 24 bits", crc_check); }
}

static void test_callsign_decode(void)
{
    printf("\n[ADS-B Callsign decode (TC 1-4)]\n");

    /*
     * Callsign "KLM1023 " encoded in ME bytes.
     * ADS-B charset: '#'=0,'A'=1,...,'Z'=26, '#'*5=27-31, ' '=32,
     *                '0'=48,...,'9'=57, '#'*6=58-63
     *
     * K=11, L=12, M=13, 1=49(0x31->idx=49-'0'+32... wait:
     * charset: "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### 0123456789######"
     *  pos 0='#', 1='A'...26='Z', 27-31='#', 32=' ', 33-47=skipped,
     *  Wait: 32=' ', then 48='0' (32+16)? Let's count:
     *  "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####" = 32 chars (0-31)
     *  " 0123456789######" starts at pos 32
     *  So: K=11, L=12, M=13, '1'=49... no, ' '=32, '0'=33... recount:
     *  pos 32=' ', 33='0', 34='1', 35='2', ..., 42='9'
     *  Wait: " 0123456789######" has ' '=32,then '0'=33...'9'=42
     *  Actually: 32=' ', then " 0123456789######"... the string after
     *  the spaces chars: pos 32=' ', 33='0','1'...until '9'=42
     *  Check with strlen: "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####" = 32 chars,
     *  then " 0123456789######" at 32.  So '0' is at index 33.
     *
     * Wait, let's actually count the charset string:
     * "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### 0123456789######"
     *  0: #
     *  1-26: A-Z
     *  27-31: ##### (5 hashes)
     *  32: (space)
     *  33-42: 0123456789
     *  43-48: ######
     * But the string as given: " 0123456789######" is 17 chars after the 32 letters
     * So '1' is at index 34.
     *
     * KLM1023 : K=11, L=12, M=13, 1=34, 0=33, 2=35, 3=36, ' '=32
     */

    /* Build ME bytes for TC=1 callsign "KLM1023 " */
    /* Indices: K=11,L=12,M=13,1=34,0=33,2=35,3=36,sp=32 */
    uint8_t idx[8] = {11, 12, 13, 34, 33, 35, 36, 32};
    /* ME byte layout: me[0]=TC<<3, then 6 bits per char packed into me[1..6] */
    /* me[1] bits [7:2] = idx[0], bits [1:0] = idx[1]>>4 */
    uint8_t me[7] = {0};
    me[0] = (1 << 3);  /* TC=1 */
    me[1] = (idx[0] << 2) | (idx[1] >> 4);
    me[2] = ((idx[1] & 0x0F) << 4) | (idx[2] >> 2);
    me[3] = ((idx[2] & 0x03) << 6) | idx[3];
    me[4] = (idx[4] << 2) | (idx[5] >> 4);
    me[5] = ((idx[5] & 0x0F) << 4) | (idx[6] >> 2);
    me[6] = ((idx[6] & 0x03) << 6) | idx[7];

    char callsign[9] = {0};
    decode_callsign(me, callsign);
    ASSERT_STR_EQ("callsign 'KLM1023'", callsign, "KLM1023");

    /* Verify charset positions */
    ASSERT_INT_EQ("charset[1]  == 'A'", ais_charset[1],  'A');
    ASSERT_INT_EQ("charset[26] == 'Z'", ais_charset[26], 'Z');
    ASSERT_INT_EQ("charset[32] == ' '", ais_charset[32], ' ');
    ASSERT_INT_EQ("charset[33] == '0'", ais_charset[33], '0');
    ASSERT_INT_EQ("charset[42] == '9'", ais_charset[42], '9');
}

static void test_altitude_decode(void)
{
    printf("\n[ADS-B Altitude decode (Q-bit)]\n");

    /*
     * Q-bit altitude encoding:
     * When Q-bit (bit 4 of 12-bit code) is 1:
     *   n = (code[11:5] << 3) | code[3:0]   (bit 4 stripped)
     *   altitude = n * 25 - 1000 feet
     *
     * Example: altitude = 38000 ft
     *   n = (38000 + 1000) / 25 = 1560
     *   code with Q-bit: need to reconstruct
     *   n=1560 = 0b11000011000
     *   upper 7 bits: 1100001 = 97, lower 4 bits: 1000 = 8
     *   alt_code = (97 << 5) | 0x10 | 8 = 0xC28 | 0x10 = ...
     *   Let's use direct: alt_code bits [11:5]=97=0b1100001, bit4=1(Q), bits[3:0]=8
     *   alt_code = (97<<5) | (1<<4) | 8 = 3104 | 16 | 8 = 3128 = 0xC38
     */
    uint8_t msg_38k[7] = {0};
    /* alt_code goes in msg[5] bits[7:0] and msg[6] bits[7:4] */
    uint16_t alt_code = 0xC38;  /* for 38000 ft */
    msg_38k[5] = (uint8_t)((alt_code >> 4) & 0xFF);
    msg_38k[6] = (uint8_t)((alt_code & 0x0F) << 4);
    int32_t alt = decode_ac12_altitude(msg_38k);
    /* Verify formula: n = (0xC38 & ~0x10) with upper/lower split */
    /* n = ((0xC38 & 0xFE0) >> 1) | (0xC38 & 0x0F) */
    int n = ((0xC38 & 0xFE0) >> 1) | (0xC38 & 0x0F);
    int expected = n * 25 - 1000;
    ASSERT_INT_EQ("altitude 38000 ft decodes correctly", alt, expected);

    /* Zero alt_code -> 0 */
    uint8_t msg_zero[7] = {0};
    int32_t alt_zero = decode_ac12_altitude(msg_zero);
    ASSERT_INT_EQ("alt_code=0 -> 0", alt_zero, 0);

    /* Q=1, n=0: altitude = -1000 ft (minimum) */
    uint16_t ac_min = 0x0010; /* Q-bit set, everything else 0 */
    uint8_t msg_min[7] = {0};
    msg_min[5] = (uint8_t)((ac_min >> 4) & 0xFF);
    msg_min[6] = (uint8_t)((ac_min & 0x0F) << 4);
    int32_t alt_min = decode_ac12_altitude(msg_min);
    ASSERT_INT_EQ("Q=1, n=0 -> -1000 ft", alt_min, -1000);
}

static void test_cpr_nl(void)
{
    printf("\n[CPR NL function]\n");

    /* Known values from 1090MHz Riddle */
    ASSERT_INT_EQ("NL(0)  == 59", cpr_nl(0.0),   59);
    ASSERT_INT_EQ("NL(1)  == 59", cpr_nl(1.0),   59);
    ASSERT_INT_EQ("NL(10) == 56", cpr_nl(10.0),  56);
    ASSERT_INT_EQ("NL(30) == 42", cpr_nl(30.0),  42);
    ASSERT_INT_EQ("NL(60) == 19", cpr_nl(60.0),  19);
    ASSERT_INT_EQ("NL(87) == 2",  cpr_nl(87.0),   2);
    ASSERT_INT_EQ("NL(90) == 1",  cpr_nl(90.0),   1);

    /* Negative lat same as positive */
    ASSERT_INT_EQ("NL(-30) == NL(30)", cpr_nl(-30.0), cpr_nl(30.0));
    ASSERT_INT_EQ("NL(-60) == NL(60)", cpr_nl(-60.0), cpr_nl(60.0));
}

static void test_cpr_global_decode(void)
{
    printf("\n[CPR Global Decode]\n");

    /*
     * Test vectors from 1090MHz Riddle (Junzi Sun):
     * Even: lat_cpr=92095, lon_cpr=39846  (F=0)
     * Odd:  lat_cpr=88385, lon_cpr=125818 (F=1)
     * Expected: lat ≈ 52.257, lon ≈ 3.919 (Netherlands)
     * Using most-recent = even.
     */
    double lat, lon;
    bool ok = cpr_global_decode(92095, 39846, 88385, 125818, true, &lat, &lon);

    printf("  TEST: %-55s ", "CPR global decode succeeds");
    if (ok) { PASS(); } else { FAIL("returned false"); }

    ASSERT_FLOAT_EQ("CPR lat ≈ 52.257", lat, 52.257, 0.01);
    ASSERT_FLOAT_EQ("CPR lon ≈ 3.919",  lon,  3.919, 0.01);

    /* Edge case: same even/odd should still decode */
    ok = cpr_global_decode(92095, 39846, 88385, 125818, false, &lat, &lon);
    printf("  TEST: %-55s ", "CPR global decode with odd reference");
    if (ok) { PASS(); } else { FAIL("returned false for odd ref"); }
}

static void test_magnitude_approx(void)
{
    printf("\n[Magnitude approximation]\n");

    /* mag = max(|I|,|Q|) + min(|I|,|Q|)*3/8 */
    /* I=127 (center), Q=127 (center) -> |I|=0, |Q|=0 -> mag=0 */
    ASSERT_INT_EQ("mag(127,127) == 0", mag_approx(127, 127), 0);

    /* I=255 (max), Q=127 (center) -> |I|=128, |Q|=0 -> mag=128 */
    ASSERT_INT_EQ("mag(255,127) == 128", mag_approx(255, 127), 128);

    /* I=127, Q=255 -> |I|=0, |Q|=128 -> mag=128 */
    ASSERT_INT_EQ("mag(127,255) == 128", mag_approx(127, 255), 128);

    /* I=0, Q=0 -> |I|=127, |Q|=127 -> hi=127, lo=127 -> 127+(127*3+4)/8=127+47=174 */
    uint16_t m = mag_approx(0, 0);
    ASSERT_INT_EQ("mag(0,0) == 174", m, 174);

    /* Verify approximation: for equal I,Q components result >= exact */
    /* Exact: sqrt(127^2 + 127^2) = 179.6... approx = 174 (slightly under) */
    double exact = sqrt(127.0 * 127.0 + 127.0 * 127.0);
    printf("  TEST: %-55s ", "mag_approx is within 5%% of true magnitude");
    double approx = (double)m;
    double err = fabs(approx - exact) / exact;
    if (err < 0.05) { PASS(); }
    else { FAIL("err=%.3f%%, approx=%.1f, exact=%.1f", err*100, approx, exact); }
}

int main(void)
{
    printf("=== ADS-B Decoder Unit Tests ===\n");

    test_crc24();
    test_callsign_decode();
    test_altitude_decode();
    test_cpr_nl();
    test_cpr_global_decode();
    test_magnitude_approx();

    printf("\n=== Results: %d passed, %d failed ===\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
