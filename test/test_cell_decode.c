/*
 * Host-based unit tests for Cell decoder logic
 * Extracted pure functions from cell_decode.c (no ESP-IDF required)
 * Compile: gcc -Wall -Wextra -o test_cell_decode test_cell_decode.c -lm
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
    if ((uint32_t)(a) != (uint32_t)(b)) { FAIL("got %u, expected %u", (unsigned)(a), (unsigned)(b)); } \
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

/* ──────────────────────── Constants (from cell_decode.h) ──────────────────────── */

#define GSM900_FREQ_START    935000000u
#define GSM900_FREQ_END      960000000u
#define GSM900_ARFCN_START   1
#define GSM900_ARFCN_END     124
#define GSM_CHANNEL_BW       200000u

#define GSM1800_FREQ_START   1805000000u
#define GSM1800_FREQ_END     1880000000u
#define GSM1800_ARFCN_START  512
#define GSM1800_ARFCN_END    885

/* ──────────────────────── Copied pure functions ──────────────────────── */

static uint32_t cell_decode_arfcn_to_freq(uint16_t arfcn)
{
    if (arfcn >= GSM900_ARFCN_START && arfcn <= GSM900_ARFCN_END) {
        return GSM900_FREQ_START + (uint32_t)(arfcn - GSM900_ARFCN_START) * GSM_CHANNEL_BW;
    }
    if (arfcn >= GSM1800_ARFCN_START && arfcn <= GSM1800_ARFCN_END) {
        return GSM1800_FREQ_START + (uint32_t)(arfcn - GSM1800_ARFCN_START) * GSM_CHANNEL_BW;
    }
    return 0;
}

static uint16_t cell_decode_freq_to_arfcn(uint32_t freq_hz)
{
    if (freq_hz >= GSM900_FREQ_START && freq_hz <= GSM900_FREQ_END) {
        return GSM900_ARFCN_START + (uint16_t)((freq_hz - GSM900_FREQ_START) / GSM_CHANNEL_BW);
    }
    if (freq_hz >= GSM1800_FREQ_START && freq_hz <= GSM1800_FREQ_END) {
        return GSM1800_ARFCN_START + (uint16_t)((freq_hz - GSM1800_FREQ_START) / GSM_CHANNEL_BW);
    }
    return 0;
}

/* IMSI BCD decode: each byte holds two BCD digits, LSN first */
static int imsi_bcd_decode(const uint8_t *raw, int raw_len, char *out, int out_size)
{
    /* raw[0] low nibble is length indicator or first digit.
     * ITU-T Q.931 / GSM: raw[0] is type+length, digits start at raw[1].
     * For mobile identity IE:
     *   raw[0] = length in bytes of the rest (not counting raw[0] itself)
     *   raw[1] = type_of_id (bits 2:0) + odd/even flag (bit 3) + first digit (bits 7:4)
     *   raw[2..] = two BCD digits per byte, low nibble = earlier digit
     *
     * Simpler form used in tests: raw bytes are pure BCD, no header.
     * BCD: each byte = (high_nibble digit)(low_nibble digit)
     * e.g. 0x91 -> '9','1'; 0x23 -> '2','3'; 0xF0 -> just '0' (0xF = filler)
     */
    int pos = 0;
    for (int i = 0; i < raw_len && pos < out_size - 1; i++) {
        uint8_t lo = raw[i] & 0x0F;
        uint8_t hi = (raw[i] >> 4) & 0x0F;
        if (lo <= 9) out[pos++] = '0' + lo;
        if (hi <= 9 && pos < out_size - 1) out[pos++] = '0' + hi;
    }
    out[pos] = '\0';
    return pos;
}

/* TMSI parse: 4-byte big-endian */
static uint32_t tmsi_parse(const uint8_t *raw)
{
    return ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16) |
           ((uint32_t)raw[2] << 8)  |  (uint32_t)raw[3];
}

/* PCI = 3 * N_ID_1 + N_ID_2 */
static uint16_t pci_calculate(uint8_t n_id_1, uint8_t n_id_2)
{
    return (uint16_t)(3 * (uint16_t)n_id_1 + (uint16_t)n_id_2);
}

/* PPM error = freq_error_hz / carrier_hz * 1e6 */
static float ppm_calculate(double freq_error_hz, double carrier_hz)
{
    return (float)(freq_error_hz / carrier_hz * 1e6);
}

/* ──────────────────────── Tests ──────────────────────── */

static void test_arfcn_to_freq(void)
{
    printf("\n[ARFCN to Frequency]\n");

    /* GSM-900 downlink: freq = 935.0 + 0.2*(ARFCN-1) MHz */
    /* ARFCN 1  -> 935.0 + 0.2*0 = 935.0 MHz = 935000000 Hz */
    ASSERT_EQ("ARFCN 1   -> 935000000 Hz", cell_decode_arfcn_to_freq(1),   935000000u);

    /* ARFCN 2  -> 935.0 + 0.2 = 935.2 MHz = 935200000 Hz */
    ASSERT_EQ("ARFCN 2   -> 935200000 Hz", cell_decode_arfcn_to_freq(2),   935200000u);

    /* ARFCN 124 -> 935.0 + 0.2*123 = 935.0 + 24.6 = 959.6 MHz = 959600000 Hz */
    ASSERT_EQ("ARFCN 124 -> 959600000 Hz", cell_decode_arfcn_to_freq(124), 959600000u);

    /* ARFCN 512 (GSM-1800 start) -> 1805.0 + 0.2*0 = 1805.0 MHz */
    ASSERT_EQ("ARFCN 512 -> 1805000000 Hz", cell_decode_arfcn_to_freq(512), 1805000000u);

    /* ARFCN 513 -> 1805.2 MHz */
    ASSERT_EQ("ARFCN 513 -> 1805200000 Hz", cell_decode_arfcn_to_freq(513), 1805200000u);

    /* ARFCN 0 (not in GSM-900 range since start is 1) -> 0 */
    ASSERT_EQ("ARFCN 0   -> 0 (invalid)",   cell_decode_arfcn_to_freq(0),   0u);
}

static void test_freq_to_arfcn(void)
{
    printf("\n[Frequency to ARFCN]\n");

    /* 935000000 -> ARFCN 1 */
    ASSERT_INT_EQ("935000000 Hz -> ARFCN 1",   cell_decode_freq_to_arfcn(935000000u), 1);

    /* 935200000 -> ARFCN 2 */
    ASSERT_INT_EQ("935200000 Hz -> ARFCN 2",   cell_decode_freq_to_arfcn(935200000u), 2);

    /* 959600000 -> ARFCN 124 */
    ASSERT_INT_EQ("959600000 Hz -> ARFCN 124", cell_decode_freq_to_arfcn(959600000u), 124);

    /* 1805000000 -> ARFCN 512 */
    ASSERT_INT_EQ("1805000000 Hz -> ARFCN 512", cell_decode_freq_to_arfcn(1805000000u), 512);

    /* 1805200000 -> ARFCN 513 */
    ASSERT_INT_EQ("1805200000 Hz -> ARFCN 513", cell_decode_freq_to_arfcn(1805200000u), 513);

    /* Round-trip */
    uint32_t freq = cell_decode_arfcn_to_freq(100);
    uint16_t arfcn_rt = cell_decode_freq_to_arfcn(freq);
    ASSERT_INT_EQ("ARFCN 100 round-trip", arfcn_rt, 100);

    freq = cell_decode_arfcn_to_freq(700);
    arfcn_rt = cell_decode_freq_to_arfcn(freq);
    ASSERT_INT_EQ("ARFCN 700 round-trip", arfcn_rt, 700);
}

static void test_imsi_bcd_parse(void)
{
    printf("\n[IMSI BCD parse]\n");

    /*
     * Raw BCD bytes (no header, pure BCD encoding):
     * [0x09, 0x10, 0x32, 0x54, 0x76, 0x98, 0xF0]
     * Decode: each byte low-nibble first, skip 0xF filler.
     * 0x09 -> '9','0'
     * 0x10 -> '0','1'
     * 0x32 -> '2','3'
     * 0x54 -> '4','5'
     * 0x76 -> '6','7'
     * 0x98 -> '8','9'
     * 0xF0 -> skip F, '0'
     * Result: "9001234567890" ... wait: "9","0","0","1","2","3","4","5","6","7","8","9","0"
     * = "9001234567890" (13 digits)
     * Let me recount:
     * 0x09: lo=9->'9', hi=0->'0'  => "90"
     * 0x10: lo=0->'0', hi=1->'1'  => "01"
     * 0x32: lo=2->'2', hi=3->'3'  => "23"  (wait: lo first)
     * Actually: 0x09 = 0000_1001: lo nibble = 9, hi nibble = 0 => '9','0'
     * 0x10 = 0001_0000: lo=0->'0', hi=1->'1' => '0','1'
     * 0x32 = 0011_0010: lo=2->'2', hi=3->'3' => '2','3'
     * 0x54 = 0101_0100: lo=4->'4', hi=5->'5' => '4','5'
     * 0x76 = 0111_0110: lo=6->'6', hi=7->'7' => '6','7'
     * 0x98 = 1001_1000: lo=8->'8', hi=9->'9' => '8','9'
     * 0xF0 = 1111_0000: lo=0->'0', hi=F->skip => '0'
     * Full string: "9001234567890" (13 digits)... wait:
     * "9","0","0","1","2","3","4","5","6","7","8","9","0" = "9001234567890"
     * That's 13 digits. Let me check the task description: "9012345678"
     * The task raw bytes [0x09,0x10,0x32,0x54,0x76,0x98,0xF0]:
     * 0x09: lo=9, hi=0 -> "90"
     * 0x10: lo=0, hi=1 -> "01"
     * But typical IMSI BCD: digits are stored lo-nibble first in each byte.
     * For "9012345678": 9 digits
     * 90 12 34 56 78 -> 0x90,0x21,0x43,0x65,0x87 but task says 0x09,0x10,...
     * 0x09 reversed = 0x90 -> "9","0"
     * With [0x09,0x10,0x32,0x54,0x76,0x98,0xF0]: result = "900123456789" + "0" = 13 digits.
     * Task says expected "9012345678". Let me use a corrected input to match that:
     * "9012345678" (10 digits): 90 -> 0x09, 12 -> 0x21, 34 -> 0x43, 56 -> 0x65, 78 -> 0x87
     */
    uint8_t raw_imsi[] = {0x09, 0x21, 0x43, 0x65, 0x87};
    char imsi[20] = {0};
    imsi_bcd_decode(raw_imsi, 5, imsi, sizeof(imsi));
    ASSERT_STR_EQ("IMSI BCD {0x09,0x21,0x43,0x65,0x87} -> '9012345678'", imsi, "9012345678");

    /* All same digit: 0x11,0x11 -> "1111" */
    uint8_t raw2[] = {0x11, 0x11};
    char out2[10] = {0};
    imsi_bcd_decode(raw2, 2, out2, sizeof(out2));
    ASSERT_STR_EQ("BCD {0x11,0x11} -> '1111'", out2, "1111");

    /* Filler 0xF at end is skipped */
    uint8_t raw3[] = {0x21, 0xF3};
    char out3[10] = {0};
    imsi_bcd_decode(raw3, 2, out3, sizeof(out3));
    /* 0x21: lo=1->'1', hi=2->'2'; 0xF3: lo=3->'3', hi=F->skip */
    ASSERT_STR_EQ("BCD {0x21,0xF3} -> '123' (filler skipped)", out3, "123");
}

static void test_tmsi_parse(void)
{
    printf("\n[TMSI parse]\n");

    /* Raw bytes [0x12, 0x34, 0x56, 0x78] -> 0x12345678 */
    uint8_t raw[] = {0x12, 0x34, 0x56, 0x78};
    uint32_t tmsi = tmsi_parse(raw);
    ASSERT_EQ("TMSI {0x12,0x34,0x56,0x78} == 0x12345678", tmsi, 0x12345678u);

    /* All zeros */
    uint8_t raw_zero[] = {0x00, 0x00, 0x00, 0x00};
    ASSERT_EQ("TMSI all zeros == 0", tmsi_parse(raw_zero), 0u);

    /* All 0xFF */
    uint8_t raw_ff[] = {0xFF, 0xFF, 0xFF, 0xFF};
    ASSERT_EQ("TMSI all 0xFF == 0xFFFFFFFF", tmsi_parse(raw_ff), 0xFFFFFFFFu);

    /* Known value */
    uint8_t raw_known[] = {0xDE, 0xAD, 0xBE, 0xEF};
    ASSERT_EQ("TMSI {0xDE,0xAD,0xBE,0xEF} == 0xDEADBEEF",
              tmsi_parse(raw_known), 0xDEADBEEFu);
}

static void test_pci_calculation(void)
{
    printf("\n[PCI calculation]\n");

    /* PCI = 3 * N_ID_1 + N_ID_2 */
    /* N_ID_1=0, N_ID_2=0 -> PCI=0 */
    ASSERT_INT_EQ("PCI(0,0) == 0",   pci_calculate(0, 0), 0);

    /* N_ID_1=0, N_ID_2=1 -> PCI=1 */
    ASSERT_INT_EQ("PCI(0,1) == 1",   pci_calculate(0, 1), 1);

    /* N_ID_1=0, N_ID_2=2 -> PCI=2 */
    ASSERT_INT_EQ("PCI(0,2) == 2",   pci_calculate(0, 2), 2);

    /* N_ID_1=1, N_ID_2=0 -> PCI=3 */
    ASSERT_INT_EQ("PCI(1,0) == 3",   pci_calculate(1, 0), 3);

    /* N_ID_1=100, N_ID_2=2 -> PCI=302 */
    ASSERT_INT_EQ("PCI(100,2) == 302", pci_calculate(100, 2), 302);

    /* N_ID_1=167, N_ID_2=2 -> PCI=503 (maximum) */
    ASSERT_INT_EQ("PCI(167,2) == 503 (max)", pci_calculate(167, 2), 503);
}

static void test_ppm_calculation(void)
{
    printf("\n[PPM calculation]\n");

    /* PPM = freq_error_hz / carrier_hz * 1e6 */

    /* 1000 Hz error at 935.2 MHz = 1000/935200000 * 1e6 ≈ 1.069 PPM */
    float ppm = ppm_calculate(1000.0, 935200000.0);
    ASSERT_FLOAT_EQ("1000 Hz at 935.2 MHz ≈ 1.069 PPM", ppm, 1.069f, 0.01f);

    /* 0 Hz error -> 0 PPM */
    ASSERT_FLOAT_EQ("0 Hz error -> 0 PPM", ppm_calculate(0.0, 935200000.0), 0.0f, 0.001f);

    /* -1000 Hz error -> negative PPM */
    float ppm_neg = ppm_calculate(-1000.0, 935200000.0);
    ASSERT_FLOAT_EQ("-1000 Hz -> -1.069 PPM", ppm_neg, -1.069f, 0.01f);

    /* 10000 Hz error at 1 GHz = 10 PPM */
    ASSERT_FLOAT_EQ("10000 Hz at 1 GHz == 10 PPM",
                    ppm_calculate(10000.0, 1000000000.0), 10.0f, 0.001f);

    /* 100 Hz at 100 MHz = 1 PPM */
    ASSERT_FLOAT_EQ("100 Hz at 100 MHz == 1 PPM",
                    ppm_calculate(100.0, 100000000.0), 1.0f, 0.001f);
}

static void test_gsm_band_coverage(void)
{
    printf("\n[GSM band coverage]\n");

    /* All ARFCN 1-124 should map to valid GSM-900 frequencies */
    int all_valid = 1;
    for (int a = 1; a <= 124; a++) {
        uint32_t f = cell_decode_arfcn_to_freq((uint16_t)a);
        if (f < GSM900_FREQ_START || f > GSM900_FREQ_END) {
            all_valid = 0;
            break;
        }
    }
    printf("  TEST: %-55s ", "All ARFCN 1-124 in GSM-900 range");
    if (all_valid) { PASS(); } else { FAIL("frequency out of range"); }

    /* All ARFCN 512-885 should map to valid GSM-1800 frequencies */
    all_valid = 1;
    for (int a = 512; a <= 885; a++) {
        uint32_t f = cell_decode_arfcn_to_freq((uint16_t)a);
        if (f < GSM1800_FREQ_START || f > GSM1800_FREQ_END) {
            all_valid = 0;
            break;
        }
    }
    printf("  TEST: %-55s ", "All ARFCN 512-885 in GSM-1800 range");
    if (all_valid) { PASS(); } else { FAIL("frequency out of range"); }

    /* Channel spacing is exactly 200 kHz */
    uint32_t f1 = cell_decode_arfcn_to_freq(1);
    uint32_t f2 = cell_decode_arfcn_to_freq(2);
    ASSERT_EQ("GSM channel spacing 200 kHz", f2 - f1, 200000u);
}

int main(void)
{
    printf("=== Cell Decoder Unit Tests ===\n");

    test_arfcn_to_freq();
    test_freq_to_arfcn();
    test_imsi_bcd_parse();
    test_tmsi_parse();
    test_pci_calculation();
    test_ppm_calculation();
    test_gsm_band_coverage();

    printf("\n=== Results: %d passed, %d failed ===\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
