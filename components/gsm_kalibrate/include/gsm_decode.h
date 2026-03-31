#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GSM constants */
#define GSM_BURST_LEN       148     /* Normal burst length in bits */
#define GSM_SCH_DATA_BITS   25      /* SCH decoded data bits */
#define GSM_SCH_ENC_BITS    78      /* SCH encoded bits (39 pairs) */
#define GSM_BCCH_ENC_BITS   456     /* BCCH encoded bits (4 bursts) */
#define GSM_BCCH_DATA_BITS  184     /* BCCH decoded data bits */
#define GSM_BCCH_BLOCK_LEN  23      /* BCCH L2 block length (bytes) */

/* Viterbi decoder constraint lengths */
#define GSM_SCH_K           5       /* SCH uses rate 1/2, K=5 (16 states) */
#define GSM_FIRE_CRC_LEN    40      /* Fire code CRC polynomial length */

/* GMSK-MLSE demodulation constants */
#define GSM_OSR             4       /* Oversampling ratio for GMSK decode */
#define GSM_CHAN_IMP_LEN    5       /* Channel impulse response length (symbols) */

/* SCH decoded info */
typedef struct {
    uint8_t  bsic;          /* Base Station Identity Code (6 bits: NCC + BCC) */
    uint8_t  ncc;           /* Network Colour Code (3 bits) */
    uint8_t  bcc;           /* Base Station Colour Code (3 bits) */
    uint16_t t1;            /* Frame number component T1 (11 bits) */
    uint8_t  t2;            /* Frame number component T2 (5 bits) */
    uint8_t  t3p;           /* Frame number component T3' (3 bits) */
    uint32_t fn;            /* Full frame number (computed from T1, T2, T3') */
    bool     valid;         /* CRC passed */
} gsm_sch_info_t;

/* Cell identity from System Information Type 3 */
typedef struct {
    uint16_t mcc;           /* Mobile Country Code (e.g., 404 = India) */
    uint16_t mnc;           /* Mobile Network Code (e.g., 45 = Airtel) */
    uint16_t lac;           /* Location Area Code */
    uint16_t cell_id;       /* Cell Identity */
    uint8_t  bsic;          /* BSIC from SCH */
    bool     valid;
} gsm_cell_id_t;

/* ── Viterbi K=5 decoder (GSM convolutional code) ── */

/**
 * Viterbi decode for GSM rate 1/2, K=5 convolutional code.
 * Generator polynomials: G0 = 0x19 (11001), G1 = 0x1B (11011)
 *
 * @param soft_bits   Input soft bits (int8: positive=1, negative=0, magnitude=confidence)
 * @param n_enc_bits  Number of encoded bits (must be even -- rate 1/2)
 * @param out_bits    Output decoded bits (packed bytes, MSB first)
 * @param out_len     Output buffer size in bytes
 * @return Number of decoded bits, or -1 on error
 */
int gsm_viterbi_k5_decode(const int8_t *soft_bits, int n_enc_bits,
                          uint8_t *out_bits, int out_len);

/* ── SCH burst decode ── */

/**
 * Detect and decode the SCH (Synchronization Channel) burst.
 * The SCH contains BSIC and frame number encoded with rate 1/2, K=5 convolutional code.
 *
 * Input: 148 soft bits of a SCH burst (after GMSK demodulation).
 * The SCH burst structure: 3 tail + 39 encoded[0] + 64 training + 39 encoded[1] + 3 tail + 8.25 guard
 *
 * @param burst_bits  148 soft demodulated bits
 * @param info        Output SCH info (BSIC, frame number)
 * @return true if decode succeeded (parity check passed)
 */
bool gsm_sch_decode(const int8_t *burst_bits, gsm_sch_info_t *info);

/* ── GMSK-MLSE demodulation ── */

/**
 * Full GMSK-MLSE demodulation pipeline (airprobe/gr-gsm algorithm).
 * Correlates training sequence, estimates channel, matched-filters,
 * then runs 16-state Viterbi MLSE equalization.
 *
 * Input IQ must be at exactly GSM_OSR (4) samples per symbol.
 * Set RTL-SDR to 1,083,334 sps for this rate.
 *
 * @param iq_re, iq_im  Float IQ samples at OSR=4 samples/symbol
 * @param n_samples     Number of IQ samples
 * @param soft_out      Output soft bits (int8, +/-127 range), 148 per burst
 * @param max_out       Maximum output soft bits
 * @return Number of soft bits produced (148 per detected burst, 0 if none)
 */
int gsm_gmsk_demod(const float *iq_re, const float *iq_im, int n_samples,
                   int8_t *soft_out, int max_out);

/* ── BCCH block decode ── */

/**
 * Decode a BCCH block from 4 normal bursts.
 * Performs: de-interleaving -> Viterbi K=5 -> Fire code CRC check -> L2 frame
 *
 * @param burst_bits  Array of 4 bursts, each 116 soft data bits
 *                    (57 + 57 from normal burst, excluding training/tail/guard)
 * @param l2_frame    Output L2 frame (23 bytes)
 * @return true if Fire code CRC passed
 */
bool gsm_bcch_decode(const int8_t burst_bits[4][116], uint8_t l2_frame[GSM_BCCH_BLOCK_LEN]);

/* ── Viterbi MLSE equalizer (16-state, Euclidean distance) ── */

/**
 * 16-state Viterbi MLSE equalizer for GMSK channel equalization.
 * Operates on symbol-rate matched-filter output.
 *
 * @param mf_re, mf_im  Matched filter output (n_sym complex samples)
 * @param n_sym          Number of symbols (148 for a burst)
 * @param cir_re, cir_im CIR or rhh (GSM_CHAN_IMP_LEN taps, normalized)
 * @param out_bits       Output hard bits (0 or 1)
 */
void gsm_viterbi_mlse(const float *mf_re, const float *mf_im,
                      int n_sym,
                      const float *cir_re, const float *cir_im,
                      uint8_t *out_bits);

/* ── System Information parsing ── */

/**
 * Parse System Information Type 3 from a BCCH L2 frame.
 * SI3 contains: Cell Identity, LAI (MCC, MNC, LAC), and cell options.
 *
 * @param l2_frame  23-byte L2 frame (decoded BCCH block)
 * @param cell_id   Output cell identity
 * @return true if the frame is SI3 and was parsed successfully
 */
bool gsm_parse_si3(const uint8_t l2_frame[GSM_BCCH_BLOCK_LEN], gsm_cell_id_t *cell_id);

/**
 * Parse System Information Type 1 from a BCCH L2 frame.
 * SI1 contains: Cell Allocation (CA) -- the list of ARFCNs used by the cell.
 */
bool gsm_parse_si1(const uint8_t l2_frame[GSM_BCCH_BLOCK_LEN], uint16_t *arfcn_list, int *n_arfcns);

#ifdef __cplusplus
}
#endif
