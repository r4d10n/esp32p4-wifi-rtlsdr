#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LTE_PSS_LEN     63      /* Zadoff-Chu sequence length */
#define LTE_PSS_ROOTS   3       /* Number of PSS roots */
#define LTE_PSS_ROOT_0  25      /* N_ID_2 = 0 */
#define LTE_PSS_ROOT_1  29      /* N_ID_2 = 1 */
#define LTE_PSS_ROOT_2  34      /* N_ID_2 = 2 */

/* Pre-computed PSS templates */
typedef struct {
    float re[LTE_PSS_ROOTS][LTE_PSS_LEN];
    float im[LTE_PSS_ROOTS][LTE_PSS_LEN];
} lte_pss_templates_t;

/* PSS correlation result */
typedef struct {
    float    magnitude;      /* Correlation peak magnitude */
    float    phase;          /* Phase of correlation peak (radians) */
    int      offset;         /* Sample offset where peak found */
    uint8_t  n_id_2;         /* Best matching root (0, 1, or 2) */
} lte_pss_peak_t;

/* Generate all 3 PSS Zadoff-Chu templates */
void lte_pss_generate(lte_pss_templates_t *templates);

/* Cross-correlate IQ data against all 3 PSS sequences.
 * Input: float IQ pairs (re[], im[]) at the LTE sample rate.
 * Searches for PSS peaks and returns the best one.
 * search_step: correlation computed every N samples (4 typical)
 * Returns number of peaks found (0 = no LTE signal). */
int lte_pss_correlate(const float *iq_re, const float *iq_im, int iq_len,
                      const lte_pss_templates_t *templates,
                      int search_step,
                      lte_pss_peak_t *best_peak);

/* Estimate frequency offset from PSS correlation results.
 * Collects phase from multiple correlation peaks and computes
 * the average frequency error in Hz.
 *
 * Input: float IQ at sample_rate, searches for PSS peaks,
 *        averages phase across all peaks above threshold.
 * Returns: frequency error in Hz, 0 if no peaks found. */
float lte_pss_freq_error(const float *iq_re, const float *iq_im, int iq_len,
                         const lte_pss_templates_t *templates,
                         float sample_rate);

#ifdef __cplusplus
}
#endif
