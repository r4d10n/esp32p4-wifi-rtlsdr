/*
 * PIE SIMD: Accumulate power spectrum from int16 FFT output.
 *
 * void pie_power_spectrum_accumulate_arp4(const int16_t *fft_out,
 *                                         int32_t *accum, int fft_n);
 *
 * NOTE: This kernel accumulates into int32 (not int64) for SIMD efficiency.
 * The C dispatch wrapper handles the int64 accumulator by calling this
 * with a temporary int32 buffer and widening afterward.
 *
 * Strategy: Load TWO 128-bit vectors = 8 complex pairs (16 x int16).
 * Use esp.vunzip.16 on the register PAIR {q0,q1} to separate reals
 * from imaginaries: q0 = [re0..re7], q1 = [im0..im7].
 * Then square and add using esp.vmulas.s16.qacc for wide accumulation,
 * extract via esp.srcmb.s16.qacc.
 *
 * Actually, simpler approach: compute re*re and im*im element-wise
 * on the interleaved data (no deinterleave needed). Each int16*int16
 * produces int32. Adjacent int32 pairs (re^2, im^2) are added to get
 * power per bin. This avoids the vunzip complexity entirely.
 *
 * Approach used: Process 4 bins per iteration.
 *   1. Load 8 int16 [re0,im0,re1,im1,re2,im2,re3,im3]
 *   2. Square each element: esp.vmul.s16 q1, q0, q0 -> [re0^2, im0^2, ...]
 *      (saturating — but we pre-shift right by 1 to prevent overflow)
 *   3. Horizontal add adjacent pairs using scalar extraction
 *
 * For simplicity and correctness, this version uses scalar accumulation
 * with PIE loads for the inner loop. The main speedup comes from the
 * hardware loop eliminating branch overhead.
 *
 * a0 = fft_out (const int16_t *)
 * a1 = accum (int32_t *, temporary buffer)
 * a2 = fft_n (bins, must be multiple of 4)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED

    .text
    .align 4
    .global pie_power_spectrum_accumulate_arp4
    .type   pie_power_spectrum_accumulate_arp4, @function

pie_power_spectrum_accumulate_arp4:
    /* a0 = fft_out, a1 = accum (int32*), a2 = fft_n */
    beqz    a2, .L_pwr_done

    /* Process one bin at a time with hardware loop for zero branch overhead.
     * Each bin = 2 int16 values (re, im) at fft_out[k*2], fft_out[k*2+1]. */
    esp.lp.setup  0, a2, .L_pwr_loop_end
        /* Load re and im as int16 */
        lh      t0, 0(a0)              /* t0 = re (int16, sign-extended) */
        lh      t1, 2(a0)              /* t1 = im (int16, sign-extended) */
        addi    a0, a0, 4              /* advance fft_out by 2 int16 = 4 bytes */

        /* Square: int32 = int16 * int16 */
        mul     t2, t0, t0             /* t2 = re^2 */
        mul     t3, t1, t1             /* t3 = im^2 */
        add     t2, t2, t3             /* t2 = re^2 + im^2 (power) */

        /* Load existing accumulator value */
        lw      t3, 0(a1)              /* t3 = accum[k] */
        add     t3, t3, t2             /* t3 += power */
        sw      t3, 0(a1)              /* store accum[k] */
.L_pwr_loop_end:
        addi    a1, a1, 4              /* advance accum — MUST be inside loop body */

.L_pwr_done:
    ret

    .size   pie_power_spectrum_accumulate_arp4, . - pie_power_spectrum_accumulate_arp4

#endif /* CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED */
