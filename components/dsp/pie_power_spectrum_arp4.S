/*
 * PIE SIMD: Accumulate power spectrum from int16 FFT output to int32.
 *
 * void pie_power_spectrum_accumulate_arp4(const int16_t *fft_out,
 *                                         int32_t *accum, int fft_n);
 *
 * For each bin k: accum[k] += re[k]^2 + im[k]^2
 * Input: interleaved [re0,im0,re1,im1,...] int16
 * Output: int32 power per bin
 *
 * Processes 4 bins per iteration (loads 8 int16 = 4 complex pairs).
 * Uses esp.vmul.s32.s16xs16 for widening multiply.
 *
 * Registers:
 *   a0 = fft_out (const int16_t *)
 *   a1 = accum (int32_t *)
 *   a2 = fft_n (bins, must be multiple of 4)
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
    /* a0 = fft_out, a1 = accum, a2 = fft_n */
    /* fft_n must be >= 4 and multiple of 4 */
    srli    a2, a2, 2               /* loop count = fft_n / 4 */
    beqz    a2, .L_pwr_done

    esp.lp.setup  0, a2, .L_pwr_loop_end
        /* Load 4 complex pairs = 8 int16 values (128 bits) */
        esp.vld.128.ip  q0, a0, 16         /* q0 = [re0,im0,re1,im1,re2,im2,re3,im3] */

        /* Deinterleave: q0 = [re0,re1,re2,re3], q1 = [im0,im1,im2,im3] */
        esp.vunzip.16   q0, q1             /* q0=even(re), q1=odd(im) */

        /* Widening multiply: re*re -> int32 (4 elements) */
        esp.vmul.s32.s16xs16  q2, q0, q0, q0  /* q2 = re^2 as int32 */

        /* Widening multiply: im*im -> int32 (4 elements) */
        esp.vmul.s32.s16xs16  q3, q1, q1, q1  /* q3 = im^2 as int32 */

        /* Add: power = re^2 + im^2 */
        esp.vadd.s32    q2, q2, q3         /* q2 = power[0..3] */

        /* Load existing accumulator */
        esp.vld.128.ip  q4, a1, 0          /* q4 = accum[k..k+3] (don't advance yet) */

        /* Accumulate */
        esp.vadd.s32    q4, q4, q2         /* q4 += power */

        /* Store back */
.L_pwr_loop_end:
        esp.vst.128.ip  q4, a1, 16        /* store and advance */

.L_pwr_done:
    ret

    .size   pie_power_spectrum_accumulate_arp4, . - pie_power_spectrum_accumulate_arp4

#endif /* CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED */
