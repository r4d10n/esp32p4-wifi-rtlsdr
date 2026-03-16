/*
 * PIE SIMD: Accumulate power spectrum from int16 FFT output.
 *
 * STATUS: DISABLED — awaiting hardware validation of PIE instruction semantics.
 *
 * This assembly kernel is NOT linked into the build (PIE_ASM_POWER_SPECTRUM=0
 * in pie_kernels.c). The C reference with __attribute__((optimize("O3"))) is
 * used instead, which generates good scalar code.
 *
 * To re-enable, the following must be verified on real ESP32-P4 hardware:
 *
 *   1. esp.vunzip.16 q0, q1 — Research indicates this rearranges within a
 *      SINGLE register, not across a register pair:
 *        Input  q0: [a0, b0, a1, b1, a2, b2, a3, b3]
 *        Output q0: [a0, a1, a2, a3, b0, b1, b2, b3]
 *      This puts both re and im halves into the SAME register (q0 lower = re,
 *      q0 upper = im), NOT split across q0/q1 as the code below assumes.
 *      The kernel needs restructuring to account for this layout.
 *
 *   2. esp.vmul.s32.s16xs16 qd, qs1, qs2, qs3 — The 4th operand likely
 *      selects which half (low/high) of the 16-bit source to widen-multiply.
 *      Exact encoding must be confirmed against the PIE TRM or xesppie.S
 *      instruction catalog.
 *
 *   3. Accumulation is int32 here but the C reference uses int64 to avoid
 *      overflow with large averaging counts. A corrected assembly version
 *      would need widening accumulation or reduced averaging.
 *
 * The C fallback in pie_kernels.c is used instead (PIE_ASM_POWER_SPECTRUM=0).
 *
 * void pie_power_spectrum_accumulate_arp4(const int16_t *fft_out,
 *                                         int64_t *accum, int fft_n);
 *
 * Registers:
 *   a0 = fft_out (const int16_t *)
 *   a1 = accum (int64_t *)
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
