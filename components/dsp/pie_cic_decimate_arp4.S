/*
 * PIE: 3rd-order CIC integrator inner loop with hardware zero-overhead loop.
 *
 * void pie_cic_integrate_arp4(const int16_t *iq_in, int in_pairs,
 *                              int32_t *accum);
 *
 * Runs the 3 sequential integrator stages at input rate using esp.lp.setup
 * for branch-free looping. Comb stages and decimation logic remain in C.
 *
 * accum layout (same as pie_cic_decimate_s16):
 *   [0,1]   = i1_re (int64 as lo,hi)
 *   [2,3]   = i1_im
 *   [4,5]   = i2_re
 *   [6,7]   = i2_im
 *   [8,9]   = i3_re
 *   [10,11] = i3_im
 *   [12..23] = comb state (untouched by this function)
 *   [24]     = counter (untouched)
 *
 * The integrator stages are inherently sequential (y[n] = y[n-1] + x[n]),
 * so SIMD vectorization is not possible. The optimization comes from:
 *   - Hardware zero-overhead loop (no branch penalty)
 *   - All 6 int64 integrator values kept in registers (no memory spills)
 *   - Efficient 64-bit add using add + sltu carry propagation
 *
 * Register allocation (callee-saved s0-s11 for integrator state):
 *   s0/s1   = i1_re lo/hi     s2/s3   = i1_im lo/hi
 *   s4/s5   = i2_re lo/hi     s6/s7   = i2_im lo/hi
 *   s8/s9   = i3_re lo/hi     s10/s11 = i3_im lo/hi
 *
 * Temporaries: t0=re, t1=im, t2=sign_ext, t3=carry, t4=old_lo
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED

    .text
    .align 4
    .global pie_cic_integrate_arp4
    .type   pie_cic_integrate_arp4, @function

pie_cic_integrate_arp4:
    /* a0=iq_in, a1=in_pairs, a2=accum */
    beqz    a1, .L_int_done

    /* Save callee-saved registers */
    addi    sp, sp, -52
    sw      s0,   0(sp)
    sw      s1,   4(sp)
    sw      s2,   8(sp)
    sw      s3,  12(sp)
    sw      s4,  16(sp)
    sw      s5,  20(sp)
    sw      s6,  24(sp)
    sw      s7,  28(sp)
    sw      s8,  32(sp)
    sw      s9,  36(sp)
    sw      s10, 40(sp)
    sw      s11, 44(sp)
    sw      ra,  48(sp)

    /* Load integrator state from accum into registers */
    lw      s0,   0(a2)            /* i1_re_lo */
    lw      s1,   4(a2)            /* i1_re_hi */
    lw      s2,   8(a2)            /* i1_im_lo */
    lw      s3,  12(a2)            /* i1_im_hi */
    lw      s4,  16(a2)            /* i2_re_lo */
    lw      s5,  20(a2)            /* i2_re_hi */
    lw      s6,  24(a2)            /* i2_im_lo */
    lw      s7,  28(a2)            /* i2_im_hi */
    lw      s8,  32(a2)            /* i3_re_lo */
    lw      s9,  36(a2)            /* i3_re_hi */
    lw      s10, 40(a2)            /* i3_im_lo */
    lw      s11, 44(a2)            /* i3_im_hi */

    /* Hardware zero-overhead loop over all input pairs */
    esp.lp.setup  0, a1, .L_int_loop_end

        /* Load int16 re and im, sign-extend to int32 */
        lh      t0, 0(a0)             /* t0 = re (sign-extended int16->int32) */
        lh      t1, 2(a0)             /* t1 = im */
        addi    a0, a0, 4

        /*
         * Integrator 1: i1 += x  (signed 64-bit add)
         *
         * For adding a sign-extended int32 to int64 stored as (lo, hi):
         *   new_lo = lo + x_lo          (x_lo = x, since x fits in int32)
         *   carry  = unsigned overflow of (lo + x_lo)
         *   new_hi = hi + x_hi + carry  (x_hi = sign extension = x >> 31)
         *
         * Carry detection for signed addend:
         *   if x >= 0: carry = (new_lo < old_lo)  i.e. sltu(new_lo, old_lo)
         *   if x <  0: carry = (new_lo >= old_lo)  i.e. NOT sltu, but we need borrow=-1
         *
         * Simpler unified approach using sign extension:
         *   hi += (x >> 31) + carry
         * where carry = (new_lo <u old_lo) when x>=0, or NOT(new_lo <u old_lo) adjusted.
         *
         * Actually the cleanest way: carry = (new_lo <u old_lo) XOR (x < 0)... no.
         *
         * Correct approach: standard 64-bit add.
         *   sign_ext = arithmetic right shift x by 31 (0 or -1)
         *   old_lo = lo
         *   lo = lo + x
         *   carry = (lo <u old_lo) if x>=0, else carry = (lo <=u old_lo)... still messy.
         *
         * Simplest correct method:
         *   old_lo = lo
         *   lo += x
         *   carry = sltu(lo, old_lo)  -- works ONLY if x >= 0
         *   For x < 0 (two's complement): lo += x where x is large unsigned
         *     e.g., x = -1 -> 0xFFFFFFFF, lo = 0 -> new_lo = 0xFFFFFFFF, no carry
         *     old_lo=0, new_lo=0xFFFFFFFF, sltu(0xFFF.., 0) = 0 -> but real carry is 0. Correct!
         *     e.g., x = -1 -> 0xFFFFFFFF, lo = 1 -> new_lo = 0, carry should be 1.
         *     old_lo=1, new_lo=0, sltu(0, 1) = 1. Correct!
         *     e.g., x = -1 -> 0xFFFFFFFF, lo = 0xFFFFFFFF -> new_lo = 0xFFFFFFFE, carry=1
         *     old_lo=0xFFFFFFFF, new_lo=0xFFFFFFFE, sltu(0xFFFFFFFE, 0xFFFFFFFF) = 1. Correct!
         *
         * So sltu(new_lo, old_lo) gives the UNSIGNED carry, and hi += sign_ext + carry
         * is the standard idiom for 64-bit add. This works for both positive and negative x.
         */

        /* i1_re += x_re */
        srai    t2, t0, 31            /* t2 = sign extension of re */
        mv      t4, s0                /* t4 = old i1_re_lo */
        add     s0, s0, t0            /* i1_re_lo += re */
        sltu    t3, s0, t4            /* t3 = carry */
        add     s1, s1, t2            /* i1_re_hi += sign_ext */
        add     s1, s1, t3            /* i1_re_hi += carry */

        /* i1_im += x_im */
        srai    t2, t1, 31
        mv      t4, s2
        add     s2, s2, t1
        sltu    t3, s2, t4
        add     s3, s3, t2
        add     s3, s3, t3

        /* Integrator 2: i2 += i1 (64-bit += 64-bit) */
        /* i2_re += i1_re */
        mv      t4, s4
        add     s4, s4, s0
        sltu    t3, s4, t4
        add     s5, s5, s1
        add     s5, s5, t3

        /* i2_im += i1_im */
        mv      t4, s6
        add     s6, s6, s2
        sltu    t3, s6, t4
        add     s7, s7, s3
        add     s7, s7, t3

        /* Integrator 3: i3 += i2 (64-bit += 64-bit) */
        /* i3_re += i2_re */
        mv      t4, s8
        add     s8, s8, s4
        sltu    t3, s8, t4
        add     s9, s9, s5
        add     s9, s9, t3

        /* i3_im += i2_im */
        mv      t4, s10
        add     s10, s10, s6
        sltu    t3, s10, t4
        add     s11, s11, s7
.L_int_loop_end:
        add     s11, s11, t3

    /* Store integrator state back to accum */
    sw      s0,   0(a2)
    sw      s1,   4(a2)
    sw      s2,   8(a2)
    sw      s3,  12(a2)
    sw      s4,  16(a2)
    sw      s5,  20(a2)
    sw      s6,  24(a2)
    sw      s7,  28(a2)
    sw      s8,  32(a2)
    sw      s9,  36(a2)
    sw      s10, 40(a2)
    sw      s11, 44(a2)

    /* Restore callee-saved registers */
    lw      s0,   0(sp)
    lw      s1,   4(sp)
    lw      s2,   8(sp)
    lw      s3,  12(sp)
    lw      s4,  16(sp)
    lw      s5,  20(sp)
    lw      s6,  24(sp)
    lw      s7,  28(sp)
    lw      s8,  32(sp)
    lw      s9,  36(sp)
    lw      s10, 40(sp)
    lw      s11, 44(sp)
    lw      ra,  48(sp)
    addi    sp, sp, 52

.L_int_done:
    ret

    .size   pie_cic_integrate_arp4, . - pie_cic_integrate_arp4

#endif /* CONFIG_IDF_TARGET_ESP32P4 && CONFIG_DSP_OPTIMIZED */
