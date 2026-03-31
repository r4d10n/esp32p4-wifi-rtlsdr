# Polyphase Fractional Resampler Design for GSM on ESP32-P4

## Resampling Requirement

| Parameter | Value |
|-----------|-------|
| Input sample rate | 2,000,000 sps (RTL-SDR standard rate) |
| Output sample rate | 1,083,333.33 sps (4 x GSM symbol rate 270,833.33) |
| Ratio (in/out) | 2,000,000 / 1,083,333.33 = 1.846154 |
| Rational approximation | 24/13 (output/input = 13/24) |
| Polyphase decomposition | Upsample by 13, lowpass filter, downsample by 24 |

### Derivation of Rational Ratio

```
Output / Input = 1,083,333.33 / 2,000,000
               = 1,083,333.33 / 2,000,000
               ~ 13 / 24  (exact: 13,000,000 / 24,000,000)

Verification: 2,000,000 * 13 / 24 = 1,083,333.33  [correct]
```

The ratio 13/24 means: for every 24 input samples consumed, produce 13 output samples.

---

## Polyphase Resampler Architecture

### Conceptual Pipeline

```
Input (2 MSPS) --> [Upsample x13] --> [Lowpass FIR] --> [Downsample /24] --> Output (1.083 MSPS)
```

### Efficient Polyphase Implementation

No actual upsampling or zero-insertion occurs. Instead, the prototype lowpass filter of length L is decomposed into 13 polyphase subfilters, each of length ceil(L/13).

For each output sample n:
1. Compute the fractional input index: `idx = n * 24 / 13`
2. Integer part selects the input sample neighborhood
3. Fractional part (0..12) selects which of the 13 polyphase subfilters to use
4. Convolve the selected subfilter with the input samples around that position

```
Output sample computation:

  phase = (output_index * 24) mod 13        -- selects subfilter
  offset = (output_index * 24) / 13         -- integer input index

  y[n] = sum_{k=0}^{K-1} h_phase[k] * x[offset - k]

  where h_phase[] is the phase-th subfilter (K = ceil(L/13) taps)
```

### Block Diagram

```
                    Input buffer x[]
                         |
                    +-----------+
                    | Polyphase |
                    | Filter    |
                    | Bank      |
                    |           |
                    | phase 0:  h[0], h[13], h[26], ...
                    | phase 1:  h[1], h[14], h[27], ...
                    | phase 2:  h[2], h[15], h[28], ...
                    |   ...
                    | phase 12: h[12], h[25], h[38], ...
                    |           |
                    +-----------+
                         |
                    Output y[] at 1.083 MSPS
```

---

## Prototype Lowpass Filter Design

### Requirements

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Prototype filter rate | 13 x 2 MSPS = 26 MSPS (conceptual) |
| Passband edge | 541,666 Hz (half of output rate) |
| Stopband edge | 1,458,334 Hz (input_rate - passband) |
| Transition bandwidth | 916,668 Hz |
| Stopband attenuation | >= 60 dB |
| Passband ripple | < 0.1 dB |

### Filter Length Estimate

Using the Fred Harris rule of thumb:
```
N_taps ~ (att_dB * f_conceptual) / (22 * transition_bw)
       ~ (60 * 26,000,000) / (22 * 916,668)
       ~ 77 taps (prototype)

Per subfilter: ceil(77 / 13) = 6 taps
```

A practical choice is 48 prototype taps (with slight relaxation), giving ceil(48/13) = 4 taps per subfilter. This provides approximately 50 dB stopband attenuation with a Kaiser window (beta=5.6).

For better performance, 78 prototype taps (6 per subfilter) achieves > 60 dB.

### Recommended Configuration

```
Prototype filter length:  78 taps (6 per subfilter x 13 phases)
Window:                   Kaiser, beta = 6.0
Stopband attenuation:     ~65 dB
Passband ripple:          < 0.05 dB
```

---

## PIE SIMD Optimization Opportunities

Reference: [ESP32_P4_PIE_SIMD_GUIDE.md](ESP32_P4_PIE_SIMD_GUIDE.md)

### Data Format

Convert float IQ samples to Q15 (int16) before resampling, convert back after:
```
int16_t sample_q15 = (int16_t)(float_sample * 32767.0f);
```

This enables full use of PIE's 16-bit integer SIMD path.

### Key PIE Instructions

| Instruction | Use in Resampler |
|-------------|-----------------|
| `esp.vld.128.ip` | Load 8 x int16 input samples at once |
| `esp.vmulas.s16.xacc.ld.ip` | Fused multiply-accumulate + next load for FIR taps |
| `esp.lp.setup` | Hardware zero-overhead loop for filter kernel inner loop |
| `esp.srs.s16` | Shift and saturate accumulator result to int16 output |

### Inner Loop Structure (per output sample)

```asm
    ; Select polyphase subfilter (6 taps)
    ; Load subfilter coefficients into QR0 (8 slots, 6 used + 2 zero-padded)
    esp.vld.128.ip  q0, coeff_ptr, 16

    ; Load 8 input samples around the computed offset
    esp.vld.128.ip  q1, input_ptr, 16

    ; Clear accumulator
    esp.zero.xacc

    ; MAC: accumulate dot product of coefficients and input
    esp.vmulas.s16.xacc  q0, q1

    ; Extract result with rounding shift
    esp.srs.s16  result_reg, sar_reg
```

With only 6 taps per subfilter, a single SIMD MAC instruction computes the entire subfilter in one cycle (8-way parallelism covers 6 taps with 2 zero-padded slots).

### Dual-Channel (I + Q) Strategy

Process I and Q channels in parallel using two vector registers:
```
QR0: [i0, i1, i2, i3, i4, i5, 0, 0]   -- 6 I input samples
QR1: [q0, q1, q2, q3, q4, q5, 0, 0]   -- 6 Q input samples
QR2: [h0, h1, h2, h3, h4, h5, 0, 0]   -- subfilter coefficients

XACC = QR0 . QR2  --> I output
XACC = QR1 . QR2  --> Q output
```

Total: 2 MAC instructions + 3 loads per output sample.

---

## Estimated Performance

### Computation Budget

| Item | Value |
|------|-------|
| Subfilter taps | 6 (with 78-tap prototype, 13 phases) |
| MACs per output sample | 6 x 2 (I + Q) = 12 MACs |
| With PIE 8-way SIMD | 2 cycles (MAC) + 3 cycles (loads) = ~5 cycles/sample |
| Output sample rate | 1,083,333 sps |
| Total cycles/sec | 5 x 1,083,333 = 5.4M cycles/sec |
| CPU utilization at 400 MHz | 5.4M / 400M = **1.35%** |

With a longer 48-tap-per-subfilter design (for higher quality):

| Item | Value |
|------|-------|
| Subfilter taps | 4 (with 48-tap prototype) |
| Total cycles/sec | ~4M cycles/sec |
| CPU utilization at 400 MHz | **1.0%** |

Either way, the resampler is negligible CPU overhead on ESP32-P4.

### Memory Budget

| Item | Bytes |
|------|-------|
| Prototype filter coefficients (78 x int16) | 156 B |
| Polyphase subfilter table (13 x 8 x int16, zero-padded) | 208 B |
| Input ring buffer (~100 samples x 2 channels x int16) | 400 B |
| **Total** | **< 1 KB** |

---

## Alternative: Direct RTL-SDR at 1,083,334 Hz

The current implementation sets the RTL-SDR sample rate directly to 1,083,334 Hz, avoiding resampling entirely. This works and is the simplest approach.

### When Resampling Becomes Necessary

The polyphase resampler (input at 2 MSPS, output at 1.083 MSPS) should be implemented if any of the following apply:

1. **Crystal PPM drift**: RTL-SDR crystal oscillators have 1-50 ppm drift. At 1,083,334 Hz this is 1-54 Hz of sample rate error. At 2,000,000 Hz (a standard rate the RTL2832U PLL locks to more cleanly) the PLL jitter is lower and PPM correction is more predictable.

2. **Multi-standard scanning**: If the same RTL-SDR session needs to scan both GSM (needing 1.083 MSPS) and LTE (needing 1.92 MSPS), using 2 MSPS as a common capture rate avoids re-tuning the sample rate between scans. The resampler produces GSM-rate output from the common 2 MSPS capture.

3. **PLL jitter performance**: The R820T/RTL2832U combination may exhibit better phase noise at round sample rates (1, 2, 2.4 MSPS) compared to the non-standard 1,083,334 Hz. This would improve SNR for weak cell detection.

4. **Shared capture buffer**: A single 2 MSPS capture can be resampled to multiple rates for different analyses (GSM at 1.083 MSPS, LTE at 1.92 MSPS, wideband FFT at 2 MSPS) without re-capturing.

### Current Recommendation

Keep the direct 1,083,334 Hz approach as default. Implement the polyphase resampler as an optional path activated when 2 MSPS input is detected or explicitly requested. The CPU cost (< 2%) is negligible on ESP32-P4.
