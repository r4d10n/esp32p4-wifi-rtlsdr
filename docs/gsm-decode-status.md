# GSM Cell ID Decoder — Detailed Context & Status

## Date: 2026-03-31

## Project
`/home/rax/exp/esp32/p4/host/esp32p4-gsm-kalibrate/` (branch: `feature/gsm-kalibrate`)

## Hardware
- ESP32-P4 (400 MHz dual-core RISC-V, 8 MB PSRAM)
- RTL-SDR Blog V4 (R828D tuner)
- Location: Kerala, India
- GSM900 band: 935-960 MHz (Airtel, BSNL, Jio, Vi)

## Current Architecture

```
RTL-SDR @ 1,083,334 sps (OSR=4 for 270,833 Hz GSM symbol rate)
  ↓
Stage 1: Wideband power scan (Goertzel at 5 sub-channels per 1 MHz step)
  ↓
Stage 2: FCCH validation (FM discriminator variance, noise-adaptive threshold)
  → PPM calibration: -1.41 PPM (σ=0.24)
  ↓
Stage 3: Cell ID decode attempt
  → DDC decim=4 → 270,833 sps (1 samp/sym, CIC anti-alias at ~135 kHz)
  → FCCH detection (airprobe accumulator: 100×OSR hits, 4×OSR max misses)
  → SCH search (±5 timeslots from FCCH, bit-domain training correlation)
  → Differential demod with soft decisions
  → SCH Viterbi K=5 decode → BCCH decode → SI3 parse
```

## Best Results

| Metric | Value |
|--------|-------|
| FCCH detection | Working on 2-5 channels per scan |
| SCH training correlation | **32/64 = 75%** (best channel) |
| SCH decode | **Fails** (needs ~80% for Viterbi K=5) |
| GSM PPM calibration | -1.41 PPM (σ=0.24) — production-ready |

## Demod Approaches Tested

| # | Approach | Train Corr | Notes |
|---|----------|-----------|-------|
| 1 | Diff demod at OSR=4 (decim=1) | 28-30/64 | No anti-alias, noise aliasing |
| 2 | Diff demod at 2 samp/sym (decim=2) | 26-30/64 | CIC too wide (270 kHz) |
| 3 | **Diff demod at 1 samp/sym (decim=4)** | **22-32/64** | **Best: CIC at 135 kHz** |
| 4 | MAFI + MLSE (correlation metric) | All zeros | Branch metric bias |
| 5 | MAFI + MLSE (Euclidean distance) | All zeros | CIR/rhh scale mismatch |
| 6 | 1-tap channel equalization + diff | 30/64 | Only fixes phase, not ISI |
| 7 | Direct phase tracking | 16/64 | Error cascade |
| 8 | GMSK mapper (Laurent) IQ correlation | 2-18/64 | Template doesn't match signal |
| 9 | GMSK mapper (Gaussian BT=0.3) IQ corr | 0-18/64 | Still doesn't match |

## Root Cause of 75% Ceiling

1. **GMSK ISI**: BT=0.3 Gaussian filter spreads each symbol over ~3 symbol periods.
   Differential demod can't handle this — it sees the convolved output, not the original bit.

2. **CIC filter is suboptimal**: The accumulate-and-dump CIC has sinc response with
   sidelobes that leak noise. A Gaussian matched filter (matched to BT=0.3 pulse)
   would give ~3 dB SNR improvement.

3. **No channel equalization**: Multipath and ISI are not compensated.
   The MLSE implementation has bugs (CIR estimation uses wrong GMSK model).

## Three Goals to Close the Gap

### Goal 1: Gaussian Matched Filter (replace CIC)
- Design a FIR lowpass filter matched to the GMSK pulse shape (BT=0.3)
- The Gaussian pulse has bandwidth ~200 kHz at -3dB
- At 1,083,334 sps: filter should be ~20-30 taps
- Apply before decimation to 1 samp/sym
- Expected gain: ~3 dB SNR → ~5% accuracy improvement

### Goal 2: Correct GMSK Pulse Model for CIR Estimation
- The Laurent approximation (instant ±90° phase) is wrong at OSR>1
- Need to generate the actual GMSK baseband pulse at the operating sample rate
- Use this for IQ-domain training correlation → correct CIR
- With correct CIR → MAFI + MLSE with Euclidean distance should work
- Expected gain: ISI compensation → ~10% accuracy improvement

### Goal 3: Polyphase Resampler (2 MSPS → 1,083,333 sps)
- Capture at 2 MSPS (stable, standard RTL-SDR rate)
- Polyphase resampler: ratio 13/24 (upsample 13, downsample 24)
- PIE SIMD: esp.vmulas.s16.xacc.ld.ip for fused MAC+load
- ~48-tap FIR filter per phase
- This combines anti-alias filtering with resampling in one step

## Key Files

| File | Lines | Purpose |
|------|-------|---------|
| gsm_kalibrate.c | ~1400 | Scan engine, Stage 1-3 |
| gsm_decode.c | ~1100 | Viterbi K=5, SCH, BCCH, SI3, GMSK-MLSE |
| gsm_decode.h | ~140 | Public API |
| lte_sync.c | ~800 | LTE PSS/SSS/CFO |
| lte_sync.h | ~65 | LTE sync API |

## Airprobe/gr-gsm Reference

The working pipeline from airprobe:
1. Resample to exactly 1,083,333 sps (OSR=4)
2. FCCH detection (phase-diff accumulator)
3. SCH at FCCH + 8 timeslots
4. GMSK-mapped training correlation (IQ domain) → CIR
5. MAFI matched filter (20-tap at OSR=4)
6. 16-state Viterbi MLSE → 148 bits
7. Conv decode → CRC → L2 frame

The key difference: airprobe uses a correct GMSK baseband pulse model (via IDFT
of the frequency-domain training) for the IQ correlation template. Our Laurent
approximation doesn't match the actual transmitted waveform.
