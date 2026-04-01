# ESP32-P4 GSM/LTE Kalibrate — Full Session Context

## Session: 2026-03-30 to 2026-03-31

## Project
- **Repo**: `r4d10n/esp32p4-wifi-rtlsdr` branch `feature/gsm-kalibrate`
- **Path**: `/home/rax/exp/esp32/p4/host/esp32p4-gsm-kalibrate/`
- **Hardware**: ESP32-P4 (400 MHz RISC-V, 8 MB PSRAM) + RTL-SDR Blog V4 (R828D)
- **Location**: Kerala, India

## What Was Built (30+ commits)

### GSM FCCH PPM Calibration (Production-Ready)
- Wideband Goertzel power scan → narrowband FCCH validation
- Noise-adaptive FM discriminator variance threshold
- Hardware verified: **PPM = -1.41, stddev = 0.24**
- 2-5 BCCH channels detected per scan on GSM900

### LTE PSS/SSS Cell Identification
- FFT-domain PSS: 128-pt FFT, Zadoff-Chu correlation, IFO ±3 shift
- SSS: brute-force 168×2 m-sequence with channel equalization
- Multi-pair MMSE FOE: 40 pairs per 200ms capture
- Band 5 India (869-894 MHz): **4 real cells detected with PCI**
- PCI=421 @ 876.0 MHz (strongest, mag=51275)

### GSM Channel Decoder Infrastructure
- Viterbi K=5: 16-state, rate 1/2, G0=0x19, G1=0x1B, soft decision
- SCH burst decode: training correlation, parity check, BSIC extraction
- BCCH block decode: 4-burst de-interleave, Fire code CRC-40
- SI3 parser: BCD MCC/MNC, LAC, Cell ID
- GMSK-MLSE pipeline: mapper, CIR estimation, MAFI, 16-state Viterbi MLSE

### GSM Cell ID Decode (75% — Needs 80%)
- FCCH→SCH timing (airprobe approach): 8 timeslots after FCCH
- SCH training search: ±5 timeslots, bit-domain correlation
- Best result: **32/64 training correlation = 75% bit accuracy**
- Viterbi K=5 needs ~80% input accuracy for reliable decode

## All Demod Approaches Tested

| # | Approach | Train Corr | Notes |
|---|----------|-----------|-------|
| 1 | Diff demod, no filter, OSR=4 (decim=1) | 28-30/64 | Noise aliasing |
| 2 | Diff demod, CIC decim=2, 2 samp/sym | 26-30/64 | Wider noise BW |
| 3 | **Diff demod, CIC decim=4, 1 samp/sym** | **22-32/64** | **BEST (75%)** |
| 4 | Gaussian FIR (BT=0.3 pulse), decim=4 | 22-24/64 | Too narrow |
| 5 | Gaussian FIR (wider 240kHz), decim=2 | 24-30/64 | Same as CIC d=2 |
| 6 | MAFI + MLSE (correlation metric) | All zeros | Bias toward 0 |
| 7 | MAFI + MLSE (Euclidean distance) | All zeros | CIR/rhh scale mismatch |
| 8 | MAFI + MLSE (CIR not rhh) | All zeros | Scale 5× off |
| 9 | MAFI + MLSE (1-tap, rhh) | All zeros | MAFI 10× > expected |
| 10 | 1-tap channel equalization + diff | 30/64 | Only fixes phase |
| 11 | Direct phase tracking | 16/64 | Error cascade |
| 12 | De-rotation (known training phase) | 0-8/64 | Phase model wrong |

## Root Cause Analysis

### Why 75% is the Ceiling for Differential Demod
- GMSK BT=0.3 creates ~1.5 symbol ISI (Gaussian filter spreads each symbol)
- Differential demod sees the convolved output, not original bits
- At 1 samp/sym, there's no oversampling to resolve ISI
- Only MAFI (matched filter) equalization can handle ISI

### Why MAFI+MLSE Doesn't Work Yet
- CIR estimated from Laurent-approximated training IQ doesn't match actual signal
- The Laurent model (instant ±90° phase jumps) is exact at symbol instants but wrong between symbols
- At OSR=4, the smooth Gaussian transitions matter for correlation
- The scale mismatch: MAFI output ≈ 0.01, expected output ≈ 0.05-0.2 (depending on CIR vs rhh)
- Airprobe uses `j × diff_nrz × prev_output` for its GMSK mapper — different from our cumulative phase

### Why Anti-Alias Filtering Matters
- Without filtering (decim=1): full ±541 kHz noise bandwidth
- CIC at decim=4: ~135 kHz cutoff, reduces noise by ~4×
- This improved correlation from 28-30/64 to 22-32/64 (best=32)
- The CIC sinc response happens to match GSM ~180 kHz BW well

## Key Technical Findings

### LTE
- Band 20 (Europe) does NOT exist in India — all B20 "detections" were false positives
- Band 5 (869-894 MHz) is the correct Indian LTE FDD band
- PSS-SSS phase rotation CFO has ±7003 Hz ambiguity — wraps at the edge
- Multi-pair FOE (40 pairs from 200ms capture) gives best accuracy
- FFT-domain PSS with IFO correction is necessary for proper detection

### GSM
- Sample rate 1,083,334 sps = exactly OSR=4 for 270,833 Hz symbol rate
- FCCH detection: airprobe accumulator (count positive phase-diffs) + variance fallback
- SCH is 8 timeslots (1250 symbols) after FCCH, not 11
- CIC at decim=4 is optimal for noise filtering (tested 5 configurations)
- The 75% accuracy is ISI-limited, not noise-limited or timing-limited

## Files Created/Modified

### New Components
| File | Lines | Purpose |
|------|-------|---------|
| gsm_kalibrate.c | ~1500 | Scan engine, Stage 1-3, DDC, FCCH, filters |
| gsm_kalibrate.h | ~170 | Public API: 10 bands, scan control, HTTP |
| gsm_decode.c | ~1100 | Viterbi K=5, SCH, BCCH, SI3, GMSK-MLSE |
| gsm_decode.h | ~150 | Decoder API + MLSE |
| lte_sync.c | ~800 | FFT PSS, SSS m-sequences, CFO, multi-pair FOE |
| lte_sync.h | ~65 | LTE sync API |
| main.c | ~245 | Application wiring, auto-scan |

### Design Documents
| File | Size | Content |
|------|------|---------|
| PIE_CHANNEL_DECODE_DESIGN.md | 42 KB | PIE SIMD for Viterbi/turbo decode |
| resampling-design.md | ~8 KB | 13/24 polyphase resampler |
| lte-scanner-research.md | ~12 KB | Airprobe/gr-gsm/srsRAN analysis |
| gsm-decode-status.md | ~8 KB | Current decoder status |
| ESP32_P4_PIE_SIMD_GUIDE.md | ~30 KB | PIE instruction reference |

## Indian Cellular Environment
- **GSM900**: 935-960 MHz — Airtel, BSNL, Jio, Vi
- **LTE Band 5**: 869-894 MHz — BSNL (limited), some Airtel
- **LTE Band 3**: 1805-1880 MHz — primary LTE band (may exceed R828D range)
- **LTE Band 40/41**: TDD — not supported by current FDD code
- **MCC**: 404/405 (India)

## Next Steps for GSM Cell ID Decode

### Priority 1: Port Airprobe MAFI+MLSE Exactly
The airprobe/gr-gsm chain works because it uses:
1. `gmsk_mapper()`: `j × (diff_nrz, 0) × prev_output` — not cumulative phase
2. MAFI at OSR=4 with channel correlation from this specific mapper
3. Viterbi MLSE with `rhh` branch metrics (Ungerboeck formulation)
4. Hardcoded `strongest_window_nr = 3` for channel estimation

This is a precision port requiring matching every sign/scale convention.

### Priority 2: Validate with Simulated Signal
Generate a known GMSK burst in code, pass through the MAFI+MLSE,
verify it decodes correctly before testing on real signals.

### Priority 3: PIE SIMD Optimization
Once the scalar C decoder works, optimize the Viterbi K=5 and MLSE
using PIE SIMD (esp.vmin.s16 for 8-way ACS, hardware loops).

## Commit History (latest 15)

```
f274e32 fix: revert to differential demod — de-rotation failed
f910079 test: comprehensive filter comparison — CIC decim=4 wins
335573f test: Gaussian FIR worse than CIC — too narrow
793b414 test: decim=4 optimal — tested decim=1,2,4
65194d2 fix: CIC anti-alias, rhh/CIR scale analysis
a4148e2 fix: multi-tap MAFI+MLSE at symbol rate
5842c02 fix: revert to differential demod (phase tracking worse)
dc05561 feat: phase sweep demod + soft decisions
4303005 fix: relax FCCH accumulator for indoor SNR
77d06f7 fix: MLSE Euclidean distance + airprobe FCCH + resample design
bcea9e9 feat: FCCH→SCH timing, symbol-spaced diff, unified 1.083 MSPS
6d15f13 debug: SCH train_corr random — burst timing wrong
75291f6 fix: differential phase slicer produces real data!
1404915 debug: MLSE all-zero output root cause
c8a1e32 feat: GMSK-MLSE decoder — airprobe algorithm in scalar C
```
