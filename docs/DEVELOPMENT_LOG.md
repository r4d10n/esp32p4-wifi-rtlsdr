# ESP32-P4 Standalone FM Radio — Complete Development Log

## Project Overview

ESP32-P4 based standalone FM radio receiver using RTL-SDR V4 (R828D tuner + RTL2832U demod) over USB 2.0 HS. Demodulates WBFM with stereo pilot detection, RDS decode, I2S speaker output (ES8311 codec), and WebSocket browser audio streaming over WiFi 6.

**Hardware:** Waveshare ESP32-P4-WIFI6 (~$30) + RTL-SDR Blog V4 (~$30)
**Transmitter:** ADALM-PLUTO SDR running `fm_stereo_rds_tx.py` at 100 MHz

---

## Branch History

| Branch | Base | Purpose | Status |
|---|---|---|---|
| `feature/websdr` | `main` | RTL-SDR USB→WiFi bridge (IQ streaming) | Working |
| `feature/standalone` | `feature/websdr` | SIMD Q15 FM radio (original attempt) | Broken (9 unfixed issues) |
| `standalone-mono-nonopt` | `feature/websdr` | Plain C float FM radio (fresh start) | **Working (v0.6.0-squelch)** |
| `standalone-optimized` | `standalone-mono-nonopt` | -O3 optimization attempt | Abandoned (stereo blend + compiler issues) |

---

## Tags (on `standalone-mono-nonopt`)

| Tag | Commit | Key Achievement |
|---|---|---|
| `v0.1.0-mono-minimal` | `a23b317` | Mono WBFM, 100% fill, plain C float |
| `v0.2.0-stereo` | `b520db6` | Stereo pilot + PLL + L-R decode |
| `v0.3.0-quality` | `904534b` | THD 0.08%, SNR 51 dB, 2-stage decimation |
| `v0.4.0-rds` | `d3c61e9` | RDS on-air: PS="MY RADIO" PI=0x1234 |
| `v0.5.0-polished` | `c7119cf` | Cascaded notch, slow AGC, 34 dB music SNR |
| `v0.6.0-squelch` | `e9826b3` | Noise squelch, **final working version** |

---

## v0.6.0-squelch Performance (Final)

| Metric | Value | Target | Status |
|---|---|---|---|
| THD (1kHz tone) | 0.098% | <0.5% | PASS |
| SNR (1kHz tone) | 49.8 dB | >40 dB | PASS |
| SNR (music) | 34.0 dB | >25 dB | PASS |
| 19kHz pilot | -46 dB | <-30 dB | PASS |
| Squelch (no signal) | RMS=0.000 | Silent | PASS |
| RDS groups (120s) | 1237 | 200 | PASS (6x) |
| RDS PS decode | "MY RADIO" | Non-empty | PASS |
| RDS PI code | 0x1234 | Correct | PASS |
| Audio Peak | 68% | >50% | PASS |
| Audio RMS | 44% | >10% | PASS |
| CPU Core 1 | 77% | <100% | PASS |
| Fill rate | 240% | >100% | PASS |
| WS reconnects | 0 | 0 | PASS |

---

## Signal Path

```
RTL-SDR USB (256 kSPS uint8 IQ)
    |
    v
[USB Host Controller] → iq_data_cb() → Ring Buffer (128 KB PSRAM)
    |
    v (Core 1 — fm_pipeline_task)
[U8→float] → (iq - 127.5) / 127.5
    |
    v
[FM Discriminator] → fast_atan2f(cross, dot) × fm_scale
    |                  9th-order minimax polynomial
    |                  <0.001° error, ~20 cycles
    v
[Goertzel 19kHz] → pilot detection (1024-sample blocks)
    |
[PLL] → uint32 phase accumulator, PI loop filter
    |
    +--→ [38kHz L-R demod] → sin_u32(phase<<1) × MPX × 2
    |                         (disabled — PLL phase error causes 41% THD)
    |
    +--→ [57kHz RDS mixing] → sin_u32(phase×3) × MPX
    |                          fractional A&D → 9500 sps
    |                          → matched filter + clock recovery PLL
    |                          → Manchester decode → CRC-10 → groups
    v
[2:1 Decimation] → 256 kSPS → 128 kSPS (box average)
    |
    v
[31-tap FIR LPF] → 15 kHz cutoff at 128 kSPS
    |                Nuttall window, transition band 4.1 kHz
    v
[Cascaded 19kHz Notch] → 2× 2nd-order IIR (r=0.95)
    |                      -80 dB+ pilot rejection
    v
[De-emphasis IIR] → 75us (NA), alpha = 1-exp(-1/(tau×fs))
    |
    v
[Squelch Gate] → IQ signal_rms threshold (0.20/0.25 hysteresis)
    |              Mutes audio when no signal present
    v
[AGC] → Peak tracker, target 70% FS
    |    Attack 100ms, release 2s, max gain 10×
    |    Frozen when squelch closed
    v
[Linear Resampler] → 128 kSPS → 48 kHz
    |
    v
[Volume] → × vol × 32767 → int16
    |
    +--→ [I2S DMA] → ES8311 codec → NS4150B amp → Speaker
    |
    +--→ [WebSocket] → 9600-sample buffer → TLS binary frames
                        → Browser AudioWorklet / paplay
```

---

## Critical Bugs Found and Fixed

### Phase 1: Original `feature/standalone` Branch (SIMD Q15)

| # | Bug | Impact | Root Cause |
|---|---|---|---|
| 1 | Noise blanker `(avg>>8)*20` threshold | **100% samples blanked** → silence | Threshold 1860 vs magnitude 24000 |
| 2 | FM discriminator `cross<<14` int32 overflow | Garbage atan2 output | Products exceed 2^31 for IQ>±5000 |
| 3 | WebSocket ~200 frames/sec | TLS drops every 2.5s | mbedTLS can't sustain binary frame rate |
| 4 | Discriminator scale 16384 (50% Q15) | -6 dB wasted headroom | Conservative mapping |
| 5 | RDS 57kHz phase constant per block | RDS decode impossible | Mixing used stale post-loop phase_acc |
| 6 | PLL lock detector `|pd_i|` always true | False stereo detection | L+R audio dominated the metric |
| 7 | RDS CRC-10 shift register wrong | Sync never matches | Polynomial implementation error |
| 8 | RDS no anti-alias before decimation | Noise buries RDS signal | Naive every-Nth sampling |
| 9 | RDS no differential BPSK decode | Wrong bit polarity | Missing XOR of successive symbols |
| 10 | RDS block_idx=0 after sync | Every block checks wrong offset | Should be 1 (next expected is B) |
| 11 | VLA 16KB on 8KB stack | Stack overflow crash | alloca(4096×3×sizeof(float)) |
| 12 | Stereo LPF 31 taps at 256kSPS | 19kHz pilot leaks through | Transition band 8.3 kHz too wide |

### Phase 2: `standalone-mono-nonopt` Branch (Plain C Float)

| # | Bug | Impact | Root Cause |
|---|---|---|---|
| 13 | CIC float integrator precision loss | Audio zeros after ~1s | Float32 mantissa (24-bit) overflows |
| 14 | CIC double precision on single FPU | 118% CPU (overloaded) | Double is software-emulated on ESP32-P4 |
| 15 | 15-tap FIR at 256kSPS | 17kHz transition band, pilot leaks | Insufficient taps for 19kHz rejection |
| 16 | malloc/free per DSP block | Jittery audio | Heap fragmentation + latency spikes |
| 17 | I2S write timeout=0 drops audio | Gaps in speaker output | Non-blocking drops when DMA full |
| 18 | RDS at 2375 sps (1 samp/symbol) | No matched filter possible | Need 9500 sps (4 samp/half-symbol) |
| 19 | RDS A&D normalized (÷count) | Signal too weak for sync | 27× amplitude loss |
| 20 | TX differential encoding reset per group | 8.1% block errors | State should carry across groups |
| 21 | AGC instant attack | Gain pumping on music | Need 100ms+ attack time |
| 22 | No squelch | Amplified noise when no signal | AGC boosts noise to full scale |

### Phase 3: `standalone-optimized` Branch (Abandoned)

| # | Bug | Impact | Root Cause |
|---|---|---|---|
| 23 | `-Ofast` breaks atan2 polynomial | THD 0.099%→0.355% | `-ffast-math` reorders float ops |
| 24 | `-O3`/`-O2` breaks IIR notch filter | THD→142-350% | Compiler reorders recursive feedback |
| 25 | Stereo blend activates → 41% THD | Unusable audio | PLL 2× phase error in 38kHz L-R ref |
| 26 | THD measurements timing-dependent | False "good" results | Blend ramps after boot → measures mono first |

---

## Key Design Decisions

### 1. 256 kSPS Direct Sampling (No CIC)
At 256 kSPS, FM bandwidth (±75 kHz = 150 kHz) fits within Nyquist (128 kHz). No CIC decimation needed, eliminating precision issues and saving CPU.

### 2. 2-Stage Decimation (256k→128k→48k)
The 15 kHz FIR cutoff needs the 19 kHz pilot in the stopband. At 256 kSPS, the transition band is too wide for reasonable tap counts. Decimating 2:1 to 128 kSPS makes a 31-tap FIR adequate (transition band = 4.1 kHz).

### 3. Plain C Float (No SIMD)
The SIMD Q15 pipeline had too many integer overflow, precision, and scaling bugs. Float eliminated all of them. The ESP32-P4's hardware FPU (rv32imafc) makes float operations reasonably fast.

### 4. Cascaded 19kHz Notch
Single-stage 2nd-order IIR gives ~-40 dB. Cascading two identical stages gives -80 dB+ rejection. The 0.8 dB passband effect is acceptable since de-emphasis already attenuates 15 kHz by -17 dB.

### 5. IQ-Level Squelch (Not Audio-Level)
Audio variance squelch failed because de-emphasis reduces variance below any threshold. IQ signal_rms (sqrt of I²+Q²) directly measures RF energy — low with no signal, high with FM signal.

### 6. Fractional A&D for RDS
Integer decimation (256000/107=2392 sps) has 0.73% rate mismatch vs 2375 symbol rate. Fractional phase accumulator (`phase_inc = 9500/256000`) gives exact rate with zero drift.

### 7. RDS Raw Sum (Not Average)
Dividing A&D output by sample count reduces amplitude 27×. The RDS decoder only needs sign detection — maximum amplitude gives maximum SNR for the matched filter + clock recovery PLL.

### 8. Stereo Disabled
The PLL tracks the 19 kHz pilot accurately, but the 2× phase multiplication for the 38 kHz L-R reference doubles the phase error. Even small PLL jitter causes massive L+R→L-R crosstalk (41% THD). Needs a Costas loop or separate 38 kHz PLL to fix.

---

## Compiler Optimization Warning

**All optimization levels above -Os break the IIR notch filter:**

| Flag | THD | Issue |
|---|---|---|
| `-Os` (default) | **0.098%** | Works correctly |
| `-O2` | 103-162% | Notch filter destabilized |
| `-O3` | 142% | Same issue, worse |
| `-Ofast` | 0.355% | atan2 polynomial affected too |

The cascaded 2nd-order IIR notch has recursive feedback (`y[n] depends on y[n-1], y[n-2]`). Higher optimization levels reorder these operations, changing the effective filter response. This is a known issue with IIR filters under aggressive optimization — the mathematical equivalence of reordered operations doesn't hold for finite-precision float arithmetic.

**Workaround:** Use `-Os` for the DSP file, or mark the notch variables as `volatile` (not tested).

---

## RDS Decoder Architecture

Based on analysis of redsea, gr-rds, and SDR++ implementations:

```
57kHz mixing (coherent, 3× PLL phase)
    |
    v
Fractional A&D (9500 sps output, exact rate)
    |
    v
8-tap Biphase Matched Filter
    [+1,+1,+1,+1,-1,-1,-1,-1] correlator
    +9 dB SNR gain over raw sampling
    |
    v
Zero-Crossing Clock Recovery PLL
    PI loop: Bn=5 Hz, zeta=1.0 (critically damped)
    Compensates decimation rate mismatch + drift
    |
    v
Manchester Symbol Pairing
    First half → diff_bit, second half → verify
    |
    v
Differential Decode
    data = (diff_bit == prev_diff_bit) ? 1 : 0
    |
    v
CRC-10 GF(2) Long Division
    Syndrome check vs offset words (A/B/C/C'/D)
    |
    v
Block Sync + Group Assembly
    block_idx=1 after sync (next expected is B)
    Lose sync after 20 consecutive bad blocks
    |
    v
Group Decode
    0A: PS name (2 chars/group, 4 segments)
    2A: RadioText (4 chars/group)
    4A: Clock Time
```

**TX Bug Fixed:** Differential encoding state was reset per group. IEC 62106 requires continuous state across groups. Fixed by carrying `_diff_prev` in the RDSEncoder class.

---

## CPU Budget (v0.6.0-squelch @ 256 kSPS)

```
Block = 16384 bytes = 8192 IQ pairs = 32ms @ 256 kSPS
Demod time: ~24,500 us/block (77% CPU)

Breakdown:
  atan2f (9th-order poly):  ~10,800 us (44%)
  FIR 31-tap @ 128kSPS:     ~4,700 us (19%)
  PLL + sin_u32 × 3:        ~2,500 us (10%)
  RDS (57kHz mix + A&D):    ~1,200 us (5%)
  Notch + deemph + AGC:     ~1,000 us (4%)
  Resample + volume:           ~800 us (3%)
  U8→float + squelch:         ~500 us (2%)
  Other (Goertzel, blend):   ~3,000 us (13%)
```

---

## Memory Usage

| Region | Total | Used | Free |
|---|---|---|---|
| Internal SRAM | 768 KB | ~245 KB | ~523 KB |
| PSRAM | 32 MB | ~450 KB | ~31.6 MB |
| Flash | 16 MB | 1.2 MB | ~14.8 MB |

---

## Test Commands

```bash
# Flash working firmware
cd /home/rax/exp/esp32/p4/host/esp32p4-standalone-mono-nonopt
source ~/esp/v5.5.1/esp-idf/export.sh
idf.py -p /dev/ttyACM1 flash

# Start FM TX (1kHz tone for THD measurement)
cd /home/rax/exp/esp32/p4/host/tools
python3 fm_stereo_rds_tx.py --freq 100.0 --left-freq 1000 --right-freq 1000 \
    --ps "TESTTONE" --rt "1kHz test" --attn 0 --loop

# Start FM TX (music + RDS)
python3 fm_stereo_rds_tx.py --freq 100.0 --ps "MY RADIO" \
    --rt "Now playing Ezhilam" --loop \
    --audio ~/exp/pluto/workshop/Ezhilam.wav --attn 0

# Capture + play audio
cd /home/rax/exp/esp32/p4/host/esp32p4-standalone
python3 test/test_browser_audio.py --host 192.168.1.233 --freq 100.0 --gain 496

# Run host DSP reference (validates all blocks)
python3 test/host_dsp_reference.py

# Run RDS loopback test
python3 test/test_rds_loopback.py

# Play saved audio
paplay /tmp/fm_music.wav
```

---

## Future Work

1. **Stereo:** Fix PLL for clean 38kHz L-R extraction (Costas loop or dual-PLL)
2. **RDS reliability:** Reduce false syncs (67.6% accuracy → 90%+), add CRC error correction
3. **SIMD FIR:** Replace float FIR inner loop with PIE `esp.vmulas.s16.xacc` (Q15 MAC, 8/cycle)
4. **SIMD atan2:** Batch conjugate multiply with `esp.cmul.s16` (cross/dot are data-parallel)
5. **Browser UI:** Add stereo badge, RDS display, MPX spectrum visualization
6. **Web radio API:** Add /api/rds endpoint to expose PS/RT/PI to browser
7. **Volume control:** Wire web UI volume slider to AGC target
8. **Frequency scan:** Auto-scan for stations with signal strength threshold
