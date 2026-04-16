# ESP32-P4 Standalone FM Receiver

A **headless FM stereo + RDS receiver** running entirely on the Waveshare ESP32-P4 with an RTL-SDR dongle. Unlike the sibling [`esp32p4-wifi-rtlsdr`](../esp32p4-wifi-rtlsdr/) project — which streams raw IQ over WiFi to a client — this firmware does **all DSP on-device** and emits audio through a **USB audio class device** (or an attached I²S codec).

```
[Antenna] ─ [RTL-SDR] ─USB HS─► [ESP32-P4] ─ FM stereo + RDS DSP ─► [USB Audio / I²S] ─► [Speakers]
                                         │
                                         └─► [Browser UI: tuning, RDS text, MPX spectrum]
```

No WiFi client required for playback. WiFi is still used for control UI and OTA, but the audio path is self-contained.

## Status

| Capability | Branch | Status |
|------------|--------|--------|
| Mono FM demodulation | `feature/standalone` | Production |
| Stereo (pilot detection + PLL) | `feature/standalone` | Production |
| RDS decode (PS, RT, PI, PTY) | `standalone-mono-nonopt` | Working, field-verified on-air |
| USB Audio Class 1 output | `feature/standalone` | Production |
| Web UI (tuning + MPX spectrum + RDS panel) | `standalone-mono-nonopt` | Working |
| IQ-level noise squelch | `standalone-mono-nonopt` | Working |
| `fast_atan2f` + sin-LUT optimization | `feature/standalone` | Merged |
| Civic-radio emulator (CI-V / CAT control) | `feature/standalone` | Experimental |
| `-O3` / `-Ofast` compiler trade-off study | `standalone-optimized` | Study complete (stayed on `-Os`) |

## Signal Path

```
RTL-SDR IQ @ 256 kSPS
    │
    ▼
┌──────────────────────┐
│  2-stage decimation  │   → 38 kSPS baseband MPX
│   (CIC + FIR)        │
└──────────┬───────────┘
           │ real MPX signal
           ▼
┌──────────────────────┐
│  19 kHz pilot PLL    │   → locks to stereo pilot, -62 dB residual
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐      ┌─────────────────────┐
│  L+R (0–15 kHz LPF)  │      │  L−R DSB-SC @38 kHz │
└──────────┬───────────┘      └──────────┬──────────┘
           ▼                             ▼
            ┌─────────────┐   ┌─────────────┐
            │  Matrix →   │   │  Cascaded   │   → audio L, R
            │  L, R       │   │  19 kHz notch│
            └─────┬───────┘   └──────┬──────┘
                  │                  │
                  ├──── De-emphasis (75 µs) ────┐
                  ▼                              ▼
            ┌─────────────┐              ┌──────────────┐
            │ USB Audio   │              │  57 kHz RDS  │
            │ Class 1     │              │  subcarrier  │
            │ 48 kSPS s16 │              │  → biphase   │
            └─────────────┘              │  → RDS blocks│
                                         └──────────────┘
```

**Key DSP techniques** (see `docs/ARCHITECTURE.md` and the commit log for derivations):

- **`fast_atan2f` + sin lookup table** — ~5× / ~10× speedup over libm on the RISC-V core
- **ESP32-P4 PIE SIMD** for filter taps and pilot mixer (see parent project's `docs/ESP32_P4_PIE_SIMD_GUIDE.md`)
- **Cascaded 19 kHz notch** (−80 dB) to keep the pilot out of the audio path
- **IQ-level noise squelch** — mutes audio when the input carrier drops below threshold, preventing noise bursts between stations
- **Bit-exact host DSP reference pipeline** — every stage has a Python/NumPy reference that matches the on-device output sample-for-sample, verified by the `test/` suite

## Hardware

Same BOM as the parent project:

| Component | Model | Notes |
|-----------|-------|-------|
| Main board | Waveshare ESP32-P4-WIFI6 or ESP32-P4-NANO | USB Host + WiFi 6 + RMII |
| SDR dongle | RTL-SDR Blog V4 (or any RTL2832U) | USB 2.0 HS |
| Audio out (choice A) | Built-in USB Audio Class 1 | No extra hardware, plug into speakers / headphones with a USB-C audio dongle |
| Audio out (choice B) | I²S codec (e.g. ES8311, MAX98357) | For dedicated line/speaker out on a board pin header |

## Quick Start

```bash
cd ~/exp/esp32/p4/host/esp32p4-standalone
source ~/esp/v5.5.1/esp-idf/export.sh

idf.py set-target esp32p4
idf.py menuconfig      # Configure WiFi (optional, for web UI) and default FM frequency
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Plug the ESP32-P4 into a host computer (or a USB speaker). It will enumerate as a standard USB Audio device — no driver needed on Linux, macOS, or Windows. Select it as the audio output device and tune via the browser UI or the optional CI-V control interface.

## Branches

| Branch | Purpose |
|--------|---------|
| `feature/standalone` | Primary development branch — stereo + perf optimizations + tests |
| `standalone-mono-nonopt` | Snapshot with RDS on-air decode verified + web UI + dev log |
| `standalone-optimized` | Compiler-flag study (`-Os` vs `-O3` vs `-Ofast`); stayed on `-Os` after RDS integrator instability at higher opt levels |

The mono-vs-stereo and optimized variants are preserved as **reference points** for the different regressions they exposed — see `docs/BUGS_AND_FIXES.md` for the full history.

## Relationship to `esp32p4-wifi-rtlsdr`

The two firmwares **share the same USB / RTL2832U / DSP core** but diverge in output:

- **`esp32p4-wifi-rtlsdr`** → streams IQ (and spectrum/audio) over the network to a remote client
- **`esp32p4-standalone`** (this repo) → demodulates and plays audio locally

Improvements to the shared DSP (PIE SIMD kernels, `fast_atan2f`, filter implementations) flow in both directions. Protocol-specific code (rtl_tcp, SpyServer, SoapyRemote) stays on the streaming side; audio-output code (USB Audio, I²S, RDS) stays here.

## License

**GPL-2.0-or-later**, inherited from the parent project (librtlsdr / xtrsdr lineage).
