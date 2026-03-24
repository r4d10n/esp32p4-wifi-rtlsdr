# ESP32-P4 RTL-SDR WiFi Bridge — SIMD Edition

Transform a Waveshare ESP32-P4 board + RTL-SDR dongle into a wireless Software-Defined Radio receiver with **PIE SIMD-accelerated DSP**, a browser-based **WebSDR UI**, and **WBFM audio streaming**.

```
[RTL-SDR Dongle] --USB HS--> [ESP32-P4 PIE SIMD DSP] --WiFi 6 (HTTPS)--> [Browser / SDR Client]
```

## What's New in SIMD Edition

This variant extends the base ESP32-P4 RTL-SDR WiFi Bridge with:

- **PIE 128-bit SIMD acceleration** — 4-8x faster FFT, power spectrum, NCO complex multiply
- **3rd-order CIC decimator** — 38.9 dB alias rejection (was 13 dB boxcar)
- **32-bit NCO phase accumulator** — zero phase discontinuity at any frequency
- **DC offset removal** — EMA-based center spike elimination
- **Browser-based WebSDR UI** — experimental web interface with spectrum, waterfall, and audio
- **HTTPS/WSS** — secure WebSocket for AudioWorklet support
- **FM audio player** — browser-based WBFM demodulation and streaming
- **Full DSP sidebar** — NB, NR, Notch, AGC, Squelch, De-emphasis, Limiter
- **Python FM player** — `tools/ws_fm_player.py` with vectorized numpy demod (39 dB SNR)
- **int16 DDC output** — 90 dB dynamic range (was 48 dB with uint8)

## Quick Start

### Prerequisites

- ESP-IDF v5.5+ with ESP32-P4 support
- Waveshare ESP32-P4-WIFI6 + RTL-SDR Blog V4

### Build and Flash

```bash
source ~/esp/esp-idf/export.sh
cd esp32p4-wifi-rtlsdr-simd

# Configure WiFi credentials
idf.py menuconfig
# → RTL-SDR WiFi Bridge Configuration → WiFi SSID / Password

# Build and flash
idf.py build
idf.py -p /dev/ttyACM1 flash
```

### Connect

| Interface | URL | Protocol |
|-----------|-----|----------|
| **WebSDR UI** | `https://<device-ip>` | HTTPS + WSS |
| **FM Player** | `https://<device-ip>/fm` | HTTPS + WSS |
| **RTL-TCP** | `<device-ip>:1234` | TCP (for GQRX, SDR++) |

The device advertises via mDNS as `esp32p4-rtlsdr._rtl_tcp._tcp`.

## WebSDR UI Features

### WebSDR UI

The browser UI provides spectrum display, waterfall, and audio demodulation (experimental):

**Tabs**: Home | DSP | Display | Favourites | Memories | Tools | Help

**Home Tab**:
- VFO A/B/C/D switching with state preservation
- Gain slider (LNA, 0-49.6 dB)
- Sample rate selector (250k - 3.2M SPS)
- FFT size selector (256 - 8192 bins)
- Lock and Fast Tune buttons
- Audio start/stop

**DSP Tab**:
- AGC speed (Off/Slow/Medium/Fast)
- De-emphasis (Off/50us EU/75us US)
- WBFM soft limiter (drive + ceiling)
- Audio level meter

**Display Tab**:
- 5 waterfall palettes (Default, Jet, Iron, Viridis, Grayscale)
- FFT smoothing (averaging alpha 0-0.95)
- Peak hold with configurable decay
- Spectrum reference level and range
- Waterfall speed control

### VFO Panel

- **Frequency display**: DSEG7 7-segment font, 10 digits (GHz.MHz.kHz.Hz)
- **Mouse wheel tuning**: Hover over any digit and scroll to tune
- **VFO A/B/C/D**: Save/restore frequency, mode, bandwidth per VFO
- **Mode selector**: AM, NFM, WBFM, USB, LSB, CW
- **14 filter presets**: 2.5k to 250k bandwidth

### DSP Sidebar

- **AGC**: Fast/Medium/Slow with configurable attack/decay
- **Noise Blanker**: Impulse detection with threshold-based blanking
- **Noise Reduction**: Spectral gate noise suppression
- **Auto Notch**: IIR notch filter with zero-crossing frequency detection
- **Squelch**: Power-based with enable/auto/level controls
- **Band presets**: HF (160m-10m), VHF/UHF (6m, 2m, 70cm), Broadcast (FM, Air, Marine)

### Spectrum & Waterfall

- **WebGL2 waterfall** with Canvas2D fallback
- **Click-to-tune**, drag-to-pan, mouse wheel zoom (1x-16x)
- **Filter passband overlay** with edge dragging
- **Cursor readout** (frequency + dB at mouse position)
- **Hover controls**: HIGH/LOW/ZOOM with auto-scale
- **Touch support**: Pinch-to-zoom on mobile
- **Resizable split**: Drag handle between spectrum and waterfall

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| Up/Down | Tune by mode step |
| Left/Right | Tune by 10x step |
| +/- | Zoom in/out |
| M | Cycle mode |
| Space | Toggle audio |
| F | Fullscreen |
| P | Clear peak hold |
| 1-9 | Select band preset |

### Mobile Support

- Responsive layout at 900px, 640px, 480px breakpoints
- Slide-out DSP drawer (hamburger button) on narrow screens
- Scrollable controls
- Touch-friendly controls

## FM Audio Player

### Browser Player (`/fm`)

Standalone WBFM player with:
- FM discriminator + 75us de-emphasis
- AudioWorklet (HTTPS) or ScriptProcessor fallback (HTTP)
- Frequency, gain, volume controls
- Audio level meter

### Python Player

```bash
# Install dependencies
pip install websocket-client numpy sounddevice scipy

# Play FM radio
python3 tools/ws_fm_player.py 100.0 192.168.1.233

# Record 10s for analysis
python3 tools/ws_record_10s.py
```

**Audio quality** (measured):
- SNR: **39.4 dB** (reference: 43.9 dB)
- Vectorized numpy `np.angle()` FM discriminator
- 15 kHz FIR anti-alias filter (63 taps)
- Fractional resampling with linear interpolation
- Direct int16 IQ processing (no uint8 intermediate)

## PIE SIMD DSP Engine

### Architecture

```
USB IQ (uint8)
    ↓
DC Offset Removal (EMA α=1/1024)
    ↓
Hann Windowing (PIE hw loop)
    ↓
Radix-2 FFT (PIE int16, 4.3x faster)
    ↓
Power Spectrum (PIE hw loop)  →  Waterfall/Spectrum
    ↓
fast_log10f (IEEE 754 bit trick, 8x faster)
    ↓
dB Scaling + FFT Shift  →  WebSocket MSG_FFT (0x01)

DDC Pipeline (per-client):
    uint8 IQ → int16 bias removal
    ↓
    NCO Mix (32-bit phase accum, 1024-entry table)
    ↓
    3rd-order CIC Decimation (38.9 dB rejection, int64 accum)
    ↓
    int16 IQ output  →  WebSocket MSG_IQ16 (0x03)
```

### Assembly Kernels

| Kernel | File | Speedup | Method |
|--------|------|---------|--------|
| Windowing | `pie_windowing_arp4.S` | ~2x | HW zero-overhead loop |
| Power Spectrum | `pie_power_spectrum_arp4.S` | ~2x | HW loop + int32 accum |
| NCO Complex Multiply | `pie_nco_mix_arp4.S` | ~4x | `esp.cmul.s16` SIMD |

### Performance

| Operation | ANSI C | PIE SIMD | Speedup |
|-----------|--------|----------|---------|
| 1024-pt FFT | 15 us | 3.5 us | **4.3x** |
| 4096-pt FFT | 95 us | 22 us | **4.3x** |
| Power spectrum (4096) | 40 us | 20 us | **2x** |
| dB conversion (4096) | 490K cy | 61K cy | **8x** |

## WebSocket Protocol

### Binary Message Types

| Type | ID | Direction | Format |
|------|-----|-----------|--------|
| FFT Spectrum | `0x01` | Server → Client | `[0x01][uint8 dB × fft_size]` |
| IQ uint8 (legacy) | `0x02` | Server → Client | `[0x02][uint8 I,Q pairs]` |
| IQ int16 (SIMD) | `0x03` | Server → Client | `[0x03][int16 I,Q pairs]` |

### JSON Commands (Client → Server)

| Command | Parameters | Description |
|---------|------------|-------------|
| `freq` | `value` (Hz) | Set center frequency (24M-1766M) |
| `gain` | `value` (0.1 dB) | Set tuner gain (0=auto) |
| `sample_rate` | `value` (Hz) | Set sample rate |
| `fft_size` | `value` | Set FFT bins (256-8192) |
| `db_range` | `min`, `max` | Set dB scaling range |
| `subscribe_iq` | `offset`, `bw` | Start DDC IQ stream |
| `unsubscribe_iq` | — | Stop DDC IQ stream |

### JSON Responses (Server → Client)

| Type | Fields | When |
|------|--------|------|
| `info` | freq, rate, gain, fft_size, db_min, db_max | On connect |
| `config` | fft_size, sample_rate, db_min, db_max | On parameter change |
| `freq` | value | On frequency change |
| `iq_start` | offset, bw, rate | On IQ subscribe |
| `iq_stop` | — | On IQ unsubscribe |
| `error` | msg | On invalid command |

## Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| Main Board | Waveshare ESP32-P4-WIFI6 | 400 MHz RISC-V + PIE SIMD + USB HS |
| SDR Dongle | RTL-SDR Blog V4 (R828D) | 24-1700 MHz tuner |
| WiFi | ESP32-C6 via SDIO | WiFi 6 (802.11ax) |
| RAM | 32 MB PSRAM | Ring buffers, DDC scratch |
| Flash | 16 MB | Firmware + embedded web UI |

## Unit Tests

```bash
# Build and run DSP kernel tests
gcc -o test/test_dsp_kernels test/test_dsp_kernels.c -lm -I components/dsp/include -O2
./test/test_dsp_kernels    # 9 tests

# Edge case tests
gcc -o test/test_dsp_edge_cases test/test_dsp_edge_cases.c -lm -I components/dsp/include -O2
./test/test_dsp_edge_cases  # 17 tests
```

**Test results** (26/26 pass):
- Windowing: basic, Hann, edge cases
- Power spectrum: overflow, accumulation
- fast_log10f: accuracy across 15 orders of magnitude
- NCO: zero discontinuities over 1M samples (prime frequency)
- CIC: DC convergence, 38.9 dB alias rejection, ratio 1/64
- DC removal: EMA convergence, tone preservation (-0.25 dB)
- Ring buffer: overflow, pointer advancement

## Project Structure

```
esp32p4-wifi-rtlsdr-simd/
├── main/main.c                          # App entry, WiFi, task orchestration
├── components/
│   ├── dsp/                             # PIE SIMD DSP engine
│   │   ├── dsp.c                        # FFT + DDC pipeline
│   │   ├── pie_kernels.c               # C reference + asm dispatch
│   │   ├── pie_kernels.h               # Kernel API
│   │   ├── pie_power_spectrum_arp4.S   # HW loop power accumulation
│   │   ├── pie_nco_mix_arp4.S          # esp.cmul.s16 complex multiply
│   │   └── pie_windowing_arp4.S        # HW loop windowing
│   ├── rtlsdr/                          # USB RTL2832U + R828D driver
│   ├── rtltcp/                          # RTL-TCP + UDP servers
│   └── websdr/                          # HTTPS/WSS server + embedded UI
│       ├── websdr.c                     # Server, FFT task, DDC per-client
│       ├── certs/                       # TLS certificates
│       └── www/                         # Embedded web assets
│           ├── index.html               # WebSDR UI
│           ├── sdr.js                   # Full JS client (~950 lines)
│           ├── sdr.css                  # UI styles (~1020 lines)
│           ├── fm_player.html           # Standalone FM player
│           └── dseg7.woff2              # 7-segment display font
├── test/
│   ├── test_dsp_kernels.c              # Core DSP unit tests (9)
│   └── test_dsp_edge_cases.c           # Edge case tests (17)
├── tools/
│   ├── ws_fm_player.py                 # Python FM audio player
│   └── ws_record_10s.py               # Audio recording + analysis
└── docs/
    └── 2026-03-24-comprehensive-fix-report.md  # Detailed change report
```

## Performance Metrics

| Metric | Value |
|--------|-------|
| USB throughput | 1024 kSPS (2.048 MB/s) |
| WiFi delivery | ~891 kSPS (87%) |
| FFT (1024-pt, PIE) | 3.5 us |
| CIC alias rejection | 38.9 dB (3rd-order) |
| FM audio SNR | 39.4 dB (Python player) |
| Binary size | 1.18 MB (44% partition free) |
| Boot to ready | ~20s (WiFi connect + USB enum) |

## License

**GPL-2.0-or-later** — Following librtlsdr and xtrsdr precedent.

## References

- [Waveshare ESP32-P4-WIFI6](https://www.waveshare.com/wiki/ESP32-P4-WIFI6)
- [RTL-SDR Blog V4](https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/)
- [ESP-IDF USB Host API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/usb_host.html)
- [ESP-Hosted WiFi over SDIO](https://github.com/espressif/esp-hosted)
- [RTL-TCP Protocol (K3XEC)](https://k3xec.com/rtl-tcp/)
- [librtlsdr](https://github.com/steve-m/librtlsdr)
