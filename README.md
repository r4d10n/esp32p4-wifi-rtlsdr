# ESP32-P4 RTL-SDR WiFi Platform (Work in Progress)

> ⚠️ **Status: Work in Progress.** This project is under active development. APIs, Kconfig options, wire protocols, and web UI may change between commits without notice. Expect sharp edges; file issues for anything that looks broken.

Transform a Waveshare ESP32-P4 board + RTL-SDR dongle into a wireless, multi-protocol Software-Defined Radio platform. Stream IQ samples over WiFi or Ethernet using `rtl_tcp`, **SpyServer**, or **SoapyRemote** — compatible with GQRX, SDR++, SDR#, CubicSDR, and other standard clients. Or run the built-in **WebSDR** browser UI with real-time FFT, waterfall, and WBFM audio.

```
[RTL-SDR Dongle] --USB 2.0 HS--> [ESP32-P4] --WiFi 6/Ethernet--> [SDR Client / Browser]
```

## What's New in SIMD Edition

This variant extends the base ESP32-P4 RTL-SDR WiFi Bridge with:

Beyond raw IQ streaming, the platform hosts a **browser-based WebSDR** (real-time spectrum + waterfall + WBFM/NFM audio, DTMF and AFSK1200 decoders) and a **WiFi Manager** captive portal for provisioning.

**Key achievement**: 96.4% spectral correlation with direct USB reference captures, proving fidelity across the WiFi bridge.

## Current Status

| Capability | Branch | Status |
|------------|--------|--------|
| `rtl_tcp` protocol (1 MSPS WiFi) | `main` | Production |
| SpyServer protocol (SDR++ native) | `main` | Production |
| SoapyRemote protocol | `main` | Production |
| WebSDR browser UI + waterfall | `main` | Production |
| WiFi Manager (captive portal) | `main` | Production |
| UDP streaming + `udp2tcp` bridge | `main` | Production |
| rtl_power FFT scanner | `main` | Beta |
| rtl_433 ISM band decoder | `main` | Beta |
| NB / NR / Notch DSP + WBFM player | `feature/websdr` → merging | Ready |
| GSM FCCH PPM calibration | `feature/gsm-kalibrate` | Research |
| Cell carrier detection / spectrum survey | `feature/gsm-lte` | Research |

A separate **[Standalone FM Receiver firmware](../esp32p4-standalone/)** (local USB audio output, stereo + RDS decode, headless operation) lives in its own worktree — see *Related Projects* below.

## Hardware

### Bill of Materials

| Component | Model | Cost | Purpose |
|-----------|-------|------|---------|
| Main Board | Waveshare ESP32-P4-WIFI6 | ~$30 | USB Host + WiFi 6 controller + Ethernet |
| SDR Dongle | RTL-SDR Blog V4 | ~$30 | RTL2832U demod + R828D tuner |
| Antenna | Any 50Ω | ~$10 | Receive antenna (frequency-dependent) |
| **Total** | | **~$70** | Complete wireless SDR receiver |

**Alternatives**:
- **ESP32-P4-NANO**: Smaller form factor, same specs
- **Other RTL2832U dongles**: Generic (0x0bda:0x2832), RTL-SDR Blog versions

### Connections

```
Waveshare ESP32-P4-WIFI6:
┌─────────────────────────────────────────────┐
│   USB-A Ports (4x, HS on one port)          │
│   ↑ Plug RTL-SDR Blog V4 here               │
│                                             │
│   Ethernet RJ45 (100 Mbps, optional)       │
│   WiFi 6 (via ESP32-C6 on SDIO)            │
│                                             │
│   GPIO for bias-T (GPIO4 by default)       │
└─────────────────────────────────────────────┘
        ↑
[RTL-SDR Blog V4 + Antenna]
```

## Features

### Driver & Transport Core

- **USB 2.0 High-Speed**: 480 Mbps raw link, ~30+ MB/s practical throughput
- **RTL2832U Demodulator**: Complete register-level driver port from librtlsdr
- **R828D Tuner**: 24-1700 MHz coverage with R828D multi-band support + Blog V4 HF/VHF/UHF switching
- **WiFi 6 (802.11ax)**: Via ESP32-C6 companion chip on SDIO, ~4-5 MB/s TCP sustained
- **Ethernet (100 Mbps)**: RMII interface for fixed installations, ~11 MB/s TCP
- **mDNS Service Discovery**: Auto-advertise as `esp32p4-rtlsdr._rtl_tcp._tcp`
- **Ring Buffer (PSRAM)**: Configurable 2-8 MB absorbs WiFi jitter
- **Full Gain Control**: LNA + Mixer + VGA stages, manual or auto-gain
- **Direct Sampling / Offset Tuning / IF Gain**: Advanced RTL2832U modes for HF and weak-signal work
- **Bias-T Power**: GPIO-controlled antenna power (default GPIO4)
- **Frequency Correction**: PPM-level accuracy via crystal calibration (FCCH-assisted tool on `feature/gsm-kalibrate`)
- **Test Mode**: IQ pattern generator for validation
- **Real-time IQ Streaming**: 1 MSPS comfortable, 2+ MSPS possible

### Streaming Protocols

- **RTL-TCP**: Standard wire protocol, multi-client capable
- **UDP**: Lower-overhead alternative with sequence tracking + `udp2tcp` bridge for legacy clients
- **SpyServer**: Native SDR++ / SDR# compatibility (DeviceInfo HELLO, gain-table mapping, configurable decimation)
- **SoapyRemote**: Cross-platform Pothos / CubicSDR compatibility

### Built-in WebSDR (Browser UI)

- Real-time FFT spectrum (256-8192 pt, **int16 PIE SIMD**, 4.3× faster than float)
- WebGL2 waterfall with drag-pan tuning and filter overlay
- WBFM streaming player (75 µs de-emphasis, 15 kHz LPF, soft limiter, AudioWorklet output)
- NFM / USB / LSB / CW demodulation with configurable filter bandwidth
- NB (narrow-band) / NR (noise reduction) / Notch DSP
- Decoders: DTMF (Goertzel), AFSK1200 (AX.25-style framing)
- Exponential reconnect, amber-theme UI, SVG S-meter, DSEG7 frequency readout
- Works on phones, tablets, and desktops without installing any client

### Network & Provisioning

- **WiFi Manager**: Captive-portal provisioning (no hard-coded credentials needed)
- **mDNS**: Discoverable hostname on local network
- **Optional Ethernet + Multicast** (Kconfig-gated)

## Performance

### Sample Rate Capability

| Transport | Max Sustainable | Notes |
|-----------|-----------------|-------|
| WiFi 6 (SDIO 25 MHz) | ~1.0 MSPS | Recommended; comfortable margin |
| WiFi 6 (SDIO 40 MHz) | ~1.5 MSPS | Advanced; PCB-dependent |
| Ethernet (100 Mbps) | 2.4+ MSPS | Recommended for high-rate operation |

**Data rates**:
- 1.024 MSPS = 2.048 MB/s
- 2.048 MSPS = 4.096 MB/s
- 2.4 MSPS = 4.8 MB/s

### Throughput Progression

Real-world testing showed continuous improvements:

| Phase | Configuration | Max (kSPS) | Issue |
|-------|---------------|-----------|-------|
| 1 | USB HS enumeration | 214 | USB driver immature |
| 2 | Demod register fixes | 558 | Zero-IF mode missing |
| 3 | IF frequency (0x19-0x1b) | 720 | EPA registers wrong |
| 4 | EPA fix (0x2148) | 891 | SDIO bottleneck identified |
| 5 | SDIO 40 MHz tuning | 813 | Regression from volatility |
| 6 | Ring buffer optimization | 1025 | Stable 1 MSPS achieved |

### Spectral Validation

IQ quality verified against direct USB reference at 936 MHz:

```
Metric                  Reference    WiFi Bridge    Match
─────────────────────────────────────────────────────────────
Mean power (dB)          -35.2        -35.1         +0.1 dB
Peak power (dB)          -15.3        -15.2         +0.1 dB
Noise floor (dB)         -80.4        -80.6         -0.2 dB
Spectral correlation     1.0000       0.964         96.4%
```

## Architecture

### System Block Diagram

```
┌──────────────────────────────────────────────────┐
│           ESP32-P4 (400 MHz RISC-V dual-core)    │
├──────────────────────────────────────────────────┤
│                                                  │
│  ┌───────────────────────────────────────────┐  │
│  │ USB Host Driver (Core 0, High Priority)   │  │
│  │ ┌─────────────┬──────────────────────────┐│  │
│  │ │ RTL2832U    │ R828D Tuner              ││  │
│  │ │ Demod ctrl  │ - 24-1700 MHz            ││  │
│  │ │ (EP0 ctrl)  │ - LNA/Mixer/VGA          ││  │
│  │ └─────────────┴──────────────────────────┘│  │
│  │ Bulk IN Reader                             │  │
│  │ (EP 0x81 async, 512-byte packets)         │  │
│  └────────┬────────────────────────────────────┘  │
│           ↓                                        │
│  ┌────────────────────────────────────────────┐   │
│  │ Ring Buffer (PSRAM, default 2 MB)          │   │
│  │ Decouples USB bursts from WiFi TCP         │   │
│  └────────┬─────────────────────────────────┬─┘   │
│           ↓                                 ↓     │
│  ┌──────────────────┐      ┌────────────────┐    │
│  │ TCP Server       │      │ UDP Server     │    │
│  │ (Core 1, Port    │      │ (Core 1, Port  │    │
│  │  1234, rtl_tcp)  │      │  1235)         │    │
│  └────────┬─────────┘      └────────┬───────┘    │
│           ↓                         ↓            │
│  ┌────────────────────────────────────────────┐  │
│  │ ESP-Hosted SDIO Bridge → ESP32-C6          │  │
│  │ (25 or 40 MHz clock, 4-bit bus)            │  │
│  └────────────────────────────────────────────┘  │
│           ↓                                       │
│  ┌────────────────────────────────────────────┐  │
│  │ WiFi 6 (802.11ax) @ 5 GHz                 │  │
│  │ Or Ethernet (100 Mbps RMII)               │  │
│  └────────────────────────────────────────────┘  │
│           ↓                                       │
└───────────────────────────────────────────────────┘
            ↓
   [SDR Clients: SDR++, GQRX, etc.]
```

### Task Architecture (FreeRTOS)

| Task | Core | Priority | Role | Buffer |
|------|------|----------|------|--------|
| USB Host Lib | 0 | 20 (high) | USB event processing | N/A |
| USB Bulk Reader | 0 | 19 | Async EP0x81 → ring buffer | 512 B × 4-8 |
| TCP/UDP Server | 1 | 10 | Accept clients, stream IQ | Ring buffer |
| WiFi/Network | 1 | 8 | ESP-Hosted + lwIP | Internal |

**Pinning**: USB and bulk reader on Core 0 (hard real-time), network on Core 1 (soft real-time).

### Memory Map

| Region | Size | Purpose |
|--------|------|---------|
| HP SRAM (L2MEM) | 768 KB | Code, stacks, USB driver buffers |
| LP SRAM | 32 KB | RTC memory, PM info |
| PSRAM | 32 MB | Ring buffer (default 2 MB), malloc pool |
| Flash | 16 MB | Firmware, NVS, SPIFFS |

**Ring buffer location**: PSRAM for minimal latency, GDMA-AXI capable for DMA transfers.

### USB Protocol Details

#### Enumeration

1. Attach RTL2SDR Blog V4 dongle (VID:PID = 0x0bda:0x2838)
2. USB host driver claims interface 0
3. Open bulk IN endpoint 0x81 (512-byte MPS at HS)
4. No OUT endpoints; all config via control transfers

#### Vendor Control Transfers (Register Access)

```
bmRequestType:  0xC0 (read) / 0x40 (write)
bRequest:       0x00
wValue:         register_address (0x0000-0xFFFF)
wIndex:         (block << 8) | flags
                  block: 0-6 (DEMOD, USB, SYS, TUNER, ROM, IR, I2C)
                  flags: 0x10 (write) or 0x00 (read)
wLength:        1-4 bytes
Timeout:        300 ms
```

**Register mapping**:
- `demod[addr]` at `block=0, reg=addr`
- `rtl2832u[addr]` at `block=1, reg=addr`
- `sys_ctlreg[addr]` at `block=2, reg=addr`
- `tuner_i2c` at `block=6` with I2C repeater

#### Bulk IN Streaming

- EP 0x81 delivers IQ samples as USB packets
- Each packet: 512 bytes at HS (unsigned 8-bit I, Q interleaved)
- Packets arrive ~1 every microsecond at 1 MSPS
- Async callback buffers into ring buffer

### RTL2832U Demod Initialization Sequence

1. **Reset demod**: write USB_SYSCTL
2. **Load baseband**: I2C eeprom read + FIR filter setup
3. **Configure sample rate**: write demod registers 1:0x9F, 1:0xA1 (resampler ratio)
4. **Probe tuner**: try each I2C address in turn
5. **Configure tuner**: frequency/gain/bandwidth
6. **Enable IF frequency**: write demod 1:0x19-0x1b (DDC center)
7. **Set AGC mode**: demod register 1:0x4D
8. **Start streaming**: open EP 0x81 for bulk IN

### RTL-TCP Protocol

#### DongleInfo Header (Server → Client, 12 bytes, big-endian)

```
Bytes 0-3:   "RTL0" (0x52544C30)
Bytes 4-7:   Tuner type (6 = R828D)
Bytes 8-11:  Gain count (29)
```

#### Command Packet (Client → Server, 5 bytes, big-endian)

```
Byte 0:      Command opcode
Bytes 1-4:   Parameter (uint32, big-endian)
```

| Cmd | Name | Parameter |
|-----|------|-----------|
| 0x01 | Set frequency | Hz |
| 0x02 | Set sample rate | samples/sec |
| 0x03 | Set gain mode | 0=auto, 1=manual |
| 0x04 | Set gain | tenths of dB |
| 0x05 | Set freq correction | PPM |
| 0x06 | Set IF gain | stage<<16\|gain |
| 0x07 | Set test mode | 0/1 |
| 0x08 | Set AGC mode | 0/1 |
| 0x09 | Set direct sampling | 0=off, 1=I, 2=Q |
| 0x0A | Set offset tuning | 0/1 |
| 0x0B | Set RTL xtal | Hz |
| 0x0C | Set tuner xtal | Hz |
| 0x0D | Set gain by index | index |
| 0x0E | Set bias tee | 0/1 |

**IQ Data Stream** (after DongleInfo):
- Continuous uint8 I, Q pairs (interleaved)
- Byte order: little-endian values, big-endian on network (handled by lwIP)
- Example: `[I0, Q0, I1, Q1, ...]` → each byte ∈ [0, 255], 0=−3.2V, 255=+3.2V

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
esp32p4-wifi-rtlsdr/
├── README.md                          # This file
├── CMakeLists.txt                     # Top-level build config
├── sdkconfig.defaults                 # ESP-IDF default config
├── partitions.csv                     # Flash partition table
│
├── main/
│   ├── main.c                         # Application entry point
│   ├── Kconfig.projbuild              # WiFi/sample rate configuration
│   └── CMakeLists.txt
│
├── components/                        # (on `main`; `feature/websdr` is a subset)
│   ├── rtlsdr/                        # RTL2832U USB host driver
│   │   ├── include/                   #   - rtlsdr.h, rtlsdr_internal.h
│   │   ├── rtlsdr.c                   # Demod + USB host + EP0x81 bulk reader
│   │   ├── tuner_r82xx.c              # R828D/R820T tuner driver
│   │   └── CMakeLists.txt
│   │
│   ├── rtltcp/                        # rtl_tcp + UDP servers
│   │   ├── rtltcp.c                   # TCP implementation
│   │   └── rtludp.c                   # UDP implementation
│   │
│   ├── spyserver/                     # SpyServer protocol (SDR++ native)
│   ├── soapyremote/                   # SoapyRemote protocol
│   ├── websdr/                        # Browser UI + embedded web assets
│   │   ├── www/                       #   HTML/JS/CSS (spectrum, waterfall, WBFM)
│   │   ├── certs/                     #   TLS certificates (optional HTTPS)
│   │   └── websdr.c                   #   HTTP + WebSocket server, DSP glue
│   │
│   ├── dsp/                           # FFT, DDC, filters (int16 PIE SIMD)
│   ├── decoders/                      # DTMF, AFSK1200, shared decoder helpers
│   ├── rtl433/                        # rtl_433 ISM-band decoder bridge
│   ├── rtlpower/                      # rtl_power FFT scanner
│   └── wifimgr/                       # Captive-portal WiFi provisioning
│
├── c6-ota-flasher/                    # ESP32-C6 WiFi firmware
│   ├── main/app_main.c                # C6 firmware entry
│   ├── CMakeLists.txt
│   └── partitions.csv
│
├── docs/
│   ├── FEASIBILITY.md                 # Project feasibility analysis
│   ├── DRIVER_ANALYSIS.md             # RTL-SDR Blog V4 driver gaps
│   ├── ARCHITECTURE.md                # Detailed architecture (new)
│   ├── BENCHMARKS.md                  # Performance data (new)
│   ├── POWER_BUDGET.md                # Power consumption analysis (new)
│   ├── HARDWARE_SETUP.md              # Hardware assembly guide (new)
│   ├── BUGS_AND_FIXES.md              # Development bug fixes (new)
│   └── plans/
│       └── 2026-03-15-full-driver-port-plan.md
│
├── tools/
│   ├── ws_fm_player.py                 # Python FM audio player
│   └── ws_record_10s.py               # Audio recording + analysis
└── docs/
    └── 2026-03-24-comprehensive-fix-report.md  # Detailed change report
```

## Roadmap

### Near-term (pending merge into `main`)

- Merge `feature/websdr` — NB/NR/notch DSP, WBFM player, expanded decoder test matrix
- Cherry-pick direct-sampling / offset-tuning / IF-gain hardening from `feature/wifimgr`

### Medium-term

- High-rate 2.4+ MSPS Ethernet primary transport
- Expanded tuner support: **E4000**, **FC0012**, **FC0013**, **FC2580** (driver framework already probes their I2C addresses)

### Research

- **GSM FCCH-based PPM calibration** (`feature/gsm-kalibrate`) — a calibration-only tool that uses the known 67.7 kHz GSM frequency-correction burst as a clock reference to measure and correct the RTL-SDR crystal offset. Same principle as the classic `kalibrate-rtl` utility.
- **Cell carrier detection / spectrum survey** (`feature/gsm-lte`) — FFT-based scanner that identifies occupied cellular channels (ARFCN / EARFCN presence) across GSM bands and LTE PSS/SSS detection for educational RF-survey use. Passive spectrum-occupancy measurements only.
- **DVB-T reception** (tracked in the sibling `esp32p4-dvbmod` experiment) — native demodulation of DVB-T transport streams using the RTL2832U's onboard COFDM demodulator.

### Standalone Track (separate product)

- **Standalone FM Receiver firmware** (see [Related Projects](#related-projects)) — headless FM stereo + RDS receiver with local USB audio output. Not a merge into `main`; maintained on its own release track.

## Development History

| Metric | Value |
|--------|-------|
| USB throughput | 1024 kSPS (2.048 MB/s) |
| WiFi delivery | ~891 kSPS (87%) |
| FFT (1024-pt, PIE) | 3.5 us |
| CIC alias rejection | 38.9 dB (3rd-order) |
| FM audio SNR | 39.4 dB (Python player) |
| Binary size | 1.18 MB (44% partition free) |
| Boot to ready | ~20s (WiFi connect + USB enum) |

## Related Projects

Additional ESP32-P4 SDR experiments live in sibling worktrees under the same parent directory. They share hardware and much of the driver core, but diverge in target use case:

| Project | Worktree / Branch | Purpose |
|---------|-------------------|---------|
| **Standalone FM Receiver** | `../esp32p4-standalone/` (`feature/standalone`) | Headless FM stereo + RDS receiver with local **USB audio output** — no WiFi client needed. Target: portable / car / bedside radio. |
| Standalone (dev tracks) | `standalone-mono-nonopt`, `standalone-optimized` | Development experiments exploring cascaded 19 kHz notch, IQ-level squelch, and `-O3`/`-Ofast` compiler-flag trade-offs for the FM chain. |
| WiFi Manager + Extras | `../esp32p4-wifi-rtlsdr-wifimgr/` (`feature/wifimgr`) | Super-set of `main` with extended network and notification features. Tested additions trickle back into `main` once validated. |

## License

**GPL-2.0-or-later** — Following librtlsdr and xtrsdr precedent.

## References

- [Waveshare ESP32-P4-WIFI6](https://www.waveshare.com/wiki/ESP32-P4-WIFI6)
- [RTL-SDR Blog V4](https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/)
- [ESP-IDF USB Host API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/usb_host.html)
- [ESP-Hosted WiFi over SDIO](https://github.com/espressif/esp-hosted)
- [RTL-TCP Protocol (K3XEC)](https://k3xec.com/rtl-tcp/)
- [librtlsdr](https://github.com/steve-m/librtlsdr)
