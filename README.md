# ESP32-P4 RTL-SDR WiFi Platform

Transform a Waveshare ESP32-P4 board + RTL-SDR dongle into a wireless, multi-protocol Software-Defined Radio platform. Stream IQ samples over WiFi or Ethernet using `rtl_tcp`, **SpyServer**, or **SoapyRemote** — compatible with GQRX, SDR++, SDR#, CubicSDR, and other standard clients. Or run the built-in **WebSDR** browser UI with real-time FFT, waterfall, and WBFM audio.

```
[RTL-SDR Dongle] --USB 2.0 HS--> [ESP32-P4] --WiFi 6/Ethernet--> [SDR Client / Browser]
```

## Overview

This project provides a complete **USB Host driver for RTL2832U-based DVB-T tuners** on the ESP32-P4 — the first ESP32 variant with USB 2.0 High-Speed support. Previous variants (S2, S3) only supported Full-Speed USB, causing babble errors and limiting sample rates to ~240 kHz. The ESP32-P4 with HS enables **1+ MSPS streaming over WiFi**.

Beyond raw IQ streaming, the platform hosts a **browser-based WebSDR** (real-time spectrum + waterfall + WBFM/NFM audio, DTMF and AFSK1200 decoders), a **WiFi Manager** captive portal for provisioning, and a growing family of **protocol-specific receiver branches** (APRS iGate, AIS maritime, ADS-B aircraft) that share the common driver/DSP core.

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
| APRS (144.39 MHz, AFSK1200, iGate) | `feature/aprs` → merging | Ready |
| AIS (161.975/162.025 MHz, GMSK 9600) | `feature/ais` → merging | Ready |
| ADS-B (1090 MHz, dump1090-style) | `feature/adsb` → merging | Ready |
| WSPR / FT8 weak-signal decode | `feature/wspr-ft8` | Experimental |
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

## Protocol Receiver Branches

The platform's driver and DSP core is shared across a family of **protocol-specific firmware variants**, each maintained in its own feature branch with a purpose-built decoder and web UI. These build on the common WebSDR foundation but tune the receiver for a specific band and demodulation chain.

| Branch | Frequency | Modulation | Decoder | HTTP Port | Status |
|--------|-----------|------------|---------|-----------|--------|
| `feature/aprs` | 144.390 MHz | AFSK 1200 | AX.25 frame sync, APRS-IS iGate forwarder | 8081 | Merge-ready |
| `feature/ais`  | 161.975 / 162.025 MHz | GMSK 9600 (dual-channel) | AIS NMEA decoder | 8082 | Merge-ready |
| `feature/adsb` | 1090 MHz | PPM (Mode S) | dump1090-style message extractor, live aircraft map | 8083 | Merge-ready |
| `feature/wspr-ft8` | HF / 6m / 2m (per config) | 8-GFSK | WSPR + FT8 (`ft8_lib`, LDPC, Costas sync), optional PSKreporter upload | configurable | Experimental |

Each branch is **independent** — pick the protocol you want, flash that branch. Merging the four "merge-ready" branches into `main` is a near-term milestone so they can coexist as runtime-selectable modules.

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

- **ESP-IDF v5.5+**: Download from [esp-idf releases](https://github.com/espressif/esp-idf/releases)
- **Python 3.7+** with numpy (for spectral analysis)
- **Build tools**: GCC, CMake
- Waveshare ESP32-P4-WIFI6 board + RTL-SDR dongle

### 1. Set Up ESP-IDF

```bash
# Download and extract ESP-IDF v5.5.1
cd ~/esp
wget https://github.com/espressif/esp-idf/releases/download/v5.5.1/esp-idf-v5.5.1.zip
unzip esp-idf-v5.5.1.zip
cd esp-idf-v5.5.1
./install.sh esp32p4
source ./export.sh
```

### 2. Flash ESP32-C6 Slave (One-Time Setup)

The ESP32-P4 cannot run WiFi natively; it uses an **ESP32-C6 companion chip** connected via SDIO. The C6 must be flashed with the esp-hosted firmware.

```bash
cd ~/esp32p4-wifi-rtlsdr/c6-ota-flasher
idf.py build

# Flash C6 over UART
# Connect Waveshare USB-UART adapter to C6 UART pins
idf.py -p /dev/ttyUSB0 flash monitor
# Wait for "esp-hosted connected"
```

**For Waveshare ESP32-P4-WIFI6**: C6 is usually pre-flashed. Verify with:
```bash
cd c6-ota-flasher
idf.py monitor
# Should show: "esp-hosted connected" on serial console
```

### 3. Configure WiFi Credentials

```bash
cd ~/esp32p4-wifi-rtlsdr
idf.py menuconfig

# Navigate to: RTL-SDR WiFi Bridge Configuration
#   → WiFi SSID: enter your network name
#   → WiFi Password: enter your password
#   (Defaults: YourSSID / YourPassword for dev testing)

# Save and exit
```

### 4. Build and Flash P4

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

**Expected output**:
```
[00:00:01] Initializing ESP-Hosted...
[00:00:02] Connecting to C6 slave over SDIO...
[00:00:03] ESP-Hosted connected to C6
[00:00:05] WiFi STA connecting to 'YourSSID'...
[00:00:08] Got IP: 192.168.1.232
[00:00:10] USB Host installed, waiting for device...
[00:00:12] RTL-SDR attached, VID:PID=0x0bda:0x2838
[00:00:13] RTL2832U demod initialized
[00:00:14] Tuner detected: R828D
[00:00:15] RTL-TCP server listening on :1234 (mDNS: esp32p4-rtlsdr._rtl_tcp._tcp)
[00:00:16] Ready to stream IQ samples
```

### 5. Connect SDR Client

**GQRX**:
```
Input → Other → rtl_tcp → esp32p4-rtlsdr (or 192.168.1.232:1234)
```

**SDR++**:
```
Source → RTL-TCP → esp32p4-rtlsdr:1234
```

**Command line**:
```bash
# Capture 5 seconds @ 936 MHz, 1 MSPS, gain 40 dB
rtl_tcp -a 192.168.1.232 -p 1234 &
sleep 1
# In another terminal:
rtl_sdr -f 936000000 -s 1024000 -g 40 - | timeout 5 xxd -l 2048 | head
```

## UDP Streaming

UDP provides **lower latency and overhead** compared to TCP, suitable for high-rate streaming. Use the **udp2tcp bridge** on the host to convert UDP streams to standard rtl_tcp format.

### Enable UDP on Device

```bash
idf.py menuconfig
# RTL-SDR WiFi Bridge Configuration
#   → IQ Transport Protocol: "UDP only" or "Both TCP and UDP"
#   → UDP streaming port: 1235 (default)
#   → UDP payload size: 1024 (default, ~2 MB/s per UDP client)

idf.py build flash monitor
```

### Build and Run udp2tcp Bridge

On your host machine (Linux/Mac):

```bash
cd tools
gcc -O2 -o udp2tcp_bridge udp2tcp_bridge.c -lpthread

# Start bridge (receives UDP from ESP32-P4, serves TCP to rtl_tcp clients)
./udp2tcp_bridge -u 192.168.1.232:1235 -t 1234 -s

# Output:
# UDP server listening on :1235
# TCP server listening on :1234
# Connected to 192.168.1.232:1235 (tuner=R828D, gains=29)
```

### Connect SDR Client to Bridge

```bash
# Now point SDR++ or GQRX to localhost:1234 (not the ESP32-P4)
# The bridge transparently forwards UDP IQ to TCP rtl_tcp protocol
```

**Advantages**:
- UDP throughput: 2-3 Mbps per client (vs TCP 4-5 Mbps)
- Lower latency (no TCP retransmits/ACKs)
- Fits in one Ethernet frame per packet
- Multiple clients possible

**Disadvantages**:
- Packet loss possible on WiFi (bridge logs dropped packets)
- Requires host-side bridge software
- No flow control (overflow = dropped samples)

## Configuration Options

Edit `main/Kconfig.projbuild` or use `idf.py menuconfig`:

### WiFi Settings

| Option | Default | Range | Notes |
|--------|---------|-------|-------|
| WiFi SSID | YourSSID | string | Network to connect to |
| WiFi Password | YourPassword | string | Network password |

### Sample Rate Settings

| Option | Default | Range | Notes |
|--------|---------|-------|-------|
| Default frequency | 100 MHz | Hz | Initial center frequency on boot |
| Default sample rate | 1.024 MSPS | Hz | Initial rate; changeable via RTL-TCP |

### Transport Configuration

| Option | Default | Values | Notes |
|--------|---------|--------|-------|
| IQ Transport | TCP | TCP, UDP, Both | Select streaming protocol |
| TCP Port | 1234 | 1024-65535 | Standard rtl_tcp port |
| UDP Port | 1235 | 1024-65535 | Custom UDP port |
| UDP Payload | 1024 | 512-1464 | Bytes of IQ per UDP packet |
| Ring Buffer | 2048 KB | 512-8192 | PSRAM buffer for jitter absorption |

### Advanced Tuning

| Option | Default | Notes |
|--------|---------|-------|
| SDIO Clock | 25 MHz | Set to 40 MHz for higher WiFi throughput (PCB-dependent) |

## Spectral Validation

Verify IQ quality using the included Python spectral analysis tool.

### Capture Reference (Direct USB)

On a host with librtlsdr installed:

```bash
# Capture at 936 MHz, 1.024 MSPS, gain 40 dB for 2 seconds
rtl_sdr -f 936000000 -s 1024000 -g 40 - | head -c $((2 * 1024000 * 2)) > reference.iq

# Generate spectral profile
cd test
python3 analyze_spectrum.py reference ../reference.iq --freq 936e6 --rate 1024000
# Output: reference_profile.json
```

### Capture via WiFi Bridge

```bash
# Capture same conditions via ESP32-P4
python3 analyze_spectrum.py capture_wifi bridge.iq --host 192.168.1.232 --port 1234

# Compare
python3 analyze_spectrum.py compare reference_profile.json bridge.iq
```

**Expected output**:
```
============================================================
SPECTRAL COMPARISON
============================================================
Reference: reference.iq (direct_usb)
Test:      bridge.iq (wifi_bridge)

Power Stats:
  Metric               Reference      Test       Diff
  ─────────────────────────────────────────────────────
  mean_db                   -35.2     -35.1     +0.1
  peak_db                   -15.3     -15.2     +0.1
  noise_floor_db            -80.4     -80.6     -0.2
  dynamic_range_db           65.1      65.4     +0.3

Spectral Shape:
  Correlation: 0.9640 (1.0 = perfect match)
  RMSE:        1.23 dB

Peaks: reference=3, test=3
  Matched: 936.012 MHz (freq_diff=-5.3 kHz, power_diff=+0.1 dB)

============================================================
MATCH SCORE: 96.4/100
EXCELLENT - Signals match closely
============================================================
```

## Power Budget

### Component Power Consumption

| Component | Current (mA) | @ 5V | @ 3.3V |
|-----------|--------------|------|--------|
| ESP32-P4 (idle) | 15 | 75 mW | 50 mW |
| ESP32-P4 (100%) | 200 | 1 W | 660 mW |
| ESP32-C6 (WiFi active) | 50 | 250 mW | 165 mW |
| RTL-SDR Blog V4 | 200 | 1 W | 660 mW |
| **Total (streaming)** | **~450** | **2.3 W** | **1.5 W** |

### Battery Runtime

Assuming typical WiFi streaming at 1 MSPS:

| Battery | Capacity | Runtime | Notes |
|---------|----------|---------|-------|
| 5000 mAh USB power bank | 5 Ah @ 5V | 5-6 hours | Cost: ~$15 |
| 18650 Li-ion 3.7V | 3 Ah | 2-3 hours | Cost: ~$5 |
| 4x AA NiMH 1.2V (6V) | 2.4 Ah | 3-4 hours | Cost: ~$10 |

### Solar Panel Sizing

For continuous operation:

- **Total draw**: 1.5 W average (WiFi streaming)
- **Solar panel**: 5-10 W @ full sun (100 mW typical indoor, 500 mW+ outdoors)
- **Battery buffer**: 5000 mAh USB power bank + 1W solar → continuous portable operation

**Recommended setup**: 10W solar panel + 10000 mAh battery for 24/7 outdoor field use.

### Power Comparison: Raspberry Pi vs ESP32-P4

| System | Idle | WiFi Streaming | Cost | Advantage |
|--------|------|-----------------|------|-----------|
| **Raspberry Pi 5** | 3 W | 8 W | $80 | Full-featured, large ecosystem |
| **Raspberry Pi Zero 2W** | 0.5 W | 2 W | $15 | Very low power, still full OS |
| **ESP32-P4 (this project)** | 0.1 W | 1.5 W | $30 | **Ultra-low power**, dedicated |

The ESP32-P4 consumes **90% less power** than even the Raspberry Pi Zero 2W, making it ideal for battery-powered field deployments.

## Supported Tuners

| Tuner | I2C Address | Frequency Range | Implemented | Notes |
|-------|-------------|-----------------|-------------|-------|
| **R828D** | 0x74 | 24-1700 MHz | Full | Current hardware, Blog V4 multi-input |
| **R820T/R820T2** | 0x34 | 24-1700 MHz | Full | Common in older dongles, ~90% compatible with R828D |
| **E4000** | 0xC8 | 52-2200 MHz | Planned | Different architecture, separate driver |
| **FC0012** | 0xC6 | 22-948.6 MHz | Planned | Requires adaptation |
| **FC0013** | 0xC6 | 22-1100 MHz | Planned | Requires adaptation |
| **FC2580** | 0xAC | 146-308, 438-924 MHz | Planned | Multi-band, requires adaptation |

**Auto-detection**: The driver probes each address in order (E4000 → FC0013 → R820T → R828D → FC2580 → FC0012) and initializes the first responding tuner.

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
│   └── udp2tcp_bridge.c               # Host-side UDP→TCP bridge
│
└── test/
    └── analyze_spectrum.py            # IQ spectral analysis
```

## Roadmap

### Near-term (pending merge into `main`)

- Merge `feature/websdr` — NB/NR/notch DSP, WBFM player, expanded decoder test matrix
- Merge `feature/aprs` — APRS iGate as runtime-selectable module
- Merge `feature/ais`  — AIS maritime decoder
- Merge `feature/adsb` — ADS-B aircraft tracker with STALL-cascade USB fix
- Cherry-pick direct-sampling / offset-tuning / IF-gain hardening from `feature/wifimgr`

### Medium-term

- Promote `feature/wspr-ft8` to production once field-verified with PSKreporter upload path
- Runtime mode switch ("receiver profile") — swap between raw IQ / APRS / AIS / ADS-B / WSPR without reflashing
- High-rate 2.4+ MSPS Ethernet primary transport
- Expanded tuner support: **E4000**, **FC0012**, **FC0013**, **FC2580** (driver framework already probes their I2C addresses)

### Research

- **GSM FCCH-based PPM calibration** (`feature/gsm-kalibrate`) — a calibration-only tool that uses the known 67.7 kHz GSM frequency-correction burst as a clock reference to measure and correct the RTL-SDR crystal offset. Same principle as the classic `kalibrate-rtl` utility.
- **Cell carrier detection / spectrum survey** (`feature/gsm-lte`) — FFT-based scanner that identifies occupied cellular channels (ARFCN / EARFCN presence) across GSM bands and LTE PSS/SSS detection for educational RF-survey use. Passive spectrum-occupancy measurements only.
- **DVB-T reception** (tracked in the sibling `esp32p4-dvbmod` experiment) — native demodulation of DVB-T transport streams using the RTL2832U's onboard COFDM demodulator.

### Standalone Track (separate product)

- **Standalone FM Receiver firmware** (see [Related Projects](#related-projects)) — headless FM stereo + RDS receiver with local USB audio output. Not a merge into `main`; maintained on its own release track.

## Development History

### Milestone: USB HS Enumeration (214 kSPS)
- **Date**: 2025-12-xx
- **Achievement**: RTL2832U device enumeration and control transfer support on ESP32-P4 HS
- **Issue**: Initial USB driver port from xtrsdr, register access working
- **Fix**: Standard ESP-IDF USB host API, claim interface, vendor control transfers

### Milestone: Demod Initialization (558 kSPS)
- **Date**: 2025-12-xx
- **Achievement**: FIR filter loading, sample rate control functional
- **Issue**: IQ samples were DC-offset, spurious tones
- **Fix**: Added missing Zero-IF disable register (demod 1:0xb1 = 0x1a, not 0x1b)

### Milestone: IF Frequency Setup (720 kSPS)
- **Date**: 2026-01-xx
- **Achievement**: Tuner IF offset registered with demod DDC
- **Issue**: Center frequency off by 3.57 MHz (IF frequency)
- **Fix**: Write demod registers 1:0x19, 1:0x1a, 1:0x1b with IF offset value

### Milestone: EPA Register Correction (891 kSPS)
- **Date**: 2026-01-xx
- **Achievement**: USB endpoint attributes fixed for High-Speed
- **Issue**: wIndex computations wrong, EPA register address off by 0x2000
- **Fix**: EPA @ 0x2148-0x2158 (not 0x0148), EPA_MAXPKT = 0x0002 for HS

### Milestone: SDIO Bottleneck Identified (813 kSPS regression)
- **Date**: 2026-02-xx
- **Achievement**: Profiling revealed WiFi 6 SDIO as throughput limit, not USB
- **Issue**: Attempt to run SDIO at 40 MHz caused volatility
- **Fix**: Revert to stable 25 MHz; 1 MSPS comfortable, 2 MSPS marginal on WiFi

### Milestone: Ring Buffer Optimization (1025 kSPS stable)
- **Date**: 2026-02-xx
- **Achievement**: Sustainable 1+ MSPS over WiFi with 2 MB ring buffer
- **Issue**: WiFi jitter caused occasional sample drops on TCP
- **Fix**: Larger ring buffer (2-8 MB configurable), TCP_NODELAY flag, dual-core pinning

### Spectral Validation (96.4% match)
- **Date**: 2026-03-15
- **Achievement**: IQ quality verified against direct USB reference at 936 MHz
- **Metric**: 96.4% spectral correlation, <2 dB dynamic range difference
- **Method**: Python spectral analyzer compares FFT profiles, peak matching, RMSE

## References and Links

### Hardware & Boards

- [Waveshare ESP32-P4-WIFI6](https://www.waveshare.com/wiki/ESP32-P4-WIFI6)
- [Waveshare ESP32-P4-NANO](https://www.waveshare.com/esp32-p4-nano.htm)
- [RTL-SDR Blog V4](https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/)

### ESP32-P4 Documentation

- [ESP32-P4 Product Page](https://www.espressif.com/en/products/socs/esp32-p4)
- [ESP-IDF USB Host API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/usb_host.html)
- [ESP-Hosted WiFi over SDIO](https://github.com/espressif/esp-hosted)
- [Known USB HS Bug (IDFGH-16415)](https://github.com/espressif/esp-idf/issues/17550)

### RTL-SDR & Protocol

- [RTL-TCP Protocol Specification (K3XEC)](https://k3xec.com/rtl-tcp/)
- [RTL-TCP Alternative Doc (hz.tools)](https://hz.tools/rtl_tcp/)
- [librtlsdr Canonical Source](https://github.com/steve-m/librtlsdr)
- [RTL-SDR Blog Fork](https://github.com/rtlsdrblog/rtl-sdr-blog)

### Prior Art & References

- [xtrsdr: RTL-SDR on ESP32-S2](https://github.com/XTR1984/xtrsdr) — Excellent code reference
- [RTL2832U on STM32](https://fallingnate.svbtle.com/rtl2832-usb-stm32-pt1) — MCU-level USB proof-of-concept
- [RTL2832U USB Overflow (IDFGH-10944)](https://github.com/espressif/esp-idf/issues/12137) — Why ESP32-P4 was needed

## Related Projects

Additional ESP32-P4 SDR experiments live in sibling worktrees under the same parent directory. They share hardware and much of the driver core, but diverge in target use case:

| Project | Worktree / Branch | Purpose |
|---------|-------------------|---------|
| **Standalone FM Receiver** | `../esp32p4-standalone/` (`feature/standalone`) | Headless FM stereo + RDS receiver with local **USB audio output** — no WiFi client needed. Target: portable / car / bedside radio. |
| Standalone (dev tracks) | `standalone-mono-nonopt`, `standalone-optimized` | Development experiments exploring cascaded 19 kHz notch, IQ-level squelch, and `-O3`/`-Ofast` compiler-flag trade-offs for the FM chain. |
| WiFi Manager + Extras | `../esp32p4-wifi-rtlsdr-wifimgr/` (`feature/wifimgr`) | Super-set of `main` with PSKreporter HTTPS upload, network forwarders, Telegram notification bot, AX.25 multi-baud DSP, and production-hardened ADS-B / AIS decoders. Features trickle back into `main` once validated. |

## License

**GPL-2.0-or-later** — Following librtlsdr and xtrsdr precedent.

This project is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

See [LICENSE](LICENSE) for full text.

---

**Questions?** Open an issue on GitHub or refer to the detailed technical documentation in `docs/`.
