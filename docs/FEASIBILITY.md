# ESP32-P4 RTL-SDR USB Host → WiFi RTL-TCP Bridge

## Feasibility Analysis & Architecture Plan

### 1. Concept

Use a Waveshare ESP32-P4 board as a standalone RTL-SDR receiver:
- ESP32-P4 acts as **USB Host** for an RTL-SDR dongle (RTL2832U + R820T/R860)
- IQ samples streamed over **WiFi** using the **RTL-TCP protocol**
- Remote SDR clients (GQRX, SDR#, SDR++, etc.) connect and control tuning

```
[RTL-SDR Dongle] --USB 2.0 HS--> [ESP32-P4] --WiFi (TCP)--> [SDR Client]
                                      ^                           |
                                      |     RTL-TCP commands      |
                                      +---------------------------+
```

---

### 2. Prior Art

#### xtrsdr (ESP32-S2 + RTL2832U) — Most Complete Attempt
- **Repo**: [github.com/XTR1984/xtrsdr](https://github.com/XTR1984/xtrsdr)
- Ported librtlsdr to ESP-IDF USB host on ESP32-S2FN4R2
- Streams to GQRX, SDR#, SDR++ over WiFi and Ethernet (W5500 SPI)
- **Max sample rates**: 240 kHz (WiFi), 300 kHz (Ethernet)
- **Fundamental limitation**: ESP32-S2 is USB Full-Speed only (12 Mbps)

#### The Babble Error Problem (IDFGH-10944)
- RTL2832U is natively a **USB High-Speed device** (512-byte MPS)
- When forced to enumerate at Full-Speed (64-byte MPS) on ESP32-S2/S3, it commits
  **USB babble errors** — sending more data than the FS MPS allows
- Result: `USB_TRANSFER_STATUS_OVERFLOW` on every bulk IN transfer
- Espressif closed as "Won't Do" — device non-compliance, not a driver bug
- **ESP32-P4 is the first ESP32 variant where RTL-SDR can enumerate correctly at HS**

#### Other Attempts
- **kvhnuke/esp32-rtl-sdr**: 2 commits, no docs, unclear status
- **STM32 port** (fallingnate.svbtle.com): Proves MCU-level USB host control of RTL2832U is viable

---

### 3. Hardware Platform

#### ESP32-P4 SoC
| Feature | Specification |
|---------|--------------|
| CPU | Dual-core RISC-V, 400 MHz (HP core) + 40 MHz (LP core), FPU |
| HP SRAM | 768 KB on-chip (L2MEM, usable as cache or direct) |
| LP SRAM | 32 KB |
| TCM | 8 KB zero-wait |
| PSRAM | Up to 32 MB stacked in-package |
| USB | **Two USB 2.0 OTG**: one **High-Speed** (480 Mbps), one Full-Speed (12 Mbps) |
| WiFi | **None built-in** — requires companion chip (ESP32-C6 via SDIO) |
| Ethernet | EMAC with RMII (100 Mbps) |
| DMA | GDMA-AHB (6ch, SRAM only) + **GDMA-AXI** (6ch, SRAM+PSRAM) + 2D-DMA |

**USB Host constraint**: Only one USB OTG peripheral can operate as host at a time (current software limitation in ESP-IDF, not hardware).

**Known ESP32-P4 USB HS bug (IDFGH-16415)**: Bulk OUT transfers silently fail (0 bytes transferred) if FIFO is misconfigured. **Fix**: set `CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED=y` in sdkconfig. Resolved in recent ESP-IDF.

#### Waveshare ESP32-P4-WIFI6 Board
- ESP32-P4 + ESP32-C6-MINI (WiFi 6 / BLE 5 via SDIO)
- **4x USB 2.0 Type-A ports** (at least one on HS peripheral)
- 16 MB Flash, 32 MB PSRAM
- Ethernet RJ45 (100 Mbps)
- MIPI-DSI/CSI connectors
- ~$25-35 USD

#### Alternative: ESP32-P4-NANO
- Smaller form factor (credit-card sized)
- Same ESP32-P4 + ESP32-C6 combo
- USB-A host port(s) available

---

### 3. RTL-SDR USB Requirements

#### USB Protocol
| Aspect | Detail |
|--------|--------|
| USB Speed | **High-Speed (480 Mbps)** — RTL2832U is a HS device |
| Device Class | Vendor-specific (`bInterfaceClass=0xFF`) |
| Control EP | EP0 — vendor control transfers for register read/write |
| Data EP | **EP 0x81** — Bulk IN for IQ sample stream (512-byte MPS at HS) |
| VID:PID | `0x0bda:0x2832` (generic) / `0x0bda:0x2838` (RTL-SDR Blog) |
| No OUT EP | All configuration via EP0 vendor control transfers only |

#### USB Control Transfer Format (Register Access)
```
bmRequestType: 0xC0 (read) / 0x40 (write)  — Vendor, device-level
bRequest:      0x00
wValue:        register address
wIndex:        (block << 8) | 0x10 (write) or (block << 8) (read)
wLength:       1-4 bytes
Timeout:       300ms
```

#### RTL2832U Initialization Sequence (from librtlsdr)
1. `libusb_claim_interface(dev, 0)`
2. Dummy vendor write to USB_SYSCTL (test comms / reset)
3. `rtlsdr_init_baseband()` — load FIR filter coefficients, configure DDC
4. Tuner probe loop over I2C repeater (tries E4000, FC0013, R820T, R828D, FC2580, FC0012)
5. Tuner-specific init (IF frequency, zero-IF or low-IF mode)

#### Sample Rate Configuration
Written to demod registers as a rational resampling ratio from 28.8 MHz crystal:
```c
rsamp_ratio = (rtl_xtal * pow(2,22)) / sample_rate;
// Written to demod block 1, regs 0x9F (hi16) and 0xA1 (lo16)
```

#### Data Rates at Common Sample Rates
| Sample Rate | IQ Throughput | Notes |
|-------------|--------------|-------|
| 0.25 MSPS | 0.5 MB/s | Narrowband FM |
| 1.0 MSPS | 2.0 MB/s | General purpose |
| 2.048 MSPS | 4.096 MB/s | RTL-TCP default |
| 2.4 MSPS | 4.8 MB/s | Common max stable |
| 3.2 MSPS | 6.4 MB/s | Theoretical max, often unstable |

Each sample = 8-bit I + 8-bit Q = 2 bytes.

---

### 4. RTL-TCP Protocol

**Default port**: 1234

#### Server → Client (on connect)
12-byte `DongleInfo` header:
```
Bytes 0-3:  Magic "RTL0" (0x52544C30)
Bytes 4-7:  Tuner type (uint32, network byte order)
Bytes 8-11: Gain count (uint32, network byte order)
```
Followed by continuous IQ stream (interleaved uint8 I,Q pairs).

#### Client → Server (commands)
5-byte packets: 1 byte command + 4 bytes argument (big-endian uint32):

| Cmd | Function | Argument |
|-----|----------|----------|
| 0x01 | Set center frequency | Hz |
| 0x02 | Set sample rate | samples/sec |
| 0x03 | Set gain mode | 0=auto, 1=manual |
| 0x04 | Set gain | tenths of dB |
| 0x05 | Set freq correction | PPM |
| 0x06 | Set IF gain | stage<<16 | gain |
| 0x07 | Set test mode | 0/1 |
| 0x08 | Set AGC mode | 0/1 |
| 0x09 | Set direct sampling | 0=off, 1=I, 2=Q |
| 0x0a | Set offset tuning | 0/1 |
| 0x0b | Set RTL xtal freq | Hz |
| 0x0c | Set tuner xtal freq | Hz |
| 0x0d | Set gain by index | index |
| 0x0e | Set bias tee (GPIO 0) | 0/1 |

All values are **big-endian** (network byte order). Commands can be sent at any time
during streaming and take effect immediately.

---

### 5. Bandwidth Budget Analysis

#### Critical Path: USB → ESP32-P4 → WiFi → Client

```
USB HS Bulk IN:  480 Mbps theoretical, ~30-40 MB/s practical
     ↓
ESP32-P4 CPU:    400 MHz dual-core RISC-V (buffer management, no DSP needed)
     ↓
PSRAM Buffer:    Ring buffer in 32 MB PSRAM
     ↓
SDIO → ESP32-C6: ~160 Mbps raw, ~36 Mbps TCP throughput (WiFi 6 HT20)
     ↓
TCP Socket:      RTL-TCP stream to client
```

#### Bottleneck Analysis

| Segment | Capacity | Required (2.048 MSPS) | Required (1.0 MSPS) | Margin |
|---------|----------|----------------------|---------------------|--------|
| USB HS Bulk IN | ~30 MB/s | 4.1 MB/s | 2.0 MB/s | **7-15x** ✅ |
| ESP32-P4 CPU | 400 MHz | Trivial (memcpy) | Trivial | ✅ |
| PSRAM bandwidth | ~80 MB/s | 8.2 MB/s (R+W) | 4.0 MB/s | **10-20x** ✅ |
| WiFi TCP throughput | **~4.5 MB/s** (36 Mbps) | **4.1 MB/s** | 2.0 MB/s | **⚠️ TIGHT / ✅** |

**WiFi is the bottleneck.**

- At **1.0 MSPS** (2.0 MB/s): **Comfortable** — 56% of WiFi capacity
- At **2.048 MSPS** (4.1 MB/s): **Very tight** — 91% of WiFi capacity, packet loss likely
- At **2.4 MSPS** (4.8 MB/s): **Exceeds** WiFi capacity, will drop samples

#### Ethernet Alternative
- 100 Mbps Ethernet → ~11 MB/s TCP → supports **all sample rates** easily
- Recommended for fixed installations

---

### 6. Feasibility Verdict

| Scenario | Feasible? | Notes |
|----------|-----------|-------|
| WiFi @ ≤1.0 MSPS | **YES** ✅ | Solid margin, reliable |
| WiFi @ 2.048 MSPS | **MARGINAL** ⚠️ | May work with tuning, risk of drops |
| WiFi @ 2.4+ MSPS | **NO** ❌ | Exceeds WiFi throughput |
| Ethernet @ any rate | **YES** ✅ | 100 Mbps handles everything |
| USB Host for RTL2832U | **YES** ✅ | HS bulk transfers supported |
| RTL-TCP server impl | **YES** ✅ | Simple protocol, well-documented |

**Overall: FEASIBLE with WiFi at moderate sample rates (≤1 MSPS comfortable, ~2 MSPS with care). Ethernet recommended for full-rate operation.**

---

### 7. Software Architecture

```
┌─────────────────────────────────────────────────┐
│                  ESP32-P4 Firmware               │
├─────────────────────────────────────────────────┤
│                                                  │
│  ┌──────────────┐    ┌────────────────────────┐ │
│  │  USB Host     │    │  RTL-TCP Server        │ │
│  │  Driver       │    │                        │ │
│  │  ┌──────────┐ │    │  - TCP listener :1234  │ │
│  │  │RTL2832U  │ │    │  - DongleInfo header   │ │
│  │  │Control   │ │    │  - IQ stream relay     │ │
│  │  │(EP0 ctrl)│ │    │  - Command parser      │ │
│  │  └──────────┘ │    │  - Multi-client?       │ │
│  │  ┌──────────┐ │    └─────────┬──────────────┘ │
│  │  │Bulk IN   │ │              │                │
│  │  │Async Read│──────> Ring ───┘                │
│  │  │(EP 0x81) │ │    Buffer                     │
│  │  └──────────┘ │   (PSRAM)                     │
│  └──────────────┘                                │
│                                                  │
│  ┌──────────────┐    ┌────────────────────────┐ │
│  │  WiFi Stack   │    │  Device Manager        │ │
│  │  (ESP-Hosted  │    │  - USB hotplug         │ │
│  │   via C6)     │    │  - RTL2832U init seq   │ │
│  └──────────────┘    │  - Tuner detection      │ │
│                       └────────────────────────┘ │
└─────────────────────────────────────────────────┘
```

#### Core Components

1. **USB Host RTL2832U Driver**
   - Enumerate device, match VID:PID
   - Claim interface, open bulk EP 0x81
   - Port librtlsdr register read/write (vendor control transfers)
   - Initialize RTL2832U + detect tuner (R820T/R820T2/R860)
   - Async bulk IN transfers with double/triple buffering

2. **Ring Buffer (PSRAM)**
   - Decouple USB read rate from TCP send rate
   - Size: 2-8 MB in PSRAM (configurable)
   - Lock-free SPSC (single-producer single-consumer) if single client

3. **RTL-TCP Server**
   - TCP listener on port 1234
   - Send 12-byte DongleInfo on connect
   - Stream IQ data from ring buffer
   - Parse 5-byte commands, translate to USB control transfers
   - Handle client disconnect gracefully

4. **WiFi / Network Stack**
   - ESP-Hosted (SDIO) for WiFi via ESP32-C6
   - lwIP TCP stack
   - Optional: mDNS for service discovery (e.g., `_rtl_tcp._tcp`)

#### Task Architecture (FreeRTOS)

| Task | Core | Priority | Role |
|------|------|----------|------|
| USB Host Library | Core 0 | High | USB event processing |
| USB Bulk Reader | Core 0 | High | Async bulk IN → ring buffer |
| TCP Server | Core 1 | Medium | Accept connections, send IQ data |
| Command Handler | Core 1 | Medium | Parse RTL-TCP commands → USB control |
| WiFi/Network | Core 1 | Medium | ESP-Hosted network stack |

---

### 8. Key Implementation Challenges

#### Challenge 1: RTL2832U USB Driver
- No existing ESP-IDF driver for RTL2832U **on ESP32-P4** (HS)
- **xtrsdr** already ported librtlsdr to ESP-IDF on ESP32-S2 (FS) — excellent code reference
- Must adapt xtrsdr's register-level init to use HS bulk transfers (512-byte MPS vs 64)
- Control transfers: `usb_host_transfer_submit()` with vendor request type (`bmRequestType=0x40/0xC0`)
- Bulk reads: submit async bulk IN transfers on EP 0x81, handle in callback
- Default librtlsdr uses 15 × 256 KB async transfer buffers — adapt sizing for ESP32-P4 PSRAM
- **References**: xtrsdr (ESP-IDF port), STM32 port (fallingnate.svbtle.com), librtlsdr source

#### Challenge 2: WiFi Throughput
- ESP32-C6 single-core 160 MHz is the weak link
- Optimize: larger TCP segments, minimize copies, TCP_NODELAY
- Consider adaptive sample rate based on link quality
- Fallback: Ethernet for reliable high-rate operation

#### Challenge 3: Buffer Management
- USB bulk transfers come in fixed-size chunks (typically 512B × N)
- TCP sends may stall briefly (WiFi jitter)
- Ring buffer in PSRAM absorbs bursts
- Must handle overflow gracefully (drop oldest samples, not newest)

#### Challenge 4: Tuner Initialization
- Must detect tuner type (R820T, R820T2, R860, E4000, FC0012, etc.)
- Each tuner has different I2C register sequences
- Start with R820T/R820T2 (most common in modern dongles)

---

### 9. Development Phases

#### Phase 1: USB Enumeration & Control
- Set up ESP-IDF project for ESP32-P4
- USB host: enumerate RTL-SDR, read descriptors
- Implement vendor control transfers (register R/W)
- Initialize RTL2832U demodulator

#### Phase 2: Tuner Init & Bulk Streaming
- Port R820T/R820T2 tuner init from librtlsdr
- Start async bulk IN transfers on EP 0x81
- Verify IQ data in buffer (known test patterns)

#### Phase 3: WiFi + RTL-TCP Server
- Set up ESP-Hosted WiFi via ESP32-C6
- Implement TCP server with RTL-TCP protocol
- Stream IQ data to client (GQRX, SDR++, etc.)
- Implement command handling (freq, gain, sample rate)

#### Phase 4: Optimization & Robustness
- Tune buffer sizes, TCP parameters
- Add mDNS service advertisement
- Handle USB disconnect/reconnect
- Adaptive sample rate based on WiFi conditions
- Power management

---

### 10. Bill of Materials

| Item | Est. Cost |
|------|-----------|
| Waveshare ESP32-P4-WIFI6 | ~$30 |
| RTL-SDR Blog V4 dongle | ~$30 |
| USB-A cable (if needed) | ~$3 |
| Antenna | ~$10 |
| **Total** | **~$73** |

---

### 11. References

#### ESP32-P4 Hardware & Software
- [ESP32-P4 Product Page](https://www.espressif.com/en/products/socs/esp32-p4)
- [ESP32-P4 Announcement](https://www.espressif.com/en/news/ESP32-P4)
- [ESP-IDF USB Host API (ESP32-P4)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/usb_host.html)
- [ESP-IoT-Solution USB Host Solutions](https://docs.espressif.com/projects/esp-iot-solution/en/latest/usb/usb_overview/usb_host_solutions.html)
- [ESP32-P4 HS Bulk OUT Bug (IDFGH-16415)](https://github.com/espressif/esp-idf/issues/17550)
- [ESP32-P4 + C6 WiFi Throughput Test](https://github.com/r4d10n/esp32p4-c6-wifi-test)

#### Waveshare Boards
- [ESP32-P4-WIFI6 Wiki](https://www.waveshare.com/wiki/ESP32-P4-WIFI6)
- [ESP32-P4-NANO](https://www.waveshare.com/esp32-p4-nano.htm)
- [ESP32-P4-Module-DEV-KIT](https://www.waveshare.com/esp32-p4-module-dev-kit.htm)
- [CNX Software — P4 credit-card board with 4x USB-A](https://www.cnx-software.com/2025/04/04/esp32-p4-credit-card-sized-board-features-ethernet-wifi-6-four-usb-ports-40-pin-gpio-header-mipi-dsi-and-csi-connectors/)

#### RTL-SDR & RTL-TCP Protocol
- [RTL-TCP Protocol Overview (K3XEC)](https://k3xec.com/rtl-tcp/)
- [RTL-TCP Protocol Overview (hz.tools)](https://hz.tools/rtl_tcp/)
- [RTL-TCP DeepWiki](https://deepwiki.com/rtlsdrblog/rtl-sdr-blog/5.1-rtl_tcp-network-iq-streaming-server)
- [librtlsdr Source (steve-m, canonical)](https://github.com/steve-m/librtlsdr)
- [librtlsdr Source (rtl-sdr-blog fork)](https://github.com/rtlsdrblog/rtl-sdr-blog)
- [Osmocom RTL-SDR Wiki](https://osmocom.org/projects/rtl-sdr/wiki/Rtl-sdr)

#### Prior Art & MCU Implementations
- [xtrsdr — RTL-SDR on ESP32-S2](https://github.com/XTR1984/xtrsdr)
- [RTL2832U on STM32 MCU (Nate Fisher)](https://fallingnate.svbtle.com/rtl2832-usb-stm32-pt1)
- [RTL2832U USB Bulk Overflow on ESP32-S3 (IDFGH-10944)](https://github.com/espressif/esp-idf/issues/12137)
