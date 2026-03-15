# Hardware Setup Guide

Step-by-step physical assembly, pin connections, and troubleshooting for the ESP32-P4 RTL-SDR WiFi Bridge.

## Required Hardware

### Bill of Materials

| Item | Model | Qty | Cost | Notes |
|------|-------|-----|------|-------|
| Main board | Waveshare ESP32-P4-WIFI6 | 1 | $30 | Alternative: ESP32-P4-NANO |
| SDR dongle | RTL-SDR Blog V4 | 1 | $30 | Generic RTL2832U also supported |
| USB cable | USB-A to USB-A or USB-C | 1 | $5 | For connecting dongle to P4 USB port |
| Antenna | Depends on frequency | 1 | $10+ | FM: λ/4 @ 100 MHz = 75 cm rod |
| Power supply | USB 5V/2A | 1 | $10 | Can use PC USB port (limited current) |
| Serial cable | USB-UART (CP2102/CH340) | 1 | $8 | For serial console (optional, debug only) |
| **Total** | | | **$83-93** | |

### Optional Components

- **Ethernet cable + switch**: For high-rate operation (2+ MSPS)
- **Solar panel + LiFePO4**: For permanent outdoor deployment
- **Enclosure**: Weatherproof housing (plastic project box, ~$5)
- **Heatsink**: For ESP32-P4 if in hot environment
- **Bias-T connector**: If antenna has active elements

## Physical Assembly

### Waveshare ESP32-P4-WIFI6 Pin Layout

```
┌──────────────────────────────────────────────────┐
│ Waveshare ESP32-P4-WIFI6 (top view)              │
├──────────────────────────────────────────────────┤
│                                                  │
│  ┌────────┐                                      │
│  │ USB-A  │  ← Main USB port (HS, plug dongle)  │
│  │ (HS)   │                                      │
│  └────────┘                                      │
│  ┌────────┐  ┌────────┐  ┌────────┐             │
│  │ USB-A  │  │ USB-A  │  │ USB-A  │  (FS)      │
│  └────────┘  └────────┘  └────────┘             │
│                                                  │
│  ┌────────────────────────────────────────────┐ │
│  │ ESP32-P4 & ESP32-C6 (stacked)              │ │
│  │ + PSRAM 32MB                               │ │
│  └────────────────────────────────────────────┘ │
│                                                  │
│  ┌────────────────────────────────────────────┐ │
│  │ GPIO Headers (2×20 pin)                    │ │
│  │  GND, 3V3, GPIO 0-9, 15-48                 │ │
│  │                                             │ │
│  │  Default bias-T: GPIO4 output               │ │
│  └────────────────────────────────────────────┘ │
│                                                  │
│  ┌────────────────────────────────────────────┐ │
│  │ Ethernet RJ45 (100 Mbps)                   │ │
│  └────────────────────────────────────────────┘ │
│                                                  │
└──────────────────────────────────────────────────┘
```

### Step 1: Identify USB Ports

The Waveshare ESP32-P4-WIFI6 has **4 USB-A ports**:

```
Port Layout (viewed from front):
┌─────────────────────┐
│ [HS1]  [FS1] [FS2]  │  Top row
│         [FS3]       │  Bottom (sometimes not populated)
└─────────────────────┘
```

**Recommended**: Plug RTL-SDR into the **leftmost port (HS)**. This is the High-Speed port; the others are Full-Speed and will cause babble errors.

**How to verify**: Check PCB silkscreen or test both—if stuck at 240 kHz, wrong port.

### Step 2: Connect RTL-SDR Dongle

1. **Power off** the board (unplug USB)
2. **Insert RTL-SDR Blog V4** into the High-Speed USB-A port
   - Use a short USB-A to micro-USB cable if dongle has micro-B connector
   - Push firmly until fully seated
3. **Attach antenna** to SMA connector on dongle
   - For FM reception: ~75 cm monopole rod at 90°
   - For broadband: WiFi antenna or discone

### Step 3: Power Connection

**Option A: Via USB (simplest)**
- Connect USB 5V/2A supply to main board's **USB device port** (usually marked with a small arrow)
- Board will power both ESP32-P4, C6, and RTL-SDR

**Option B: Via barrel jack (if available)**
- Some Waveshare boards have optional 5.5×2.1 mm barrel jack
- 5V positive center, GND outer sleeve
- Provides more stable power than USB bus

**Option C: Via GPIO (advanced)**
- For persistent outdoor installations, solder 5V/GND wires to GPIO header
- Use an inline power connector for serviceability
- Requires care with polarity

### Step 4: Serial Console (Optional)

For debugging, connect a USB-UART adapter:

```
Waveshare     USB-UART
────────────────────────
GPIO1 (TX)  ← RX
GPIO0 (RX)  → TX
GND         ← GND
3V3         → 3V3 (optional)
```

- **Baud rate**: 115200
- **Terminal**: `picocom /dev/ttyUSB0 -b 115200` (Linux)
  or PuTTY (Windows)

This is optional; you can use `idf.py monitor` via the main USB port.

### Step 5: Ethernet (Optional)

For sustained 2+ MSPS operation, connect to Ethernet:

```
Waveshare RJ45 ← Standard Cat5e/Cat6 cable to switch/router

Features:
  ├─ Auto-negotiation (10/100 Mbps)
  ├─ No software config needed (DHCP auto)
  └─ Provides ~11 MB/s TCP (2.4+ MSPS capable)
```

If both WiFi and Ethernet connected, WiFi is primary; Ethernet acts as fallback.

## ESP32-P4 Pin Mapping (for reference)

```
GPIO   Name          Function          Default
────────────────────────────────────────────────
0      GPIO0         UART RX (strapping pin)
1      GPIO1         UART TX
2      GPIO2         strap pin (HS USB select)
3      GPIO3         U0RXD
4      GPIO4         BIAS_TEE (output)
33-39  SDIO pins     WiFi (SDIO to C6)
       CLK,CMD,D0-D3

35     GPIO35        SDIO CLK
33     GPIO33        SDIO CMD
34-39  GPIO34-39     SDIO D0-D3
```

**Bias-T control**: Default GPIO4
- Set HIGH (3.3V) to enable antenna power (~5V on connector)
- Set LOW (0V) to disable

To use different GPIO:
```bash
idf.py menuconfig
# Advanced → Bias-T GPIO (change to your pin)
```

## First Boot

### 1. Initial Connection

```bash
# Terminal 1: Build and flash
cd ~/esp32p4-wifi-rtlsdr
source ~/esp/v5.5.1/esp-idf/export.sh
idf.py menuconfig
# Set WiFi SSID / Password
# (Change defaults from dev credentials if needed)

idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### 2. Expected Log Output

```
[00:00:00] ESP-IDF v5.5.1 initializing...
[00:00:01] Initializing ESP-Hosted...
[00:00:02] Connecting to C6 slave over SDIO...
[00:00:03] ESP-Hosted connected to C6
[00:00:04] WiFi STA starting...
[00:00:05] WiFi STA connecting to 'YourSSID'...
[00:00:08] Got IP: 192.168.1.232
[00:00:09] USB Host library installed
[00:00:10] Waiting for RTL-SDR device...
```

At this point, **plug in the RTL-SDR dongle** (if not already connected).

```
[00:00:12] RTL-SDR device detected!
[00:00:12] VID:PID = 0x0bda:0x2838 (RTL-SDR Blog V4)
[00:00:13] RTL2832U demodulator initializing...
[00:00:14] Writing FIR filter coefficients...
[00:00:15] Tuner detection sequence...
[00:00:15] Probing at I2C 0xC8 (E4000): no response
[00:00:16] Probing at I2C 0xC6 (FC0013): no response
[00:00:16] Probing at I2C 0x34 (R820T): no response
[00:00:17] Probing at I2C 0x74 (R828D): FOUND
[00:00:18] R828D tuner detected!
[00:00:19] R828D initialization complete
[00:00:20] RTL-TCP server listening on 0.0.0.0:1234
[00:00:20] UDP streaming disabled (compile-time option)
[00:00:20] mDNS service registered: esp32p4-rtlsdr._rtl_tcp._tcp.local
[00:00:21] Ready for IQ streaming
```

### 3. Get IP Address

From the log output, note the IP address (example: 192.168.1.232). If DHCP failed:

```bash
# Find the board via mDNS
dns-sd -B _rtl_tcp._tcp local

# Or scan for it:
nmap -sn 192.168.1.0/24 | grep -i "rtl\|esp"
```

### 4. Connect SDR Client

**SDR++** (recommended):
1. Source → RTL-TCP
2. Address: `192.168.1.232` (or `esp32p4-rtlsdr.local`)
3. Port: `1234`
4. Connect

**GQRX**:
1. Input → Other → rtl_tcp
2. Same address/port

**Command line**:
```bash
rtl_sdr -a 192.168.1.232 -f 936000000 -s 1024000 -g 40 - | xxd | head
```

## Troubleshooting

### Board Won't Flash

**Symptom**: `esptool.py: A fatal error occurred`

**Solutions**:
```bash
# 1. Check USB cable (some are charge-only, not data)
# 2. Try different USB port on your computer
# 3. Put board in download mode:
#    - Hold "Boot" button
#    - Press "Reset" button
#    - Release "Boot"
#    - Immediately run: idf.py flash

# 4. Erase entire flash:
idf.py erase-flash
idf.py build flash monitor
```

### WiFi Won't Connect

**Symptom**: Log shows `WIFI_EVENT_STA_DISCONNECTED` repeated

**Check**:
```bash
# 1. Verify SSID and password in Kconfig.projbuild
idf.py menuconfig
# RTL-SDR WiFi Bridge Configuration
#   → WiFi SSID: (check spelling, case-sensitive)
#   → WiFi Password: (check spaces, special chars)

# 2. Try 2.4 GHz band (5 GHz may not be supported on all boards)

# 3. Check WiFi channel (1-11 in US, 1-13 in Europe)
#    Spectrum analyzer on your router settings

# 4. Increase WiFi timeout in main/main.c:
//   esp_wifi_connect() → add retry loop
```

### RTL-SDR Not Detected

**Symptom**: Log shows "Waiting for RTL-SDR device..." but never "detected"

**Check**:
```bash
# 1. Is dongle in the HIGH-SPEED (leftmost) USB port?
#    If in FS port → transfer will stall

# 2. Try different USB cable
#    (some cables lack HS capability)

# 3. Check dongle power consumption
#    Blog V4 draws 200 mA; ensure PSU can supply

# 4. Look at USB host log verbosity
idf.py menuconfig
# → Component Config → USB Host
#   → Log (level) [Debug]
idf.py build flash monitor

# 5. Manual USB enumeration test
#    Connect to laptop with libusb tools:
lsusb | grep 0bda:2838
```

### No IQ Data Received

**Symptom**: Can connect to RTL-TCP server, but no samples arrive

**Check**:
```bash
# 1. Antenna connected to RTL-SDR?

# 2. Verify demod registers
#    Add debug logging in rtlsdr.c:
//   rtlsdr_demod_write_reg(...) → log wValue, data

# 3. Test with known strong signal
#    (local FM station, WiFi routers broadcast beacon)
# Frequency ranges:
#   FM broadcast: 88-108 MHz
#   WiFi 2.4 GHz: 2412-2472 MHz
#   WiFi 5 GHz:   5150-5850 MHz

# 4. Check ring buffer
#    Add debug: printf("ring buffer: %d / %d bytes\n", ...);
```

### WiFi Throughput Very Low (< 500 kSPS)

**Symptom**: Connects successfully, but sample rate capped at 500 kSPS

**Check**:
```bash
# 1. WiFi signal strength
#    Log shows RSSI (dBm); target > −70 dBm
#    If < −80 dBm, move closer or reorient antenna

# 2. WiFi channel congestion
#    Use WiFi scanner app: "WiFi Analyzer" (Android)
#    Switch to less-crowded channel in router

# 3. Ethernet available?
#    Switch to Ethernet for 2.4+ MSPS guaranteed:
#    Connect RJ45 cable → same local subnet

# 4. SDIO clock speed
idf.py menuconfig
# → RTL-SDR WiFi Bridge Config
#   → SDIO Clock: 40 MHz (try if stable)
#   Reset if unstable: revert to 25 MHz

# 5. TCP window size tuning (expert)
#    In lwIP config, increase TCP_WND (default 65534)
```

### USB Device Keeps Disconnecting

**Symptom**: "USB device gone" / "attach/detach loop"

**Check**:
```bash
# 1. Power supply insufficient
#    Blog V4: 200 mA @ 5V
#    Plus ESP32-P4: 150 mA
#    Total: 350 mA needed (use 2A+ supply)

# 2. USB cable too long or low quality
#    Use <2 meter cable, high-speed rated

# 3. USB hub in between (if using hub)
#    Try direct connection to board

# 4. Thermal throttle
#    Feel the RTL-SDR dongle (should be warm, not hot)
#    Add heatsink if hot
```

### Antenna Doesn't Pick Up Signals

**Symptom**: Low signal levels even for strong local transmitters

**Check**:
```bash
# 1. Antenna orientation
#    For vertical monopole (rod): point UP
#    For dipole: point perpendicular to transmitter
#    For WiFi: experiment (typically horizontal or diagonal)

# 2. Antenna impedance mismatch
#    RTL-SDR wants 50 Ω, cheap antennas may be 75 Ω
#    High VSWR (voltage standing wave ratio) causes loss
#    Try different antenna to isolate

# 3. Wrong frequency range
#    Blog V4 covers 24-1700 MHz
#    Antenna must be designed for that frequency
#    Example: FM antenna won't work well for 2.4 GHz

# 4. Bias-T not enabled (if active antenna)
#    Check: rtlsdr_set_bias_tee(dev, 1);
#    GPIO4 must output 3.3V

# 5. RF interference
#    Try shielded USB cable
#    Avoid placing near high-power devices (microwaves, WiFi)
```

### Spectral Output Distorted

**Symptom**: FFT shows noise floor, no clear peaks

**Check**:
```bash
# 1. Wrong center frequency
#    Tune to known transmitter frequency
#    FM: 104.5 MHz (local station)
#    WiFi: 2437 MHz (WiFi channel 6)

# 2. AGC disabled or gain too low
#    Set manual gain to 40 dB (high):
#    rtlsdr_set_tuner_gain_mode(dev, 1);  // manual
#    rtlsdr_set_tuner_gain(dev, 400);     // 40.0 dB

# 3. Sample rate mismatch
#    Capture must use exact sample rate as SDR set it to
#    Default: 1.024 MSPS
#    Check: rtlsdr_get_sample_rate(dev);

# 4. I/Q balance
#    Check DC offset and imbalance in I/Q data
#    Should be centered near 127.5 (mid-range of uint8)
```

## Performance Optimization

### WiFi Throughput Tuning

```c
// In main.c, after WiFi init:

// 1. Enable WiFi power save (reduces latency)
wifi_ps_type_t ps_type = WIFI_PS_NONE;  // no sleep
esp_wifi_set_ps(ps_type);

// 2. Set TCP_NODELAY (disable Nagle algorithm)
int flag = 1;
setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

// 3. Increase TCP window size
int window_size = 65534;
setsockopt(client_fd, SOL_SOCKET, SO_RCVBUF, &window_size, sizeof(window_size));
```

### USB Stability

```bash
# In sdkconfig.defaults or menuconfig:
CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED=y  # Critical for HS stability

# Increase USB transfer timeout if experiencing stalls
idf.py menuconfig
# → Component Config → USB Host
#   → Transfer Timeout (ms): 5000 (default 1000)
```

### Ring Buffer Tuning

```bash
idf.py menuconfig
# → RTL-SDR WiFi Bridge Config
#   → Ring buffer size: increase from 2 MB to 4-8 MB if dropping packets
#     (uses more PSRAM, but absorbs WiFi jitter better)
```

## Field Deployment Checklist

- [ ] RTL-SDR dongle in HIGH-SPEED USB port (leftmost)
- [ ] Antenna attached and oriented correctly
- [ ] Power supply capable of 2A @ 5V
- [ ] WiFi SSID/password configured and tested
- [ ] Can connect from SDR client (ping/RTL-TCP connection)
- [ ] Spectral output validated against known signals
- [ ] Ring buffer size adequate for WiFi conditions (test at 1 MSPS)
- [ ] Bias-T enabled (if using active antenna)
- [ ] Serial console accessible (if troubleshooting needed)
- [ ] Ethernet cable available (fallback if WiFi marginal)

## Next Steps

- See [ARCHITECTURE.md](ARCHITECTURE.md) for register-level details
- See [BENCHMARKS.md](BENCHMARKS.md) for throughput validation
- See [POWER_BUDGET.md](POWER_BUDGET.md) for battery/solar sizing

---

**Support**: Open an issue with logs and platform details if stuck.
