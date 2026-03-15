# esp32p4-wifi-rtlsdr

ESP32-P4 USB Host driver for RTL-SDR dongles — streams IQ samples over WiFi/Ethernet using the RTL-TCP protocol.

## What

Turns a Waveshare ESP32-P4 board + RTL-SDR dongle into a wireless SDR receiver. Remote clients (GQRX, SDR++, SDR#, etc.) connect via the standard `rtl_tcp` protocol to receive IQ samples and control tuning parameters.

```
[RTL-SDR Dongle] --USB 2.0 HS--> [ESP32-P4] --WiFi/Ethernet--> [SDR Client]
```

## Why ESP32-P4

The ESP32-P4 is the **first ESP32 variant with USB 2.0 High-Speed** (480 Mbps). Previous variants (S2, S3) only support Full-Speed (12 Mbps), which causes USB babble errors with RTL2832U devices and caps sample rates at ~240 kHz.

## Hardware

- **Waveshare ESP32-P4-WIFI6** (or ESP32-P4-NANO) — ~$30
- **RTL-SDR Blog V4** (or any RTL2832U-based dongle) — ~$30
- Antenna

## Performance

| Transport | Max Sample Rate | Status |
|-----------|----------------|--------|
| WiFi (ESP32-C6, WiFi 6) | ~1 MSPS comfortable, ~2 MSPS marginal | Primary target |
| Ethernet (100 Mbps) | 2.4+ MSPS | Recommended for full-rate |

## Building

```bash
# Set up ESP-IDF v5.5+
. ~/esp/v5.5.1/esp-idf/export.sh

# Configure WiFi credentials
idf.py menuconfig
# → RTL-SDR WiFi Bridge Configuration → WiFi SSID / Password

# Build, flash, monitor
idf.py build flash monitor
```

## Usage

1. Flash the ESP32-P4 board
2. Connect an RTL-SDR dongle to the USB-A port
3. Wait for WiFi connection and RTL-SDR enumeration
4. Connect from an SDR client:

```bash
# GQRX: Input → Other → rtl_tcp=<esp32-ip>:1234
# SDR++: Source → RTL-TCP → <esp32-ip>:1234
# Command line:
rtl_tcp -a <esp32-ip> -p 1234
```

mDNS advertises the service as `esp32p4-rtlsdr._rtl_tcp._tcp`.

## Architecture

```
main.c              — App entry, WiFi init, USB host setup, task orchestration
components/rtlsdr/  — RTL2832U USB host driver + R820T/R820T2 tuner control
components/rtltcp/  — RTL-TCP server (TCP listener, command parser, IQ streamer)
```

See [docs/FEASIBILITY.md](docs/FEASIBILITY.md) for detailed technical analysis.

## Prior Art

- [xtrsdr](https://github.com/XTR1984/xtrsdr) — RTL-SDR on ESP32-S2 (Full-Speed USB, ~240 kHz max)
- [librtlsdr](https://github.com/steve-m/librtlsdr) — Canonical RTL-SDR library
- [RTL2832U on STM32](https://fallingnate.svbtle.com/rtl2832-usb-stm32-pt1) — MCU USB host proof-of-concept

## License

GPL-2.0-or-later (following librtlsdr)
