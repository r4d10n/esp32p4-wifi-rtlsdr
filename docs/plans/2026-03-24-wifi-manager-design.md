# ESP32-P4 WiFi Manager & Service Configuration Portal

**Date:** 2026-03-24
**Status:** Design Complete — Ready for Implementation
**Component:** `wifimgr` (new ESP-IDF component)

---

## 1. Overview

A runtime WiFi/Ethernet configuration manager with a web-based portal for the ESP32-P4 RTL-SDR platform. Replaces compile-time WiFi credentials (Kconfig) with persistent, secure, multi-network configuration. Extends to full service management for all SDR protocol servers, decoder applications, notification channels, and an LLM chatbot interface.

### Goals

1. **Zero-config first boot**: Device starts SoftAP with captive portal when no networks are saved
2. **Multi-network roaming**: Signal-strength-based auto-selection across saved networks
3. **Secure credential storage**: NVS encryption for WiFi passwords and API keys
4. **Complete service management**: Enable/disable/configure all SDR services from one UI
5. **Notification dispatch**: Telegram/Discord alerts with per-service event filtering
6. **LLM chatbot** (Phase 2): Natural language queries routed to cloud LLM APIs with tool calling
7. **Ethernet support**: Full DHCP/static IP configuration for wired deployments

### Non-Goals

- Mesh networking / ESP-NOW
- Bluetooth provisioning (phones can use the web portal)
- On-device ML inference
- SpyServer/SoapySDR full protocol implementation (stubs only in Phase 1)

---

## 2. Architecture

### 2.1 Component Stack

```
+-----------------------------------------------------+
|              Config Web Portal (SPA)                 |
|   HTML/JS/CSS embedded in firmware (~30KB gzipped)   |
|   Tabs: WiFi | Ethernet | SDR | Services |          |
|         Notifications | Chatbot | System             |
+-----------------------------------------------------+
|              REST API Layer (JSON)                    |
|   /api/wifi/*  /api/eth/*  /api/sdr/*               |
|   /api/services/*  /api/notify/*  /api/chat/*        |
|   /api/system/*                                      |
+-----------------------------------------------------+
|           WiFi Manager Core (wifimgr)                |
|   - Network scanner + RSSI-sorted auto-connect       |
|   - AP fallback with hybrid captive portal           |
|   - UART '+++' pattern detection (hardware ISR)      |
|   - BOOT button long-press (3s, iot_button)          |
|   - Ethernet link management                         |
+-----------------------------------------------------+
|              Config Store                            |
|   NVS (encrypted): WiFi creds, API keys/tokens       |
|   LittleFS (128KB): services.json, sdr.json,        |
|                      ethernet.json, notify.json      |
+-----------------------------------------------------+
|           Notification Dispatcher                    |
|   FreeRTOS queue (32 slots) -> single task           |
|   Per-service rate limiting + retry (3x exp backoff) |
|   Telegram Bot API + Discord Webhook HTTPS POST      |
+-----------------------------------------------------+
|           LLM Chatbot Gateway (Phase 2)              |
|   Context gatherer -> Prompt builder -> HTTPS POST   |
|   Tool registry -> Safe executor -> Response router  |
+-----------------------------------------------------+
```

### 2.2 State Machine

```
BOOT
  |
  v
CHECK_CONFIG -----> [no saved networks?] ----> AP_MODE
  |
  v
SCAN_NETWORKS
  |
  v
SORT_BY_RSSI (filter below threshold, default -75 dBm)
  |
  v
TRY_CONNECT (strongest match first)
  |
  +---> [success] ----> CONNECTED ----> START_SERVICES
  |                        ^                |
  +---> [fail] -> next    |                v
  |     network by RSSI   |           RUNNING
  |         |              |             |
  |     [all failed]       |     [UART '+++' OR BOOT 3s]
  |         |              |             |
  |     cycle_count++      |             v
  |         |              |      AP_MODE (concurrent APSTA)
  |   [count < 3?]         |             |
  |     |       |          |       [user configures]
  |    yes      no         |             |
  |     |       |          +-------------+
  |     v       v
  |   SCAN    AP_MODE
  |           (SoftAP only)
  |              |
  |        [user configures + saves]
  |              |
  +<-------------+
```

**Key behaviors:**
- CONNECTED + trigger -> APSTA mode (config portal on :80, services continue on existing IP)
- AP_MODE standalone -> SoftAP only, portal on 192.168.4.1:80
- 3 full scan cycles before fallback (~30-45 seconds)
- Ethernet checked in parallel; if link up, services start on Ethernet while WiFi continues scanning

### 2.3 Partition Table

```csv
# Name,      Type, SubType,  Offset,    Size,     Flags
nvs,         data, nvs,      0x9000,    0x6000,
phy_init,    data, phy,      0xf000,    0x1000,
factory,     app,  factory,  0x10000,   0x1E0000,
storage,     data, littlefs, 0x1F0000,  0x20000,
```

- `nvs` (24KB): WiFi credentials (encrypted namespace), PHY calibration
- `factory` (1.875MB): Main firmware (reduced 128KB from 2MB)
- `storage` (128KB): LittleFS for JSON config files

### 2.4 Config File Layout (LittleFS `/storage/`)

```
/storage/
  sdr.json          # Global SDR parameters
  services.json     # Per-service enable/config
  ethernet.json     # Ethernet settings
  notify.json       # Telegram/Discord config + per-service events
  chatbot.json      # LLM provider config (Phase 2)
```

WiFi credentials stored in NVS encrypted namespace `wifi_creds`:
- `wifi_count` (uint8): Number of saved networks (max 8)
- `wifi_N_ssid` (string): SSID for network N
- `wifi_N_pass` (string): Password for network N
- `wifi_N_auth` (uint8): Auth mode (WPA2/WPA3/Open)

API keys stored in NVS encrypted namespace `api_keys`:
- `tg_bot_token`, `dc_webhook_url`, `llm_api_key`

---

## 3. Trigger Mechanisms

### 3.1 UART Pattern Detection

Uses ESP-IDF hardware pattern detection ISR — zero CPU polling.

```c
// Detect "+++" with baud-rate-relative timing
uart_enable_pattern_det_baud_intr(
    UART_NUM_0,     // Console UART
    '+',            // Pattern character
    3,              // 3 consecutive '+'
    9,              // chr_tout: 9 baud cycles between chars
    0,              // post_idle: immediate
    0               // pre_idle: immediate
);
```

On `UART_PATTERN_DET` event in UART task -> set `config_mode_requested` flag -> state machine transitions to AP_MODE.

### 3.2 BOOT Button Long-Press

Uses `espressif/button` managed component (v4.1+).

```c
button_handle_t btn = iot_button_new_gpio_device(
    .button_gpio = GPIO_NUM_0,   // BOOT button
    .active_level = 0            // Active low
);

iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, config_mode_cb, NULL);
// Long press threshold: 3000ms (configurable)
```

### 3.3 Automatic Fallback

After 3 full scan-connect cycles with no success (~30-45s), transitions to AP_MODE automatically.

---

## 4. WiFi Configuration

### 4.1 Multi-Network Management

- Store up to 8 WiFi networks in NVS
- On boot/reconnect: scan all channels, match against saved networks
- Sort matches by RSSI (strongest first)
- Skip networks below configurable RSSI threshold (default -75 dBm)
- Try each in order until connected or all exhausted

### 4.2 SoftAP Configuration

| Parameter | Default | Notes |
|-----------|---------|-------|
| AP SSID | `ESP32P4-SDR-Setup` | Configurable |
| AP Password | `sdrsetup` | Min 8 chars, configurable |
| AP Channel | 1 | Auto-matched to STA in APSTA mode |
| AP Max Connections | 4 | SoftAP limit |
| AP IP | 192.168.4.1 | DHCP server on .4.0/24 |

### 4.3 Hybrid Captive Portal

Respond to OS-specific probe URLs on the SoftAP HTTP server (port 80):

| OS | Probe URL | Response |
|----|-----------|----------|
| Android | `GET /generate_204` | HTTP 302 -> `/` |
| iOS/macOS | `GET /hotspot-detect.html` | HTTP 302 -> `/` |
| Windows | `GET /connecttest.txt` | HTTP 302 -> `/` |
| Chrome OS | `GET /generate_204` | HTTP 302 -> `/` |

All probes redirect to the config portal root. The portal itself returns HTTP 200. This triggers the captive portal popup on phones/laptops without requiring a DNS server.

### 4.4 APSTA Coexistence

When entering config mode from CONNECTED state:
- Switch to `WIFI_MODE_APSTA`
- STA remains connected (services continue)
- AP starts on same channel as STA (ESP-IDF requirement)
- Config portal accessible on both AP IP (192.168.4.1) and STA IP

---

## 5. Ethernet Configuration

### 5.1 Hardware Support

ESP32-P4 internal EMAC with RMII interface. Supported on Olimex ESP32-P4 (IP101 PHY). Not available on Waveshare ESP32-P4-WIFI6 (no Ethernet port).

### 5.2 Parameters (ethernet.json)

```json
{
  "enable": false,
  "dhcp": true,
  "static_ip": "192.168.1.100",
  "static_mask": "255.255.255.0",
  "static_gw": "192.168.1.1",
  "static_dns": "8.8.8.8",
  "phy_type": "IP101",
  "phy_addr": -1,
  "mdc_gpio": 31,
  "mdio_gpio": 27,
  "prefer_over_wifi": true
}
```

### 5.3 PHY Support Matrix

| PHY | Kconfig Symbol | Notes |
|-----|---------------|-------|
| IP101 | `CONFIG_ETH_PHY_IP101` | Olimex ESP32-P4 default |
| LAN8720 | `CONFIG_ETH_PHY_LAN8720` | Common in custom boards |
| RTL8201 | `CONFIG_ETH_PHY_RTL8201` | Alternative |

### 5.4 Runtime Behavior

- Ethernet init runs at boot if `enable: true`
- Link status monitored via `ETH_EVENT_CONNECTED` / `ETH_EVENT_DISCONNECTED`
- If `prefer_over_wifi: true` and Ethernet link is up, route default gateway through Ethernet
- WiFi remains active for clients connecting via AP mode
- Ethernet link status shown in System tab

---

## 6. Global SDR Parameters (sdr.json)

```json
{
  "center_freq": 100000000,
  "sample_rate": 250000,
  "gain_mode": "auto",
  "tuner_gain_tenth_db": 0,
  "rtl_agc": false,
  "tuner_agc": false,
  "ppm_correction": 0,
  "direct_sampling": "off",
  "offset_tuning": false,
  "offset_freq_hz": 0,
  "bias_tee": false,
  "dc_offset_correction": true,
  "iq_imbalance_correction": false,
  "invert_iq": false,
  "max_total_users": 5,
  "hostname": "esp32p4-rtlsdr"
}
```

### 6.1 Parameter Details

**Frequency & Tuning:**

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `center_freq` | uint32 | 22MHz–1.7GHz | 100MHz | Center frequency in Hz |
| `sample_rate` | uint32 | 225k–3.2M | 250000 | Sample rate in Hz |
| `ppm_correction` | int16 | -200…+200 | 0 | Crystal oscillator PPM error correction |

**Gain Control:**

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `gain_mode` | enum | auto/manual | auto | Automatic vs manual tuner gain |
| `tuner_gain_tenth_db` | uint16 | 0–496 | 0 | Manual gain in tenths of dB |
| `rtl_agc` | bool | — | false | RTL2832U demodulator AGC |
| `tuner_agc` | bool | — | false | R828D hardware tuner AGC |

R820T/R828D valid gain values (29 steps, tenths dB):
`0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254, 280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496`

**Sampling Modes:**

| Parameter | Type | Values | Default | Description |
|-----------|------|--------|---------|-------------|
| `direct_sampling` | enum | off/I-ADC/Q-ADC | off | Direct ADC input for HF (0–28.8MHz). Requires hardware mod. I-ADC=pins 1-2, Q-ADC=pins 4-5 |
| `offset_tuning` | bool | — | false | Enable offset tuning for external frequency converters |
| `offset_freq_hz` | int32 | ±200MHz | 0 | Offset in Hz. Positive=upconverter (e.g., +125MHz), negative=downconverter |
| `invert_iq` | bool | — | false | Swap I/Q channels (some converters invert spectrum) |

**IQ Correction:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dc_offset_correction` | bool | true | Remove DC spike at center frequency (mixer leakage) |
| `iq_imbalance_correction` | bool | false | Correct I/Q amplitude/phase mismatch |

**Hardware:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `bias_tee` | bool | false | +5V DC on antenna connector for external LNA. GPIO4 default |

**System:**

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `max_total_users` | uint8 | 1–10 | 5 | Max concurrent clients across all services |
| `hostname` | string(32) | — | "esp32p4-rtlsdr" | mDNS hostname |

### 6.2 Max Users Estimation

Based on ESP32-P4 capabilities (dual-core 400MHz, 32MB SPIRAM, WiFi ~15Mbps usable):

| Service | BW per client | CPU per client | Suggested max |
|---------|--------------|----------------|---------------|
| RTL-TCP (250 kSPS) | 500 KB/s | ~5% | 3 |
| RTL-UDP (250 kSPS) | 500 KB/s | ~3% | 5 (multicast=1) |
| SpyServer (FFT only) | ~50 KB/s | ~8% (FFT) | 5 |
| SpyServer (IQ+FFT) | ~550 KB/s | ~10% | 2 |
| WebSDR (FFT+DDC) | ~100 KB/s | ~12% | 3 |
| SoapySDR Remote | 500 KB/s | ~5% | 2 |

WiFi bottleneck: ~15 Mbps usable = ~1.8 MB/s. With 250 kSPS IQ = 500 KB/s per stream, max ~3 concurrent IQ streams over WiFi. Ethernet removes this bottleneck (~12 MB/s usable).

---

## 7. Service Configuration (services.json)

### 7.1 SDR Protocol Services

#### RTL-TCP (Existing)

```json
{
  "rtl_tcp": {
    "enable": true,
    "port": 1234,
    "bind_addr": "0.0.0.0",
    "max_clients": 1
  }
}
```

#### RTL-UDP (Existing)

```json
{
  "rtl_udp": {
    "enable": true,
    "port": 1235,
    "payload_size": 1024,
    "multicast_enable": false,
    "multicast_group": "239.1.2.3",
    "multicast_port": 1236
  }
}
```

#### SpyServer (Stub — Phase 2 Implementation)

```json
{
  "spyserver": {
    "enable": false,
    "port": 5555,
    "max_clients": 5,
    "allow_control": true,
    "fft_fps": 15,
    "fft_bin_bits": 16,
    "force_8bit": true,
    "buffer_size_ms": 50,
    "session_timeout_min": 0,
    "owner_name": "",
    "antenna_type": "",
    "antenna_location": ""
  }
}
```

SpyServer wire protocol: TCP port 5555, 16-byte message header (ProtocolID, MessageType, StreamType, SequenceNumber, BodySize). Message types: DeviceInfo(0), ClientHandshake(1), FFT data, IQ data. Data formats: UINT8, INT16, FLOAT. Reference: `spyserver_protocol.h`.

#### SoapySDR Remote (Stub — Phase 2 Implementation)

```json
{
  "soapysdr": {
    "enable": false,
    "port": 55132,
    "protocol": "udp",
    "format": "CS8",
    "mtu": 1500,
    "announce_mdns": true
  }
}
```

SoapyRemote: TCP RPC on port 55132 for control, UDP for IQ streaming. Discovery via mDNS `_SoapySDR._tcp`. Client args: `remote:prot=udp`, `remote:format=CS8|CS16`.

### 7.2 Web Application Services

#### WebSDR (Existing)

```json
{
  "websdr": {
    "enable": true,
    "port": 8080,
    "max_clients": 3,
    "fft_rate_hz": 20,
    "tls_enable": true
  }
}
```

#### ADS-B Decoder (Stub)

```json
{
  "adsb": {
    "enable": false,
    "freq_hz": 1090000000,
    "output_format": "json",
    "feed_flightaware": false,
    "feed_fr24": false,
    "feed_host": "",
    "feed_port": 30005,
    "max_range_nm": 250
  }
}
```

#### AIS Decoder (Stub)

```json
{
  "ais": {
    "enable": false,
    "freq1_hz": 161975000,
    "freq2_hz": 162025000,
    "output_format": "nmea",
    "forward_host": "",
    "forward_port": 10110
  }
}
```

#### APRS Decoder (Stub)

```json
{
  "aprs": {
    "enable": false,
    "freq_hz": 144390000,
    "callsign": "",
    "ssid": 10,
    "igate_enable": false,
    "aprs_is_server": "rotate.aprs2.net",
    "aprs_is_port": 14580,
    "passcode": ""
  }
}
```

#### GSM/LTE Scanner (Stub)

```json
{
  "gsm_scanner": {
    "enable": false,
    "scan_bands": ["GSM850", "GSM900", "DCS1800", "PCS1900"],
    "dwell_time_ms": 200,
    "nco_lookup": true,
    "report_interval_s": 300
  }
}
```

#### FT8/WSPR Decoder (Stub)

```json
{
  "ft8_wspr": {
    "enable": false,
    "mode": "FT8",
    "band": "20m",
    "freq_hz": 14074000,
    "callsign": "",
    "grid_locator": "",
    "upload_pskreporter": false,
    "upload_wsprnet": false
  }
}
```

#### WB/NB FM Player (Existing in WebSDR)

```json
{
  "fm_player": {
    "enable": true,
    "default_freq_hz": 100000000,
    "de_emphasis_us": 75,
    "stereo": false,
    "wbfm_bandwidth_hz": 150000,
    "nbfm_bandwidth_hz": 12500
  }
}
```

#### rtl_433 Decoder (Stub)

```json
{
  "rtl_433": {
    "enable": false,
    "frequency": [433920000, 315000000],
    "hop_interval_s": 600,
    "sample_rate": 250000,
    "protocols": [],
    "output_format": "json",
    "mqtt_enable": false,
    "mqtt_host": "",
    "mqtt_port": 1883,
    "mqtt_topic": "rtl_433/{model}/{id}",
    "signal_level_threshold_db": -10
  }
}
```

rtl_433 parameters: 300+ protocol decoders, frequency hopping between up to 4 frequencies, output to JSON/CSV/MQTT/InfluxDB/syslog. Key modes: `pulse_detect` (auto/classic/minmax/level), `convert` (native/si/customary).

#### rtl_power Spectrum Monitor (Stub)

```json
{
  "rtl_power": {
    "enable": false,
    "freq_start_hz": 88000000,
    "freq_stop_hz": 108000000,
    "bin_size_hz": 10000,
    "interval_s": 60,
    "window_func": "hamming",
    "crop_percent": 0,
    "output_format": "csv",
    "cloud_upload_enable": false,
    "cloud_provider": "google_drive",
    "cloud_path": "/rtl_power/",
    "cloud_auth_token": "",
    "upload_interval_min": 60,
    "generate_spectrogram_png": true
  }
}
```

rtl_power sweep parameters: Window functions (rectangle, hamming, blackman, blackman-harris, hann-poisson, bartlett, youssef). Output CSV: `date, time, Hz_low, Hz_high, Hz_step, samples, dBm...`. Cloud upload via HTTPS POST (Google Drive API v3, Dropbox, generic WebDAV).

---

## 8. Notification System (notify.json)

### 8.1 Channel Configuration

```json
{
  "telegram": {
    "enable": false,
    "bot_token": "",
    "chat_id": "",
    "rate_limit_s": 60,
    "max_retries": 3
  },
  "discord": {
    "enable": false,
    "webhook_url": "",
    "rate_limit_s": 60,
    "max_retries": 3
  }
}
```

### 8.2 Telegram Bot API Reference

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `POST /bot<TOKEN>/sendMessage` | JSON body | Text with MarkdownV2 or HTML |
| `POST /bot<TOKEN>/sendPhoto` | multipart/form-data | PNG spectrogram (max 10MB) |
| `POST /bot<TOKEN>/sendDocument` | multipart/form-data | CSV data files |
| `POST /bot<TOKEN>/getUpdates` | JSON body | Long-poll for incoming messages (chatbot) |

Rate limits: 1 msg/s private, 20 msg/min groups. HTTP 429 -> exponential backoff.

### 8.3 Discord Webhook API Reference

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `POST /api/webhooks/<ID>/<TOKEN>` | JSON body | Message with embeds |
| `POST /api/webhooks/<ID>/<TOKEN>` | multipart/form-data | File attachments |

Rate limits: 5 requests/2 seconds per webhook. Embed limits: title 256 chars, description 4096 chars, max 10 embeds, 6000 chars total.

### 8.4 Per-Service Event Configuration

Each service has a `notify` block in services.json:

```json
{
  "adsb": {
    "enable": true,
    "notify": {
      "telegram": true,
      "discord": false,
      "events": ["new_aircraft", "emergency_squawk"],
      "throttle_s": 30
    }
  }
}
```

### 8.5 Event Types Per Service

| Service | Events | Payload Type |
|---------|--------|-------------|
| rtl_power | `periodic_scan_complete` | PNG spectrogram + CSV |
| ADS-B | `new_aircraft`, `emergency_squawk`, `interesting_aircraft` | Text: callsign, alt, pos, heading |
| AIS | `new_vessel`, `cpa_alert` | Text: MMSI, name, pos, speed, CPA |
| APRS | `new_station`, `message_received` | Text: callsign, pos, message |
| GSM Scanner | `periodic_cell_report`, `new_cell_found` | Text: MCC/MNC, LAC, CID, ARFCN, dBm |
| FT8/WSPR | `new_decode`, `rare_grid`, `dx_spot` | Text: callsign, grid, SNR, freq |
| FM (RDS) | `rds_station_change`, `rds_radiotext` | Text: PI, PS name, RT |
| rtl_433 | `device_decoded`, `new_device` | Text: protocol, ID, sensor data |
| System | `wifi_reconnect`, `eth_link_change`, `error`, `boot` | Text: status, uptime, heap |

### 8.6 Dispatcher Architecture

```
Service Event --> xQueueSend(notify_queue, &event, 0)
                       |
                       v
              Notification Task (Core 1, priority 3)
                       |
                       v
              Rate Limiter Check
              (per-service, per-channel timestamp map)
                       |
                  [throttled?]
                  /         \
                yes          no
                 |            |
               drop     Format Message
                         /        \
                    Telegram    Discord
                        |          |
                   HTTPS POST  HTTPS POST
                   (esp_http_client, TLS)
                        |          |
                   [success?]  [success?]
                   /     \     /     \
                 yes     no  yes    no
                  |       |   |      |
                done   retry done  retry
                      (3x, exp backoff)
```

Queue: 32 slots, `sizeof(notify_event_t)` ~128 bytes each. Task stack: 8KB. Total RAM: ~12KB.

---

## 9. LLM Chatbot Gateway (Phase 2)

### 9.1 Configuration (chatbot.json)

```json
{
  "enable": false,
  "provider": "gemini",
  "api_key": "",
  "model": "gemini-2.0-flash-lite",
  "web_enable": true,
  "telegram_enable": false,
  "discord_enable": false,
  "max_history": 10,
  "allowed_tools": ["query"],
  "system_prompt_extra": ""
}
```

### 9.2 Provider API Endpoints

| Provider | Endpoint | Auth Header |
|----------|----------|-------------|
| Gemini | `POST generativelanguage.googleapis.com/v1beta/models/{model}:generateContent` | `?key=API_KEY` |
| OpenAI | `POST api.openai.com/v1/chat/completions` | `Authorization: Bearer API_KEY` |
| Claude | `POST api.anthropic.com/v1/messages` | `x-api-key: API_KEY`, `anthropic-version: 2023-06-01` |

### 9.3 Recommended Models (Cost for Embedded Use)

| Provider | Model | Input $/M | Output $/M | Notes |
|----------|-------|-----------|------------|-------|
| Google | gemini-2.0-flash-lite | FREE | FREE | Best for prototyping |
| Google | gemini-2.5-flash | $0.075 | $0.30 | Best budget |
| OpenAI | gpt-4o-mini | $0.15 | $0.60 | Cost-effective |
| Anthropic | claude-haiku-4-5 | $1.00 | $5.00 | Most capable budget |

### 9.4 Tool Definitions

**Query tools (safe, read-only):**

| Tool | Description | Returns |
|------|-------------|---------|
| `get_sdr_status` | Current freq, rate, gain, connected clients | JSON |
| `get_adsb_aircraft` | Recent ADS-B decodes (last 60s) | JSON array |
| `get_ais_vessels` | Recent AIS decodes | JSON array |
| `get_aprs_stations` | Recent APRS decodes | JSON array |
| `get_gsm_cells` | Last cell scan results | JSON array |
| `get_ft8_decodes` | Recent FT8/WSPR spots | JSON array |
| `get_rtl433_devices` | Recent rtl_433 decoded devices | JSON array |
| `get_spectrum` | FFT snapshot for freq range | JSON (dBm array) |
| `get_system_info` | Uptime, heap, WiFi RSSI, clients | JSON |
| `get_notifications` | Recent notification log | JSON array |

**Control tools (requires `allowed_tools: ["query", "control"]`):**

| Tool | Description | Parameters |
|------|-------------|-----------|
| `set_frequency` | Tune to frequency | `freq_hz` (uint32) |
| `set_gain` | Set tuner gain | `mode` (auto/manual), `gain_db` (float) |
| `set_sample_rate` | Change sample rate | `rate` (uint32) |
| `enable_service` | Enable/disable a service | `service` (string), `enable` (bool) |
| `set_alert` | Configure notification alert | `service`, `condition`, `threshold` |
| `reboot` | Reboot device | — |

### 9.5 Message Flow

```
User message (Web/TG/Discord)
  |
  v
Chat Transport Layer
  |
  v
Build conversation (system prompt + history + user message)
  |
  v
Gather auto-context (active services summary, ~500 tokens)
  |
  v
HTTPS POST to LLM API (with tool definitions)
  |
  v
Parse response
  |
  +---> [text response] --> Return to user
  |
  +---> [tool_call] --> Execute tool
                          |
                          v
                    Gather tool result
                          |
                          v
                    HTTPS POST again (with tool result)
                          |
                          v
                    Parse final response --> Return to user
```

### 9.6 Resource Estimate

| Resource | Usage | Notes |
|----------|-------|-------|
| PSRAM | ~50-100KB | Prompt 16KB + response 16KB + history 32KB + HTTP buffers |
| Flash | ~15KB | Chat UI HTML/JS + prompt templates |
| CPU | Negligible | JSON ser/de only |
| Network | ~2-10KB/query | Dominated by LLM API latency (1-5s) |

---

## 10. Web Portal UI

### 10.1 Tab Structure

| Tab | Contents |
|-----|----------|
| **WiFi** | Live scan results (RSSI bars), saved networks (drag reorder), add/edit/delete, connect/test buttons, current status |
| **Ethernet** | Enable toggle, DHCP/static switch, IP/mask/GW/DNS, PHY config, link status LED |
| **SDR** | Frequency input, sample rate dropdown, gain slider + auto toggle, direct sampling mode, offset tuning, PPM, AGC, IQ correction, bias-T, invert IQ |
| **Services** | Accordion list — each service: enable toggle + expandable config form. Status indicator (running/stopped/error) |
| **Notifications** | Telegram/Discord credentials, test button, per-service event matrix (checkboxes), throttle settings |
| **Chatbot** | Provider dropdown, API key input, model select, enable toggles (web/TG/Discord), tool permission (query/control), test button |
| **System** | Hostname, max users, uptime, heap/PSRAM bar, firmware version, reboot button, backup/restore config, factory reset (with confirmation) |

### 10.2 Design Principles

- Vanilla JS, no framework (matches existing `sdr.js` pattern)
- Single HTML file with inline CSS (embedded via `EMBED_FILES`)
- All config via `fetch()` to REST API (JSON)
- Responsive: works on phone in captive portal
- Total size target: < 30KB gzipped
- Color scheme: neutral/dark, functional

### 10.3 REST API

**WiFi:**

| Endpoint | Method | Body/Response |
|----------|--------|---------------|
| `/api/wifi/scan` | GET | `{"networks": [{"ssid": "...", "rssi": -42, "auth": "WPA2", "saved": true}]}` |
| `/api/wifi/networks` | GET | `{"networks": [{"ssid": "...", "auth": "WPA2"}]}` (no passwords returned) |
| `/api/wifi/networks` | POST | `{"ssid": "...", "password": "...", "auth": "WPA2"}` -> add |
| `/api/wifi/networks/{ssid}` | PUT | `{"password": "..."}` -> update |
| `/api/wifi/networks/{ssid}` | DELETE | Remove saved network |
| `/api/wifi/connect` | POST | `{"ssid": "..."}` -> test connection |
| `/api/wifi/status` | GET | `{"connected": true, "ssid": "...", "ip": "...", "rssi": -45, "mode": "STA"}` |

**Ethernet:**

| Endpoint | Method | Body/Response |
|----------|--------|---------------|
| `/api/eth/config` | GET | Full ethernet.json |
| `/api/eth/config` | PUT | Partial update |
| `/api/eth/status` | GET | `{"link": true, "speed": "100M", "ip": "...", "mac": "..."}` |

**SDR:**

| Endpoint | Method | Body/Response |
|----------|--------|---------------|
| `/api/sdr/config` | GET | Full sdr.json |
| `/api/sdr/config` | PUT | Partial update -> applies to RTL-SDR immediately |

**Services:**

| Endpoint | Method | Body/Response |
|----------|--------|---------------|
| `/api/services` | GET | `{"rtl_tcp": {"enable": true, "status": "running"}, ...}` |
| `/api/services/{name}` | GET | Full service config |
| `/api/services/{name}` | PUT | Partial update -> restart service if running |

**Notifications:**

| Endpoint | Method | Body/Response |
|----------|--------|---------------|
| `/api/notify/config` | GET | Full notify.json (tokens masked) |
| `/api/notify/config` | PUT | Update channels |
| `/api/notify/test` | POST | `{"channel": "telegram"}` -> send test message |

**Chatbot:**

| Endpoint | Method | Body/Response |
|----------|--------|---------------|
| `/api/chat/config` | GET/PUT | Chatbot config (key masked) |
| `/api/chat/message` | POST | `{"message": "..."}` -> `{"reply": "...", "tool_calls": [...]}` |
| `/api/chat/history` | GET | Recent conversation |
| `/api/chat/history` | DELETE | Clear history |

**System:**

| Endpoint | Method | Body/Response |
|----------|--------|---------------|
| `/api/system/info` | GET | `{"uptime_s": 3600, "heap_free": 4200000, "psram_free": 31000000, "version": "1.0.0", "hostname": "..."}` |
| `/api/system/reboot` | POST | `{"confirm": true}` -> reboot |
| `/api/system/backup` | GET | Download all config as single JSON |
| `/api/system/restore` | POST | Upload config JSON -> validate -> apply |
| `/api/system/factory-reset` | POST | `{"confirm": true}` -> erase NVS + LittleFS -> reboot |

---

## 11. mDNS Service Advertisement

```c
mdns_hostname_set(config->hostname);  // "esp32p4-rtlsdr"
mdns_instance_name_set("ESP32-P4 RTL-SDR");

// Advertise active services with TXT records
mdns_txt_item_t txt_tcp[] = {
    {"board", "esp32-p4"},
    {"version", FIRMWARE_VERSION},
    {"tuner", "R828D"},
    {"sample_rate", "250000"}
};
mdns_service_add(NULL, "_rtl_tcp", "_tcp", 1234, txt_tcp, 4);
mdns_service_add(NULL, "_http", "_tcp", 8080, NULL, 0);        // WebSDR
mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);          // Config portal
// Conditionally add SpyServer, SoapySDR if enabled
```

Discoverable via: `dns-sd -B _rtl_tcp._tcp local.`

---

## 12. Security Considerations

1. **WiFi passwords**: NVS encrypted namespace (`nvs_flash_secure_init`)
2. **API keys** (Telegram, Discord, LLM): Same encrypted NVS namespace
3. **Config portal**: HTTP only on SoftAP (no TLS needed for local AP). HTTPS on STA network.
4. **REST API**: No authentication on SoftAP (captive portal). Optional basic auth on STA network (configurable).
5. **LLM tool calling**: Separate "query" (read-only) and "control" (write) permission levels
6. **Backup/restore**: Passwords and API keys included in backup (encrypted at rest, user responsibility in transit)
7. **Factory reset**: Erases NVS + LittleFS completely, requires physical confirmation (button) or double-POST

---

## 13. Implementation Phases

### Phase 1 (This PR)

- [ ] `wifimgr` component: state machine, scanner, auto-connect, AP mode
- [ ] Captive portal (hybrid detection)
- [ ] UART `+++` detection + BOOT button long-press
- [ ] NVS encrypted storage for WiFi credentials
- [ ] LittleFS mount + JSON config read/write
- [ ] Ethernet configuration and init
- [ ] REST API (all endpoints)
- [ ] Web portal UI (all tabs, services as config-only)
- [ ] Notification dispatcher (Telegram + Discord)
- [ ] mDNS multi-service advertisement
- [ ] Integration with existing main.c (replace Kconfig WiFi)
- [ ] Updated partition table

### Phase 2 (Future)

- [ ] SpyServer protocol implementation
- [ ] SoapySDR Remote protocol implementation
- [ ] LLM chatbot gateway
- [ ] Cloud upload for rtl_power (Google Drive API)
- [ ] Decoder service implementations (use existing repos as reference)
- [ ] OTA firmware update via web portal

---

## 14. File Structure

```
components/wifimgr/
  CMakeLists.txt
  Kconfig
  include/
    wifimgr.h              # Public API
    wifimgr_config.h        # Config structures
    wifimgr_api.h           # REST API handler declarations
    wifimgr_notify.h        # Notification dispatcher
  wifimgr.c                 # State machine, scanner, AP mode
  wifimgr_config.c          # NVS + LittleFS config read/write
  wifimgr_api.c             # REST API handlers
  wifimgr_portal.c          # Captive portal + HTTP server (port 80)
  wifimgr_notify.c          # Telegram/Discord dispatcher
  wifimgr_ethernet.c        # Ethernet init and management
  www/
    portal.html             # Single-page config UI
    portal.js               # UI logic
    portal.css              # Styling
```

---

## 15. Dependencies

**Managed components (add to idf_component.yml):**
- `espressif/esp_littlefs` — LittleFS filesystem
- `espressif/button` (v4.1+) — GPIO button with debounce/long-press
- `espressif/cJSON` — JSON parsing (likely already present via ESP-IDF)

**ESP-IDF built-in:**
- `nvs_flash` — NVS with encryption
- `esp_wifi` — WiFi STA/AP/APSTA modes
- `esp_eth` — Ethernet EMAC + PHY drivers
- `esp_http_server` — HTTP server (multiple instances)
- `esp_http_client` — HTTPS client (for notifications)
- `mdns` — mDNS service discovery
- `uart` — UART pattern detection
- `lwip` — TCP/IP stack
