# WebSDR System Analysis: Browser-Based SDR with Client-Side DSP

## Vision

Turn the ESP32-P4 RTL-SDR bridge into a **web-accessible SDR receiver** where
any device with a browser becomes an SDR client — no software installation needed.

```
┌─────────────────────────────────────────────────────────────────┐
│  Browser (Phone/Tablet/Laptop)                                  │
│  ┌────────────┐ ┌──────────────┐ ┌────────────────────────┐   │
│  │ Waterfall  │ │ FFT Spectrum │ │ Audio Output            │   │
│  │ (Canvas2D/ │ │ (Canvas/     │ │ (Web Audio API)         │   │
│  │  WebGL)    │ │  WebGL)      │ │                         │   │
│  └─────┬──────┘ └──────┬───────┘ └────────┬───────────────┘   │
│        │               │                  │                    │
│  ┌─────┴───────────────┴──────────────────┴───────────────┐   │
│  │  JavaScript/WASM DSP Engine                             │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────┐ │   │
│  │  │ FFT      │ │ DDC/     │ │ Demod    │ │ Decoder   │ │   │
│  │  │ (WASM)   │ │ Decimate │ │ FM/AM/   │ │ (digital  │ │   │
│  │  │          │ │          │ │ SSB/CW   │ │  modes)   │ │   │
│  │  └──────────┘ └──────────┘ └──────────┘ └───────────┘ │   │
│  └─────────────────────┬──────────────────────────────────┘   │
│                        │ WebSocket / WebRTC                    │
└────────────────────────┼──────────────────────────────────────┘
                         │
              ┌──────────┴──────────┐
              │  ESP32-P4 Server    │
              │  ┌────────────────┐ │
              │  │ HTTP + WS      │ │  ← Serves HTML/JS/WASM
              │  │ Server         │ │  ← Streams IQ or FFT
              │  ├────────────────┤ │
              │  │ Optional:      │ │
              │  │ Server-side    │ │  ← Reduces bandwidth
              │  │ FFT + DDC     │ │
              │  ├────────────────┤ │
              │  │ RTL-SDR USB   │ │
              │  │ Host Driver   │ │
              │  └────────────────┘ │
              └─────────────────────┘
```

---

## Existing WebSDR Systems — Lessons Learned

### OpenWebRX
- **Architecture**: Python server, server-side DSP (csdr/pycsdr), WebSocket to browser
- **Client**: HTML5 Canvas waterfall, Web Audio for playback
- **DSP split**: Server does FFT + demod, client just renders + plays audio
- **Bandwidth**: ~50-200 KB/s per client (audio + compressed FFT)
- **Lesson**: Server-heavy DSP works but limits client count on embedded

### KiwiSDR
- **Architecture**: FPGA-based DDC, custom web interface
- **Client**: JavaScript FFT, client-side demodulation
- **DSP split**: FPGA does DDC, browser does demod + visualization
- **Bandwidth**: ~100-400 KB/s per client
- **Lesson**: Offloading DDC to hardware enables many clients

### NovaSDR (2025)
- **Architecture**: Rust backend, SoapySDR, OpenCL FFT, WebSocket streaming
- **Client**: React/TypeScript, Zstd-compressed waterfalls, FLAC audio
- **Innovation**: Zstd compression on FFT data, GPU-accelerated server FFT
- **Bandwidth**: Very efficient due to compression
- **Lesson**: Compression is key for low-bandwidth links

### WebSDR.org
- **Architecture**: Custom server, Java/HTML5 client
- **Client**: Client-side DSP in Java applet / HTML5
- **Bandwidth**: Audio-only streaming, very efficient
- **Lesson**: Audio-only streaming is ultra-low bandwidth

---

## Transport Options Comparison

### WebSocket (WS/WSS)

```
Browser ◄──── WebSocket (TCP) ────► ESP32-P4
```

| Aspect | Detail |
|--------|--------|
| **Protocol** | Full-duplex over TCP, frames with opcode + length |
| **Binary support** | Yes (opcode 0x02 for binary frames) |
| **Max frame** | ~5000 bytes practical on ESP32 (RAM limited) |
| **Throughput ESP32** | **~1.5-3 Mbps** (WebSocket over lwIP) |
| **Latency** | ~10-50ms (TCP + framing overhead) |
| **Firewall** | Passes through most firewalls (port 80/443) |
| **TLS** | Yes (WSS), enables HTTPS hosting |
| **Multi-client** | Yes (each gets own connection) |
| **Browser API** | `new WebSocket(url)` — trivial |
| **ESP-IDF support** | `esp_websocket_server` component exists |

**Estimated IQ rates via WebSocket**:
```
Raw IQ (uint8):  ~750 kSPS max  (1.5 MB/s ÷ 2 bytes)
Compressed IQ:   ~1500 kSPS     (with Zstd/LZ4 ~2:1 ratio)
FFT only:        Unlimited      (1 KB/frame, ~50 frames/sec = 50 KB/s)
Audio only:      Unlimited      (16 kHz mono = 32 KB/s)
```

### WebRTC DataChannel

```
Browser ◄──── DTLS/SCTP (UDP) ────► ESP32-P4 (via STUN/TURN)
```

| Aspect | Detail |
|--------|--------|
| **Protocol** | SCTP over DTLS over UDP — peer-to-peer |
| **Binary support** | Yes (ArrayBuffer) |
| **Throughput** | **~2-5 Mbps** (less overhead than WS for streaming) |
| **Latency** | **~1-5ms** (UDP-based, no head-of-line blocking) |
| **Reliability** | Configurable: ordered/reliable OR unordered/lossy |
| **NAT traversal** | Built-in (STUN/TURN/ICE) — works across internet |
| **TLS** | Always encrypted (DTLS) |
| **Complexity** | **VERY HIGH** on ESP32 — needs DTLS, ICE, SCTP |
| **ESP-IDF support** | **None** — would need full WebRTC stack port |

### Server-Sent Events (SSE)

```
Browser ◄──── HTTP SSE (one-way) ────► ESP32-P4
Browser ────► HTTP POST (commands) ──► ESP32-P4
```

| Aspect | Detail |
|--------|--------|
| **Protocol** | HTTP long-poll, text-based events |
| **Binary support** | Base64 encoded only (33% overhead) |
| **Throughput** | ~500 KB/s (text encoding overhead) |
| **Latency** | ~20-100ms |
| **Verdict** | **Not suitable** for IQ data (text-only) |

### Raw HTTP Chunked Transfer

```
Browser ◄──── HTTP/1.1 chunked ────► ESP32-P4
```

| Aspect | Detail |
|--------|--------|
| **Protocol** | Standard HTTP with Transfer-Encoding: chunked |
| **Binary support** | Yes (with proper Content-Type) |
| **Throughput** | Similar to WebSocket (~1.5 Mbps) |
| **Latency** | ~10-50ms |
| **Simplicity** | Very simple server implementation |
| **Limitation** | One-way only — commands need separate request |

---

## Recommended Architecture: Hybrid SpyServer + WebSDR

### Design: "ESP32-P4 WebSDR"

```
┌── ESP32-P4 Server ──────────────────────────────────┐
│                                                      │
│  RTL-SDR ──► USB Bulk IN ──► IQ Ring Buffer          │
│                                  │                   │
│                    ┌─────────────┼─────────────┐     │
│                    ▼             ▼             ▼     │
│              ┌──────────┐ ┌──────────┐ ┌─────────┐ │
│              │ FFT      │ │ rtl_tcp  │ │ rtl_udp │ │
│              │ Engine   │ │ (1234)   │ │ (1235)  │ │
│              │ (1024pt) │ └──────────┘ └─────────┘ │
│              └────┬─────┘                           │
│                   │                                  │
│              ┌────┴──────────────────────────────┐  │
│              │ WebSocket Server (port 80/8080)    │  │
│              │                                    │  │
│              │ Channel A: FFT frames (50Hz)       │  │ ← ~50 KB/s
│              │   - 1024 bins, uint8, compressed   │  │
│              │                                    │  │
│              │ Channel B: IQ slice (on request)   │  │ ← ~50-400 KB/s
│              │   - DDC to client BW, uint8/int16  │  │
│              │                                    │  │
│              │ Channel C: Commands (bidirectional) │  │ ← ~1 KB/s
│              │   - Freq, gain, mode, BW           │  │
│              └────────────────────────────────────┘  │
│                                                      │
│              ┌────────────────────────────────────┐  │
│              │ HTTP Server (SPIFFS/LittleFS)       │  │
│              │ Serves: index.html, sdr.js, sdr.wasm│  │
│              └────────────────────────────────────┘  │
└──────────────────────────────────────────────────────┘
         │
         │  WiFi (WebSocket over TCP)
         ▼
┌── Browser Client ───────────────────────────────────┐
│                                                      │
│  ┌──────────────────────────────────────────────┐   │
│  │ sdr.js / sdr.wasm (client-side DSP)          │   │
│  │                                               │   │
│  │  WebSocket ──► IQ/FFT Parser                  │   │
│  │                    │                          │   │
│  │           ┌────────┼────────┐                 │   │
│  │           ▼        ▼        ▼                 │   │
│  │     ┌─────────┐ ┌──────┐ ┌────────────┐     │   │
│  │     │Waterfall│ │Demod │ │ Digital    │     │   │
│  │     │(WebGL)  │ │FM/AM/│ │ Decoder    │     │   │
│  │     │         │ │SSB   │ │ (optional) │     │   │
│  │     └─────────┘ └──┬───┘ └────────────┘     │   │
│  │                     ▼                         │   │
│  │              ┌────────────┐                   │   │
│  │              │ Web Audio  │                   │   │
│  │              │ API Output │                   │   │
│  │              └────────────┘                   │   │
│  └──────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────┘
```

### Data Flow Modes

#### Mode 1: FFT-Only (Waterfall Browsing) — ~50 KB/s
```
ESP32-P4:  IQ → FFT (1024-pt) → compress → WebSocket
Browser:   decompress → render waterfall + spectrum
```
- Client sees full spectrum, clicks to tune
- **No IQ data transmitted** until client selects frequency
- Multiple clients: each gets same FFT (broadcast-like)
- **Perfect for browsing/scanning on phone**

#### Mode 2: FFT + Narrowband IQ — ~100-450 KB/s
```
ESP32-P4:  IQ → FFT (for waterfall)
           IQ → DDC (to client freq/BW) → compress → WebSocket
Browser:   FFT → waterfall
           IQ → demodulate (FM/AM/SSB) → Web Audio → speakers
```
- Client sees waterfall AND hears audio
- DDC on ESP32-P4 sends only the slice client needs
- **Narrowband (25 kHz): ~50 KB/s IQ + 50 KB/s FFT = 100 KB/s**
- **Wideband FM (200 kHz): ~400 KB/s IQ + 50 KB/s FFT = 450 KB/s**

#### Mode 3: Full IQ (Advanced Users) — ~1.5 MB/s
```
ESP32-P4:  IQ → compress → WebSocket
Browser:   decompress → full client-side DSP
```
- For power users who want raw IQ in browser
- Limited by WebSocket throughput (~750 kSPS)
- Client does all DSP (FFT, demod, decode)

### Client-Side DSP Stack (JavaScript/WASM)

```
┌─────────────────────────────────────────────┐
│ sdr-core.wasm (compiled from C/Rust)        │
│ ┌─────────────────────────────────────────┐ │
│ │ FFT (kiss_fft or custom, 1024-8192pt)   │ │  ← for spectrum display
│ │ DDC (complex mixer + CIC + FIR)         │ │  ← for narrowband extract
│ │ Demodulators:                           │ │
│ │   FM (wideband + narrowband)            │ │
│ │   AM (envelope detection)               │ │
│ │   SSB (Weaver or filter method)         │ │
│ │   CW (narrow BPF + envelope)            │ │
│ │ AGC (fast/slow/off)                     │ │
│ │ Squelch (carrier/noise)                 │ │
│ │ Filters (BPF, LPF, notch)              │ │
│ └─────────────────────────────────────────┘ │
├─────────────────────────────────────────────┤
│ sdr-ui.js (TypeScript/Vanilla JS)           │
│ ┌─────────────────────────────────────────┐ │
│ │ Waterfall renderer (WebGL / Canvas2D)   │ │
│ │ Spectrum analyzer (Canvas2D)            │ │
│ │ Frequency scale + cursor               │ │
│ │ Tuning controls (click/drag/scroll)     │ │
│ │ Mode selector (FM/AM/SSB/CW/RAW)       │ │
│ │ Gain/squelch/BW sliders                │ │
│ │ S-meter display                         │ │
│ │ Audio output (Web Audio API)            │ │
│ └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

### Bandwidth Budget per Client

| Mode | FFT | IQ | Audio | Total | WiFi Clients |
|------|-----|-----|-------|-------|-------------|
| Browse only | 50 KB/s | 0 | 0 | **50 KB/s** | ~30 |
| NFM listen | 50 KB/s | 50 KB/s | 0 | **100 KB/s** | ~15 |
| WBFM listen | 50 KB/s | 400 KB/s | 0 | **450 KB/s** | ~3 |
| Full IQ | 0 | 1500 KB/s | 0 | **1500 KB/s** | ~1 |

WiFi capacity: ~1.5 MB/s via WebSocket. Multiple browse-only clients are feasible!

---

## Transport Recommendation

### WebSocket (Primary) — Recommended

| Pro | Detail |
|-----|--------|
| Universal browser support | Every modern browser, mobile included |
| Binary frames | Efficient IQ/FFT transfer |
| Bidirectional | Commands + data on same connection |
| ESP-IDF support | `esp_websocket_server` or lwIP raw |
| Firewall friendly | Port 80/443, passes proxies |
| TLS possible | WSS for secure remote access |

### WebRTC DataChannel — Future Enhancement

| Pro | Detail |
|-----|--------|
| Lower latency | UDP-based, ~1-5ms vs 10-50ms WS |
| NAT traversal | Works across internet without port forwarding |
| Lossy mode | Perfect for real-time IQ (drop old, send new) |

| Con | Detail |
|-----|--------|
| **No ESP-IDF support** | Need full DTLS + ICE + SCTP stack |
| Signaling server needed | For ICE candidate exchange |
| Complexity | ~10,000+ LOC for WebRTC stack |

**Verdict**: WebRTC is ideal in theory but impractical on ESP32 today. Revisit when ESP-IDF adds WebRTC support or use a cloud relay.

---

## Implementation Plan

### Phase 1: Minimal WebSDR (FFT waterfall only)
**Effort: Medium (~1000 LOC server + 500 LOC client)**

Server (ESP32-P4):
1. Add FFT computation (1024-point, from IQ ring buffer)
2. Compress FFT bins to uint8 (dB scale, 0-255)
3. WebSocket server on port 8080
4. Serve static files from SPIFFS/LittleFS
5. Stream FFT frames at 20-50 Hz

Client (Browser):
1. HTML5 page with Canvas waterfall
2. WebSocket connection to ESP32-P4
3. FFT frame renderer (Canvas2D or WebGL)
4. Click-to-tune (sends frequency command via WS)
5. Basic controls (frequency, gain)

**Result**: Browse the spectrum from any phone/tablet!

### Phase 2: Add Narrowband Audio
**Effort: Medium (~800 LOC server DDC + 1000 LOC client DSP)**

Server:
1. Simple DDC (complex mixer + CIC decimator)
2. Send decimated IQ slice via WebSocket binary
3. Per-client freq/BW selection

Client:
1. FM/AM demodulator in JavaScript or WASM
2. Web Audio API for playback
3. Mode selector (NFM, AM, USB, LSB)

**Result**: Listen to radio from any browser!

### Phase 3: WASM DSP Engine
**Effort: High (~2000 LOC Rust/C compiled to WASM)**

1. Compile libcsdr or custom DSP to WASM
2. High-performance FFT, filters, demod
3. Digital mode decoders (POCSAG, ADS-B, etc.)
4. Full IQ mode for power users

### Phase 4: Multi-user + Auth
1. User authentication (simple password)
2. Per-user frequency/gain limits
3. Admin panel for monitoring
4. HTTPS/WSS for secure remote access

---

## SpyServer-like vs WebSDR: Combined Benefits

The ideal system combines SpyServer's server-side efficiency with WebSDR's
browser accessibility:

```
┌────────────────────────────────────────────────────────┐
│              ESP32-P4 Hybrid SDR Server                 │
│                                                        │
│  RTL-SDR ──► IQ Buffer ──┬──► rtl_tcp (:1234)         │ ← SDR++/GQRX
│                           ├──► rtl_udp (:1235)         │ ← High-rate
│                           ├──► SpyServer (:5555)       │ ← SDR# multi-client
│                           └──► WebSDR (:8080)          │ ← Any browser!
│                                  │                     │
│                           ┌──────┴──────┐              │
│                           │ FFT Engine  │              │
│                           │ DDC Engine  │              │
│                           │ WebSocket   │              │
│                           │ HTTP/SPIFFS │              │
│                           └─────────────┘              │
└────────────────────────────────────────────────────────┘

Client options:
  📱 Phone browser    → WebSDR (waterfall + audio)     50-450 KB/s
  💻 SDR++ desktop    → rtl_tcp or SpyServer           744-2048 KB/s
  🔧 GNU Radio        → rtl_tcp or SoapyRemote         2048 KB/s
  📡 Multi-user       → SpyServer DDC                  50 KB/s each
  ⚡ High performance → rtl_udp + bridge               2048 KB/s
```

This makes the ESP32-P4 RTL-SDR bridge the most versatile remote SDR
receiver ever built on a microcontroller — serving every type of client
from a phone browser to GNU Radio, all simultaneously.
