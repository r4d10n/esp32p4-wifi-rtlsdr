# Protocol Comparison: SpyServer vs SoapyRemote vs rtl_tcp

## Overview

| Feature | rtl_tcp (current) | SpyServer | SoapyRemote |
|---------|-------------------|-----------|-------------|
| **Developer** | Osmocom/Community | Airspy/SDR# | Pothosware |
| **Transport** | TCP only | TCP | RPC(TCP) + Stream(UDP/TCP) |
| **Clients** | SDR++, GQRX, SDR#, CubicSDR | SDR#, SDR++, SpyServer clients | Any SoapySDR app |
| **Compression** | None (raw uint8 IQ) | PCM 8/16/24/32, DINT4 | Native sample format |
| **Server-side DSP** | None | DDC, decimation, FFT | None |
| **Multi-client** | No (one at a time) | Yes (each picks own BW/freq) | No |
| **Bandwidth saving** | None | Up to 50x reduction | Configurable MTU/window |
| **Protocol spec** | 12-byte header + raw stream | Documented header + commands | RPC + windowed datagrams |
| **Implementation** | Simple (~200 lines) | Medium (~1500 lines) | Complex (~3000 lines) |

---

## SpyServer Protocol

### Architecture
```
┌─────────────┐    TCP     ┌─────────────────────────────┐
│  SDR#/SDR++ │◄──────────►│  SpyServer                  │
│  Client     │            │  ┌─────────────────────────┐ │
│             │            │  │ DDC (Digital Down Conv)  │ │
│  Waterfall ◄──── FFT ───│  │ Per-client decimation    │ │
│  IQ Data  ◄──── IQ ────│  │ Server-side FFT          │ │
│  Controls ──── Cmds ───►│  └─────────────────────────┘ │
└─────────────┘            │  ┌─────────────────────────┐ │
                           │  │ RTL-SDR / Airspy HW     │ │
┌─────────────┐    TCP     │  └─────────────────────────┘ │
│  Client 2   │◄──────────►│                              │
└─────────────┘            └─────────────────────────────┘
```

### Protocol Details

**Message Header (8 bytes)**:
```
Offset  Size  Field
0       4     Protocol ID + Message Type
4       4     Body Size
```

**Commands (client → server)**:
| ID | Command | Purpose |
|----|---------|---------|
| 0 | HELLO | Handshake (protocol version + client name) |
| 1 | GET_SETTING | Query device capability |
| 2 | SET_SETTING | Configure parameter |
| 3 | PING | Keep-alive |

**Settings**:
| ID | Setting | Type |
|----|---------|------|
| 0 | Streaming mode | IQ, FFT, IQ+FFT |
| 1 | Streaming enabled | bool |
| 2 | Gain | uint32 |
| 3-5 | IQ format/freq/decimation | IQ config |
| 6-12 | FFT format/freq/decimation/dB | FFT config |

**Stream Types**:
| Type | Data |
|------|------|
| STATUS | Device info, capabilities |
| IQ | Complex samples (configurable format) |
| AF | Audio frequency data |
| FFT | Spectral data for waterfall |

**IQ Formats**: UINT8, INT16, INT24, FLOAT32, DINT4 (4-bit differential)

### Key Advantage: Server-Side DDC
SpyServer performs **Digital Down Conversion on the server** — the client requests
a specific center frequency and bandwidth within the full capture range. The server
decimates and sends only the requested slice.

**Bandwidth comparison at narrowband FM (25 kHz)**:
```
rtl_tcp:    1.024 MSPS × 2 bytes = 2,048 KB/s  (full raw stream)
SpyServer:  25 kSPS × 2 bytes    =    50 KB/s   (decimated to client BW)
                                    ≈ 40x less bandwidth!
```

### Multi-Client DDC
```
Full capture: 1.024 MSPS @ 936 MHz
├── Client 1: 25 kSPS @ 935.800 MHz (NFM)    →  50 KB/s
├── Client 2: 25 kSPS @ 936.100 MHz (NFM)    →  50 KB/s
└── Client 3: 200 kSPS @ 936.000 MHz (WBFM)  → 400 KB/s
                                        Total:  500 KB/s vs 2,048 KB/s raw
```

---

## SoapyRemote Protocol

### Architecture
```
┌──────────────┐    TCP (RPC)    ┌───────────────────────┐
│  SDR++ /     │◄───────────────►│  SoapyRemote Server   │
│  CubicSDR /  │                 │                       │
│  GNU Radio   │    UDP (Stream) │  SoapySDR HAL         │
│  Any Soapy   │◄───────────────►│  ┌───────────────┐    │
│  Client      │                 │  │ RTL-SDR driver │    │
└──────────────┘                 │  └───────────────┘    │
     ↑                           └───────────────────────┘
     │ SoapySDR API
     │ (transparent — client doesn't know it's remote)
```

### Protocol Layers

**1. RPC Layer (TCP)**:
- Binary encoding, network byte order
- Request/reply pattern
- Device discovery (avahi/mDNS)
- All SoapySDR API calls: setFrequency, setSampleRate, setGain, etc.
- Configuration and metadata

**2. Streaming Layer (UDP default, TCP optional)**:
- Windowed datagram protocol
- Configurable MTU (default ~1500 bytes)
- Flow control via receiver feedback
- Configurable socket buffer window (10s of MB recommended)

**Parameters**:
| Key | Default | Purpose |
|-----|---------|---------|
| `remote:prot` | udp | Stream transport (udp/tcp) |
| `remote:mtu` | ~1500 | Datagram size |
| `remote:window` | OS default | Socket buffer size |
| `remote` | hostname:port | Server address |

### Key Advantage: Universal SDR API
SoapyRemote makes any remote SDR appear as a **local SoapySDR device**.
Applications don't need any remote-specific code — they use the standard
SoapySDR API and the SoapyRemote plugin handles networking transparently.

---

## Comparison for ESP32-P4 Implementation

### Implementation Complexity

```
┌──────────────┬───────────┬──────────────────────────────────────┐
│ Protocol     │ Est. LOC  │ What's needed                        │
├──────────────┼───────────┼──────────────────────────────────────┤
│ rtl_tcp      │   ~200    │ DONE — TCP server + 14 commands      │
│ rtl_udp      │   ~250    │ DONE — UDP server + header           │
│ SpyServer    │  ~1500    │ Protocol handler, DDC engine,        │
│              │           │ FFT computation, multi-client,       │
│              │           │ decimation, format conversion        │
│ SoapyRemote  │  ~3000    │ RPC server (full SoapySDR API),      │
│              │           │ windowed UDP streamer, flow control,  │
│              │           │ device discovery (avahi/mDNS)        │
└──────────────┴───────────┴──────────────────────────────────────┘
```

### CPU Requirements on ESP32-P4

| Feature | rtl_tcp | SpyServer | SoapyRemote |
|---------|---------|-----------|-------------|
| IQ forwarding | ~5% CPU | ~5% CPU | ~5% CPU |
| DDC/decimation | N/A | ~20-40% CPU | N/A |
| FFT computation | N/A | ~15-30% CPU | N/A |
| Multi-client mgmt | N/A | ~5% CPU | N/A |
| RPC handling | N/A | ~2% CPU | ~5% CPU |
| **Total extra CPU** | **0%** | **40-75%** | **10%** |

SpyServer's DDC engine is computationally expensive — the ESP32-P4's 400 MHz
dual-core RISC-V can handle it for narrow bandwidths but would struggle with
wideband FFT at high sample rates.

### Client Compatibility

| Client | rtl_tcp | SpyServer | SoapyRemote |
|--------|---------|-----------|-------------|
| SDR++ | ✓ Native | ✓ Native | ✓ Via SoapySDR |
| SDR# | ✓ Native | ✓ Native (best) | ✗ |
| GQRX | ✓ Native | ✗ | ✓ Via SoapySDR |
| CubicSDR | ✓ | ✗ | ✓ Via SoapySDR |
| GNU Radio | ✓ (osmosdr) | ✗ | ✓ Via SoapySDR |
| OpenWebRX | ✓ | ✓ | ✓ Via SoapySDR |
| dump1090 | ✓ | ✗ | ✗ |
| Custom apps | ✓ (simple) | △ (complex) | ✓ (SoapySDR lib) |

### Bandwidth Efficiency

```
Scenario: Listening to FM station (200 kHz BW) at 1.024 MSPS capture

rtl_tcp:     2,048 KB/s  (sends everything, client decimates)
SpyServer:     400 KB/s  (server decimates to 200 kSPS, sends slice)
SoapyRemote: 2,048 KB/s  (sends everything, like rtl_tcp)
Our UDP:     2,048 KB/s  (sends everything + 0.8% header overhead)

Winner: SpyServer (5x less bandwidth for narrowband use)
```

```
Scenario: Wideband capture (full 1.024 MSPS)

rtl_tcp:     2,048 KB/s
SpyServer:   2,048 KB/s  (full IQ mode, no saving)
SoapyRemote: 2,048 KB/s
Our UDP:     2,048 KB/s

All equal for wideband use.
```

---

## Benefits Analysis

### SpyServer Benefits
1. **Massive bandwidth reduction** for narrowband monitoring (5-50x)
2. **Multi-client** — multiple users share one dongle
3. **Server-side FFT** — client sees waterfall without full IQ bandwidth
4. **SDR# native** — best experience in SDR#
5. **DINT4 compression** — 4-bit differential reduces bandwidth further

### SpyServer Drawbacks
1. **Heavy server-side DSP** — DDC + FFT on ESP32-P4 is CPU-intensive
2. **Closed-source reference** — protocol reverse-engineered, not standardized
3. **TCP only** — no UDP option for lower latency
4. **Limited client support** — mainly SDR# and SDR++

### SoapyRemote Benefits
1. **Universal client compatibility** — any SoapySDR app works transparently
2. **Standard API** — well-documented, open-source
3. **UDP streaming with flow control** — windowed datagrams prevent overflow
4. **Device abstraction** — client doesn't know or care about RTL-SDR specifics
5. **GNU Radio integration** — Soapy source block works remotely

### SoapyRemote Drawbacks
1. **Complex implementation** — must implement full SoapySDR API over RPC
2. **No bandwidth reduction** — sends full raw IQ (no DDC)
3. **No multi-client** — one client per server instance
4. **Heavy dependency** — client needs SoapySDR + SoapyRemote plugin

---

## Recommendation

### Priority Matrix

| Protocol | Impact | Effort | WiFi Benefit | Unique Value |
|----------|--------|--------|-------------|--------------|
| **SpyServer** | HIGH | HIGH | **HUGE** (5-50x BW reduction) | Multi-client, narrowband efficiency |
| **SoapyRemote** | MEDIUM | VERY HIGH | None | Universal app compatibility |

### Verdict

**SpyServer is the better investment for ESP32-P4** because:

1. **WiFi bandwidth is our bottleneck** — SpyServer's DDC directly addresses this
   by sending only the bandwidth the client needs. A narrowband FM listener would
   use 50 KB/s instead of 2,048 KB/s — fitting comfortably within WiFi capacity
   even at the lowest rates.

2. **Multi-client is uniquely valuable** for a shared remote receiver — multiple
   users can each tune to different frequencies within the capture bandwidth.

3. **The ESP32-P4 has enough CPU** for narrow-bandwidth DDC (25-200 kHz slices
   from 1 MSPS capture) though wideband FFT would need optimization.

4. **SDR++ supports SpyServer natively** — no bridge tool needed.

**SoapyRemote** is lower priority because it doesn't reduce bandwidth (same as
rtl_tcp/UDP) and the implementation is significantly more complex. It's mainly
useful for GNU Radio integration, which can also use rtl_tcp via osmosdr.

### Suggested Implementation Approach

```
Phase 1: SpyServer Lite (narrowband DDC only)
  - Implement protocol handshake + SET_SETTING commands
  - Single-channel DDC (CIC + FIR decimation)
  - UINT8 and INT16 IQ formats
  - Single client initially
  → Enables 50-400 KB/s narrowband streaming over WiFi

Phase 2: SpyServer FFT
  - Add server-side FFT computation (1024-point)
  - Stream FFT data for waterfall display
  → Client gets waterfall without full IQ bandwidth

Phase 3: Multi-client
  - Per-client DDC channels
  - Bandwidth allocation and management
  → Multiple users share one ESP32-P4 + RTL-SDR

Phase 4: SoapyRemote (optional)
  - Full SoapySDR RPC server
  - For GNU Radio / CubicSDR users
```
