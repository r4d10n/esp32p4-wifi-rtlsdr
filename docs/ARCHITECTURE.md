# ESP32-P4 RTL-SDR WiFi Bridge — Technical Architecture

Complete system architecture, data flow, memory layout, USB protocols, and register maps for the ESP32-P4 RTL-SDR USB Host driver.

## System Block Diagram

```
┌────────────────────────────────────────────────────────────────────┐
│                  ESP32-P4 (400 MHz dual-core RISC-V)               │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│  ┌────────────────────────────────────────────────────────────┐   │
│  │  USB 2.0 High-Speed OTG Peripheral                         │   │
│  │  └─ Bulk IN EP 0x81 (512-byte MPS)                        │   │
│  │  └─ Control EP0 (vendor requests)                         │   │
│  └────────────────┬─────────────────────────────────────────┘   │
│                   ↓                                              │
│  ┌────────────────────────────────────────────────────────────┐   │
│  │  RTL2832U Demodulator + R828D Tuner (USB Device)           │   │
│  │  ├─ Demod control: vendor CTL transfers                   │   │
│  │  ├─ Tuner init: I2C via demod repeater                    │   │
│  │  └─ IQ output: bulk IN stream                             │   │
│  └────────────────┬─────────────────────────────────────────┘   │
│                   ↓                                              │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  Core 0: USB Host Driver + Bulk Reader (Real-time)         │  │
│  │  ├─ USB host library event callbacks (High prio)          │  │
│  │  ├─ Async bulk IN transfers (EP 0x81)                    │  │
│  │  └─ Ring buffer writer (PSRAM)                           │  │
│  └──────────────┬─────────────────────────────────────────────┘  │
│                │                                                │
│  ┌─────────────↓──────────────────────────────────────────────┐  │
│  │  PSRAM Ring Buffer (default 2 MB, configurable 512KB-8MB)  │  │
│  │  ├─ Decouples USB bursts from TCP/UDP streaming           │  │
│  │  ├─ Lock-free SPSC (single-producer single-consumer)      │  │
│  │  └─ GDMA-AXI capable (zero-copy potential)               │  │
│  └──────────────┬────────────────────────────────────┬────────┘  │
│                │                                    │           │
│  ┌─────────────↓──────────────────┐  ┌─────────────↓─────────┐  │
│  │  Core 1: TCP/UDP Servers        │  │  Softmax Tasks      │  │
│  │  ├─ RTL-TCP on :1234           │  │  ├─ WiFi (low prio) │  │
│  │  ├─ UDP streaming on :1235     │  │  └─ mDNS advert     │  │
│  │  └─ Ring buffer reader         │  └─────────────────────┘  │
│  └─────────────┬──────────────────┘                            │
│                │                                                │
│  ┌─────────────↓──────────────────────────────────────────────┐  │
│  │  lwIP TCP/UDP Stack                                         │  │
│  │  ├─ TCP_NODELAY enabled (streaming)                        │  │
│  │  └─ Segment size optimized                                │  │
│  └─────────────┬──────────────────────────────────────────────┘  │
│                │                                                │
│  ┌─────────────↓──────────────────────────────────────────────┐  │
│  │  ESP-Hosted SDIO Bridge (25 or 40 MHz clock)               │  │
│  │  ├─ SDIO Host (P4) ↔ SDIO Slave (C6)                     │  │
│  │  └─ 4-bit bus, CRC protected                             │  │
│  └─────────────┬──────────────────────────────────────────────┘  │
│                │                                                │
├────────────────↓──────────────────────────────────────────────────┤
│                │                                                  │
└────────────────↓──────────────────────────────────────────────────┘
                 │
        ┌────────↓────────┐
        │  ESP32-C6 Slave │
        │  (WiFi 6 modem) │
        └────────┬────────┘
                 ↓
        ┌──────────────────┐
        │ 802.11ax WiFi    │
        │ @ 5 GHz (HT20)   │
        │ or Ethernet      │
        └────────┬─────────┘
                 ↓
         [SDR Clients]
```

## Data Flow: USB → Ring Buffer → TCP/UDP → WiFi

```
1. USB High-Speed Bulk IN (480 Mbps theoretical)
   │
   ├─ RTL2832U outputs 512-byte packets
   │  (Interleaved uint8 I, Q samples)
   │
   └─ Packets arrive ~1 per microsecond at 1 MSPS
      (= 512B/µs = 512 MB/s @ 1 MSPS, handled by USB controller DMA)

2. Core 0 USB Event Handler
   │
   ├─ USB host driver invokes async callback on packet complete
   │  (High priority, preempts everything except interrupts)
   │
   └─ Callback passes buffer to ring buffer writer

3. Ring Buffer Writer (PSRAM)
   │
   ├─ Lock-free SPSC append
   │  (Single-producer: USB, single-consumer: TCP/UDP)
   │
   ├─ Wraps at RING_SIZE (default 2 MB)
   │  (Overflow: oldest samples discarded, not newest)
   │
   └─ Produces write index, signals Core 1 via event

4. Core 1 Ring Buffer Reader (TCP/UDP)
   │
   ├─ Medium priority, can be preempted
   │
   ├─ RTL-TCP task:
   │  ├─ Pops from ring buffer
   │  ├─ Sends via TCP socket to client
   │  └─ Handles RTL-TCP command packets
   │
   └─ RTL-UDP task:
      ├─ Pops from ring buffer
      ├─ Wraps with 8-byte header (seq + timestamp)
      └─ Sends UDP datagrams to subscribed clients

5. lwIP Stack (TCP/UDP)
   │
   ├─ Segments data into 1460-byte TCP segments (MSS)
   │
   ├─ MTU-aware fragmentation for UDP (1500-byte Ethernet frames)
   │
   └─ Pushes to SDIO bridge

6. ESP-Hosted SDIO Bridge
   │
   ├─ SDIO clock: 25 MHz (default) or 40 MHz (advanced)
   │
   ├─ 4-bit bus (25 MHz × 4 bits × 2 edges = ~50 MB/s theoretical)
   │  (Practical: ~36 Mbps WiFi TCP, ~5 MB/s sustained)
   │
   └─ Forwards to ESP32-C6 slave

7. ESP32-C6 WiFi 6 Modem
   │
   ├─ 802.11ax @ 5 GHz HT20 (80 MHz channels)
   │
   ├─ TCP throughput: ~4-5 MB/s (36 Mbps @ -50 dBm RSSI)
   │  (UDP: 2-3 MB/s after retransmits)
   │
   └─ Or Ethernet: 100 Mbps (11 MB/s TCP)

8. Remote SDR Client
   │
   ├─ SDR++ / GQRX / rtl_tcp CLI
   │
   └─ Receives IQ stream + controls tuning via RTL-TCP commands
```

## Task Architecture (FreeRTOS)

### Core 0 (Real-Time USB)

```
┌──────────────────────────────────────────────┐
│  USB Host Library                            │
│  Priority: 20 (high)                        │
│  ├─ Event dispatcher                        │
│  ├─ Control transfer handler (EP0)          │
│  ├─ Bulk transfer callbacks                 │
│  └─ Device enumeration                      │
└────────────┬─────────────────────────────────┘
             ↓
┌──────────────────────────────────────────────┐
│  USB Bulk IN Reader                          │
│  Priority: 19 (just below lib)              │
│  ├─ Submit async transfers                  │
│  ├─ Buffer management (16 KB × 4-8 bufs)    │
│  ├─ Ring buffer writer                      │
│  └─ Packet loss counter                     │
└────────────┬─────────────────────────────────┘
             ↓
         PSRAM Ring Buffer
         (single writer from USB)
```

**Stack size**: 4 KB (minimal, USB driver uses heap)
**Affinity**: CORE_0 (hard)
**Preemption**: Disabled from other tasks (USB critical)

### Core 1 (Networking & Application)

```
┌──────────────────────────────────────────────┐
│  RTL-TCP Server Task                         │
│  Priority: 10 (medium)                      │
│  ├─ Listen on :1234                         │
│  ├─ Accept one client at a time             │
│  ├─ Ring buffer reader (pop samples)        │
│  ├─ Send DongleInfo header                  │
│  ├─ Stream IQ data                          │
│  └─ Parse RTL-TCP commands                  │
└────────────┬──────────────────────────────┬──┘
             │                              │
             ↓                              ↓
    Ring Buffer (shared)        RTL-SDR Device (via USB)
```

```
┌──────────────────────────────────────────────┐
│  RTL-UDP Server Task (optional)              │
│  Priority: 9 (medium)                       │
│  ├─ Listen on :1235                         │
│  ├─ Track subscriber address                │
│  ├─ Ring buffer reader (pop samples)        │
│  ├─ Wrap with seq + timestamp header        │
│  ├─ Send UDP datagrams                      │
│  └─ Accept commands on same socket          │
└────────────┬──────────────────────────────┬──┘
             │                              │
             ↓                              ↓
    Ring Buffer (shared)        RTL-SDR Device (via USB)
```

```
┌──────────────────────────────────────────────┐
│  WiFi / Network Stack                        │
│  Priority: 8 (soft real-time)               │
│  ├─ ESP-Hosted event dispatcher             │
│  ├─ lwIP TCP/UDP processors                 │
│  ├─ SDIO bridge communication               │
│  └─ Automatic (managed by esp_wifi)         │
└──────────────────────────────────────────────┘
```

### Synchronization

- **Ring buffer**: Lock-free SPSC (no mutexes needed)
- **USB control**: Semaphore for blocking vendor transfers
- **Client state**: Atomic flags for connect/disconnect

## Memory Map

### SRAM Layout

```
┌─────────────────────────────────────────┐
│  HP SRAM (768 KB, L2MEM)                │
├─────────────────────────────────────────┤
│  FreeRTOS heap                          │  ~300 KB
│  - USB driver heap                      │  ~100 KB
│  - USB transfer buffers (16KB × 8)      │  ~128 KB
│  - Task stacks (4 tasks × 4-8 KB)       │  ~32 KB
│  - lwIP internal (tcp/udp)              │  ~100 KB
├─────────────────────────────────────────┤
│  Code + RO data + initialized data      │  ~320 KB
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│  LP SRAM (32 KB)                        │
├─────────────────────────────────────────┤
│  RTC memory (4 KB)                      │
│  PM info + reserves                     │  Remaining
└─────────────────────────────────────────┘
```

### PSRAM Layout

```
┌──────────────────────────────────────────┐
│  PSRAM (32 MB, default allocation)       │
├──────────────────────────────────────────┤
│  Ring Buffer (2 MB default, configurable)│  2 MB
│  ├─ Write pointer: USB callback          │
│  ├─ Read pointer: TCP/UDP task           │
│  └─ Wrap mask: (RING_SIZE - 1)          │
├──────────────────────────────────────────┤
│  FreeRTOS malloc pool                    │  ~10 MB
│  ├─ Socket buffers                      │
│  ├─ TCP window buffers                  │
│  └─ Misc allocations                    │
├──────────────────────────────────────────┤
│  Available for user app / future         │  ~20 MB
└──────────────────────────────────────────┘
```

### Flash Partition Table

```
Name         Offset  Size     Purpose
────────────────────────────────────────────
nvs          0x000   0x020    NVS (WiFi credentials)
otadata      0x020   0x002    OTA data
ota_0        0x030   0x100    OTA slot 1 (1 MB)
ota_1        0x130   0x100    OTA slot 2 (1 MB)
spiffs       0x230   0x1D0    SPIFFS (1856 KB, reserved)
────────────────────────────────────────────
Total        0x400   0x400    (16 MB flash)
```

## USB Protocol

### Device Identification

```
RTL-SDR Blog V4:
  idVendor:       0x0BDA (Realtek)
  idProduct:      0x2838 (RTL-SDR Blog)
  bDeviceClass:   0xFF (Vendor-specific)
  bInterfaceClass: 0xFF (Vendor-specific)

Generic RTL2832U:
  idVendor:       0x0BDA
  idProduct:      0x2832
  (Identical protocol, may need different tuner probing)
```

### Endpoints

```
Endpoint 0 (Control)
  ├─ Address: 0x00
  ├─ Type: Control
  ├─ Max packet size: 64 bytes (FS/HS)
  └─ Purpose: Vendor control transfers for register R/W

Endpoint 0x81 (Bulk IN)
  ├─ Address: 0x81 (IN from device)
  ├─ Type: Bulk
  ├─ Max packet size: 512 bytes @ HS (64 bytes @ FS)
  └─ Purpose: IQ sample stream (uint8 I, Q interleaved)

No OUT endpoints
  └─ All configuration via control transfers only
```

### Control Transfer Format (Register Access)

#### Request Structure

```
bmRequestType:  0xC0 (read) | 0x40 (write)
                bit 7 = direction (0=out, 1=in)
                bit 6:5 = type (10=vendor)
                bit 4:0 = recipient (00000=device)

bRequest:       0x00 (always)

wValue:         Register address
                └─ 16-bit address (0x0000-0xFFFF)

wIndex:         (block << 8) | flags
                └─ block: register block selector
                   └─ 0: DEMOD
                   └─ 1: USB
                   └─ 2: SYS
                   └─ 3: TUNER
                   └─ 4: ROM
                   └─ 5: IR
                   └─ 6: I2C
                └─ flags: 0x10 (write), 0x00 (read)

wLength:        1-4 bytes (register value)

bInterval:      N/A (control transfers are not periodic)

Timeout:        300 ms (default)
```

#### Control Transfer Examples

```c
// Read demod register (block=0, addr=0x00)
bmRequestType = 0xC0;  // vendor in
bRequest = 0x00;
wValue = 0x0000;       // register address
wIndex = (0 << 8) | 0x00;  // block 0, read flag
wLength = 1;

// Write demod register (block=0, addr=0x1a)
bmRequestType = 0x40;  // vendor out
bRequest = 0x00;
wValue = 0x001a;       // register address
wIndex = (0 << 8) | 0x10;  // block 0, write flag
wLength = 1;
data[0] = value;

// Read tuner register via I2C repeater (block=6)
bmRequestType = 0xC0;
wValue = i2c_addr << 8 | register;
wIndex = (6 << 8) | 0x00;  // block 6 (I2C), read
wLength = 1;

// Write tuner register via I2C repeater
bmRequestType = 0x40;
wValue = i2c_addr << 8 | register;
wIndex = (6 << 8) | 0x10;  // block 6 (I2C), write
wLength = 1;
```

## RTL2832U Register Map

### Demodulator Block (Block 0)

```
Address  Name              Purpose
─────────────────────────────────────────────
0x00     DEMOD_CTRL        Control register
0x08     ADC_CTRL          ADC input mux
0x0D     SOFT_RST          Soft reset flag
0x10     DVBT_CTRL         DVB-T control
0x14     ADC_ENABLE        ADC enable
0x15     SPEC_INV          Spectrum inversion

0x19     IF_FREQ_H         IF frequency (high)
0x1A     IF_FREQ_M         IF frequency (middle)
0x1B     IF_FREQ_L         IF frequency (low)

0x9F     RS_RATIO_H        Resampler ratio (high)
0xA0     RS_RATIO_M        Resampler ratio (middle)
0xA1     RS_RATIO_L        Resampler ratio (low)

0xB1     ZERO_IF           Zero-IF mode flag
0x4D     AGC_CTRL          AGC enable

0x3F     FREQ_CORR_H       Frequency correction (high)
0x3E     FREQ_CORR_M       Frequency correction (middle)
0x3D     FREQ_CORR_L       Frequency correction (low)
```

### USB Block (Block 1)

```
Address  Name              Purpose
─────────────────────────────────────────────
0x02     USB_SYSCTL        USB system control
0x04     USB_CTRL          USB control
0x06     FIFO_CTRL         FIFO control
0x08     EPA_MAXPKT        Endpoint A max packet size
0x0E     EPA_CTL           Endpoint A control
```

### System Control Block (Block 2)

```
Address  Name              Purpose
─────────────────────────────────────────────
0x00     GPO               GPIO output register
0x01     GPOE              GPIO output enable
0x02     GPD               GPIO data direction
0x04     GPH               GPIO high/low select
```

### R828D Tuner Registers (via I2C @ 0x74)

```
Address  Name              Purpose
─────────────────────────────────────────────
0x00     CHIP_ID           Chip identifier
0x05     IF_SELECT         Input/filter select
0x06     FILTER            Filter control
0x08     XTAL_CAP          Crystal load capacitance
0x0A     LNA_GAIN          LNA gain control
0x0C     VGA_GAIN          VGA gain control
0x0D     MIXER_GAIN        Mixer gain control
0x14     PLL_DIV_NUM       PLL divider number
0x15     PLL_FRAC_NUM      PLL fractional number
0x19     PDF_BIAS          PDF bias current
0x1A     RF_MUX            RF multiplexer
0x1B     TF_SELECT         Tracking filter select
```

## RTL-TCP Protocol

### Connection Sequence

```
Client                          ESP32-P4
   │                               │
   ├─ connect(192.168.1.232:1234)─→│
   │                          (TCP handshake)
   │                               │
   │←──── DongleInfo (12 bytes) ────│
   │ {Magic="RTL0", tuner=6, gains=29}
   │                               │
   ├────────→ [5-byte command] ────→│
   │ (e.g., set freq 936 MHz)      │
   │                               │
   │←──────── [IQ stream] ──────────│
   │ continuous uint8 pairs (I,Q)   │
   │                               │
   ├────────→ [more commands] ─────→│
   │                               │
   │←──────── [more IQ] ───────────│
   │  ...                          │
   │                               │
   ├─ close() ────────────────────→│
   │                               │
```

### DongleInfo Header (12 bytes, big-endian)

```
Byte Offset    Field          Size    Value
────────────────────────────────────────────
0-3            Magic          4       0x52544C30 ("RTL0")
4-7            Tuner Type     4       6 (R828D)
8-11           Gain Count     4       29
```

### Command Packet (5 bytes, big-endian)

```
Byte Offset    Field          Size    Value
────────────────────────────────────────────
0              Command        1       0x01-0x0E
1-4            Parameter      4       Big-endian uint32
```

### IQ Data Stream

```
Format:    Interleaved uint8 I, Q pairs
Encoding:  I[i] ∈ [0, 255] maps to [-3.2V, +3.2V]
           127.5 = 0.0V nominal
Example:   [I₀, Q₀, I₁, Q₁, I₂, Q₂, ...]
Endianness: Native byte order (little-endian on x86),
            TCP stack handles network byte order
```

## R828D Tuner Driver

### Initialization Sequence

```
1. Write init_array
   └─ 16 register writes to default values

2. Set TV Standard (SDR mode)
   ├─ Write system frequency selector
   ├─ Trigger filter calibration
   │  └─ PLL to 56 MHz
   │  └─ Charge pump current max
   │  └─ Wait and read calibration code
   └─ Save calibration result

3. Set System Frequency
   ├─ Based on center frequency
   ├─ Configure LNA/mixer/filter
   └─ Set charge pump / discharge current

4. Set Frequency (user call)
   ├─ Call r82xx_set_mux (frequency-dependent RF config)
   │  └─ 28-entry freq_ranges[] table
   │  └─ Set open-drain control, RF mux, tracking filter
   ├─ Call r82xx_set_pll (PLL lock)
   │  └─ N + (SDM code / 2^22) divider
   │  └─ VCO coarse + fine tuning
   │  └─ Read back VCO fine tune and adjust
   └─ Call r82xx_set_vga_gain (post-tuning gain)

5. Set Bandwidth
   ├─ Recompute IF frequency from bandwidth
   └─ Return new IF to caller (demod update)

6. Set Gain (LNA + mixer + VGA)
   └─ Interleaved gain steps across stages
```

### PLL Tuning

```
Center frequency → PLL divider:
  div_num = xtal / 2^22
  n_syn = freq / (xtal / 2^22)
  n_int = n_syn >> 16
  n_frac = n_syn & 0xFFFF

SDM (Sigma-Delta Modulator):
  Iterative bit-shifting for stable PLL lock
  Reduces spurious emissions
```

### Frequency Ranges (Blog V4)

```
0-28.8 MHz       HF band: Cable2 input, upconvert +28.8 MHz, GPIO5 control
28.8-250 MHz     VHF band: Cable1 input
250-2000 MHz     UHF band: Air input

Each frequency range specifies:
  ├─ RF multiplexer setting
  ├─ Tracking filter capacitor bank
  ├─ Crystal load capacitance
  ├─ Drain control
  └─ Notch filter configuration
```

## ESP-Hosted SDIO Transport

### SDIO Physical Layer

```
Clock frequency:  25 MHz (default) or 40 MHz (advanced)
Bus width:        4-bit
Voltage:          3.3V
Protocol:         SDIO 2.0 with CRC protection
```

### Pin Mapping (Waveshare ESP32-P4-WIFI6)

```
ESP32-P4        ESP32-C6
─────────────────────────
GPIO35 (CLK)  ← CLK
GPIO33 (CMD)  ↔ CMD (bidirectional)
GPIO34 (D0)   ↔ DAT0
GPIO37 (D1)   ↔ DAT1
GPIO38 (D2)   ↔ DAT2
GPIO39 (D3)   ↔ DAT3
GND           ← GND
+3.3V         ← +3.3V
```

### Data Frame Structure

```
SDIO Block Header (8 bytes, little-endian):
  Offset  Field         Size   Purpose
  ──────────────────────────────────────
  0-1     Length        2      Payload length (< 4096)
  2-3     Offset        2      Reserved / flags
  4-7     Seq number    4      Packet sequence

Followed by:
  ├─ WiFi frame (TCP/UDP/IP packet)
  └─ CRC-7 check
```

### WiFi Throughput Calculation

```
Physical layer:    25 MHz × 4 bits × 2 edges = 50 MB/s
                   (40 MHz variant: 80 MB/s)

Protocol overhead: ~25% (SDIO framing, CRC, gaps)
Usable link:       ~36-40 MB/s @ 25 MHz
                   (~60 MB/s @ 40 MHz)

WiFi 6 air rate:   802.11ax MCS8 (80 MHz channel)
                   ~240 Mbps theoretical
                   ~150 Mbps at −50 dBm RSSI

TCP overhead:      IP (20B) + TCP (20B) + options (4B) = 44 bytes per segment
                   1460 bytes payload / 1504 total = 97% efficiency

Measured sustained: ~4-5 MB/s on 2.4 GHz, ~36 Mbps wireless
                    (1.024 MSPS = 2.048 MB/s @ 512 samples/ms)
```

## Bottleneck Analysis

### Critical Path: USB → WiFi

```
Stage               Capacity        Required @ 2 MSPS
──────────────────────────────────────────────────────
USB HS Bulk IN      ~30-40 MB/s    4.1 MB/s       ✓
ESP32-P4 CPU        400 MHz dual   Trivial        ✓
PSRAM bandwidth     ~80 MB/s       8.2 MB/s (R+W) ✓
SDIO (25 MHz 4-bit) ~36 MB/s       4.1 MB/s       ✓
WiFi 6 TCP          ~4-5 MB/s      4.1 MB/s       ⚠️ TIGHT
```

**Bottleneck**: WiFi 6 TCP throughput (36 Mbps sustained over the air)

**Solutions**:
- 1.0 MSPS: Comfortable (2 MB/s << 4.5 MB/s limit)
- 2.0 MSPS: Marginal (4.1 MB/s ≈ 4.5 MB/s, expect packet loss ~5-10%)
- 2.4+ MSPS: Use Ethernet (100 Mbps → 11 MB/s TCP)

## Sample Rate Configuration

### Demodulator Resampler Ratio

```
Crystal frequency: 28.8 MHz (RTL2832U default XTAL)

Sample rate configuration:
  rsamp_ratio = (xtal × 2^22) / sample_rate
              = (28.8e6 × 2^22) / sample_rate

Example @ 1.024 MSPS:
  rsamp_ratio = (28.8e6 × 4194304) / 1024000
              = 117961728

Demod registers (block=1, little-endian):
  reg 0x9F = (rsamp_ratio >> 16) & 0xFF
  reg 0xA0 = (rsamp_ratio >> 8) & 0xFF
  reg 0xA1 = rsamp_ratio & 0xFF

Plus masking: rsamp_ratio &= 0x0FFFFFFC (2-bit LSB must be 0)
```

### Valid Sample Rate Ranges

```
Narrow range:   225,001 - 300,000 Hz    (narrow tuners)
Wide range:     900,001 - 3,200,000 Hz  (most dongles)
Default:        1,024,000 Hz            (1.024 MSPS)
```

## Synchronization and Locking

### Ring Buffer (Lock-Free SPSC)

```
Producer (USB callback):
  write_index += bytes_written;
  write_index &= (RING_SIZE - 1);  // Wrap

Consumer (TCP/UDP):
  bytes = read_index - write_index;  // Atomic read
  if (bytes > 0) {
    consume(ring_buffer[read_index], bytes);
    read_index = (read_index + bytes) & mask;
  }

No mutexes needed: Single producer, single consumer
Wraparound safe: Power-of-2 size allows modulo via bitwise AND
```

### USB Control Transfer Lock

```
For blocking vendor control transfers (register R/W):
  ├─ Semaphore: USB_CTL_DONE (binary semaphore)
  ├─ Acquire before submit_control_transfer
  ├─ Release in control_callback (ISR)
  └─ Timeout: 300 ms
```

---

**See also**: [BENCHMARKS.md](BENCHMARKS.md) for performance measurements, [HARDWARE_SETUP.md](HARDWARE_SETUP.md) for wiring details, [BUGS_AND_FIXES.md](BUGS_AND_FIXES.md) for register map discoveries.
