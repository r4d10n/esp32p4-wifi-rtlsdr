# Power Budget Analysis — ESP32-P4 RTL-SDR WiFi Bridge

Comprehensive power consumption analysis, battery/solar sizing, and comparison with alternative platforms.

## Component Power Breakdown

### Typical Operating Conditions

**Setup**: 1.024 MSPS WiFi streaming @ 936 MHz, 40 dB gain, 5 GHz WiFi, −50 dBm RSSI

| Component | Mode | Voltage | Current | Power | Notes |
|-----------|------|---------|---------|-------|-------|
| **ESP32-P4 HP Core** | Streaming | 3.3V | 120 mA | 396 mW | Both cores active, 400 MHz |
| **ESP32-P4 LP Core** | Idle | 3.3V | 5 mA | 16 mW | RTC only |
| **PSRAM (32 MB)** | Read/write | 3.3V | 30 mA | 99 mW | Continuous access for ring buffer |
| **USB Host PHY** | Active | 3.3V | 40 mA | 132 mW | High-Speed bulk transfers |
| **ESP32-C6 (WiFi)** | WiFi 6 RX/TX | 3.3V | 60 mA | 198 mW | SDIO + RF active |
| **RTL-SDR Blog V4** | Streaming | 5V | 200 mA | 1000 mW | Full current draw |
| **Misc (regulators, filters)** | — | 3.3V | 20 mA | 66 mW | Board overhead |
| **—** | | | **475 mA** | **1907 mW** | **Total @ USB 5V** |

### Power Breakdown by Mode

```
Idle (USB only, no WiFi):
  ESP32-P4:     120 mW
  RTL-SDR:      1000 mW
  ─────────────────────
  Total:        1120 mW

WiFi active, no streaming:
  ESP32-P4:     200 mW
  ESP32-C6:     200 mW
  RTL-SDR:      1000 mW (always on)
  ─────────────────────
  Total:        1400 mW

Streaming (1.024 MSPS, WiFi):
  ESP32-P4:     400 mW
  PSRAM:        100 mW
  USB PHY:      130 mW
  ESP32-C6:     200 mW
  RTL-SDR:      1000 mW
  ─────────────────────
  Total:        1830 mW (1.5 W nominal)
```

### Waveshare Board Regulation

The Waveshare ESP32-P4-WIFI6 uses:
- **USB 5V input**: Typical 5A capable
- **5V → 3.3V regulator**: MP2315 (4.5A capable)
- **5V → regulated supplies**: USB port can supply 2-3A comfortably

**Implication**: Can power RTL-SDR Blog V4 (200 mA @ 5V) directly from USB input without external supply.

## Battery Runtime Estimates

### Lithium-Ion (Li-ion) Options

#### Small Capacity (Portable)

| Battery | Voltage | Capacity | Energy | Runtime (1.5W) | Weight | Cost |
|---------|---------|----------|--------|----------------|--------|------|
| 18650 3.7V | 3.7V | 3000 mAh | 11.1 Wh | 4 hours | 45 g | $5 |
| 18650 3.7V | 3.7V | 5000 mAh | 18.5 Wh | 6.5 hours | 70 g | $8 |
| 26650 3.7V | 3.7V | 5000 mAh | 18.5 Wh | 6.5 hours | 90 g | $10 |

#### USB Power Banks

| Battery | Capacity | Voltage | Energy | Runtime (1.5W) | Weight | Cost |
|---------|----------|---------|--------|----------------|--------|------|
| Anker PowerCore 5K | 5000 mAh | 5V | 25 Wh | 16 hours | 160 g | $15 |
| Anker PowerCore 10K | 10000 mAh | 5V | 50 Wh | 33 hours | 310 g | $25 |
| Anker PowerCore 26K | 26000 mAh | 5V | 130 Wh | 87 hours | 730 g | $60 |

**Recommendation**: 5000-10000 mAh power bank for portable field work (best weight-to-runtime ratio).

#### LiFePO4 (Safer, Longer Life)

| Battery | Voltage | Capacity | Energy | Runtime (1.5W) | Cycles | Cost |
|---------|---------|----------|--------|----------------|--------|------|
| 18650 LiFePO4 | 3.2V | 1500 mAh | 4.8 Wh | 3 hours | 3000+ | $12 |
| 26650 LiFePO4 | 3.2V | 3500 mAh | 11.2 Wh | 7.5 hours | 3000+ | $20 |

**Advantage**: Safer chemistry, longer lifespan (3000+ cycles vs 300-500 for Li-ion). Better for long-term deployments.

### Alkaline/NiMH Batteries

| Type | Voltage | Capacity | Energy | Runtime (1.5W) | Cost |
|------|---------|----------|--------|----------------|------|
| 4x AA alkaline | 6V | 2500 mAh (3000-hour rating) | 15 Wh | 10 hours | $4 |
| 4x AA NiMH 2500 mAh | 4.8V | 2500 mAh (rechargeable) | 12 Wh | 8 hours | $8 |
| 8x AA NiMH | 4.8V | 2500 mAh | 12 Wh (in series) | 8 hours | $16 |

**Advantage**: No special charging, shelf-stable. **Disadvantage**: Higher cost per Wh, environmental impact.

## Solar Panel Sizing

### Typical Solar Irradiance

| Condition | Power Density | Device Equivalent |
|-----------|---------------|-------------------|
| Peak sun (noon, clear) | 1000 W/m² | 100% |
| Partly cloudy | 600 W/m² | 60% |
| Heavy cloud | 200 W/m² | 20% |
| Deep indoors | 50 W/m² | 5% |

### Panel Sizing for Continuous Operation

**Power requirement**: 1.5 W continuous (WiFi streaming)

**Minimum panel size**:
```
Panel power = Device power / Efficiency
Panel power = 1.5 W / (0.85 × 0.9)  [regulator × MPPT efficiency]
            = 1.96 W required

Panel types:
  2W monocrystalline (4-5 cm²):  ~$5     (5V, direct regulation)
  5W monocrystalline (10 cm²):   ~$12    (5V, with MPPT controller)
  10W monocrystalline (20 cm²):  ~$20    (18V, requires MPPT)
```

**Realistic solar setup** (outdoor, field deployment):

| System | Panel | MPPT | Battery | Cost | Notes |
|--------|-------|------|---------|------|-------|
| Portable (low sun) | 10W | Optional | 10Ah Li-ion | $50 | Works morning/afternoon |
| Field (moderate sun) | 20W | Required | 20Ah LiFePO4 | $100 | All-day operation if cloudy |
| Robust (variable) | 30W | Required | 30Ah LiFePO4 | $150 | 24/7 even in winter |

**Example**: 10W solar + 10Ah Li-ion battery

```
Sunny day (6 hours @ 800 W/m² average):
  Panel input:      10W × 0.8 = 8W actual
  Charger overhead: −15% = 6.8W charging
  Device draw:      −1.5W = 5.3W surplus
  Battery charge gain: 5.3W × 6h = 31.8 Wh

Cloudy day (no useful solar):
  Battery capacity: 37 Wh (10 Ah @ 3.7V)
  Device draw:      1.5W
  Runtime:          37 Wh ÷ 1.5W = 25 hours

Night + next cloudy day:
  Total runtime:    25 + 25 = 50 hours before recharge

Conclusion: Sustainable with 10W solar + 10Ah for 3-day field deployment in variable weather
```

## Duty Cycle and Energy Savings

### Measurement vs Continuous Streaming

Many field applications don't need continuous streaming; periodic measurement reduces power significantly.

```
Measurement protocol:
  ├─ 1 second stream: 2 MB captured
  ├─ 10 second gap: ESP32-P4 CPU idle, C6 WiFi standby
  └─ Repeat

Power consumption:
  Active (1s):     1.5W × 1s = 1.5 Ws
  Idle (9s):       0.3W × 9s = 2.7 Ws
  ─────────────────────────────────
  Per 10s cycle:   4.2 Ws = 0.42W average

Advantage: 72% power reduction vs continuous streaming
Battery runtime: 10Ah @ 0.42W = 357 hours ≈ 15 days
```

### Scenario: Frequency Scan

```
Example: Scan 900-1000 MHz in 1 MHz steps
  Channels:         100
  Per-channel time: 500 ms
  Total time:       50 seconds
  Power draw:       1.5W continuous
  Energy:           1.5W × 50s = 75 Wh

Battery (5000 mAh Li-ion @ 3.7V = 18.5 Wh):
  Scans possible:   18.5 ÷ 75 = 0.25 scans (one scan uses 4x battery capacity!)

Optimized scan (10 ms per channel, dump to local storage):
  Total time:       1 second
  Power draw:       1.5W
  Energy:           1.5 Ws
  Battery life:     18.5 Wh ÷ 1.5 W = 12 hours continuous scanning
  Or:               >100 full frequency scans on single charge
```

**Recommendation**: For field deployments, use duty-cycled measurement (periodic scans) rather than continuous streaming.

## Power Comparison: Raspberry Pi vs ESP32-P4

### Specifications Comparison

| Metric | RPi 5 | RPi Zero 2W | RPi 4 | **ESP32-P4** |
|--------|-------|------------|-------|-------------|
| **Processor** | BCM2712 2.4GHz × 4 | RP3A0 × 4 (1GHz) | BCM2711 × 4 (1.5GHz) | RISC-V × 2 (400MHz) |
| **RAM** | 2-8 GB | 512 MB | 1-8 GB | 768 KB + 32 MB PSRAM |
| **WiFi** | None (USB) | Built-in | None (USB) | **Built-in (C6)** |
| **USB Host** | Via PCH | Single port | Via PCH | **Native HS** |
| **Cost** | $80 | $15 | $55 | **$30** |

### Power Consumption Comparison

| Mode | RPi 5 | RPi Zero 2W | RPi 4 | **ESP32-P4** | Power Ratio |
|------|-------|------------|-------|-------------|-------------|
| **Idle** | 2.6 W | 0.4 W | 0.5 W | **0.15 W** | 3× better than Zero |
| **WiFi on, no traffic** | 4.2 W | 1.2 W | 2.0 W | **0.3 W** | 4× better |
| **Streaming (2 MB/s TCP)** | 8.0 W | 2.0 W | 3.5 W | **1.5 W** | 1.3× better |
| **Peak (CPU 100%)** | 12 W | 4.0 W | 6.0 W | **2.0 W** | 2× better |

### Field Deployment Scenario (30-day battery operation)

**Requirement**: Scan 100 MHz wide band every 5 minutes, store results locally.

```
ESP32-P4 Solution:
  Scan duration:    1 second @ 1.5W = 1.5 Ws
  Idle duration:    299 seconds @ 0.3W = 89.7 Ws
  Per 5-min cycle:  91.2 Ws = 0.30W average

  30-day power:     0.30W × 30 days × 24h = 216 Wh
  Battery needed:   20 Ah @ 3.7V = 74 Wh (too small)
                    20000 mAh power bank = 100 Wh (OK)

  Cost:             $30 (board) + $25 (battery) + $30 (solar) = $85

Raspberry Pi Zero 2W Solution:
  Scan duration:    1 second @ 2W = 2 Ws
  Idle duration:    299 seconds @ 1.2W = 359 Ws
  Per 5-min cycle:  361 Ws = 1.20W average

  30-day power:     1.20W × 30 × 24 = 864 Wh
  Battery needed:   ~230 Wh (multi-pack lithium)

  Cost:             $15 (board) + $100 (battery) + $50 (solar) = $165

Savings with ESP32-P4:
  Power:            75% reduction (0.30W vs 1.20W)
  Battery:          73% reduction (100 Wh vs 230 Wh)
  Cost:             49% reduction ($85 vs $165)
  Weight:           60% reduction (300g vs 750g)
```

**Conclusion**: ESP32-P4 is **significantly more efficient** for power-constrained field deployments.

## Recommended Power Solutions by Use Case

### Urban/Indoor Portable (2-hour mission)
```
Budget option:
  Waveshare ESP32-P4-WIFI6:  $30
  5000 mAh USB power bank:   $15
  ─────────────────────────────
  Total:                     $45
  Runtime:                   ~15 hours (far exceeds need)
```

### Field Expedition (1-week battery-only)
```
Standard setup:
  Waveshare board:           $30
  2x 5000 mAh power banks:   $30 (serialize or parallel)
  ─────────────────────────────
  Total:                     $60
  Energy:                    ~100 Wh
  Runtime @ 1.5W:            ~67 hours (2.8 days)
  Plus duty cycle:           ~7 days if duty-cycled

Note: One set per 2-3 days, rotate between sun-charging and use
```

### Off-Grid Permanent (30+ days)
```
Recommended:
  Waveshare board:               $30
  10W solar panel (monocrystal): $20
  30 Ah LiFePO4 battery:         $80 (with charger)
  MPPT controller:               $30
  ─────────────────────────────────
  Total:                         $160

  Capacity:           30 Ah × 3.2V = 96 Wh
  Output capability:  ~60 Wh usable (80% discharge)
  Solar input:        10W × 6h/day avg = 60 Wh/day (good weather)

  Operating model:
    Daytime (6h):     Powered by solar + charging battery
    Night (18h):      Battery discharge (1.5W × 18h = 27 Wh)
    Balance:          +33 Wh/day surplus → unlimited operation

  Weather resilience: 5 days battery alone before recharge needed
```

### Vehicle Installation (12V car)
```
Direct 12V input:
  Waveshare board (5V input) requires:
    ├─ Car 12V → 5V buck converter: $10 (4A capable)
    └─ 1 Ah load is acceptable on 12V accessory circuit

  Advantage: Unlimited power from alternator
  Note: Car alternator cycles (12-14.5V); buck converter regulates to 5V
```

## Power Optimization Tips

### Firmware Level

1. **Disable unused cores**:
   ```c
   // If not using dual-core, disable LP core for 40 mW savings
   esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
   ```

2. **Reduce clock frequency** (trade performance for power):
   ```c
   // Use 160 MHz instead of 400 MHz for non-critical applications
   esp_pm_config_esp32p4_t pm_config = {
       .max_freq_mhz = 160,
       .min_freq_mhz = 80,
   };
   ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
   // Saves ~200 mW, reduces throughput to ~500 kSPS
   ```

3. **WiFi power save mode**:
   ```c
   wifi_ps_type_t ps_type = WIFI_PS_MODEM;  // Modem sleep
   esp_wifi_set_ps(ps_type);
   // Saves ~100 mW when idle, minimal latency impact
   ```

### Hardware Level

1. **External USB power supply**: Use dedicated 5V 2A supply instead of USB bus
   - Eliminates computer idle power drain
   - Stable voltage for best USB performance

2. **Solar panel orientation**: Face south (north hemisphere), 30° tilt
   - Increases average power 15-25% vs horizontal

3. **Thermal management**: Keep in 20-30°C range
   - Regulators are more efficient at moderate temperature
   - Below 10°C, battery voltage sags; add thermal insulation

## Battery Chemistry Comparison

| Chemistry | Voltage | Density | Cost/Wh | Cycles | Safety | Temp |
|-----------|---------|---------|---------|--------|--------|------|
| **Li-ion** | 3.7V | High | $0.10 | 500 | Thermal risk | 0-45°C |
| **LiFePO4** | 3.2V | Med | $0.20 | 3000 | Excellent | −10 to 60°C |
| **Li-Po** | 3.7V | High | $0.12 | 300 | Fire risk | 0-40°C |
| **Alkaline** | 1.5V | Low | $0.50 | 1 | Safe | −10 to 65°C |
| **NiMH** | 1.2V | Low | $0.15 | 1000 | Safe | −10 to 60°C |

**Recommendation for field**: **LiFePO4** — safest rechargeable, best cycle life, wide temperature range. Best for durability in remote locations.

---

**See also**: [HARDWARE_SETUP.md](HARDWARE_SETUP.md) for power connector details, [BENCHMARKS.md](BENCHMARKS.md) for power efficiency measurements.
