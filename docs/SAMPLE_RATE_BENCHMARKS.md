# Sample Rate Benchmarks: 250 kSPS to 3.2 MSPS

## Test Setup
- Hardware: Waveshare ESP32-P4-WIFI6 + RTL-SDR Blog V4 (R828D)
- WiFi: ESP32-C6, SDIO 25 MHz, 4-bit, WiFi 6 HT20
- Measurement: 8 seconds per rate, TCP then UDP sequential
- Date: 2026-03-15

## Results

```
Rate         │ Expected │ TCP kSPS   TCP%   Mbps │ UDP kSPS   UDP%  Loss%   Mbps
──────────── │ ──────── │ ──────── ────── ────── │ ──────── ────── ────── ──────
250 kSPS     │     250  │   262.9  105.2%   4.2 │   249.8   99.9%   0.0%   4.0 ✓✓
500 kSPS     │     500  │   237.1   47.4%   3.8 │   235.3   47.1%   0.0%   3.8 ✗✗
900 kSPS     │     900  │   736.7   81.9%  11.8 │   899.3   99.9%   0.0%  14.4 △✓
1.000 MSPS   │    1000  │   625.7   62.6%  10.0 │   998.4   99.8%   0.0%  16.0 ✗✓
1.024 MSPS   │    1024  │   619.3   60.5%   9.9 │   610.3   59.6%   0.2%   9.8 ✗✗*
1.200 MSPS   │    1200  │   733.8   61.2%  11.7 │   904.5   75.4%   0.1%  14.5 ✗△
1.400 MSPS   │    1400  │   644.9   46.1%  10.3 │     0.0    0.0%   0.0%   0.0 ✗✗
1.600 MSPS   │    1600  │   516.5   32.3%   8.3 │     0.0    0.0%   0.0%   0.0 ✗✗
1.800 MSPS   │    1800  │   737.1   41.0%  11.8 │     0.0    0.0%   0.0%   0.0 ✗✗
2.000 MSPS   │    2000  │   514.6   25.7%   8.2 │     0.0    0.0%   0.0%   0.0 ✗✗
2.048 MSPS   │    2048  │   588.6   28.7%   9.4 │     0.0    0.0%   0.0%   0.0 ✗✗
2.400 MSPS   │    2400  │   693.9   28.9%  11.1 │     0.0    0.0%   0.0%   0.0 ✗✗
2.800 MSPS   │    2800  │   627.7   22.4%  10.0 │     0.0    0.0%   0.0%   0.0 ✗✗
3.200 MSPS   │    3200  │   736.7   23.0%  11.8 │     0.0    0.0%   0.0%   0.0 ✗✗
```
✓ = >95% delivery   △ = 70-95%   ✗ = <70%
*1.024 MSPS UDP affected by sequential test interference (isolated: 100%)

## Key Findings

### USB HS Internal Transfer
- **All rates 250 kSPS to 3.2 MSPS work on USB HS**
- USB 2.0 HS bulk throughput: ~30+ MB/s, only 6.4 MB/s needed at 3.2 MSPS
- The ESP32-P4 USB host controller handles all standard RTL-SDR rates

### WiFi Delivery Ceiling
- **TCP ceiling: ~737 kSPS** (~11.8 Mbps) regardless of USB input rate
- **UDP ceiling: ~1000 kSPS** (~16 Mbps) with near-zero loss
- Bottleneck: ESP32-C6 single-core 160 MHz processing WiFi TX
- Above 1.2 MSPS: WiFi cannot deliver, use Ethernet

### Recommended Operating Points

| Use Case | Rate | Transport | Delivery |
|----------|------|-----------|----------|
| Narrowband FM/AM | 250 kSPS | TCP or UDP | 100% |
| General SDR | 900 kSPS | UDP | 99.9% |
| Max WiFi rate | 1.000 MSPS | UDP | 99.8% |
| Full bandwidth | 2.4 MSPS | Ethernet | ~100% (est.) |

### Avoid: 500 kSPS
Falls in the RTL2832U resampler gap (300-900 kSPS range has irregular
USB bulk transfer timing). Use 250 kSPS or 900 kSPS+ instead.

### UDP Drops Above 1.2 MSPS
The UDP server accumulator overflows when WiFi can't keep up, causing
the client to appear disconnected. TCP degrades more gracefully because
flow control throttles the sender. Future improvement: UDP should
degrade to dropping individual packets rather than disconnecting.
