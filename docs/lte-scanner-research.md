# LTE Cell Scanner Research — Implementation Learnings

## Research Date: 2026-03-30

## Projects Studied

| Project | URL | RTL-SDR | PSS Method | FOE Method | PBCH |
|---------|-----|---------|-----------|-----------|------|
| LTE-Cell-Scanner (Evrytania) | github.com/Evrytania/LTE-Cell-Scanner | 1.92 MSPS | Time-domain, freq-shifted templates | PSS-SSS + CRS | Yes |
| LTE-Cell-Scanner (JiaoXianjun) | github.com/JiaoXianjun/LTE-Cell-Scanner | 1.92 MSPS | Same + OpenCL GPU | Same | Yes |
| srsRAN_4G | github.com/srsran/srsRAN_4G | 1.92 MSPS | FFT convolution | CP + PSS-split + IFO | Yes |
| gr-LTE | github.com/kit-cel/gr-lte | 1.92 MSPS | Schmidl-Cox + FFT | CP correlation | Partial |
| OpenLTE | openlte.sourceforge.net | 1.92-30.72 MSPS | Coarse + fine | Multi-stage | Yes |

## Critical Findings

### 1. PSS Detection: Time-Domain with Frequency-Shifted Templates

LTE-Cell-Scanner generates **137-sample time-domain PSS templates** (128 OFDM + 9 CP), then creates
frequency-shifted copies at every 5 kHz across ±35 kHz (or wider based on PPM). Each shifted template
is slid across the entire capture buffer at **step = 1 sample**.

```
PSS_td = IDFT(ZC_freq_domain) * sqrt(128/62)
PSS_td_with_cp = [last_9_samples, PSS_td]  // 137 samples
PSS_shifted[f] = PSS_td_with_cp * exp(j*2π*f*n/Fs)  // for each f in search grid
```

The correlation peak directly indicates both timing and coarse frequency. This handles large PPM
offsets (up to 100 PPM) without IFO search.

**Key insight**: Including the CP in the template (137 vs 128 samples) provides 7% more correlation
gain and naturally aligns to CP boundaries.

### 2. Multi-Frame PSS-SSS FOE with MMSE Weighting

The single most important technique for accurate frequency estimation. LTE-Cell-Scanner accumulates
PSS-SSS phase rotation across ALL pairs in the capture:

```
For each PSS/SSS pair (16 pairs in 80ms):
  H_pss[k] = rx_pss[k] * conj(pss_ref[k])     // channel from PSS
  H_sss[k] = rx_sss[k] * conj(sss_ref[k])     // channel from SSS

  // MMSE weight per subcarrier:
  noise_power = estimate_from_smoothing_residual(H_pss)
  w[k] = |H_pss[k]|² / (2*|H_pss[k]|²*noise + noise²)

  // Accumulate weighted cross-correlation:
  M += sum_k( conj(H_sss[k]) * H_pss[k] * w[k] )

freq_error = angle(M) / (2*π * T_pss_sss)
```

Where T_pss_sss = 137/1920000 = 71.35 µs (time between SSS and PSS).

With 16 pairs and MMSE weighting: sub-100 Hz accuracy.

### 3. Three-Stage Frequency Pipeline (srsRAN)

```
Total CFO = cfo_cp + cfo_pss_split + cfo_integer
```

- **cfo_cp**: CP autocorrelation averaged over 7 symbols (effective 63-sample correlation)
- **cfo_pss_split**: PSS correlation split into two halves, phase difference
- **cfo_integer**: PSS correlation at ±1 subcarrier shift (IFO)
- All three use **EMA filtering** for stability across frames

### 4. CP Autocorrelation at 1.92 MSPS — How srsRAN Makes It Work

srsRAN averages CP correlation across **all 7 symbols in a slot**:

```c
for (int i = 0; i < max_offset; i++) {
    corr[i] = 0;
    for (int n = 0; n < 7; n++) {
        cplen = (n % 7) ? 9 : 10;
        corr[i] += dot_product_conj(&input[i], &input[i + 128], cplen) / 7;
        inputPtr += 128 + cplen;
    }
}
```

9 samples × 7 symbols = 63 effective samples. The key is averaging, not individual CP windows.

**Our bug**: We implemented averaging correctly but searched 960 offsets, finding noise peaks.
srsRAN limits the search range and uses the CP sync only for fractional CFO, not absolute timing.

### 5. CRS-Based Superfine FOE (Third Stage)

After PSS-SSS FOE brings residual below ±1 kHz:

```
Accumulate phase between consecutive CRS pilot symbols (0.5ms apart)
residual_f = angle(pilot_correlation) / (2π × 0.0005)
freq_superfine = freq_fine + residual_f
```

Achieves sub-10 Hz accuracy. Requires knowing PCI (for CRS positions).

### 6. PBCH Decode Chain

```
IQ → CP removal → 128-pt FFT → extract 72 subcarriers
→ CRS channel estimation (4 antenna port hypotheses)
→ CRS-based timing/frequency correction
→ PBCH RE extraction (240 REs per subframe)
→ QPSK soft demod with noise variance
→ Rate de-matching (1920 → 120 bits)
→ Tail-biting Viterbi (K=7, rate 1/3, 64 states)
→ CRC-16 with antenna mask (0x0000/0xFFFF/0x5555)
→ Parse MIB: bandwidth(3), PHICH(3), SFN(8), spare(10)
```

Computational cost on ESP32-P4: ~10 µs for Viterbi, ~200 µs total including FFT.

## What Our Implementation Was Missing

| Issue | Impact | Fix |
|-------|--------|-----|
| Single PSS/SSS pair for FOE | High variance, false SSS corrupts result | Accumulate across ALL pairs (16-40) |
| No MMSE weighting | Noisy subcarriers dominate phase estimate | Weight by |H|²/(2|H|²σ²+σ⁴) |
| FFT-domain PSS only | Can't handle large initial PPM well | Time-domain with freq-shifted templates |
| CP sync searching 960 offsets | Finds noise peaks | Limit search or use only for fractional CFO |
| No CRS tracking | Can't refine below ~1 kHz | Implement CRS pilot-based PLL |

## Implementation Priority

1. **Multi-pair MMSE PSS-SSS FOE** — highest impact, should reduce stddev from ~4 to <0.5 PPM
2. **Time-domain PSS with freq grid** — handles large PPM, improves detection reliability
3. **PBCH decode** — trivial on ESP32-P4, gives bandwidth + SFN
4. **CRS tracking** — sub-10 Hz accuracy for connected-mode applications

## References

- [LTE-Cell-Scanner source (Evrytania)](https://github.com/Evrytania/LTE-Cell-Scanner)
- [srsRAN_4G sync implementation](https://github.com/srsran/srsRAN_4G/tree/master/lib/src/phy/sync)
- [Daniel Estévez — LTE Sync Signals](https://destevez.net/2022/04/lte-downlink-synchronization-signals/)
- [Daniel Estévez — LTE PBCH/PDCCH](https://destevez.net/2022/07/lte-downlink-pbch-and-pdcch/)
- [EURASIP — Complete Cell Search (2017)](https://jwcn-eurasipjournals.springeropen.com/articles/10.1186/s13638-017-0886-3)
- [KIT gr-LTE paper (2013)](https://www.cel.kit.edu/download/SDR-WInnComm-Europe-2013_DemelKoslowskiJondral.pdf)
