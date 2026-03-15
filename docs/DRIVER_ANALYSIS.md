# RTL-SDR Blog V4 Driver Analysis — Gaps vs Reference Implementation

## Source: https://github.com/rtlsdrblog/rtl-sdr-blog

## Critical Differences (will affect IQ data quality)

### 1. Missing IF Frequency Register Setup
**Reference** (librtlsdr.c after tuner detect, before tuner init):
```c
rtlsdr_set_if_freq(dev, R82XX_IF_FREQ);  // 3,570,000 Hz
```
This writes to demod registers `1:0x19`, `1:0x1a`, `1:0x1b`:
```c
if_freq = ((freq * TWO_POW(22)) / rtl_xtal) * (-1);
reg 1:0x19 = (if_freq >> 16) & 0x3f;
reg 1:0x1a = (if_freq >> 8) & 0xff;
reg 1:0x1b = if_freq & 0xff;
```
**Our code**: Never sets IF frequency registers. The RTL2832U DDC doesn't know where the tuner IF is.
**Impact**: SEVERE — IQ signal will be offset from center, causing incorrect frequency display.

### 2. Wrong Demod Register After Tuner Detect
**Reference**:
```c
rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1a, 1);  // disable Zero-IF
rtlsdr_demod_write_reg(dev, 0, 0x08, 0x4d, 1);  // In-phase ADC only
rtlsdr_demod_write_reg(dev, 1, 0x15, 0x01, 1);  // enable spectrum inversion
```
**Our code** (in init_baseband):
```c
rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1b, 1);  // WRONG: 0x1b vs 0x1a
rtlsdr_demod_write_reg(dev, 1, 0x15, 0x00, 1);  // WRONG: 0x00 vs 0x01
// Missing: 0:0x08 = 0x4d
```
**Impact**: Spectrum inversion wrong, ADC input configuration wrong.

### 3. PLL Implementation Heavily Simplified
**Reference** PLL has:
- VCO current set to MAX: `reg 0x12 = 0x06` (full register, not just bits)
- Read-back of VCO fine tune from `data[4] & 0x30` to adjust div_num
- ni/si decomposition: `ni = (nint-13)/4; si = nint-4*ni-13; reg 0x14 = ni + (si<<6)`
- SDM calculator uses iterative bit-shifting, not simple division
- pw_sdm flag in reg 0x12 bit 3
- PLL autotune changes from 128kHz to 8kHz after lock
- PLL lock verification with retry and max VCO current boost

**Our code**: Simplified PLL with basic N/SDM division, no fine-tune readback, no ni/si split, no pw_sdm.
**Impact**: PLL may not lock reliably at some frequencies, tuning accuracy degraded.

### 4. Missing r82xx_set_mux() — Frequency-Dependent RF Frontend Config
**Reference** has a `freq_ranges[]` table with 28 entries covering 0-2000 MHz, each specifying:
- `open_d` — drain control
- `rf_mux_ploy` — RF mux and polyphase filter select
- `tf_c` — tracking filter capacitor bank
- `xtal_cap20p/10p/0p` — crystal load capacitance

Registers written: `0x17[3]`, `0x1a[7:6,1:0]`, `0x1b[7:0]`, `0x10[3,1:0]`, `0x08[5:0]`, `0x09[5:0]`

**Our code**: Simplified 3-band RF mux (≤50M, ≤340M, >340M), no tracking filter, no xtal cap.
**Impact**: Poor sensitivity and selectivity, especially at band edges.

### 5. Missing r82xx_set_tv_standard / r82xx_sysfreq_sel
**Reference** has detailed per-band system frequency selection with:
- LNA top current, mixer top, LNA discharge
- Mixer filter cap, pre-detect threshold
- AGC clock rate selection
- Channel filter extension

**Our code**: Very simplified version with only a few registers.
**Impact**: Suboptimal gain distribution and filter response.

### 6. Blog V4 Multi-Input Switching
**Reference** (Blog V4 specific):
```
HF (≤28.8 MHz): Cable2 input, upconvert by 28.8 MHz, GPIO5 control
VHF (28.8-250 MHz): Cable1 input
UHF (>250 MHz): Air input
Notch filters: controlled per frequency range
```
Registers: `0x06[3]`, `0x05[6]`, `0x05[5]`, GPIO5 via rtlsdr_set_bias_tee_gpio

**Our code**: Standard R828D switching only (Cable1/Air at 345 MHz).
**Impact**: HF reception won't work, VHF/UHF input selection suboptimal for Blog V4.

### 7. Bandwidth Returns IF Frequency
**Reference**: `r82xx_set_bw()` returns `priv->int_freq` (the IF frequency changes with BW).
The caller then calls `rtlsdr_set_if_freq()` to update the demod DDC.
**Our code**: Doesn't return int_freq or update demod IF registers.
**Impact**: IF offset drifts when changing bandwidth.

### 8. Missing VGA Gain Control
**Reference**: `r82xx_set_vga_gain()` writes `reg 0x0c[3:0]` for VGA gain, called during freq change.
**Our code**: No VGA gain control.
**Impact**: Gain control only uses LNA+mixer, missing VGA stage.

### 9. GPIO Register Addresses
**Reference**:
```
GPO  = 0x3001  (not 0x0000)
GPOE = 0x3003
GPD  = 0x3004
```
**Our code**: Uses `RTLSDR_BLOCK_SYS, 0x0000` for bias tee — wrong address.

---

## Multi-Tuner Architecture Plan

### Supported Tuners
| Tuner | I2C Addr | Chip Enum | Source File |
|-------|----------|-----------|-------------|
| R820T/R820T2 | 0x34 | CHIP_R820T | tuner_r82xx.c |
| R828D | 0x74 | CHIP_R828D | tuner_r82xx.c |
| E4000 | 0xC8 | - | tuner_e4k.c |
| FC0012 | 0xC6 | - | tuner_fc0012.c |
| FC0013 | 0xC6 | - | tuner_fc0013.c |
| FC2580 | 0xAC | - | tuner_fc2580.c |

### Interface (function table pattern from librtlsdr)
```c
typedef struct {
    esp_err_t (*init)(rtlsdr_dev_t *dev);
    esp_err_t (*exit)(rtlsdr_dev_t *dev);
    esp_err_t (*set_freq)(rtlsdr_dev_t *dev, uint32_t freq);
    int       (*set_bw)(rtlsdr_dev_t *dev, int bw);  // returns IF freq
    esp_err_t (*set_gain)(rtlsdr_dev_t *dev, int gain);
    esp_err_t (*set_if_gain)(rtlsdr_dev_t *dev, int stage, int gain);
    esp_err_t (*set_gain_mode)(rtlsdr_dev_t *dev, int manual);
} rtlsdr_tuner_iface_t;
```

### Priority Order
1. **R828D Blog V4** — current hardware, needs full driver match
2. **R820T/R820T2** — most common, shares 90% of R828D code
3. **E4000** — different architecture, separate driver needed
4. **FC0012/FC0013/FC2580** — less common, lower priority

### Implementation Strategy
- Port the complete `tuner_r82xx.c` from rtl-sdr-blog (not simplified)
- Include the full `freq_ranges[]` table
- Implement proper `r82xx_set_mux()`, `r82xx_set_pll()`, `r82xx_sysfreq_sel()`
- Add Blog V4 detection via USB string descriptors
- Add `rtlsdr_set_if_freq()` to demod driver
- Fix GPIO register addresses
