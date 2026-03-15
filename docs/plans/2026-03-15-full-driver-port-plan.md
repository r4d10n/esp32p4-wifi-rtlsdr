# Full Driver Port Plan — rtl-sdr-blog → ESP32-P4

## Overview
Port the complete R82xx tuner driver and RTL2832U register sequences from
`rtl-sdr-blog` fork to ESP32-P4, maintaining multi-tuner compatibility.

## Tasks

### Task 1: Update rtlsdr_internal.h — Tuner Interface & Structs
**Files**: `components/rtlsdr/include/rtlsdr_internal.h`

- Add `r82xx_config` struct (i2c_addr, xtal, rafael_chip, max_i2c_msg_len, use_predetect)
- Add `r82xx_priv` struct (cfg, regs[NUM_REGS], fil_cal_code, input, has_lock, int_freq, rf_freq, init_done, buf[])
- Add `r82xx_freq_range` struct (freq, open_d, rf_mux_ploy, tf_c, xtal_cap variants)
- Add tuner vtable `rtlsdr_tuner_iface_t` (init, exit, set_freq, set_bw, set_gain, set_if_gain, set_gain_mode)
- Add `r82xx_priv` and `r82xx_config` fields to `rtlsdr_dev` struct
- Add rtlsdr_set_if_freq, rtlsdr_set_gpio_output, rtlsdr_set_gpio_bit declarations
- Add Blog V4 detection flag and USB string fields (manufact, product)

**Verify**: Compiles without errors

### Task 2: Port complete tuner_r82xx.c
**Files**: `components/rtlsdr/tuner_r82xx.c`
**Source**: `/tmp/rtl-sdr-blog/src/tuner_r82xx.c`

Port ALL functions, adapting libusb I2C calls to our rtlsdr_write_reg/read_reg:

- `r82xx_write()` — chunked I2C with shadow register update (keep our working version)
- `r82xx_write_reg()`, `r82xx_write_reg_mask()` — single register writes
- `r82xx_read()` — I2C read with bit reversal
- `r82xx_set_mux()` — full freq_ranges[] table (28 entries), TF/RF mux/xtal cap
- `r82xx_set_pll()` — complete PLL with VCO MAX hack (0x06), vco_fine_tune, ni/si, iterative SDM, autotune 128k→8k
- `r82xx_sysfreq_sel()` — full SDR mode register sequence (lna_top, mixer_top, thresholds, cp_cur, div_buf_cur, filter_cur, agc_clk, discharge)
- `r82xx_set_tv_standard()` — with filter calibration loop (PLL to 56MHz, trigger, read fil_cal_code)
- `r82xx_set_freq()` — Blog V4: upconversion, notch filter, 3-band input (HF/VHF/UHF), GPIO5. Standard R828D: cable1/air at 345MHz. Call set_mux, set_vga_gain, set_pll.
- `r82xx_set_gain()` — interleaved LNA+mixer with VGA, correct mixer auto polarity
- `r82xx_set_vga_gain()` — reg 0x0c with mask 0x9f
- `r82xx_set_bandwidth()` — complete BW table with narrow-mode HP filter computation, return int_freq
- `r82xx_init()` — write init_array (fix reg 0x06 to 0x30), set_tv_standard, sysfreq_sel
- `r82xx_standby()` — power-down sequence

Key adaptations from libusb:
- `priv->cfg->xtal` → access via dev struct
- `rtlsdr_check_dongle_model()` → use is_blog_v4 flag set during probe
- `rtlsdr_set_bias_tee_gpio()` → call through rtlsdr.c function
- `fprintf(stderr, ...)` → `ESP_LOGE/LOGW/LOGI`
- `usleep_range()` → `vTaskDelay(pdMS_TO_TICKS())`
- Return `esp_err_t` instead of int where appropriate

**Verify**: Compiles, init sequence matches reference log

### Task 3: Update rtlsdr.c — Demod & GPIO Functions
**Files**: `components/rtlsdr/rtlsdr.c`

Add functions:
- `rtlsdr_set_if_freq(dev, freq)` — write demod 1:0x19, 1:0x1a, 1:0x1b
- `rtlsdr_set_gpio_output(dev, gpio)` — read/write GPD(0x3004), GPOE(0x3003)
- `rtlsdr_set_gpio_bit(dev, gpio, val)` — read/write GPO(0x3001)
- `rtlsdr_set_sample_freq_correction(dev, ppm)` — write demod 1:0x3f, 1:0x3e

Fix functions:
- `rtlsdr_set_bias_tee()` → use set_gpio_output + set_gpio_bit with GPIO 0
- `rtlsdr_set_bias_tee_gpio(dev, gpio, on)` → new, supports GPIO 0-7 (needed for Blog V4 GPIO5)
- `rtlsdr_set_sample_rate()` — add `rsamp_ratio &= 0x0ffffffc`, demod soft-reset, call set_sample_freq_correction
- `rtlsdr_set_freq_correction()` — call set_sample_freq_correction + retune

Update init sequence in `rtlsdr_init()`:
- After tuner probe, before tuner init:
  ```
  demod_write_reg(1, 0xb1, 0x1a, 1)  // disable Zero-IF for R82xx
  demod_write_reg(0, 0x08, 0x4d, 1)  // In-phase ADC only
  rtlsdr_set_if_freq(dev, 3570000)    // set IF frequency
  demod_write_reg(1, 0x15, 0x01, 1)  // enable spectrum inversion
  ```
- Read USB string descriptors for Blog V4 detection (manufact/product from device desc)
- Store `r82xx_config` and `r82xx_priv` in dev struct
- Set `is_blog_v4` flag based on string match

Update `rtlsdr_set_sample_rate()`:
- Call tuner set_bw BEFORE writing ratio (reference order)
- Capture set_bw return value (IF freq), call rtlsdr_set_if_freq with it
- Retune center freq after BW change

**Verify**: Build passes, demod registers correct on hardware

### Task 4: Update rtlsdr.h — Public API Updates
**Files**: `components/rtlsdr/include/rtlsdr.h`

- Add `rtlsdr_set_bias_tee_gpio()` declaration
- Add `rtlsdr_get_tuner_gains()` to return correct per-tuner gain table
- Ensure all RTL-TCP command handlers have matching API functions

**Verify**: All headers compile, no missing declarations

### Task 5: Build & Hardware Test
- Build for ESP32-P4
- Flash and verify:
  - R828D tuner detected and initialized with full calibration
  - PLL locks across frequency range
  - Filter calibration completes successfully
  - IQ streaming works
  - SDR++ connects and receives
- Capture WiFi IQ samples at 936 MHz
- Compare spectral profile with direct USB reference

### Task 6: Commit & Push
- Commit with detailed message
- Push to GitHub
- Run spectral comparison test
