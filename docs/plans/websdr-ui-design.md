# WebSDR Rich UI Design Document

## ESP32-P4 RTL-SDR Web Interface -- Comprehensive Design Blueprint

**Date**: 2026-03-16
**Status**: Design Phase -- Research Complete
**Target**: Browser-based SDR interface served from ESP32-P4 over WiFi/Ethernet

---

## Table of Contents

1. [SDR Software Analysis](#1-sdr-software-analysis)
2. [Hardware Rig UI Analysis](#2-hardware-rig-ui-analysis)
3. [Feature Comparison Matrix](#3-feature-comparison-matrix)
4. [Recommended Layout](#4-recommended-layout)
5. [Color Palette](#5-color-palette)
6. [Component Specifications](#6-component-specifications)
7. [DSP Feature List](#7-dsp-feature-list)
8. [Implementation Priority](#8-implementation-priority)
9. [Technology Choices](#9-technology-choices)
10. [Design Alternatives](#10-design-alternatives)
11. [Constraints and Trade-offs](#11-constraints-and-trade-offs)

---

## 1. SDR Software Analysis

### 1.1 Wolf SDR (UA3REO DDC Transceiver) -- GOLDEN REFERENCE

**Source**: github.com/XGudron/UA3REO-DDC-Transceiver

The Wolf SDR is a complete DDC/DUC transceiver with a 7-inch capacitive touchscreen. Its screen layout is the gold standard for information density without clutter.

**Screen Layout (top to bottom)**:
- **Top bar**: Frequency display (large digital readout, colored digits), mode indicator (CW/LSB/USB/AM/FM/WFM/DIGI), VFO A/B labels, RIT/XIT offset, filter bandwidth readout
- **Status row**: S-meter (analog style with needle), power meter, SWR bar, voltage/temperature indicators
- **Center (dominant area)**: Spectrum + Waterfall (panorama up to 384 kHz wide), occupying ~65% of screen height
- **Bottom bar**: Quick-access buttons for bands, functions, menu navigation (optimized for 7-inch touch)

**Key UI Features**:
- FFT styles: gradient, fill, dots, contour, gradient+contour
- Waterfall colors: 7 schemes (blue-yellow-red, black-yellow-red, black-yellow-green, black-red, black-green, black-blue, black-white)
- 3D spectrum modes (lines, pixels)
- Peak hold, lens (center magnification), FFT compressor
- Color themes: black, white, black-with-colored-frequency
- Touchpad gesture recognition (click, hold, swipe)
- Dual receiver mixing: A+B stereo, A&B, A-only, B-only
- Per-digit encoder tuning with acceleration

**DSP Features**:
- AGC with adjustable attack rate and K-weighting
- DNR (noise reduction) with adjustable level
- Noise blanker with threshold control
- Auto-notch and manual notch filter
- HPF/LPF adjustable per-mode
- Gaussian CW filter
- RX/TX equalizer (7-band)
- CESSB compression
- VOX, VAD (voice activity detection)
- FM squelch, carrier squelch
- CW decoder, FT8/FT4 decoder, RTTY decoder, SSTV decoder, RDS decoder

**Takeaway for our design**: The Wolf layout proves that frequency display + S-meter + spectrum/waterfall + control bar can fit cleanly on a single screen. The proportions (small header, dominant spectrum, compact controls) are optimal.

---

### 1.2 SDR Console V3

**Source**: sdr-radio.com

**Signature Feature -- Single-Control Dynamic Range/Reference Level**:
SDR Console V3 is famous for its display control approach. When hovering the mouse over the center-right of the spectrum display, overlay controls appear:
- **Scale**: Vertical axis dB range toggle
- **Low / High**: Adjustable boundaries for dB scale. "High" can be set to AUTO, which dynamically tracks signal peaks and adjusts the vertical scale automatically
- **Zoom**: Horizontal frequency zoom with magnify/shrink buttons
- **Contrast strip**: A vertical strip on the right edge of the waterfall that you drag up/down to adjust waterfall color mapping thresholds

**Spectrum Display Features**:
- Hang & Decay smoothing (default averaging mode)
- Multiple smoothing algorithms
- Mixed mode: spectrum above, waterfall below, resizable split
- Fully customizable color palette with built-in colormap editor
- Resolution enhancement via FFT overlapping
- Speed: typically 40 lines/second
- Timestamps on waterfall
- 3D waterfall mode (perspective view)
- Signal markers and frequency annotations

**Takeaway**: The auto-scaling reference level and the hover-reveal overlay controls are excellent UX patterns for a web interface. We should implement the "High=AUTO" concept.

---

### 1.3 FlexRadio SmartSDR

**Source**: flexradio.com

**UI Architecture**:
- "Panafall" = joined Panadapter + Waterfall with shared horizontal frequency axis
- Multiple "Slice Receivers" visible as colored overlays on the panadapter (each with its own frequency, mode, filter)
- Each slice has independent demodulation, filter bandwidth, mode
- Up to 4+ slices on a single panadapter
- Dual panadapter mode: VFO A and VFO B side-by-side or stacked

**S-Meter**: Digital bar-graph integrated into slice receiver panel, with signal strength displayed above VFO panel

**Frequency Control**: Click anywhere on the panadapter to place a slice. Drag slice center to tune. Drag slice edges to resize filter bandwidth.

**Key Innovations**:
- Slice receiver concept (multiple independent demodulators on one wideband display)
- API-driven architecture (SmartSDR API provides complete command/control/status/metering/streaming)
- Off-screen slice indicator shows where a slice is tuned relative to visible panadapter

**Takeaway**: The "slice" concept is aspirational for multi-demodulator support. For single-demodulator (our case), the filter overlay with draggable edges is the key pattern to adopt.

---

### 1.4 SDR# (SDRSharp)

**Source**: airspy.com / rtl-sdr.com

**UI Layout**:
- **Left panel**: Collapsible tabbed sections (Radio, Audio, AGC, FFT Display), each with +/- toggle
- **Top-right**: Large frequency display with per-digit scroll tuning
- **Center-right**: Spectrum display (upper) + Waterfall display (lower)
- **Right edge**: Zoom slider, Contrast slider, Range slider, Offset slider

**Control Panels**:
- **Radio**: Mode selection (NFM, WFM, AM, LSB, USB, CW, DSB, RAW), bandwidth, frequency step, shift, squelch
- **Audio**: Sample rate, input/output device
- **AGC**: Enable/disable, threshold, decay, slope, hang
- **FFT Display**: Resolution, window function, view mode, averaging

**Spectrum Features**:
- Red tuning line with shaded bandwidth overlay
- Click-to-tune on spectrum
- Peak hold
- Gradient fill options
- Frequency annotations

**Plugin Architecture**: Plugins installed by copying DLLs to a subfolder. Plugins can add panels, modify DSP chain, overlay on spectrum. Hundreds of community plugins exist.

**Takeaway**: The collapsible left-panel pattern is proven for complex settings. The per-digit frequency scroll is the standard interaction pattern.

---

### 1.5 OpenWebRX / OpenWebRX+

**Source**: openwebrx.de, github.com/jketterl/openwebrx

This is the most directly relevant reference -- it IS a browser-based SDR.

**Technical Architecture**:
- WebSocket binary protocol with type byte prefix (0x01 = FFT, 0x03 = secondary FFT)
- ADPCM compression on FFT stream reduces bandwidth from 2 Mbit/s to ~200 kbit/s per client
- Multiple HTML canvas elements for waterfall (dynamic creation, managed by JS)
- Separate canvas for frequency scale
- Server-side FFT with configurable size, FPS, overlap factor

**Waterfall Rendering**:
- `buildWaterfallColors()` creates gradient from scheme
- `updateWaterfallColors()` adjusts min/max thresholds
- Auto and continuous color adjustment modes
- New line rendered at top, previous content scrolled down
- Multi-canvas approach for memory efficiency

**Touch/Mouse Interaction**:
- Click waterfall to tune demodulator
- Drag to pan frequency
- Mouse wheel to zoom (maintaining center frequency)
- Touch panning and zooming supported
- `canvas_mousedown/mousemove/mouseup` event handlers convert pixel to frequency

**DSP Features (OpenWebRX+)**:
- Noise reduction via spectral subtraction (adjustable threshold, range widened to 20 dB)
- Squelch with slider and SQ toggle button
- AGC for SSB/AM/NFM modes (configurable)
- Frequency scanner monitoring bookmarked frequencies
- Tuning steps

**Takeaway**: The multi-canvas waterfall approach, ADPCM compression, and binary WebSocket protocol are directly applicable. Our existing architecture already follows a similar pattern (type-byte binary messages).

---

### 1.6 SDRAngel

**Source**: github.com/f4exb/sdrangel

**UI Architecture**:
- Workspace-based: multiple resizable/movable component windows within workspaces
- Components: device panel, main spectrum, channel panels, feature panels
- Multiple workspaces switchable via side tabs
- Auto-layout via "Stack" button (right-click)
- Drag/drop for window repositioning

**Multi-Channel Design**:
- One or more channel Rx plugins per device
- Each channel has: offset frequency dial, demodulator settings, meter
- Main device dial for center frequency
- Channel dial for offset within passband

**Takeaway**: The multi-workspace concept is too complex for our embedded target. However, the channel-offset-from-center frequency model matches our DDC architecture perfectly.

---

### 1.7 GQRX

**Source**: gqrx.dk

**UI Layout**:
- **Upper-right**: Receiver input and options tabs
- **Lower-right**: FFT settings and audio output tabs
- **Center**: Spectrum (pandapter) above, Waterfall below
- **Resize handle**: "Pandapter <-> WF" slider to adjust split ratio

**Key Features**:
- Peak detection with circles on peaks ("Peak D" button)
- Peak envelope hold ("Peak H" button)
- Zoom: R (reset to 100%), C (center on original freq), D (center on demod freq)
- Adaptive FFT averaging: noise floor averaged more than strong signals
- FFT averaging only affects pandapter, NOT waterfall
- Audio FFT display

**Takeaway**: The adaptive averaging algorithm (averaging factor dependent on signal level) is an excellent approach. The pandapter/waterfall resize slider is a standard UX pattern we should include.

---

### 1.8 MaiaSDR

**Source**: maia-sdr.org

**Architecture (most technically relevant for our project)**:
- FPGA computes 4096-point windowed FFT at full sample rate
- Non-coherent integration reduces data rate (multiple FFT vectors averaged)
- Async Rust HTTP server (maia-httpd) on ARM CPU
- WebSocket streams waterfall data
- RESTful API for device control
- Web UI (maia-wasm) in Rust + WebAssembly

**WebGL2 Waterfall Rendering**:
- Waterfall data (dB values) written to a 2D texture line-by-line via `texSubImage2D()`
- Only changed parts of texture updated (not full redraw)
- "Doubly-mapped buffer" trick for scrolling: a rectangle is shaded using the texture
- Fragment shader does colormap lookup via 1D texture
- GLSL ES 3.0 `texture()` function for colormap application

**Performance**: Supports up to 61.44 Msps waterfall display

**Takeaway**: WebGL2 waterfall rendering is vastly more efficient than Canvas `putImageData()`. The texture-based scrolling avoids expensive pixel copying. This is the correct approach for high performance on mobile browsers. However, it adds implementation complexity.

---

### 1.9 CubicSDR

**Source**: cubicsdr.com

**UI Layout**:
- **Main spectrum + waterfall**: Central display, resizable split
- **Modem spectrum + waterfall**: Secondary channelized display showing demodulated signal
- **Tuning bar**: Three adjustable values (center freq, modem freq, bandwidth) with per-digit right-click snap
- **Modulation selector**: Dropdown (AM, FM, FMS, NBFM, LSB, USB, DSB, I/Q)

**Visual Bandwidth Selection**:
- Drag modem center to tune
- Drag modem edge to change bandwidth
- Shift+click on waterfall to add new modem (vs click to retune existing)
- Right-drag up/down on spectrum to adjust gain visually

**Control Meters**: Squelch, Audio Gain, Spectrum Averaging, Waterfall Speed, Manual Gain -- all as vertical slider meters

**Takeaway**: The right-drag for gain control and the visual modem edge dragging are elegant interaction patterns.

---

### 1.10 Thetis/PowerSDR

**Source**: github.com/ramdor/Thetis

**UI Design**:
- Classic ham radio transceiver aesthetic
- Large VFO A and VFO B frequency displays
- Analog-style multimeter with customizable items (rotator, band/filter/mode/antenna indicators, LED indicators)
- Per-digit mouse wheel tuning on frequency display
- Auto-height multimeter containers
- Database manager for settings persistence

**Meter System**:
- Multiple meter items: S-meter, power, SWR, ALC, compression
- Meter VFO control: select VFOA, VFOB, or BOTH
- Interactive elements within meter items

**Takeaway**: The multi-item meter concept (S-meter + power + SWR in one container) is useful for our design, particularly when we add TX support later.

---

## 2. Hardware Rig UI Analysis

### 2.1 ICOM IC-7300 / IC-7610

- **Spectrum scope**: Class-leading resolution, sweep speed, and dynamic range
- **Touch-to-tune**: First touch magnifies area around signal, second touch tunes to it (two-tap zoom-and-tune)
- **Scroll mode**: Automatically keeps operating signal within scope range
- **Waterfall**: Shows signal strength over time, reveals weak signals not apparent on spectrum
- **Audio scope**: FFT scope (frequency components) + oscilloscope (time domain waveform)
- **Meters**: ALC, compression, SWR, and other diagnostics as bar graphs
- **Display split**: Bandscope + waterfall + meters, or bandscope + waterfall + audio scope

### 2.2 Yaesu FTDX-101D

- **7-inch TFT**: 800x480 resolution, touch panel
- **3DSS (3-Dimensional Spectrum Stream)**: Unique 3D waterfall perspective view
- **Spectrum span**: 1 kHz to 1 MHz in 10 steps, 30 FPS sweep speed, 100 dB display range
- **Dual multi-function meters**: Gorgeous analog-style meters
- **Dual spectrum**: Two spectra side-by-side or stacked (for dual VFO operation)

### 2.3 Elecraft K4

- **7-inch color LCD**: Most screen dedicated to panadapter
- **Dual panadapter mode**: VFO A and VFO B side-by-side, maximizing waterfall height
- **Mini-pan**: Narrow-range resampled high-resolution spectrum for fine tuning
- **Bar graph meters**: Between VFO displays and panadapters
- **Three multi-function knobs**: Intuitive controls for panadapter manipulation

### 2.4 FlexRadio 6000 Series

- **Panafall concept**: Joined panadapter + waterfall with shared frequency axis
- **Multiple slices**: Up to 4+ independent receivers visible on one display
- **Floating/dockable panels**: Per-slice panafall and radio panels
- **"+RX" button**: Add new slice receiver to panadapter
- **Off-screen indicator**: Shows direction to slices outside visible range

### Key Hardware UI Patterns

| Feature | IC-7300 | FTDX-101D | K4 | Flex 6000 |
|---------|---------|-----------|-----|-----------|
| Touch-to-tune | Two-tap zoom+tune | Tap to tune | Tap to tune | Click to place slice |
| Spectrum + WF split | Fixed ratio | Fixed ratio | Adjustable | Adjustable |
| Dual VFO display | Stacked | Side-by-side | Side-by-side | Multiple slices |
| Meters | Bar graph | Analog dual | Bar graph | Digital bar |
| Display resolution | 4.3" 480x272 | 7" 800x480 | 7" | PC-driven |
| DSP controls on-screen | Menu-based | Touch + knobs | Touch + knobs | Software |

---

## 3. Feature Comparison Matrix

### 3.1 Software SDR Feature Matrix

| Feature | Wolf | SDR Console | SmartSDR | SDR# | OpenWebRX | SDRAngel | GQRX | MaiaSDR | CubicSDR | Thetis |
|---------|------|-------------|----------|------|-----------|----------|------|---------|----------|--------|
| **Platform** | Embedded | Windows | Windows/iOS | Windows | Web browser | Desktop | Linux | Web browser | Cross-plat | Windows |
| **Spectrum display** | Yes | Yes | Yes | Yes | No (WF only) | Yes | Yes | No | Yes | Yes |
| **Waterfall** | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes (WebGL2) | Yes | Yes |
| **Click-to-tune** | Touch | Yes | Yes (slice) | Yes | Yes | Yes | Yes | Yes | Yes | Yes |
| **Drag-to-pan** | No | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes |
| **Filter overlay** | Yes | Yes | Yes (slice) | Yes | Yes | Yes | Yes | No | Yes (modem) | Yes |
| **Drag filter edges** | No | Yes | Yes | Yes | No | No | No | No | Yes | No |
| **Zoom-to-cursor** | No | Yes | Yes | Yes | Yes | Yes | Yes (3 modes) | No | Yes | Yes |
| **Peak hold** | Yes | Yes | No | Yes | No | No | Yes | No | Yes | Yes |
| **S-meter** | Analog | Digital | Bar | Digital | No | Per-channel | Digital | No | No | Multi-meter |
| **Per-digit tune** | Encoder | No | No | Mouse wheel | No | No | No | No | No | Mouse wheel |
| **Dual VFO** | Yes (A+B) | Yes | Yes (slices) | No | No | Multi-device | No | No | Multi-modem | Yes |
| **AGC** | Yes (K-weight) | Yes | Yes | Yes | Yes (SSB/AM/NFM) | Yes | Yes | No | No | Yes |
| **Noise Reduction** | DNR | Yes | Yes | Yes | Spectral sub. | No | No | No | No | Yes |
| **Noise Blanker** | Yes | Yes | Yes | Yes | No | No | No | No | No | Yes |
| **Notch Filter** | Auto+Manual | Yes | Yes | Yes | No | No | No | No | No | Yes |
| **Squelch** | FM + VAD | Yes | Yes | Yes | Yes | Yes | Yes | No | Yes | Yes |
| **Equalizer** | 7-band RX/TX | No | No | No | No | No | No | No | No | Yes |
| **Colormap editor** | 7 presets | Full editor | Limited | Limited | Gradient | No | No | Shader-based | No | No |
| **Adaptive averaging** | No | Hang&Decay | No | No | No | No | Signal-dependent | No | Yes | No |
| **WebSocket streaming** | N/A | N/A | API | N/A | Yes (binary) | N/A | N/A | Yes | N/A | N/A |
| **Touch support** | Yes (7") | No | Yes (iOS) | No | Yes | No | No | Yes | No | No |
| **FFT compression** | N/A | N/A | N/A | N/A | ADPCM | N/A | N/A | FPGA integ. | N/A | N/A |

---

## 4. Recommended Layout

### 4.1 Screen Layout (Wolf SDR inspired, adapted for browser)

```
+============================================================================+
| HEADER BAR (40px)                                                          |
| [LED] Status | [=== 145.800.000 MHz ===] | Mode: [NFM] | [S-METER====] | |
+============================================================================+
|                                                                            |
|   SPECTRUM DISPLAY (~25% height, min 120px)                                |
|   ┌────────────────────────────────────────────────────────────────────┐   |
|   │  dB  ╲                                                             │   |
|   │ -20  │ ╲     ╱╲        ╱╲╲                                        │   |
|   │ -40  │  ╲___╱  ╲    __╱  ╲╲___      ╱╲                           │   |
|   │ -60  │         ╲╲__╱         ╲╲____╱  ╲_________                 │   |
|   │ -80  │                                            ╲__________    │   |
|   │      │  ┊    filter   ┊                                          │   |
|   │      │  ┊◄──overlay──►┊     ← passband with yellow edges         │   |
|   │      │  ┊  red center ┊                                          │   |
|   │      ├──┴─────────────┴──────────────────────────────────────────│   |
|   │      │ 145.700  145.750  145.800  145.850  145.900  MHz         │   |
|   └────────────────────────────────────────────────────────────────────┘   |
|   ═══════════════ RESIZE HANDLE (4px, draggable) ═══════════════════════   |
|   ┌────────────────────────────────────────────────────────────────────┐   |
|   │                                                                    │   |
|   │   WATERFALL DISPLAY (flex: fills remaining, min 150px)             │   |
|   │   ████████████████████████████████████████████████████████████████ │   |
|   │   ████████▓▓▓▓▓████████████▓▓████████████████████████████████████ │   |
|   │   ██████▓▓░░░░▓▓██████████▓▓▓▓██████████████████████████████████ │   |
|   │   █████▓░░░░░░░▓███████████▓▓███████████████████████████████████ │   |
|   │   ██████▓▓░░░░▓▓█████████████████████████████████████████████████ │   |
|   │   ████████▓▓▓▓▓████████████████████████████████████████████████   │   |
|   │                                                                    │   |
|   └────────────────────────────────────────────────────────────────────┘   |
+============================================================================+
| CONTROL BAR (compact, ~48px, wraps on mobile)                              |
| [Mode] [Tune±] [Gain] [Vol] [Sql] [Rng/Ref] [Zoom] [Rate] [FFT] [WF]    |
+============================================================================+
```

### 4.2 Proportions (Desktop 1920x1080)

| Region | Height | Percentage | Min Height |
|--------|--------|------------|------------|
| Header bar | 40px | ~4% | 36px |
| Spectrum | ~240px | ~22% | 120px |
| Resize handle | 4px | <1% | 4px |
| Waterfall | ~748px | ~69% | 150px |
| Control bar | 48px | ~4% | 40px |

### 4.3 Proportions (Mobile 375x667)

| Region | Height | Percentage |
|--------|--------|------------|
| Header bar (compact) | 36px | ~5% |
| Spectrum | ~100px | ~15% |
| Resize handle | 4px | <1% |
| Waterfall | ~479px | ~72% |
| Control bar (wrapping) | 48px | ~7% |

### 4.4 Layout Principles

1. **Spectrum + Waterfall dominate**: Combined they should always be >85% of viewport
2. **Header is information-dense**: Frequency, mode, S-meter, connection status in one row
3. **Controls are compact**: Single row of grouped controls, with separators between groups
4. **Resize handle between spectrum/waterfall**: Draggable, with double-click to reset to default split
5. **All controls reachable by touch**: Minimum 36px touch targets
6. **No sidebars on mobile**: Everything flows top-to-bottom

---

## 5. Color Palette

### 5.1 Dark Theme (Primary)

Based on analysis of Wolf SDR (dark theme), SDR# (dark mode), GQRX, and hardware rig displays:

```
/* ── Backgrounds ── */
--bg-primary:        #0a0a0a;    /* Main background (near-black, not pure black) */
--bg-secondary:      #111111;    /* Header/control bar background */
--bg-surface:        #1a1a1a;    /* Button/input surface */
--bg-elevated:       #222222;    /* Hover states, dropdowns */
--bg-spectrum:       #0d0d0d;    /* Spectrum plot background */

/* ── Grid/Graticule ── */
--grid-line:         #1e1e1e;    /* Major grid lines */
--grid-line-minor:   #151515;    /* Minor grid lines */
--grid-text:         #446644;    /* Grid labels (frequency, dB) */

/* ── Accent Colors (Green Phosphor Theme) ── */
--accent-primary:    #00ff41;    /* Primary accent (green phosphor) */
--accent-dim:        #00cc33;    /* Dimmer variant for sliders, secondary elements */
--accent-dark:       #003300;    /* Active button background */
--accent-glow:       #00ff4160;  /* Box-shadow glow (40% opacity) */

/* ── Spectrum Trace Colors ── */
--trace-primary:     #00ff41;    /* Main spectrum trace (classic green phosphor) */
--trace-peak:        rgba(255, 60, 60, 0.4);  /* Peak hold trace (dim red) */
--trace-average:     rgba(0, 180, 255, 0.3);  /* Averaging trace (dim cyan) */
--trace-fill:        rgba(0, 255, 65, 0.08);  /* Fill under trace (very dim green) */

/* ── Filter Overlay ── */
--filter-fill:       rgba(255, 255, 255, 0.10);  /* Filter passband fill */
--filter-edge:       rgba(255, 220, 0, 0.75);    /* Filter edge lines (yellow) */
--filter-center:     #ff3333;                      /* Center frequency line (red) */
--filter-bw-text:    rgba(255, 220, 0, 0.85);    /* Bandwidth label */

/* ── S-Meter ── */
--smeter-low:        #00aa00;    /* S0-S5 (green) */
--smeter-mid:        #aaaa00;    /* S6-S9 (yellow) */
--smeter-high:       #ff4400;    /* S9+ (red-orange) */
--smeter-needle:     #ffffff;    /* Needle color */
--smeter-face:       #0a0a0a;   /* Meter face background */
--smeter-scale:      #888888;   /* Scale markings */

/* ── Text ── */
--text-primary:      #cccccc;    /* Primary text */
--text-secondary:    #888888;    /* Secondary/status text */
--text-dim:          #556655;    /* Labels, units */
--text-freq:         #00ff41;    /* Frequency display (green phosphor) */
--text-freq-sep:     #336633;    /* Frequency digit separators */
--text-value:        #00cc33;    /* Control values */

/* ── Interactive ── */
--border-default:    #2a2a2a;    /* Default borders */
--border-hover:      #556655;    /* Hover borders */
--border-active:     #00ff41;    /* Active/selected borders */
--border-header:     #00ff41;    /* Header bottom accent line */

/* ── Status Indicators ── */
--status-connected:  #00ff41;    /* Connected LED */
--status-error:      #ff3333;    /* Error/disconnected LED */
--status-warning:    #ffaa00;    /* Warning state */
```

### 5.2 Waterfall Colormaps

Each colormap is a 256-entry array mapping FFT power (0-255) to RGB.

**Colormap 0 -- "Jet" (default, matches current)**:
```
0-63:   Black -> Blue     (#000000 -> #0000FF)
64-127: Blue -> Cyan -> Green (#0000FF -> #00FF00)
128-191: Green -> Yellow   (#00FF00 -> #FFFF00)
192-255: Yellow -> Red     (#FFFF00 -> #FF0000)
```

**Colormap 1 -- "Iron" (thermal)**:
```
0-63:   Black -> Dark Purple  (#000000 -> #300040)
64-127: Purple -> Red         (#300040 -> #FF0000)
128-191: Red -> Orange/Yellow (#FF0000 -> #FFD000)
192-255: Yellow -> White      (#FFD000 -> #FFFFFF)
```

**Colormap 2 -- "Viridis" (perceptually uniform, colorblind-safe)**:
```
0:   #440154  (dark violet)
64:  #31688e  (teal-blue)
128: #35b779  (green)
192: #90d743  (yellow-green)
255: #fde725  (yellow)
```

**Colormap 3 -- "Green Phosphor" (classic SDR)**:
```
0-127:   Black -> Dark Green  (#000000 -> #003300)
128-200: Dark Green -> Green  (#003300 -> #00FF41)
200-255: Green -> Bright Green/White (#00FF41 -> #AAFFAA)
```

**Colormap 4 -- "Grayscale"**:
```
0-255: Black -> White (#000000 -> #FFFFFF)
```

---

## 6. Component Specifications

### 6.1 Frequency Display

**Requirements**: Large, readable, per-digit tuning via mouse wheel/touch scroll

**Design**:
```
┌──────────────────────────────────────────┐
│  1 4 5 . 8 0 0 . 0 0 0  MHz            │
│  ▲                                       │
│  Each digit is a separate <span>         │
│  with data-step attribute               │
│  Mouse wheel on digit increments/        │
│  decrements by that digit's place value  │
└──────────────────────────────────────────┘
```

**Specifications**:
- Font: DSEG7-Classic (SIL Open Font License) -- true 7-segment LED aesthetic; fallback to system monospace if DSEG fails to load
- Font size: 28px desktop, 20px tablet, 16px mobile
- Digit color: `--text-freq` (#00ff41)
- Separator color: `--text-freq-sep` (#336633)
- Unit label: `--text-dim`, 13px
- Hover effect: Background highlight `#1a3a1a`, brighter digit color `#80ff80`
- Digit grouping: `GGG.MMM.KKK.HHH` with period separators (groups of 3)
- Touch: Swipe up/down on digit to increment/decrement (with momentum)
- Active digit highlight: Underline or subtle glow on the digit being changed
- Step indication: Tiny step label below frequency (e.g., "1 kHz step")

**Per-Digit Tuning Interaction**:
1. Hover over any digit -- digit highlights
2. Scroll wheel up -- increment that digit's place value
3. Scroll wheel down -- decrement that digit's place value
4. Carry/borrow propagates naturally (e.g., incrementing 9 in kHz rolls to next MHz)
5. Clamp to valid RTL-SDR range: 24 MHz - 1766 MHz

### 6.2 S-Meter

**Design Choice**: SVG-based analog needle meter (mimics real ham radio S-meters)

```
┌─────────────────────────────────────┐
│         S1 3 5 7 9 +20 +40 +60     │  ← Scale arc
│        ╱                    ╲       │
│       ╱     ╱╱╱╱╱╱╱         ╲      │  ← Colored arc segments
│      ╱     ╱ needle ╱         ╲     │
│     ╱     ╱╱╱╱╱╱╱╱╱╱          ╲    │
│    ╱         ↑                  ╲   │
│   ╱      pivot point             ╲  │
│  ╱───────────●───────────────────╲  │
│              ▲                      │
│         Peak hold marker            │
└─────────────────────────────────────┘
```

**Specifications**:
- Rendering: Inline SVG for resolution independence
- Size: 160x50px desktop (compact horizontal meter), scales proportionally
- Scale: S0-S1-S3-S5-S7-S9 (green arc), S9+10/+20/+30/+40/+50/+60 (red arc)
- Needle: CSS `transform: rotate()` with `transition: transform 80ms ease-out` for smooth movement
- Peak hold: Small marker (tick or dot) at the highest recent reading, decays after 2 seconds
- Signal text: Digital readout below meter (e.g., "S7" or "S9+12")
- dBm readout: Optional small text showing actual dBm value

**S-Unit to dBm Mapping** (at 50 ohms):
```
S0  = < -127 dBm
S1  = -121 dBm
S2  = -115 dBm
S3  = -109 dBm
S4  = -103 dBm
S5  = -97 dBm
S6  = -91 dBm
S7  = -85 dBm
S8  = -79 dBm
S9  = -73 dBm
S9+10 = -63 dBm
S9+20 = -53 dBm
S9+40 = -33 dBm
S9+60 = -13 dBm
```

**Alternative (bar graph)**: For mobile/compact layouts, fall back to a horizontal bar:
```
S  [▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░░░░░░░░] S7 -85dBm
   ←── green ──→←yel→←─ red ─→
```

### 6.3 Spectrum Display

**Rendering**: HTML5 Canvas 2D (spectrum is redrawn every frame, Canvas is efficient for this)

**Features**:
- **Trace**: 1.5px line, `--trace-primary` color, anti-aliased
- **Peak hold**: 1px line, `--trace-peak` color, decays at configurable rate (default: 0.5 dB/frame)
- **Fill under trace**: Optional gradient fill from trace to bottom (very low opacity)
- **Grid**: Horizontal dB lines every 10 dB, vertical frequency lines every ~20% of visible span
- **Labels**: Frequency labels at bottom, dB labels at left margin
- **Noise floor line**: Horizontal dashed line at estimated noise floor (adaptive tracking)
- **Filter overlay**: Semi-transparent rectangle showing demodulator passband with:
  - Yellow edge lines (draggable to resize bandwidth)
  - Red center frequency line
  - Bandwidth text label ("12.5 kHz BW")
- **Click-to-tune**: Click anywhere on spectrum to move demodulator center frequency
- **Drag-to-pan**: Drag horizontally to shift center frequency
- **Cursor readout**: Tooltip showing frequency + power at mouse position
- **Zoom-to-cursor**: Mouse wheel zooms in/out centered on cursor position

**Averaging Modes**:
1. **None**: Raw FFT output
2. **Moving average**: Weighted combination of current and previous frame (configurable alpha 0.1-0.9)
3. **Adaptive** (GQRX-style): Averaging factor depends on signal level; noise floor smoothed more than peaks
4. **Peak hold with decay**: Maximum value held, decays linearly at configurable rate

### 6.4 Waterfall Display

**Design Decision**: Canvas 2D with ImageData scrolling (current approach) for P0, with WebGL2 upgrade path for P2.

**Current approach (Canvas 2D)**:
- `ImageData` buffer, scroll via `data.copyWithin()` (shift rows down by one row stride)
- Write new row at top from FFT data mapped through colormap
- Filter overlay drawn on top

**WebGL2 upgrade path (MaiaSDR approach)**:
- FFT data written to 2D texture via `texSubImage2D()` (single row update)
- Circular buffer index tracks current write row
- Fragment shader maps dB values through 1D colormap texture
- Scrolling via vertex offset (no pixel copying)
- Benefits: GPU-accelerated, handles high FFT rates without frame drops

**Waterfall Features**:
- **Speed control**: Minimum interval between waterfall row updates (5-100ms)
- **Colormap selector**: Dropdown to choose from predefined colormaps (see Section 5.2)
- **Time markers**: Optional timestamp annotations on waterfall edge (every N seconds)
- **Cursor readout**: Frequency + power at mouse position (shared with spectrum)
- **Auto-scroll**: New data always appears at top
- **Touch drag**: Mid-waterfall region drag shifts visible frequency band

### 6.5 Resize Handle (Spectrum/Waterfall Split)

**Specifications**:
- Height: 4px, full width
- Color: `--border-default` (#2a2a2a), brightens to `--border-hover` on hover
- Cursor: `ns-resize` on hover
- Behavior: Drag up/down to change spectrum/waterfall height ratio
- Constraints: Spectrum minimum 80px, waterfall minimum 120px
- Double-click: Reset to default 25%/75% split
- Touch: Wider hit area (16px) for touch devices

### 6.6 Control Bar

**Layout**: Horizontal flex container with control groups separated by vertical dividers

**Control Groups (left to right)**:

1. **Mode buttons**: `[AM] [NFM] [WBFM] [USB] [LSB] [CW]`
   - Toggle buttons, only one active at a time
   - Active: green border + background
   - Switching mode automatically sets default filter bandwidth

2. **Tune offset**: Slider + value display
   - Range: +/- half sample rate
   - Fine step mode: hold Shift for 10 Hz steps

3. **Gain**: Slider + value display
   - Range: Auto (0) to max tuner gain
   - "Auto" label when at 0
   - Shows dB value when manual

4. **Volume**: Slider + percentage display

5. **Squelch**: Slider + value display
   - Visual feedback: squelch threshold line on spectrum

6. **Dynamic Range / Reference Level**: Combined control (SDR Console-inspired)
   - Range slider: dB span of spectrum display (10-120 dB)
   - Reference slider: top-of-display dB level (-80 to +40 dBm)
   - Alternative: Single "Auto" button that tracks noise floor and strongest signal

7. **Zoom**: `[-] [value] [+] [1:1]` button group
   - Values: 1x, 2x, 4x, 8x
   - 1:1 resets to full span

8. **Rate / FFT**: Dropdowns for sample rate and FFT size

9. **Waterfall speed**: Slider

10. **Utility buttons**: `[Peak Clear] [Audio On/Off]`

### 6.7 Header Bar

**Layout**: Single row, horizontal flex, 40px height

```
[LED] Connected | [====== 145.800.000 MHz ======] | NFM | 1024 kSPS | [S-METER] | 12.5k BW | [Audio]
```

**Elements (left to right)**:
1. **Status LED**: 8px circle, green (connected) or red (disconnected) with glow
2. **Status text**: "Connected" / "Disconnected" / "Reconnecting..."
3. **Frequency display**: Central, largest element (see 6.1)
4. **Mode indicator**: Current demodulation mode badge
5. **Sample rate**: e.g., "1024 kSPS"
6. **S-Meter**: Inline analog or bar meter (see 6.2)
7. **Bandwidth**: Current filter bandwidth ("12.5k BW")
8. **Audio button**: Toggle audio on/off with active state indicator

---

## 7. DSP Feature List

### 7.1 Server-Side DSP (ESP32-P4, C code)

These run on the ESP32-P4's dual-core RISC-V at 400 MHz with SIMD (PIE instructions).

| Feature | Status | Implementation Notes |
|---------|--------|---------------------|
| FFT (256-16384 bins) | **Exists** | `dsp_fft_compute()` with Blackman-Harris window |
| DDC (Digital Down Converter) | **Exists** | `dsp_ddc_create/process()` with CIC + FIR decimation |
| dB range mapping | **Exists** | `dsp_fft_set_range()` |
| FFT averaging (exponential) | **To add** | Alpha-weighted running average before dB conversion |
| Noise floor tracking | **To add** | Median of lower 25% of FFT bins, updated per frame |
| AGC (server-side) | **To add** | Fast/medium/slow attack/decay on DDC output |
| Noise Blanker | **To add** | Pulse detection + interpolation in time domain, before FFT |
| IF shift | **To add** | Adjust DDC center offset independently of display center |

### 7.2 Client-Side DSP (Browser, JavaScript)

These run in the browser using Web Audio API and JavaScript.

| Feature | Status | Implementation Notes |
|---------|--------|---------------------|
| FM demodulation | **Exists** | Atan2-based discriminator in `dm()` function |
| AM demodulation | **Exists** | Envelope detection (sqrt(I^2+Q^2)) with DC removal |
| SSB demodulation (USB/LSB) | **Exists** | Phasing method (I+Q or I-Q) |
| Audio resampling | **Exists** | Linear interpolation from DDC rate to 48 kHz |
| Volume control | **Exists** | Web Audio GainNode |
| Squelch (power-based) | **Exists** | RMS power threshold in ScriptProcessor |
| Noise Reduction (spectral sub.) | **To add** | FFT of audio, estimate+subtract noise floor, IFFT |
| Audio AGC | **To add** | Peak tracking with attack/decay envelope follower |
| Notch filter (manual) | **To add** | Biquad IIR notch at user-specified frequency |
| Auto-notch | **To add** | Detect strongest tone in audio spectrum, apply notch |
| De-emphasis (FM) | **To add** | Single-pole IIR filter: 50us or 75us time constant |
| Audio equalizer | **To add** | Chain of BiquadFilterNodes (bass/mid/treble) |
| CW filter (narrow) | **To add** | Steep bandpass using cascaded biquads |

### 7.3 DSP Priority and Effort

| Feature | Priority | Effort | Impact |
|---------|----------|--------|--------|
| FM de-emphasis | P0 | Low (5 lines) | High -- WBFM sounds much better |
| Audio AGC | P0 | Medium | High -- prevents volume jumps |
| FFT averaging | P0 | Low | Medium -- cleaner spectrum display |
| Noise floor tracking | P1 | Medium | Medium -- enables auto-scaling |
| Spectral noise reduction | P1 | High | High -- usability for weak signals |
| Noise blanker | P1 | High | Medium -- helps near power lines |
| Manual notch | P2 | Medium | Medium -- removes interference |
| Auto-notch | P2 | High | Medium -- convenience feature |
| Audio equalizer | P2 | Low | Low -- nice to have |
| CW narrow filter | P3 | Medium | Low -- CW is niche for RTL-SDR |

---

## 8. Implementation Priority

### P0 -- Must Have (Core WebSDR Experience)

These features are required for the UI to be considered a functional SDR interface.

| Feature | Component | Effort |
|---------|-----------|--------|
| Dark theme with green phosphor palette | CSS | Low |
| Large frequency display with per-digit tuning | HTML+JS | Medium |
| Spectrum display with trace + grid + labels | Canvas | Medium (refactor existing) |
| Waterfall with selectable colormaps | Canvas | Medium (refactor existing) |
| Filter passband overlay with draggable edges | Canvas+JS | Medium |
| Click-to-tune on spectrum/waterfall | JS (exists, refine) | Low |
| Drag-to-pan frequency | JS (exists, refine) | Low |
| Zoom-to-cursor (mouse wheel) | JS (exists, refine) | Low |
| S-meter (bar graph minimum, SVG analog stretch) | SVG or CSS | Medium |
| Mode selection buttons | HTML (exists, refine) | Low |
| Gain/Volume/Squelch sliders | HTML (exists) | Low |
| Dynamic range + reference level sliders | HTML (exists, refine) | Low |
| Audio toggle with Web Audio demodulation | JS (exists) | Low |
| Spectrum/waterfall resize handle | JS+CSS | Low |
| Responsive layout (mobile-friendly) | CSS | Medium |
| FM de-emphasis (50/75us) | JS (client DSP) | Low |
| Audio AGC | JS (client DSP) | Medium |
| FFT averaging (moving average) | JS (client) or C (server) | Low |
| Peak hold with decay | JS (refactor existing) | Low |

### P1 -- Should Have (Professional Quality)

| Feature | Component | Effort |
|---------|-----------|--------|
| DSEG7 font for frequency display | CSS font-face | Low |
| SVG analog needle S-meter | SVG+CSS | Medium |
| Noise floor tracking + auto-scale | JS + server | Medium |
| Spectral noise reduction | JS (Web Audio) | High |
| Noise blanker (server-side) | C (ESP32) | High |
| Cursor readout (frequency + power) | JS overlay | Low |
| Waterfall time markers | Canvas | Low |
| Squelch threshold line on spectrum | Canvas | Low |
| Keyboard shortcuts | JS | Low |
| Touch gesture refinements (swipe, pinch) | JS | Medium |
| Band quick-select buttons | HTML+JS | Low |
| Memory channels (localStorage) | JS | Medium |
| Settings persistence (localStorage) | JS | Low |

### P2 -- Nice to Have (Advanced Features)

| Feature | Component | Effort |
|---------|-----------|--------|
| WebGL2 waterfall rendering | WebGL2 | High |
| Manual notch filter | JS biquad | Medium |
| Auto-notch filter | JS (FFT peak detect) | Medium |
| Audio equalizer (bass/treble) | Web Audio BiquadFilter | Low |
| Colormap editor (custom gradients) | HTML+Canvas | Medium |
| Frequency annotations / band markers | Canvas overlay | Medium |
| Context menu (right-click on spectrum) | HTML+JS | Medium |
| Multi-demodulator (multiple filter overlays) | JS+server DDC | High |
| 3D spectrum mode | WebGL | High |
| Server-side AGC | C (ESP32) | Medium |
| Adaptive FFT averaging (GQRX-style) | C or JS | Medium |
| Waterfall ADPCM compression | C (server) + JS (client) | Medium |

### P3 -- Aspirational (Future Enhancements)

| Feature | Component | Effort |
|---------|-----------|--------|
| Dual VFO (A/B with split) | JS+server | High |
| Digital mode decoders (FT8/RTTY) | JS or WASM | Very High |
| RDS decoder for WBFM | JS | High |
| Band stacking registers | JS (localStorage) | Medium |
| DX cluster overlay on spectrum | JS+WebSocket | High |
| Recording (IQ or audio to file) | JS (MediaRecorder) | Medium |
| Preset management (import/export) | JS (JSON) | Low |
| PWA offline support | ServiceWorker | Medium |
| WASM-based DSP modules | Rust->WASM | Very High |

---

## 9. Technology Choices

### 9.1 Rendering Technology per Component

| Component | Recommended | Rationale | Alternative |
|-----------|-------------|-----------|-------------|
| **Spectrum trace** | Canvas 2D | Redrawn every frame; Canvas is optimal for per-frame line drawing | WebGL (overkill for line plot) |
| **Waterfall (P0)** | Canvas 2D + ImageData | Already implemented, adequate for 20-40 FPS at 1024 bins | -- |
| **Waterfall (P2)** | WebGL2 + textures | GPU-accelerated scrolling, no pixel copy, handles 60+ FPS | Canvas with OffscreenCanvas |
| **S-meter needle** | Inline SVG + CSS transform | Resolution-independent, smooth animation via CSS transitions | Canvas (loses SVG scalability) |
| **S-meter bar** | CSS (div with gradient) | Simplest possible, works everywhere | SVG |
| **Frequency display** | HTML spans + DSEG7 font | Per-digit interaction via event handlers, CSS font | Canvas (loses text selection) |
| **Filter overlay** | Canvas 2D (drawn on spectrum/waterfall) | Needs to be part of spectrum render pipeline | SVG overlay (z-index issues) |
| **Grid/labels** | Canvas 2D | Drawn as part of spectrum rendering pass | SVG overlay |
| **Resize handle** | HTML div + JS drag | Standard DOM interaction pattern | -- |
| **Control bar** | HTML + CSS flexbox | Standard form controls, accessible | -- |
| **Cursor readout** | HTML tooltip div | Positioned absolutely, follows mouse | Canvas text |

### 9.2 Audio Pipeline

```
WebSocket (binary, type=0x02)
    → IQ packet queue (array of Uint8Array)
    → ScriptProcessorNode.onaudioprocess (or AudioWorkletProcessor for P2)
        → Demodulate (FM/AM/SSB)
        → [De-emphasis filter]        ← P0
        → [Noise reduction]           ← P1
        → [Notch filter]              ← P2
        → [AGC envelope follower]     ← P0
        → [Equalizer chain]           ← P2
        → Resample to 48 kHz
        → Squelch gate
        → GainNode (volume)
        → AudioDestination (speakers)
```

**P0**: ScriptProcessorNode (deprecated but universally supported, already implemented)
**P2 upgrade**: AudioWorkletProcessor (modern, runs in separate thread, lower latency)

### 9.3 WebSocket Protocol

Current protocol (binary messages with type byte prefix) is sound:
- `0x01` + FFT data (uint8 array, length = fft_size)
- `0x02` + IQ data (uint8 interleaved I/Q)
- JSON text messages for commands and status

**Proposed additions for P1**:
- `0x03` + S-meter data (single byte: 0-255 signal strength, sent at ~10 Hz)
- `0x04` + Noise floor estimate (single byte, sent at ~2 Hz)

This avoids computing S-meter values from FFT data in the client, which is currently approximate (averaging center bins).

### 9.4 Font Delivery

Since this runs on an ESP32 with limited flash, fonts must be compact:

| Font | Size (woff2) | Purpose | Decision |
|------|-------------|---------|----------|
| DSEG7-Classic-Regular | ~12 KB | Frequency display | Include (embedded in HTML as base64 or served as /dseg7.woff2) |
| System monospace | 0 KB | All other text | Use `'Courier New', 'Consolas', monospace` |

The DSEG7 font adds 12 KB to the firmware. This is acceptable given total web assets are currently ~15 KB (HTML + CSS + JS). Alternative: use CSS-only 7-segment rendering (0 KB but less authentic).

---

## 10. Design Alternatives

### 10.1 S-Meter: Analog Needle vs. Bar Graph vs. Digital

| Option | Pros | Cons |
|--------|------|------|
| **A: SVG Analog Needle** | Most visually appealing; matches ham radio tradition; smooth animation via CSS; resolution-independent | More complex to implement (~80 lines SVG + JS); takes more header space (~160px wide) |
| **B: CSS Bar Graph** (current) | Simplest implementation (exists already); compact width (~80px); works on all browsers | Less visually distinctive; no peak hold; doesn't "feel" like a real radio |
| **C: Digital Readout Only** | Most compact (40px); shows exact dBm value; works on tiny screens | No visual impact; requires reading numbers instead of glancing |

**Recommendation**: Implement B (bar) as P0 baseline, A (SVG needle) as P1 upgrade. Show both on desktop (needle in header, bar as fallback on mobile).

### 10.2 Waterfall Rendering: Canvas 2D vs. WebGL2

| Option | Pros | Cons |
|--------|------|------|
| **A: Canvas 2D + ImageData** (current) | Already implemented; simple code; universal browser support; adequate for 20 FPS / 1024 bins | CPU-bound pixel copying (copyWithin); scales poorly above 2048 bins or 40 FPS; mobile battery impact |
| **B: WebGL2 + Texture Scrolling** (MaiaSDR approach) | GPU-accelerated; zero-copy scrolling; handles 60+ FPS; colormap changes are instant (shader recompile); mobile-friendly (GPU is fast) | Complex implementation (~200 lines shader + JS); WebGL2 required (97% browser support); debugging is harder |
| **C: OffscreenCanvas + Worker** | Rendering in worker thread frees main thread; still Canvas 2D but decoupled | Browser support for OffscreenCanvas is ~93%; still CPU-bound; more complex message passing |

**Recommendation**: Keep A for P0 (it works). Implement B as P2 upgrade when higher FFT sizes or frame rates are needed. Skip C (middle ground with limited benefit).

### 10.3 Audio Pipeline: ScriptProcessorNode vs. AudioWorklet

| Option | Pros | Cons |
|--------|------|------|
| **A: ScriptProcessorNode** (current) | Already implemented; simple callback API; universal support | Deprecated; runs on main thread (can cause UI jank); fixed buffer sizes |
| **B: AudioWorkletProcessor** | Modern API; runs in dedicated audio thread; lower latency; better performance | Requires separate JS file for worklet; more complex message passing; Chrome 66+, Firefox 76+, Safari 14.1+ |

**Recommendation**: Keep A for P0. Migrate to B as P2 when adding complex DSP chain (NR + notch + EQ). The performance improvement matters most when multiple DSP stages are chained.

### 10.4 Frequency Display Font: DSEG7 vs. CSS-Only vs. System Monospace

| Option | Pros | Cons |
|--------|------|------|
| **A: DSEG7 woff2 font** | Authentic 7-segment LED look; ~12 KB; proper font metrics | Extra 12 KB in firmware; font loading delay on first visit |
| **B: CSS-only 7-segment** | Zero extra bytes; pure CSS borders to form segments | Complex CSS (~100 lines); fragile; no proper font kerning |
| **C: System monospace** (current) | Zero cost; already works; readable | Looks generic; not "radio-like" |

**Recommendation**: A (DSEG7) for P1. It transforms the visual identity for minimal cost. Keep C as fallback in @font-face declaration.

### 10.5 Layout: Fixed Header vs. Overlay Controls

| Option | Pros | Cons |
|--------|------|------|
| **A: Fixed header + fixed control bar** (Wolf-style, current) | Always visible; predictable; touch-friendly | Consumes ~90px of vertical space permanently |
| **B: Auto-hiding header/controls** | Maximizes spectrum/waterfall area; cleaner look | Requires hover/tap to reveal; slower access to controls; confusing on touch |
| **C: Overlay controls on spectrum** (SDR Console-style) | Zero space overhead; appears on hover | Obscures spectrum data; hard to use on touch; complex implementation |

**Recommendation**: A for all screen sizes. The 90px overhead is acceptable given the information density achieved. On mobile (<480px), compress controls further with icon-only buttons and collapsible groups.

---

## 11. Constraints and Trade-offs

### 11.1 ESP32-P4 Hardware Constraints

| Constraint | Impact on UI |
|-----------|-------------|
| Flash size (limited) | Total web assets (HTML+CSS+JS+fonts) must stay under ~100 KB compressed. Currently ~15 KB. Budget for rich UI: ~80 KB remaining. |
| PSRAM bandwidth | FFT computation and IQ streaming compete for memory bandwidth. Server-side DSP additions must be profiled. |
| WiFi throughput (~4-5 MB/s) | FFT data at 1024 bins * 20 FPS = ~20 KB/s (trivial). IQ streaming for audio is ~50-200 KB/s. Plenty of headroom. |
| Max 3 WebSocket clients | UI must work well for 1-3 simultaneous users. No client-heavy computation that would multiply server load. |
| CPU (dual RISC-V 400 MHz) | Complex server-side DSP (noise blanker, AGC) must be profiled. FFT + DDC already consume significant cycles at 1 MSPS. |

### 11.2 Browser Constraints

| Constraint | Impact |
|-----------|--------|
| Mobile battery | WebGL2 waterfall is GPU-efficient but still consumes power. Provide frame rate limiter. |
| Touch targets | Minimum 36px for all interactive elements. Slider thumbs need 44px touch area. |
| Audio autoplay policy | Audio cannot start without user gesture. "Audio" button click is the required gesture (already handled). |
| WebSocket reconnection | Must gracefully reconnect on WiFi drops. Current 2-second retry is good. |
| Single-threaded JS | Heavy client-side DSP (noise reduction FFT) can block UI. AudioWorklet migration solves this. |

### 11.3 Key Trade-off Tensions

| Tension | Resolution |
|---------|-----------|
| **Visual richness vs. firmware size** | DSEG7 font (12 KB) + richer CSS is worth the cost. Stay under 100 KB total. Gzip compression on ESP32 HTTP server would help (~60% reduction). |
| **Client DSP vs. server DSP** | Demodulation stays client-side (per-client mode independence). Noise blanker goes server-side (operates on raw IQ). AGC can be either; client-side is more flexible. |
| **Canvas 2D simplicity vs. WebGL2 performance** | Start with Canvas 2D. WebGL2 upgrade is isolated to waterfall component only -- can be swapped without changing data flow. |
| **Feature completeness vs. code size** | JavaScript must stay compact (~15-20 KB uncompressed for P0). Use IIFE pattern (current), avoid frameworks. Each P1/P2 feature adds ~1-3 KB. |
| **Minified JS vs. readability** | Ship minified (saves flash + bandwidth). Source stays readable in repo. Build step optional -- manual minification is fine for this scale. |

---

## Appendix A: Existing UI Baseline

The current implementation (`components/websdr/www/`) provides:

**What exists and works well**:
- WebSocket connection with auto-reconnect (sdr.js:72-77)
- Binary message parsing with type byte (sdr.js:87-90)
- FFT spectrum rendering with green trace on dark background (sdr.js:108-124)
- Waterfall with jet colormap (sdr.js:125-131)
- Per-digit frequency display with mouse wheel tuning (sdr.js:52-70)
- Filter passband overlay with center+edge lines (sdr.js:96-107)
- Click-to-tune on spectrum/waterfall (sdr.js:196-199)
- Drag-to-pan frequency (sdr.js:186-189)
- Drag filter edges to resize bandwidth (sdr.js:174-175, 190-192)
- Zoom-to-cursor with mouse wheel (sdr.js:205-208)
- FM/AM/SSB demodulation (sdr.js:137-140)
- Audio via ScriptProcessorNode with resampling (sdr.js:141-168)
- Peak hold (sdr.js:113-116)
- Responsive layout with mobile breakpoints (sdr.css:41-42)

**What needs improvement**:
- S-meter is a simple text + tiny bar (sdr.css:15-17) -- needs analog meter or richer bar
- Frequency display uses system font, not 7-segment (sdr.css:9)
- No spectrum averaging -- trace is noisy
- No noise reduction or audio AGC -- weak signals are hard to hear
- No FM de-emphasis -- WBFM audio sounds harsh
- No resize handle between spectrum and waterfall
- Controls are cramped on mobile
- No keyboard shortcuts
- No settings persistence
- Colormap is hardcoded (jet only)
- No cursor frequency/power readout
- No squelch threshold visualization on spectrum

---

## Appendix B: Reference Sources

### Software SDR
- [Wolf SDR (UA3REO-DDC-Transceiver)](https://github.com/XGudron/UA3REO-DDC-Transceiver)
- [SDR Console V3](https://www.sdr-radio.com/console)
- [FlexRadio SmartSDR](https://www.flexradio.com/)
- [SDR# (SDRSharp) Users Guide](https://www.rtl-sdr.com/sdrsharp-users-guide/)
- [OpenWebRX](https://www.openwebrx.de/)
- [OpenWebRX+](https://fms.komkon.org/OWRX/)
- [SDRAngel Wiki](https://github.com/f4exb/sdrangel/wiki/Quick-start)
- [GQRX SDR](https://www.gqrx.dk/)
- [MaiaSDR](https://maia-sdr.org/)
- [CubicSDR Documentation](https://cubicsdr.readthedocs.io/en/latest/application-window.html)
- [Thetis (PowerSDR fork)](https://github.com/ramdor/Thetis)

### Hardware Rig UI
- [ICOM IC-7300 Spectrum Scope](https://k0pir.us/icom-7300-spectrum-scope-settings-video/)
- [ICOM IC-7610 Display Variations](https://k0pir.us/icom-7610-spectrum-scope-display-variations/)
- [Yaesu FTDX-101D](https://yaesu.com/indexVS.cfm?cmd=DisplayProducts&ProdCatID=102&encProdID=959169DE998192AB87295E90077D740D)
- [Elecraft K4 Transceiver](https://elecraft.com/products/k4-transceiver)
- [FlexRadio FLEX-6000 User Manual](https://www.manualslib.com/manual/1945780/Flexradio-Systems-Flex-6000-Signature-Series.html)

### Fonts and Visual Resources
- [DSEG 7-Segment Font](https://www.keshikan.net/fonts-e.html) (SIL Open Font License)
- [SVG Gauge Library](https://github.com/naikus/svg-gauge)
- [Viridis Colormap](https://cran.r-project.org/web/packages/viridis/vignettes/intro-to-viridis.html)

### Technical Implementation
- [OpenWebRX Waterfall Implementation (DeepWiki)](https://deepwiki.com/jketterl/openwebrx/2.2-waterfall-display)
- [MaiaSDR FOSDEM 2024 Slides](https://archive.fosdem.org/2024/events/attachments/fosdem-2024-1841-maia-sdr-an-open-source-fpga-based-project-for-ad936x-zynq-radios/slides/22686/slides_destevez_fosdem2024_ZzM6DmF.pdf)
- [Canvas Waterfall Plot](https://github.com/jledet/waterfall)
- [Web Audio API (MDN)](https://developer.mozilla.org/en-US/docs/Web/API/Web_Audio_API)
- [UHSDR Noise Reduction Wiki](https://github.com/df8oe/UHSDR/wiki/Noise-reduction)
