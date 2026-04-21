# BrowSDR-inspired UI port (feature/webgl)

Clean-room port. Uses BrowSDR's README/live UX as spec; no source copied.

## Scope (decided)

- Path A: server-side DSP on ESP32-P4 stays authoritative.
- Vue 3 + TypeScript + Vite frontend, vanilla JS UI replaced.
- Ported BrowSDR features: (1) Vue/TS stack, (3) POCSAG, (5) bookmarks,
  (6) activity scanner, (7) RDS, (10) NFM/AM/USB/LSB/DSB/CW in addition to WBFM.
- Deferred: (2) true multi-VFO, (4) Whisper, (8) WebRTC, (9) replacing current waterfall.

## Layout

```
web/                    # Vite project (source)
  index.html
  src/
    main.ts             # Vue entry
    App.vue
    ws/client.ts        # WebSocket protocol wrapper
    dsp/                # browser-side helpers (audio mixer, RDS decode, POCSAG)
    widgets/            # waterfall, spectrum, s-meter, dseg7 freq
    stores/             # Pinia (or refs) state
    types.ts            # protocol types shared with C (hand-written)
  vite.config.ts
  package.json
  tsconfig.json
components/websdr/
  dist/                 # Vite build output (embedded by CMake, gitignored)
  websdr.c              # extended with new commands
  CMakeLists.txt        # EMBED_FILES glob from dist/
  www/                  # legacy vanilla UI — kept until port is at parity, then removed
```

## Build flow

1. `cd web && npm install && npm run build` — writes `components/websdr/dist/`.
2. `idf.py build` — embeds `dist/*` via CMake.
3. `idf.py flash monitor`.

A helper script `tools/build-web.sh` will chain 1+2.

## Milestones

M1. Vite scaffold, Vue shell, WS client, parity with current frequency tune + FFT
    waterfall. Ship as replacement for `index.html`/`sdr.js`/`sdr.css`.
M2. DSP controls ribbon (gain, rate, fft_size, db_range, squelch, bw, nb/nr/notch).
M3. Demod modes (#10) — C adds `mode` command + per-mode DSP routing.
M4. RDS (#7) — C extracts RDS groups from WBFM 57 kHz subcarrier, emits text frames.
M5. Bookmarks (#5) — pure client, localStorage.
M6. POCSAG (#3) — C-side FSK + POCSAG decoder, or browser-side from demod audio.
M7. Activity scanner (#6) — C-side peak detect across a scan window.

Each milestone is independently shippable and reviewable.

## Protocol additions (to be finalized per milestone)

See `components/websdr/include/` (to be created) for a canonical `ws-protocol.h`
that both the C handler and `web/src/types.ts` mirror by hand.
