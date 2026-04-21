<script setup lang="ts">
import { onMounted, onUnmounted, ref } from 'vue';
import { WsClient } from './ws/client';
import type { ServerMsg } from './types';
import Waterfall from '@/widgets/Waterfall.vue';
import FreqDisplay from '@/widgets/FreqDisplay.vue';

const connState = ref<'connecting' | 'open' | 'closed'>('connecting');
const freqHz = ref(100_000_000);
const sampleRate = ref(1_024_000);
const gain = ref(0);
const fftSize = ref(1024);
const dbMin = ref(-120);
const dbMax = ref(-20);
const lastFft = ref<Float32Array | null>(null);

let ws: WsClient | null = null;

function onText(msg: ServerMsg) {
  switch (msg.type) {
    case 'info':
      freqHz.value = msg.freq;
      sampleRate.value = msg.rate;
      gain.value = msg.gain;
      break;
    case 'config':
      fftSize.value = msg.fft_size;
      sampleRate.value = msg.sample_rate;
      break;
    case 'freq':
      freqHz.value = msg.value;
      break;
  }
}

function onBinary(buf: ArrayBuffer, kind: 'fft' | 'audio') {
  if (kind === 'fft') {
    // FFT frame is a Float32Array of dB values, length == fftSize.
    lastFft.value = new Float32Array(buf);
  }
  // audio kind handled by audio pipeline in a later milestone
}

function tune(deltaHz: number) {
  const next = freqHz.value + deltaHz;
  freqHz.value = next;
  ws?.send({ cmd: 'freq', value: next });
}

function setRate(rate: number) {
  sampleRate.value = rate;
  ws?.send({ cmd: 'rate', value: rate });
}

function setGain(g: number) {
  gain.value = g;
  ws?.send({ cmd: 'gain', value: g });
}

onMounted(() => {
  ws = new WsClient({
    onText,
    onBinary,
    onStateChange: (s) => (connState.value = s),
  });
});

onUnmounted(() => ws?.close());
</script>

<template>
  <div class="layout">
    <header class="topbar">
      <span>
        <span class="status-dot" :class="connState" />
        ESP32-P4 WebSDR
      </span>
      <FreqDisplay :hz="freqHz" @step="tune" />
      <span class="meta">
        {{ (sampleRate / 1e6).toFixed(3) }} MS/s · gain
        {{ gain === 0 ? 'auto' : (gain / 10).toFixed(1) + ' dB' }}
      </span>
    </header>

    <section class="ribbon">
      <div class="group">
        <label>Rate</label>
        <select :value="sampleRate" @change="(e) => setRate(+(e.target as HTMLSelectElement).value)">
          <option value="250000">250k</option>
          <option value="1024000">1.024M</option>
          <option value="1200000">1.2M</option>
          <option value="2048000">2.048M</option>
          <option value="2400000">2.4M</option>
        </select>
      </div>
      <div class="group">
        <label>Gain</label>
        <input
          type="range" min="0" max="496" step="1"
          :value="gain" @input="(e) => setGain(+(e.target as HTMLInputElement).value)"
        />
        <span class="val">{{ gain === 0 ? 'Auto' : (gain / 10).toFixed(1) + ' dB' }}</span>
      </div>
      <div class="group">
        <label>dB range</label>
        <input type="number" :value="dbMin" @change="(e) => (dbMin = +(e.target as HTMLInputElement).value, ws?.send({ cmd: 'db_range', min: dbMin, max: dbMax }))" />
        <input type="number" :value="dbMax" @change="(e) => (dbMax = +(e.target as HTMLInputElement).value, ws?.send({ cmd: 'db_range', min: dbMin, max: dbMax }))" />
      </div>
    </section>

    <main class="canvas">
      <Waterfall :fft="lastFft" :db-min="dbMin" :db-max="dbMax" />
    </main>
  </div>
</template>

<style scoped>
.layout { display: flex; flex-direction: column; height: 100%; }
.topbar {
  display: flex; justify-content: space-between; align-items: center;
  padding: 8px 12px; background: var(--panel);
  border-bottom: 1px solid var(--amber-dim);
}
.topbar .meta { opacity: 0.7; font-size: 0.9em; }
.ribbon {
  display: flex; gap: 16px; padding: 8px 12px;
  background: var(--panel); border-bottom: 1px solid var(--grid);
}
.group { display: flex; align-items: center; gap: 6px; }
.group label { font-size: 0.8em; color: var(--amber); text-transform: uppercase; }
.val { min-width: 60px; font-variant-numeric: tabular-nums; }
.canvas { flex: 1; position: relative; overflow: hidden; }
</style>
