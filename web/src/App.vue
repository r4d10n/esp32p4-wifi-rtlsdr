<script setup lang="ts">
import { onMounted, onUnmounted, reactive, ref } from 'vue';
import { WsClient } from './ws/client';
import type { ServerMsg, DspSettings } from './types';
import Waterfall from '@/widgets/Waterfall.vue';
import FreqDisplay from '@/widgets/FreqDisplay.vue';
import Ribbon from '@/widgets/Ribbon.vue';
import { AudioPipeline } from '@/dsp/audio';

const connState = ref<'connecting' | 'open' | 'closed'>('connecting');
const freqHz = ref(100_000_000);
const sampleRate = ref(1_024_000);
const gain = ref(0);
const fftSize = ref(1024);
const dbMin = ref(-120);
const dbMax = ref(-20);
const lastFft = ref<Float32Array | null>(null);
const audioOn = ref(false);

const dsp = reactive<DspSettings>({
  mode: 'WBFM',
  bwHz: 200_000,
  squelchDb: -120,
  noiseReduction: 0,
  notchHz: 0,
  deemphasisUs: 50,
  stereo: false,
  volume: 0.8,
});

let ws: WsClient | null = null;
let audio: AudioPipeline | null = null;

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
    case 'iq_start':
      audio?.setIqRate(msg.rate);
      break;
    case 'iq_stop':
      // handled by caller via audioOn
      break;
  }
}

function onBinary(buf: ArrayBuffer, kind: 'fft' | 'iq') {
  if (kind === 'fft') {
    // C sends uint8 0..255; map to dB via db_range client-side for display.
    const u8 = new Uint8Array(buf);
    const f32 = new Float32Array(u8.length);
    const span = dbMax.value - dbMin.value;
    for (let i = 0; i < u8.length; i++) f32[i] = dbMin.value + (u8[i] / 255) * span;
    lastFft.value = f32;
  } else if (audio) {
    audio.feed(buf);
  }
}

function tune(deltaHz: number) {
  const next = Math.max(0, freqHz.value + deltaHz);
  freqHz.value = next;
  ws?.send({ cmd: 'freq', value: next });
}

async function toggleAudio() {
  if (audioOn.value) {
    ws?.send({ cmd: 'unsubscribe_iq' });
    await audio?.stop();
    audio = null;
    audioOn.value = false;
    return;
  }
  audio = new AudioPipeline(dsp);
  await audio.start();
  ws?.send({ cmd: 'subscribe_iq', offset: 0, bw: dsp.bwHz });
  audioOn.value = true;
}

function updateDsp(patch: Partial<DspSettings>) {
  Object.assign(dsp, patch);
  audio?.update(patch);
  // Bandwidth and mode change the server-side DDC output rate — need to
  // resubscribe so the backend rebuilds the DDC with the new parameters.
  if ((patch.bwHz || patch.mode) && audioOn.value) {
    ws?.send({ cmd: 'subscribe_iq', offset: 0, bw: dsp.bwHz });
  }
}

onMounted(() => {
  ws = new WsClient({
    onText, onBinary,
    onStateChange: (s) => (connState.value = s),
  });
});
onUnmounted(() => { ws?.close(); audio?.stop(); });
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
        {{ (sampleRate / 1e6).toFixed(3) }} MS/s ·
        gain {{ gain === 0 ? 'auto' : (gain / 10).toFixed(1) + ' dB' }} ·
        {{ dsp.mode }}
      </span>
    </header>

    <Ribbon
      :sample-rate="sampleRate"
      :gain="gain"
      :db-min="dbMin" :db-max="dbMax"
      :fft-size="fftSize"
      :dsp="dsp"
      :audio-on="audioOn"
      @rate="(v) => { sampleRate = v; ws?.send({ cmd: 'rate', value: v }); }"
      @gain="(v) => { gain = v; ws?.send({ cmd: 'gain', value: v }); }"
      @db-range="(mn, mx) => { dbMin = mn; dbMax = mx; ws?.send({ cmd: 'db_range', min: mn, max: mx }); }"
      @fft-size="(v) => { fftSize = v; ws?.send({ cmd: 'fft_size', value: v }); }"
      @dsp="updateDsp"
      @audio-toggle="toggleAudio"
    />

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
.canvas { flex: 1; position: relative; overflow: hidden; }
</style>
