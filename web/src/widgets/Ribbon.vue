<script setup lang="ts">
import { ref } from 'vue';
import type { DspSettings, DemodMode } from '@/types';

const props = defineProps<{
  sampleRate: number;
  gain: number;
  dbMin: number;
  dbMax: number;
  fftSize: number;
  dsp: DspSettings;
  audioOn: boolean;
}>();

const emit = defineEmits<{
  (e: 'rate', v: number): void;
  (e: 'gain', v: number): void;
  (e: 'dbRange', min: number, max: number): void;
  (e: 'fftSize', v: number): void;
  (e: 'dsp', v: Partial<DspSettings>): void;
  (e: 'audioToggle'): void;
}>();

const tab = ref<'home' | 'dsp' | 'display' | 'favourites' | 'help'>('home');
const MODES: DemodMode[] = ['WBFM', 'NFM', 'AM', 'USB', 'LSB', 'DSB', 'CW'];
const BW_PRESETS: Record<DemodMode, number[]> = {
  WBFM: [150000, 200000, 256000],
  NFM:  [6250, 12500, 25000],
  AM:   [5000, 6000, 9000, 10000],
  USB:  [2400, 2700, 3000],
  LSB:  [2400, 2700, 3000],
  DSB:  [5000, 6000],
  CW:   [250, 500, 800, 1000],
};

function onMode(m: DemodMode) {
  const bw = BW_PRESETS[m][0];
  emit('dsp', { mode: m, bwHz: bw });
}
</script>

<template>
  <div class="ribbon">
    <div class="tabs">
      <button v-for="t in ['home','dsp','display','favourites','help'] as const"
              :key="t" :class="{ active: tab === t }" @click="tab = t">
        {{ t[0].toUpperCase() + t.slice(1) }}
      </button>
    </div>

    <div v-if="tab === 'home'" class="panel">
      <div class="group">
        <label>Mode</label>
        <div class="segmented">
          <button v-for="m in MODES" :key="m"
                  :class="{ active: props.dsp.mode === m }" @click="onMode(m)">
            {{ m }}
          </button>
        </div>
      </div>
      <div class="sep" />
      <div class="group">
        <label>BW</label>
        <select :value="props.dsp.bwHz"
                @change="(e) => emit('dsp', { bwHz: +(e.target as HTMLSelectElement).value })">
          <option v-for="bw in BW_PRESETS[props.dsp.mode]" :key="bw" :value="bw">
            {{ bw >= 1000 ? (bw / 1000) + ' kHz' : bw + ' Hz' }}
          </option>
        </select>
      </div>
      <div class="sep" />
      <div class="group">
        <label>Audio</label>
        <button :class="{ active: props.audioOn }" @click="emit('audioToggle')">
          {{ props.audioOn ? 'Stop' : 'Start' }}
        </button>
      </div>
      <div class="sep" />
      <div class="group">
        <label>Volume</label>
        <input type="range" min="0" max="100" step="1"
               :value="props.dsp.volume * 100"
               @input="(e) => emit('dsp', { volume: +(e.target as HTMLInputElement).value / 100 })" />
      </div>
      <div class="sep" />
      <div class="group">
        <label>Rate</label>
        <select :value="props.sampleRate"
                @change="(e) => emit('rate', +(e.target as HTMLSelectElement).value)">
          <option value="250000">250k</option>
          <option value="1024000">1.024M</option>
          <option value="1200000">1.2M</option>
          <option value="2048000">2.048M</option>
          <option value="2400000">2.4M</option>
        </select>
      </div>
      <div class="sep" />
      <div class="group">
        <label>Gain</label>
        <input type="range" min="0" max="496" step="1"
               :value="props.gain"
               @input="(e) => emit('gain', +(e.target as HTMLInputElement).value)" />
        <span class="val">{{ props.gain === 0 ? 'Auto' : (props.gain / 10).toFixed(1) + ' dB' }}</span>
      </div>
    </div>

    <div v-else-if="tab === 'dsp'" class="panel">
      <div class="group">
        <label>Squelch (dB)</label>
        <input type="range" min="-120" max="0" step="1"
               :value="props.dsp.squelchDb"
               @input="(e) => emit('dsp', { squelchDb: +(e.target as HTMLInputElement).value })" />
        <span class="val">{{ props.dsp.squelchDb }} dB</span>
      </div>
      <div class="sep" />
      <div class="group">
        <label>Notch (Hz)</label>
        <input type="number" step="10" min="0" max="15000"
               :value="props.dsp.notchHz"
               @change="(e) => emit('dsp', { notchHz: +(e.target as HTMLInputElement).value })" />
      </div>
      <div class="sep" />
      <div class="group">
        <label>NR</label>
        <input type="range" min="0" max="100" step="1"
               :value="props.dsp.noiseReduction * 100"
               @input="(e) => emit('dsp', { noiseReduction: +(e.target as HTMLInputElement).value / 100 })" />
        <span class="val">{{ (props.dsp.noiseReduction * 100).toFixed(0) }}%</span>
      </div>
      <div class="sep" />
      <div class="group">
        <label>De-emph (µs)</label>
        <div class="segmented">
          <button :class="{ active: props.dsp.deemphasisUs === 0 }" @click="emit('dsp', { deemphasisUs: 0 })">Off</button>
          <button :class="{ active: props.dsp.deemphasisUs === 50 }" @click="emit('dsp', { deemphasisUs: 50 })">50</button>
          <button :class="{ active: props.dsp.deemphasisUs === 75 }" @click="emit('dsp', { deemphasisUs: 75 })">75</button>
        </div>
      </div>
    </div>

    <div v-else-if="tab === 'display'" class="panel">
      <div class="group">
        <label>FFT size</label>
        <select :value="props.fftSize"
                @change="(e) => emit('fftSize', +(e.target as HTMLSelectElement).value)">
          <option v-for="n in [256,512,1024,2048,4096,8192]" :key="n" :value="n">{{ n }}</option>
        </select>
      </div>
      <div class="sep" />
      <div class="group">
        <label>dB min</label>
        <input type="number" :value="props.dbMin"
               @change="(e) => emit('dbRange', +(e.target as HTMLInputElement).value, props.dbMax)" />
      </div>
      <div class="group">
        <label>dB max</label>
        <input type="number" :value="props.dbMax"
               @change="(e) => emit('dbRange', props.dbMin, +(e.target as HTMLInputElement).value)" />
      </div>
    </div>

    <div v-else-if="tab === 'favourites'" class="panel">
      <slot name="favourites" />
    </div>

    <div v-else-if="tab === 'help'" class="panel help">
      <p>Wheel over a frequency digit to tune. Click a digit to +1 decade, right-click to −1 decade.</p>
      <p>Start Audio to enable demodulation. DSP tab exposes squelch, notch, NR, and de-emphasis.</p>
    </div>
  </div>
</template>

<style scoped>
.ribbon { background: var(--panel); border-bottom: 1px solid var(--grid); }
.tabs {
  display: flex; gap: 0; background: #0d0906;
  border-bottom: 1px solid var(--amber-dim);
}
.tabs button {
  flex: 0 0 auto; border: 0; border-right: 1px solid var(--grid);
  border-radius: 0; padding: 6px 18px;
  background: transparent; color: var(--text); opacity: 0.6;
}
.tabs button.active { background: var(--panel); opacity: 1; color: var(--amber); }
.panel {
  display: flex; flex-wrap: wrap; gap: 10px; align-items: center;
  padding: 10px 12px;
}
.group { display: flex; align-items: center; gap: 6px; }
.group label { font-size: 0.75em; color: var(--amber); text-transform: uppercase; }
.sep { width: 1px; height: 28px; background: var(--grid); }
.val { min-width: 60px; font-variant-numeric: tabular-nums; font-size: 0.9em; }
.segmented { display: flex; border: 1px solid var(--amber-dim); border-radius: 2px; }
.segmented button { border: 0; border-right: 1px solid var(--amber-dim); border-radius: 0; padding: 4px 10px; }
.segmented button:last-child { border-right: 0; }
.help p { margin: 4px 12px; opacity: 0.8; }
</style>
