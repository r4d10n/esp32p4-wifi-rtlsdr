<script setup lang="ts">
import { onMounted, onUnmounted, ref, watch } from 'vue';

const props = defineProps<{
  fft: Float32Array | null;
  dbMin: number;
  dbMax: number;
}>();

const canvas = ref<HTMLCanvasElement | null>(null);
const spectrum = ref<HTMLCanvasElement | null>(null);

// Plain Canvas2D waterfall. We keep the existing project's WebGL2 waterfall
// as a later swap-in — M1's goal is parity of *behavior*, not pixel-perfect
// replication. Canvas2D is simpler to reason about while the data flow is
// being ported; the hot path (ImageData blit) is still GPU-accelerated on
// every modern browser.

let gl: CanvasRenderingContext2D | null = null;
let sctx: CanvasRenderingContext2D | null = null;
let width = 0;
let height = 0;
let rowImage: ImageData | null = null;
let resizeObserver: ResizeObserver | null = null;

function colorFor(db: number, min: number, max: number): [number, number, number] {
  // Amber-black colormap, matches current project's theme.
  const t = Math.max(0, Math.min(1, (db - min) / (max - min)));
  const r = Math.round(255 * Math.min(1, t * 1.8));
  const g = Math.round(176 * t);
  const b = Math.round(16 * t);
  return [r, g, b];
}

function resize() {
  if (!canvas.value || !spectrum.value) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const rect = canvas.value.parentElement!.getBoundingClientRect();
  width = Math.floor(rect.width * dpr);
  height = Math.floor((rect.height - 120) * dpr); // reserve top for spectrum
  canvas.value.width = width;
  canvas.value.height = height;
  spectrum.value.width = width;
  spectrum.value.height = Math.floor(120 * dpr);
  rowImage = null;
}

function drawRow(fft: Float32Array) {
  if (!gl || !sctx) return;
  if (!rowImage || rowImage.width !== width) {
    rowImage = gl.createImageData(width, 1);
  }
  // Scroll existing image down by 1px
  gl.drawImage(gl.canvas, 0, 1);
  // Fill the new top row from the FFT bins
  const data = rowImage.data;
  for (let x = 0; x < width; x++) {
    const bin = Math.floor((x / width) * fft.length);
    const [r, g, b] = colorFor(fft[bin], props.dbMin, props.dbMax);
    const o = x * 4;
    data[o] = r; data[o + 1] = g; data[o + 2] = b; data[o + 3] = 255;
  }
  gl.putImageData(rowImage, 0, 0);

  // Spectrum line on top
  const sw = spectrum.value!.width;
  const sh = spectrum.value!.height;
  sctx.fillStyle = '#0d0906';
  sctx.fillRect(0, 0, sw, sh);
  sctx.strokeStyle = '#ffb000';
  sctx.lineWidth = 1;
  sctx.beginPath();
  for (let x = 0; x < sw; x++) {
    const bin = Math.floor((x / sw) * fft.length);
    const t = Math.max(0, Math.min(1, (fft[bin] - props.dbMin) / (props.dbMax - props.dbMin)));
    const y = sh - t * sh;
    if (x === 0) sctx.moveTo(x, y); else sctx.lineTo(x, y);
  }
  sctx.stroke();
}

watch(() => props.fft, (fft) => { if (fft) drawRow(fft); });

onMounted(() => {
  if (!canvas.value || !spectrum.value) return;
  gl = canvas.value.getContext('2d');
  sctx = spectrum.value.getContext('2d');
  resize();
  resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(canvas.value.parentElement!);
});

onUnmounted(() => resizeObserver?.disconnect());
</script>

<template>
  <div class="wrap">
    <canvas ref="spectrum" class="spectrum" />
    <canvas ref="canvas" class="waterfall" />
  </div>
</template>

<style scoped>
.wrap { position: absolute; inset: 0; display: flex; flex-direction: column; }
.spectrum { width: 100%; height: 120px; display: block; }
.waterfall { width: 100%; flex: 1; display: block; image-rendering: pixelated; }
</style>
