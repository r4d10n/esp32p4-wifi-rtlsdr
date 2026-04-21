<script setup lang="ts">
import { computed } from 'vue';

const props = defineProps<{ hz: number }>();
const emit = defineEmits<{ (e: 'step', deltaHz: number): void }>();

// Format 100_000_000 → "100.000.000" with digit-level interaction: wheel and
// click on a digit tunes by that decade.
const digits = computed(() => {
  const s = props.hz.toString().padStart(10, '0');
  return s.split('').map((d, i) => ({ d, decade: 10 ** (s.length - 1 - i) }));
});

function onWheel(ev: WheelEvent, decade: number) {
  ev.preventDefault();
  emit('step', ev.deltaY < 0 ? decade : -decade);
}
</script>

<template>
  <div class="freq">
    <template v-for="(g, i) in digits" :key="i">
      <span
        class="digit"
        :class="{ lead: g.d === '0' && i < digits.length - 4 }"
        @wheel="(e) => onWheel(e, g.decade)"
        @click="emit('step', g.decade)"
        @contextmenu.prevent="emit('step', -g.decade)"
      >{{ g.d }}</span>
      <span v-if="i === 0 || (digits.length - 1 - i) % 3 === 2" class="sep">.</span>
    </template>
    <span class="unit">Hz</span>
  </div>
</template>

<style scoped>
.freq {
  font-family: 'DSEG7 Classic', ui-monospace, monospace;
  font-size: 2em;
  color: var(--amber);
  letter-spacing: 2px;
  user-select: none;
}
.digit {
  display: inline-block;
  min-width: 0.6em;
  text-align: center;
  cursor: ns-resize;
}
.digit:hover { color: #fff; text-shadow: 0 0 6px var(--amber); }
.digit.lead { opacity: 0.2; }
.sep { opacity: 0.5; margin: 0 1px; }
.unit { font-size: 0.5em; opacity: 0.6; margin-left: 6px; }
</style>
