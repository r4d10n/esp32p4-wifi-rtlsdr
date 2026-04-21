<script setup lang="ts">
import type { RdsState } from '@/dsp/rds';
defineProps<{ rds: RdsState | null }>();

const PTY_NAMES = [
  'None', 'News', 'Affairs', 'Info', 'Sport', 'Educate', 'Drama', 'Culture',
  'Science', 'Varied', 'Pop M', 'Rock M', 'Easy M', 'Light M', 'Classics',
  'Other M', 'Weather', 'Finance', 'Children', 'Social', 'Religion', 'Phone In',
  'Travel', 'Leisure', 'Jazz', 'Country', 'Nation M', 'Oldies', 'Folk M',
  'Document', 'Test', 'Alarm',
];
</script>

<template>
  <div v-if="rds" class="rds">
    <div class="row">
      <span class="label">PS</span>
      <span class="ps">{{ rds.ps }}</span>
    </div>
    <div class="row small">
      <span class="label">PI</span>
      <span class="mono">{{ rds.pi ? rds.pi.toString(16).padStart(4, '0').toUpperCase() : '—' }}</span>
      <span class="label">PTY</span>
      <span>{{ PTY_NAMES[rds.pty] ?? rds.pty }}</span>
      <span v-if="rds.tp" class="tp">TP</span>
    </div>
    <div class="row rt">
      <span class="label">RT</span>
      <span class="rt-text">{{ rds.rt.trimEnd() || '—' }}</span>
    </div>
    <div class="row small stats">
      groups {{ rds.groupCount }} · blocks {{ rds.blockCount }}
    </div>
  </div>
  <div v-else class="rds off">RDS inactive (WBFM mode only)</div>
</template>

<style scoped>
.rds {
  position: absolute; top: 8px; right: 8px;
  background: rgba(26, 18, 11, 0.92);
  border: 1px solid var(--amber-dim);
  padding: 8px 12px; border-radius: 2px;
  max-width: 320px; font-size: 0.85em;
  z-index: 2;
}
.rds.off { color: var(--amber-dim); font-style: italic; }
.row { display: flex; gap: 8px; align-items: center; margin: 2px 0; }
.row.small { font-size: 0.85em; opacity: 0.8; }
.row.rt { flex-direction: column; align-items: flex-start; }
.label { color: var(--amber); font-size: 0.75em; min-width: 24px; text-transform: uppercase; }
.ps { font-family: 'DSEG7 Classic', ui-monospace, monospace; font-size: 1.4em; color: var(--amber); letter-spacing: 1px; }
.mono { font-family: ui-monospace, monospace; }
.rt-text { font-family: ui-monospace, monospace; word-break: break-word; }
.tp { color: #ff4040; font-weight: bold; }
.stats { opacity: 0.5; font-size: 0.75em; }
</style>
