<script setup lang="ts">
import { computed, ref } from 'vue';
import { useBookmarks, type Bookmark } from '@/stores/bookmarks';
import type { DemodMode, DspSettings } from '@/types';

const props = defineProps<{
  currentFreq: number;
  currentMode: DemodMode;
  currentBw: number;
}>();

const emit = defineEmits<{
  (e: 'recall', b: Bookmark): void;
}>();

const bm = useBookmarks();
const newName = ref('');
const newCategory = ref('General');

const grouped = computed(() => bm.byCategory());

function formatFreq(hz: number): string {
  if (hz >= 1e6) return (hz / 1e6).toFixed(3) + ' MHz';
  if (hz >= 1e3) return (hz / 1e3).toFixed(1) + ' kHz';
  return hz + ' Hz';
}

function saveCurrent() {
  const name = newName.value.trim() || formatFreq(props.currentFreq);
  bm.add({
    name,
    category: newCategory.value.trim() || 'General',
    freqHz: props.currentFreq,
    mode: props.currentMode,
    bwHz: props.currentBw,
  });
  newName.value = '';
}

function exportJson() {
  const blob = new Blob([JSON.stringify(bm.state.items, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = 'websdr-bookmarks.json';
  a.click();
  URL.revokeObjectURL(url);
}

async function importJson(ev: Event) {
  const file = (ev.target as HTMLInputElement).files?.[0];
  if (!file) return;
  try {
    const text = await file.text();
    const parsed = JSON.parse(text);
    if (!Array.isArray(parsed)) return;
    for (const b of parsed as Partial<Bookmark>[]) {
      if (typeof b.freqHz === 'number' && typeof b.mode === 'string') {
        bm.add({
          name: b.name ?? formatFreq(b.freqHz),
          category: b.category ?? 'General',
          freqHz: b.freqHz,
          mode: b.mode as DspSettings['mode'],
          bwHz: typeof b.bwHz === 'number' ? b.bwHz : 12500,
          notes: b.notes,
        });
      }
    }
  } catch { /* ignore bad file */ }
  (ev.target as HTMLInputElement).value = '';
}
</script>

<template>
  <div class="bookmarks">
    <div class="add-row">
      <input v-model="newName" placeholder="Name (blank = current freq)" />
      <input v-model="newCategory" placeholder="Category" class="cat" />
      <button @click="saveCurrent">＋ Save current</button>
      <span class="sep" />
      <button @click="exportJson">Export</button>
      <label class="import">
        Import
        <input type="file" accept="application/json" @change="importJson" hidden />
      </label>
    </div>

    <div class="groups">
      <div v-for="(list, cat) in grouped" :key="cat" class="group">
        <div class="cat-label">{{ cat }}</div>
        <div class="chips">
          <div v-for="b in list" :key="b.id" class="chip"
               @click="emit('recall', b)" :title="`${formatFreq(b.freqHz)} · ${b.mode} · ${(b.bwHz / 1000).toFixed(1)} kHz`">
            <span class="name">{{ b.name }}</span>
            <span class="meta">{{ formatFreq(b.freqHz) }} · {{ b.mode }}</span>
            <button class="del" @click.stop="bm.remove(b.id)" title="Delete">✕</button>
          </div>
        </div>
      </div>
      <div v-if="bm.state.items.length === 0" class="empty">
        No bookmarks yet — set a frequency/mode and hit "Save current".
      </div>
    </div>
  </div>
</template>

<style scoped>
.bookmarks { display: flex; flex-direction: column; gap: 8px; width: 100%; }
.add-row { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; }
.add-row input { padding: 3px 6px; min-width: 140px; }
.add-row input.cat { min-width: 100px; }
.add-row .sep { flex: 1; }
.import {
  background: var(--panel); color: var(--text);
  border: 1px solid var(--amber-dim); padding: 4px 8px; cursor: pointer; border-radius: 2px;
}
.import:hover { border-color: var(--amber); }
.groups { display: flex; flex-wrap: wrap; gap: 12px; }
.cat-label { font-size: 0.75em; color: var(--amber); text-transform: uppercase; margin-bottom: 4px; }
.chips { display: flex; flex-wrap: wrap; gap: 6px; }
.chip {
  display: flex; flex-direction: column;
  background: var(--bg); border: 1px solid var(--amber-dim); border-radius: 2px;
  padding: 4px 26px 4px 8px; cursor: pointer; position: relative; min-width: 140px;
}
.chip:hover { border-color: var(--amber); }
.chip .name { color: var(--amber); font-size: 0.9em; }
.chip .meta { font-size: 0.75em; opacity: 0.7; font-family: ui-monospace, monospace; }
.chip .del {
  position: absolute; top: 2px; right: 2px;
  border: 0; background: transparent; color: var(--amber-dim);
  padding: 2px 4px; line-height: 1; border-radius: 2px;
}
.chip .del:hover { background: var(--accent); color: #fff; }
.empty { opacity: 0.6; font-style: italic; padding: 8px; }
</style>
