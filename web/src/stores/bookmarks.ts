import { reactive, watch } from 'vue';
import type { DemodMode } from '@/types';

export interface Bookmark {
  id: string;
  name: string;
  category: string;
  freqHz: number;
  mode: DemodMode;
  bwHz: number;
  notes?: string;
}

const STORAGE_KEY = 'websdr.bookmarks.v1';

function load(): Bookmark[] {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return defaults();
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return defaults();
    return parsed;
  } catch {
    return defaults();
  }
}

function defaults(): Bookmark[] {
  // A handful of useful starter bookmarks — categories demonstrate the
  // grouping UI; freqs are generic so they work anywhere in the world.
  return [
    { id: crypto.randomUUID(), name: 'FM 100.0',  category: 'Broadcast', freqHz: 100_000_000, mode: 'WBFM', bwHz: 200_000 },
    { id: crypto.randomUUID(), name: 'Air tower', category: 'Airband',   freqHz: 118_100_000, mode: 'AM',   bwHz: 9_000 },
    { id: crypto.randomUUID(), name: 'Marine 16', category: 'Marine',    freqHz: 156_800_000, mode: 'NFM',  bwHz: 12_500 },
  ];
}

const state = reactive<{ items: Bookmark[] }>({ items: load() });

watch(() => state.items, (v) => {
  try { localStorage.setItem(STORAGE_KEY, JSON.stringify(v)); } catch { /* quota */ }
}, { deep: true });

export function useBookmarks() {
  function add(b: Omit<Bookmark, 'id'>): Bookmark {
    const entry = { ...b, id: crypto.randomUUID() };
    state.items.push(entry);
    return entry;
  }
  function remove(id: string) {
    const i = state.items.findIndex((b) => b.id === id);
    if (i >= 0) state.items.splice(i, 1);
  }
  function update(id: string, patch: Partial<Omit<Bookmark, 'id'>>) {
    const b = state.items.find((x) => x.id === id);
    if (b) Object.assign(b, patch);
  }
  function categories(): string[] {
    return Array.from(new Set(state.items.map((b) => b.category))).sort();
  }
  function byCategory(): Record<string, Bookmark[]> {
    const out: Record<string, Bookmark[]> = {};
    for (const b of state.items) (out[b.category] ??= []).push(b);
    return out;
  }
  return { state, add, remove, update, categories, byCategory };
}
