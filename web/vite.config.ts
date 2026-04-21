import { defineConfig } from 'vite';
import vue from '@vitejs/plugin-vue';
import { fileURLToPath, URL } from 'node:url';

// Build output is embedded by the ESP-IDF `websdr` component via EMBED_FILES.
// Keep the file list small and stable: single JS bundle, single CSS bundle,
// plus the root HTML. No code splitting — the ESP can only embed files it
// knows about at CMake configure time.
export default defineConfig({
  plugins: [vue()],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  build: {
    outDir: fileURLToPath(new URL('../components/websdr/dist', import.meta.url)),
    emptyOutDir: true,
    assetsInlineLimit: 0,
    // Output filenames intentionally match the legacy embedded assets
    // (sdr.js / sdr.css / index.html), so the ESP-IDF C side can keep the
    // same `_binary_*_start` symbols and EMBED_FILES paths — we only change
    // the source directory (www/ → dist/) in CMakeLists.txt.
    rollupOptions: {
      output: {
        entryFileNames: 'sdr.js',
        chunkFileNames: 'sdr.js',
        assetFileNames: (info) => {
          if (info.name && info.name.endsWith('.css')) return 'sdr.css';
          return 'assets/[name][extname]';
        },
        manualChunks: undefined,
      },
    },
  },
  server: {
    // During `npm run dev`, proxy the WebSocket to the ESP32 so we can
    // iterate on UI without reflashing. Set VITE_ESP_HOST to the device IP.
    proxy: {
      '/ws': {
        target: process.env.VITE_ESP_HOST
          ? `ws://${process.env.VITE_ESP_HOST}`
          : 'ws://esp32p4.local',
        ws: true,
        changeOrigin: true,
      },
    },
  },
});
