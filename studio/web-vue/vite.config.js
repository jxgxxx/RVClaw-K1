import { defineConfig } from 'vite';
import vue from '@vitejs/plugin-vue';
import Components from 'unplugin-vue-components/vite';
import { ElementPlusResolver } from 'unplugin-vue-components/resolvers';

const board = 'http://100.112.170.125:8017';
const proxy = Object.fromEntries(
  ['/api', '/stream', '/inspection-photos', '/inspection-evidence', '/inspection-progress.json', '/voice-health.json']
    .map((path) => [path, { target: board, changeOrigin: true }]),
);

export default defineConfig({
  base: '/',
  plugins: [
    vue(),
    Components({ resolvers: [ElementPlusResolver({ importStyle: 'css' })] }),
  ],
  build: {
    outDir: '../web',
    emptyOutDir: true,
    assetsDir: 'assets',
    assetsInlineLimit: 4096,
  },
  server: { proxy },
});
