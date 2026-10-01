import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// The site is meant to be hosted under a subpath of the owner's website, so the
// base is a build-time decision rather than a hardcoded '/'. Vite rewrites every
// asset URL from it; this app never builds an absolute path of its own.
//
//   AK_BASE=/aerialkit/configurator/ npm run build
//
// A base that does not both start and end with '/' is the classic way a subpath
// deployment 404s on its own assets, so it is normalized here rather than
// trusted.
function normalizeBase(raw: string | undefined): string {
  const value = (raw ?? '/').trim();
  if (value === '' || value === '/') return '/';
  return '/' + value.replace(/^\/+/, '').replace(/\/+$/, '') + '/';
}

export default defineConfig({
  base: normalizeBase(process.env.AK_BASE),
  plugins: [react()],
  build: {
    outDir: 'dist',
    sourcemap: true,
    // No vendor chunk splitting: the app is small and one request is simpler to
    // reason about when checking a deployment.
    chunkSizeWarningLimit: 900,
  },
  test: {
    environment: 'jsdom',
    globals: true,
    setupFiles: ['./vitest.setup.ts'],
    include: ['tests/**/*.test.{ts,tsx}'],
  },
});
