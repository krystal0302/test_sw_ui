import { resolve } from 'path';
import dotenv from 'dotenv';
dotenv.config();

import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

const {PORT = 8888} = process.env;

export default defineConfig({
  plugins: [react()],
  server: {
    port: PORT,
    open: '/wrapper.html',
    proxy: {
      '/api': {
        target: `http://localhost:${PORT}`,
        changeOrigin: true,
      },
    },
  },
  build: {
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'wrapper.html'),
      }
    },
    outDir: 'dist',
    // outDir: 'dist/public',
  },
})
