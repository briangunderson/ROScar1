/**
 * aio-camera.js — MJPEG stream panel (always active)
 * Uses a plain <img> tag; web_video_server provides the stream URL.
 */

import { HOST, PORTS, onAppEvent, toast } from './aio-app.js';

let quality    = 80;
let resolution = '640x480';
let streaming  = false;

let topic = '/image_raw';
const PANEL = '#panel-camera';
const RETRY_INTERVAL = 3000; // ms — retry stream when offline

let retryTimer = null;

export function initCamera() {
  setupControls();
  onAppEvent((ev) => {
    if (ev === 'connected') startStream();
  });
  // CV feed toggle: switch between /image_raw and /image_annotated
  window.addEventListener('cv-feed-change', (e) => {
    topic = e.detail.annotated ? '/image_annotated' : '/image_raw';
    startStream();
  });
  // Periodic retry: when the stream is down (e.g. camera node not yet
  // running after a mode switch), re-attempt every few seconds.
  retryTimer = setInterval(() => {
    if (!streaming) startStream();
  }, RETRY_INTERVAL);
}

// ── Build stream URL ───────────────────────────────────────────────────────
function streamUrl() {
  const [w, h] = resolution.split('x');
  return `http://${HOST}:${PORTS.video}/stream?` +
    `topic=${topic}&quality=${quality}&width=${w}&height=${h}&type=mjpeg`;
}

// ── Start / Stop ───────────────────────────────────────────────────────────
let streamTimeout = null;

function startStream() {
  const img     = document.querySelector(`${PANEL} #camera-img`);
  const overlay = document.querySelector(`${PANEL} #camera-overlay`);
  if (!img || !overlay) return;

  clearTimeout(streamTimeout);

  img.onload = () => {
    clearTimeout(streamTimeout);
    overlay.style.display = 'none';
    streaming = true;
  };
  img.onerror = () => {
    clearTimeout(streamTimeout);
    overlay.style.display = '';
    streaming = false;
  };

  // Force the browser to re-request by busting any cached failed response
  img.src = '';
  img.src = streamUrl();
  overlay.style.display = 'none'; // optimistic hide

  // If no frame arrives within 5s, show offline overlay
  streamTimeout = setTimeout(() => {
    if (!streaming) {
      overlay.style.display = '';
    }
  }, 5000);
}

// ── Controls ───────────────────────────────────────────────────────────────
function setupControls() {
  // Quality buttons (scoped to camera panel)
  document.querySelectorAll(`${PANEL} [data-quality]`).forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll(`${PANEL} [data-quality]`).forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      quality = parseInt(btn.dataset.quality, 10);
      startStream();
    });
  });

  // Resolution buttons (scoped to camera panel)
  document.querySelectorAll(`${PANEL} [data-res]`).forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll(`${PANEL} [data-res]`).forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      resolution = btn.dataset.res;
      startStream();
    });
  });
}
