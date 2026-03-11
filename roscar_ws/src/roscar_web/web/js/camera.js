/**
 * camera.js — MJPEG stream from web_video_server
 * Uses a plain <img> tag; web_video_server provides the stream URL.
 */

import { HOST, PORTS, onAppEvent } from './app.js';

let quality    = 50;
let resolution = '640x480';
let streaming  = false;

const TOPIC = '/image_raw';

export function initCamera() {
  setupControls();
  onAppEvent((ev, tab) => {
    if (ev === 'tabchange') {
      if (tab === 'camera') startStream();
      else stopStream();
    }
  });
}

// ── Build stream URL ───────────────────────────────────────────────────────
function streamUrl() {
  const [w, h] = resolution.split('x');
  // Don't encodeURIComponent the topic — web_video_server doesn't decode %2F,
  // so it can't find the topic. ROS topic chars (/ _ alnum) are URL-safe.
  return `http://${HOST}:${PORTS.video}/stream?` +
    `topic=${TOPIC}&quality=${quality}&width=${w}&height=${h}&type=mjpeg`;
}

// ── Start / Stop ───────────────────────────────────────────────────────────
let streamTimeout = null;

function startStream() {
  const img     = document.getElementById('camera-img');
  const overlay = document.getElementById('camera-overlay');

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

  img.src = streamUrl();
  overlay.style.display = 'none'; // optimistic hide

  // If no frame arrives within 5s, show offline overlay
  streamTimeout = setTimeout(() => {
    if (!streaming) {
      overlay.style.display = '';
    }
  }, 5000);
}

function stopStream() {
  clearTimeout(streamTimeout);
  const img = document.getElementById('camera-img');
  img.src = '';
  document.getElementById('camera-overlay').style.display = '';
  streaming = false;
}

// ── Controls ───────────────────────────────────────────────────────────────
function setupControls() {
  // Quality buttons
  document.querySelectorAll('[data-quality]').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-quality]').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      quality = parseInt(btn.dataset.quality, 10);
      if (streaming) startStream();
    });
  });

  // Resolution buttons
  document.querySelectorAll('[data-res]').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-res]').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      resolution = btn.dataset.res;
      if (streaming) startStream();
    });
  });

  // Fullscreen
  document.getElementById('cam-fullscreen').addEventListener('click', () => {
    const frame = document.querySelector('.camera-frame');
    if (document.fullscreenElement) {
      document.exitFullscreen();
    } else {
      frame.requestFullscreen && frame.requestFullscreen();
    }
  });
}
