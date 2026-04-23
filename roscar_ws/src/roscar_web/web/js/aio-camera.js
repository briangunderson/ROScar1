/**
 * aio-camera.js — MJPEG stream panel (always active)
 * Uses a plain <img> tag; web_video_server provides the stream URL.
 *
 * Design note on OFFLINE detection:
 *   Chrome fires `<img>.onload` ONCE for MJPEG streams (on the initial frame
 *   decode), not per-frame. And `.onerror` fires spuriously on transient
 *   multipart boundary glitches. Neither is a reliable "stream alive" signal
 *   for `<img>`. So this module uses the simplest possible UI policy:
 *     - Show OFFLINE overlay until the first frame decodes.
 *     - Hide overlay forever after the first frame.
 *     - Ignore onerror (MJPEG streams trip it routinely even when healthy).
 *     - On genuine connection loss, the browser keeps displaying the last
 *       frame rather than clearing the element; a visibly-stale image is
 *       strictly better than a flickering OFFLINE/black cycle.
 *   If the stream needs to be rebuilt (mode/quality/resolution change, or
 *   after 10s+ where the connection was obviously dropped and the user
 *   expects refresh), startStream() reassigns img.src which kicks off a
 *   fresh connection and a fresh onload.
 */

import { HOST, PORTS, CV_HOST, CV_PORT, onAppEvent, toast } from './aio-app.js';

let quality    = 80;
let resolution = '640x480';
let topic      = '/image_raw';
let cvMode     = false;
let haveFrame  = false;   // set true once the first frame decoded
let currentUrl = '';      // last URL we assigned to img.src

const PANEL = '#panel-camera';

export function initCamera() {
  setupControls();
  // The MJPEG stream is served by web_video_server over plain HTTP — it
  // does NOT depend on rosbridge being connected. Start the stream
  // unconditionally on init so the feed comes up even if rosbridge is
  // still negotiating or has hit a transient error.
  startStream();
  onAppEvent((ev) => {
    // If we never got a first frame (e.g. robot stack wasn't running yet
    // when we started), retry once we're connected to rosbridge — by that
    // point the robot nodes are almost always up.
    if (ev === 'connected' && !haveFrame) startStream();
  });
  // CV feed toggle: switch between /image_raw and /image_annotated
  window.addEventListener('cv-feed-change', (e) => {
    cvMode = e.detail.annotated;
    topic  = cvMode ? '/image_annotated' : '/image_raw';
    // URL changes → need a fresh connection
    haveFrame = false;
    startStream();
  });
}

// ── Build stream URL ───────────────────────────────────────────────────────
function streamUrl() {
  const [w, h] = resolution.split('x');
  // CV feeds come from the GPU PC's web_video_server; raw feeds from the Pi's
  const host = (cvMode && CV_HOST) ? CV_HOST : HOST;
  const port = (cvMode && CV_HOST) ? CV_PORT : PORTS.video;
  return `http://${host}:${port}/stream?` +
    `topic=${topic}&quality=${quality}&width=${w}&height=${h}&type=mjpeg`;
}

// ── Start / Restart stream ─────────────────────────────────────────────────
function startStream() {
  const img     = document.querySelector(`${PANEL} #camera-img`);
  const overlay = document.querySelector(`${PANEL} #camera-overlay`);
  if (!img || !overlay) return;

  const url = streamUrl();

  // Hook up handlers (idempotent — setting .onload replaces any prior).
  img.onload = () => {
    haveFrame = true;
    overlay.style.display = 'none';
  };
  img.onerror = () => {
    // Transient MJPEG framing issues trip this even on healthy streams.
    // Stay silent; don't revert to OFFLINE.
  };

  // Skip reconnect if URL unchanged and we've already got a frame — otherwise
  // the stream would be torn down + rebuilt on every settings change that
  // didn't actually change the URL.
  if (img.src === url && haveFrame) return;
  currentUrl = url;
  img.src = '';   // abort any pending request
  img.src = url;  // start fresh stream
}

// ── Controls ───────────────────────────────────────────────────────────────
function setupControls() {
  // Quality buttons (scoped to camera panel)
  document.querySelectorAll(`${PANEL} [data-quality]`).forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll(`${PANEL} [data-quality]`).forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      quality = parseInt(btn.dataset.quality, 10);
      haveFrame = false;  // URL changed → wait for new first frame
      startStream();
    });
  });

  // Resolution buttons (scoped to camera panel)
  document.querySelectorAll(`${PANEL} [data-res]`).forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll(`${PANEL} [data-res]`).forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      resolution = btn.dataset.res;
      haveFrame = false;
      startStream();
    });
  });
}
