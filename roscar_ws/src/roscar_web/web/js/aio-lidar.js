/**
 * aio-lidar.js — Subscribe to /scan and render a polar mini-radar in a
 * dedicated canvas inside #lidar-container.  Always renders (RAF loop),
 * auto-sizes via ResizeObserver.
 */

import { onAppEvent } from './aio-app.js';

let getRos;
let scanSub  = null;
let canvas   = null;
let ctx      = null;
let lastScan = null;
let rafId    = null;

const MAX_DISPLAY_RANGE = 4.0;           // metres
const GRID_INTERVALS    = [0.5, 1, 2];   // concentric ring distances

// ── Public init ──────────────────────────────────────────────────────────────
export function initLidar(getRosFn) {
  getRos = getRosFn;

  onAppEvent((ev) => {
    if (ev === 'connected') subscribeScans();
  });

  initCanvas();
  startRender();
}

// ── ROS subscription ─────────────────────────────────────────────────────────
function subscribeScans() {
  const ros = getRos();
  if (!ros) return;
  if (scanSub) { try { scanSub.unsubscribe(); } catch (_) { /* ignore */ } }

  scanSub = new ROSLIB.Topic({
    ros,
    name: '/scan',
    messageType: 'sensor_msgs/LaserScan',
    throttle_rate: 500,   // 2 Hz
  });
  scanSub.subscribe((msg) => { lastScan = msg; });
}

// ── Canvas setup + ResizeObserver ────────────────────────────────────────────
function initCanvas() {
  canvas = document.getElementById('lidar-canvas');
  if (!canvas) return;
  ctx = canvas.getContext('2d');

  const container = document.getElementById('lidar-container');
  if (container && typeof ResizeObserver !== 'undefined') {
    new ResizeObserver(() => sizeCanvas()).observe(container);
  }
  sizeCanvas();
}

function sizeCanvas() {
  if (!canvas) return;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (w === 0 || h === 0) return;
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width  = w;
    canvas.height = h;
  }
}

// ── RAF loop ─────────────────────────────────────────────────────────────────
function startRender() {
  if (rafId) return;
  function frame() {
    render();
    rafId = requestAnimationFrame(frame);
  }
  rafId = requestAnimationFrame(frame);
}

// ── Render ───────────────────────────────────────────────────────────────────
function render() {
  if (!canvas || !ctx) return;
  sizeCanvas();

  const w = canvas.width;
  const h = canvas.height;
  if (w === 0 || h === 0) return;
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2;
  const cy = h / 2;
  const radius = Math.min(w, h) / 2 - 4;

  // ── Grid circles at fixed metre intervals ──
  ctx.strokeStyle = 'rgba(255,255,255,0.12)';
  ctx.lineWidth = 1;
  for (const dist of GRID_INTERVALS) {
    const r = (dist / MAX_DISPLAY_RANGE) * radius;
    if (r > radius) continue;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();
  }

  // ── Forward indicator (line from centre upward) ──
  ctx.strokeStyle = 'rgba(255,255,255,0.25)';
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx, cy - radius);
  ctx.stroke();

  // ── Laser points ──
  if (lastScan) {
    const scan  = lastScan;
    const n     = scan.ranges.length;
    const maxR  = Math.min(scan.range_max || MAX_DISPLAY_RANGE, MAX_DISPLAY_RANGE);
    const scale = radius / maxR;
    const aMin  = scan.angle_min;
    const aInc  = scan.angle_increment;

    ctx.fillStyle = '#00D4FF';
    for (let i = 0; i < n; i++) {
      const d = scan.ranges[i];
      if (!isFinite(d) || d < scan.range_min || d > maxR) continue;

      const angle = aMin + i * aInc;
      // ROS scan frame: 0 = forward, CCW positive
      // Screen coords: forward = up (-y), left = left (-x)
      const px = cx + Math.sin(angle) * d * scale;
      const py = cy - Math.cos(angle) * d * scale;
      ctx.fillRect(px - 1, py - 1, 2, 2);
    }
  }

  // ── Robot dot at centre ──
  ctx.fillStyle = '#FF8C00';
  ctx.beginPath();
  ctx.arc(cx, cy, 4, 0, Math.PI * 2);
  ctx.fill();
}
