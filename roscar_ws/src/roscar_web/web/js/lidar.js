/**
 * lidar.js — Subscribe to /scan and render a polar scan on the Drive tab canvas.
 * Rendered as an overlay inside the right joystick zone (mini radar view).
 * Kept very lightweight since it runs on the same page as teleop.
 */

import { onAppEvent } from './app.js';

let getRos;
let scanSub   = null;
let canvas    = null;
let ctx       = null;
let lastScan  = null;
let rafId     = null;

export function initLidar(getRosFn) {
  getRos = getRosFn;
  onAppEvent((ev, tab) => {
    if (ev === 'connected') subscribeScans();
    if (ev === 'tabchange') {
      if (tab === 'drive') startRender();
      else stopRender();
    }
  });
}

function subscribeScans() {
  const ros = getRos();
  if (!ros) return;
  if (scanSub) { try { scanSub.unsubscribe(); } catch(_) {} }

  scanSub = new ROSLIB.Topic({
    ros,
    name: '/scan',
    messageType: 'sensor_msgs/LaserScan',
    throttle_rate: 500,  // 2 Hz to keep CPU down
  });
  scanSub.subscribe((msg) => { lastScan = msg; });
}

// ── Canvas creation ────────────────────────────────────────────────────────
function ensureCanvas() {
  if (canvas) return true;
  const zone = document.getElementById('joy-rotate');
  if (!zone) return false;
  canvas = document.createElement('canvas');
  canvas.style.cssText = 'position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:5;opacity:0.55';
  zone.appendChild(canvas);
  ctx = canvas.getContext('2d');
  return true;
}

function startRender() {
  if (!ensureCanvas()) return;
  if (rafId) return;
  function frame() {
    render();
    rafId = requestAnimationFrame(frame);
  }
  rafId = requestAnimationFrame(frame);
}

function stopRender() {
  if (rafId) { cancelAnimationFrame(rafId); rafId = null; }
}

// ── Render ─────────────────────────────────────────────────────────────────
function render() {
  if (!canvas || !ctx) return;

  const w = canvas.offsetWidth;
  const h = canvas.offsetHeight;
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2;
  const cy = h / 2;
  const r  = Math.min(w, h) / 2 - 2;

  // Grid circles
  ctx.strokeStyle = 'rgba(0,212,255,0.12)';
  ctx.lineWidth = 1;
  for (let i = 1; i <= 3; i++) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * i / 3, 0, Math.PI * 2);
    ctx.stroke();
  }

  if (!lastScan) return;

  const scan   = lastScan;
  const n      = scan.ranges.length;
  const maxR   = scan.range_max || 12.0;
  const scale  = r / maxR;
  const aMin   = scan.angle_min;
  const aInc   = scan.angle_increment;

  ctx.fillStyle = '#00D4FF';
  for (let i = 0; i < n; i++) {
    const d = scan.ranges[i];
    if (!isFinite(d) || d <= scan.range_min || d > maxR) continue;
    const angle = aMin + i * aInc;
    // ROS scan: 0=forward, CCW+. Screen: -y=up, so rotate -π/2
    const sa = -angle - Math.PI / 2;
    const px = cx + Math.cos(sa) * d * scale;
    const py = cy + Math.sin(sa) * d * scale;
    ctx.fillRect(px - 1, py - 1, 2, 2);
  }

  // Robot dot
  ctx.fillStyle = '#FF8C00';
  ctx.beginPath();
  ctx.arc(cx, cy, 4, 0, Math.PI * 2);
  ctx.fill();
}
