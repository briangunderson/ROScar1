/**
 * aio-lidar-cam.js — Depth overlay on the camera feed.
 *
 * Projects 2D lidar scan points that fall within the camera's horizontal FOV
 * onto a canvas overlaid on the MJPEG camera image.  Renders depth-coloured
 * bars and a smoothed nearest-object distance label.
 */

import { onAppEvent } from './aio-app.js';

// ── Tunables ────────────────────────────────────────────────────────────────
const CAMERA_HFOV  = 60 * (Math.PI / 180);   // horizontal FOV in radians (~60° Logitech)
const HALF_FOV     = CAMERA_HFOV / 2;
const MAX_RANGE    = 4.0;                     // metres — clamp display beyond this
const WARN_RANGE   = 0.5;                     // proximity-warning threshold (metres)
const BAR_ALPHA    = 0.4;                     // base opacity of depth bars
const NEAREST_SHOW = 2.0;                     // only show nearest label when < this
const SMOOTH_ALPHA = 0.3;                     // EMA smoothing for nearest callout (0–1, lower = smoother)

let getRos;
let scanSub   = null;
let canvas    = null;
let ctx       = null;
let lastScan  = null;
let enabled   = true;
let rafId     = null;

// Smoothed nearest-object state (prevents jitter)
let smoothDist = Infinity;
let smoothX    = 0;

// ── Colour ramp: close (warm white) → mid (light blue) → far (dim blue) ────
function depthColour(d) {
  const t = Math.min(d / MAX_RANGE, 1);       // 0 = close, 1 = far
  let r, g, b;
  if (t < 0.3) {
    // warm white → light blue
    const s = t / 0.3;
    r = Math.round(255 - 120 * s);
    g = Math.round(255 - 80 * s);
    b = 255;
  } else {
    // light blue → dim steel blue
    const s = (t - 0.3) / 0.7;
    r = Math.round(135 - 75 * s);
    g = Math.round(175 - 75 * s);
    b = Math.round(255 - 100 * s);
  }
  return { r, g, b };
}

function rgba(c, a) {
  return `rgba(${c.r},${c.g},${c.b},${a})`;
}

// ── Public API ──────────────────────────────────────────────────────────────
export function initLidarCam(getRosFn) {
  getRos = getRosFn;
  canvas = document.getElementById('lidar-cam-canvas');
  if (!canvas) return;
  ctx = canvas.getContext('2d');

  const frame = canvas.parentElement;
  if (frame && typeof ResizeObserver !== 'undefined') {
    new ResizeObserver(() => sizeCanvas()).observe(frame);
  }
  sizeCanvas();

  onAppEvent((ev) => {
    if (ev === 'connected') subscribeScan();
  });

  // Toggle button
  const btn = document.getElementById('lidar-cam-toggle');
  if (btn) {
    btn.classList.add('active');
    btn.addEventListener('click', () => {
      enabled = !enabled;
      btn.classList.toggle('active', enabled);
      if (!enabled && ctx) ctx.clearRect(0, 0, canvas.width, canvas.height);
    });
  }

  startRender();
}

export function setLidarCamEnabled(on) {
  enabled = on;
  const btn = document.getElementById('lidar-cam-toggle');
  if (btn) btn.classList.toggle('active', enabled);
}

// ── ROS subscription ────────────────────────────────────────────────────────
function subscribeScan() {
  const ros = getRos();
  if (!ros) return;
  if (scanSub) { try { scanSub.unsubscribe(); } catch (_) { /* */ } }

  scanSub = new ROSLIB.Topic({
    ros,
    name: '/scan',
    messageType: 'sensor_msgs/LaserScan',
    throttle_rate: 100,   // 10 Hz
  });
  scanSub.subscribe((msg) => { lastScan = msg; });
}

// ── Canvas sizing ───────────────────────────────────────────────────────────
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

// ── Render loop ─────────────────────────────────────────────────────────────
function startRender() {
  if (rafId) return;
  function frame() {
    render();
    rafId = requestAnimationFrame(frame);
  }
  rafId = requestAnimationFrame(frame);
}

function render() {
  if (!canvas || !ctx) return;
  sizeCanvas();
  const W = canvas.width;
  const H = canvas.height;
  if (W === 0 || H === 0) return;
  ctx.clearRect(0, 0, W, H);
  if (!enabled || !lastScan) return;

  const scan = lastScan;
  const aMin = scan.angle_min;
  const aInc = scan.angle_increment;
  const n    = scan.ranges.length;
  const rMin = scan.range_min || 0.05;

  // ── 1. Extract points within camera FOV ──────────────────────────────────
  const points = [];
  let frameDist = Infinity;
  let frameX    = 0;

  for (let i = 0; i < n; i++) {
    const d = scan.ranges[i];
    if (!isFinite(d) || d < rMin || d > MAX_RANGE) continue;

    const angle = aMin + i * aInc;
    if (angle < -HALF_FOV || angle > HALF_FOV) continue;

    // Project angle → pixel column
    //   angle  0        → centre (W/2)
    //   angle +HALF_FOV → left edge (0)     [ROS CCW+]
    //   angle -HALF_FOV → right edge (W)
    const x = W * (0.5 - angle / CAMERA_HFOV);
    points.push({ x, dist: d });

    if (d < frameDist) { frameDist = d; frameX = x; }
  }

  // ── 2. Smooth nearest-object tracking ─────────────────────────────────────
  if (isFinite(frameDist)) {
    if (!isFinite(smoothDist)) {
      smoothDist = frameDist;
      smoothX    = frameX;
    } else {
      smoothDist = smoothDist + SMOOTH_ALPHA * (frameDist - smoothDist);
      smoothX    = smoothX    + SMOOTH_ALPHA * (frameX    - smoothX);
    }
  }

  // ── 3. Depth bars ────────────────────────────────────────────────────────
  const barWidth = Math.max(2, Math.ceil(W / 180));
  for (const p of points) {
    const normD = Math.min(p.dist / MAX_RANGE, 1);
    // Height: close → up to 60% of frame,  far → 8%
    const barH  = H * (0.08 + 0.52 * (1 - normD));
    const col   = depthColour(p.dist);
    const yTop  = (H - barH) / 2;

    // Gradient: solid at centre, transparent at top & bottom
    const alpha = BAR_ALPHA * (1 - normD * 0.4);
    const grad = ctx.createLinearGradient(0, yTop, 0, yTop + barH);
    grad.addColorStop(0,    rgba(col, 0));
    grad.addColorStop(0.35, rgba(col, alpha));
    grad.addColorStop(0.5,  rgba(col, alpha));
    grad.addColorStop(0.65, rgba(col, alpha));
    grad.addColorStop(1,    rgba(col, 0));

    ctx.fillStyle = grad;
    ctx.fillRect(p.x - barWidth / 2, yTop, barWidth, barH);
  }

  // ── 4. Thin centre depth strip ────────────────────────────────────────────
  const stripH = Math.max(1, Math.round(H * 0.008));
  const stripY = (H - stripH) / 2;
  for (const p of points) {
    const col = depthColour(p.dist);
    ctx.fillStyle = rgba(col, 0.7);
    ctx.fillRect(p.x - barWidth / 2, stripY, barWidth, stripH);
  }

  // ── 5. Proximity edge tint ────────────────────────────────────────────────
  if (smoothDist < WARN_RANGE) {
    const intensity = 0.12 * (1 - smoothDist / WARN_RANGE);
    const vg = ctx.createRadialGradient(
      W / 2, H / 2, Math.min(W, H) * 0.4,
      W / 2, H / 2, Math.max(W, H) * 0.75
    );
    vg.addColorStop(0, 'rgba(255,80,60,0)');
    vg.addColorStop(1, `rgba(255,80,60,${intensity})`);
    ctx.fillStyle = vg;
    ctx.fillRect(0, 0, W, H);
  }

  // ── 6. Nearest distance label (smoothed, only when close enough) ──────────
  if (isFinite(smoothDist) && smoothDist < NEAREST_SHOW) {
    const col   = depthColour(smoothDist);
    const label = smoothDist.toFixed(2) + ' m';
    const lx    = Math.max(30, Math.min(W - 30, smoothX));
    const ly    = H * 0.15;

    // Thin leader line
    ctx.strokeStyle = rgba(col, 0.3);
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(lx, H / 2);
    ctx.lineTo(lx, ly + 8);
    ctx.stroke();

    // Label pill
    ctx.font = '10px -apple-system, system-ui, sans-serif';
    const tw = ctx.measureText(label).width + 10;
    const pillH = 16;
    const px = lx - tw / 2;
    const py = ly - pillH / 2;

    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.beginPath();
    ctx.roundRect(px, py, tw, pillH, 3);
    ctx.fill();

    ctx.fillStyle = rgba(col, 0.9);
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, lx, ly);
  }
}
