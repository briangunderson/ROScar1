/**
 * aio-lidar-cam.js — Sci-fi HUD depth overlay on the camera feed.
 *
 * Projects 2D lidar scan points that fall within the camera's horizontal FOV
 * onto a canvas overlaid on the MJPEG camera image.  Renders depth-coloured
 * bars, a proximity-warning vignette, distance readouts, and a sweep animation.
 */

import { onAppEvent } from './aio-app.js';

// ── Tunables ────────────────────────────────────────────────────────────────
const CAMERA_HFOV  = 60 * (Math.PI / 180);   // horizontal FOV in radians (~60° Logitech)
const HALF_FOV     = CAMERA_HFOV / 2;
const MAX_RANGE    = 4.0;                     // metres — clamp display beyond this
const WARN_RANGE   = 0.6;                     // proximity-warning threshold (metres)
const CRIT_RANGE   = 0.3;                     // critical proximity threshold
const BAR_ALPHA    = 0.55;                    // base opacity of depth bars
const SWEEP_PERIOD = 3000;                    // ms for one full sweep cycle
const LABEL_COUNT  = 3;                       // max distance labels to show

let getRos;
let scanSub   = null;
let canvas    = null;
let ctx       = null;
let lastScan  = null;
let enabled   = true;
let rafId     = null;

// ── Colour ramp: close (red) → mid (amber/yellow) → far (cyan) ─────────────
function depthColour(d) {
  const t = Math.min(d / MAX_RANGE, 1);       // 0 = close, 1 = far
  let r, g, b;
  if (t < 0.25) {
    // red → orange
    const s = t / 0.25;
    r = 255; g = Math.round(80 * s); b = 0;
  } else if (t < 0.5) {
    // orange → yellow
    const s = (t - 0.25) / 0.25;
    r = 255; g = Math.round(80 + 175 * s); b = 0;
  } else if (t < 0.75) {
    // yellow → green-cyan
    const s = (t - 0.5) / 0.25;
    r = Math.round(255 * (1 - s)); g = 255; b = Math.round(100 * s);
  } else {
    // green-cyan → cyan
    const s = (t - 0.75) / 0.25;
    r = 0; g = Math.round(255 - 43 * s); b = Math.round(100 + 155 * s);
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
    throttle_rate: 100,   // 10 Hz — smoother than the radar view
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
  if (!enabled || !lastScan) {
    if (enabled) drawHUDFrame(W, H, null);
    return;
  }

  const scan = lastScan;
  const aMin = scan.angle_min;
  const aInc = scan.angle_increment;
  const n    = scan.ranges.length;
  const rMin = scan.range_min || 0.05;

  // ── 1. Extract points within camera FOV ──────────────────────────────────
  // Each point: { x: pixel column (0..W), dist: metres, angle: radians }
  const points = [];
  let minDist  = Infinity;
  let minX     = 0;

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
    points.push({ x, dist: d, angle });

    if (d < minDist) { minDist = d; minX = x; }
  }

  // ── 2. Depth bars ────────────────────────────────────────────────────────
  // Each bar is a vertical gradient stripe.  Height ∝ 1/distance (closer=taller).
  const barWidth = Math.max(2, Math.ceil(W / 160));
  for (const p of points) {
    const normD = Math.min(p.dist / MAX_RANGE, 1);
    // Height: close → up to 80% of frame,  far → 10%
    const barH  = H * (0.10 + 0.70 * (1 - normD));
    const col   = depthColour(p.dist);
    const yTop  = (H - barH) / 2;

    // Gradient: solid at centre, transparent at top & bottom
    const grad = ctx.createLinearGradient(0, yTop, 0, yTop + barH);
    grad.addColorStop(0,    rgba(col, 0));
    grad.addColorStop(0.3,  rgba(col, BAR_ALPHA * (1 - normD * 0.5)));
    grad.addColorStop(0.5,  rgba(col, BAR_ALPHA * (1 - normD * 0.3)));
    grad.addColorStop(0.7,  rgba(col, BAR_ALPHA * (1 - normD * 0.5)));
    grad.addColorStop(1,    rgba(col, 0));

    ctx.fillStyle = grad;
    ctx.fillRect(p.x - barWidth / 2, yTop, barWidth, barH);
  }

  // ── 3. Depth strip (horizontal band across the centre) ───────────────────
  const stripH = Math.max(2, Math.round(H * 0.012));
  const stripY = (H - stripH) / 2;
  for (const p of points) {
    const col = depthColour(p.dist);
    ctx.fillStyle = rgba(col, 0.9);
    ctx.fillRect(p.x - barWidth / 2, stripY, barWidth, stripH);
  }

  // ── 4. Sweep line (rotating scan effect) ─────────────────────────────────
  const sweepT  = (performance.now() % SWEEP_PERIOD) / SWEEP_PERIOD;
  const sweepX  = sweepT * W;
  const sweepW  = W * 0.08;
  const sweepGrad = ctx.createLinearGradient(sweepX - sweepW, 0, sweepX, 0);
  sweepGrad.addColorStop(0, 'rgba(0,212,255,0)');
  sweepGrad.addColorStop(1, 'rgba(0,212,255,0.08)');
  ctx.fillStyle = sweepGrad;
  ctx.fillRect(sweepX - sweepW, 0, sweepW, H);

  // ── 5. Proximity vignette ────────────────────────────────────────────────
  if (minDist < WARN_RANGE) {
    const intensity = minDist < CRIT_RANGE
      ? 0.35 + 0.15 * Math.sin(performance.now() * 0.008)   // pulsing at critical
      : 0.15 * (1 - (minDist - CRIT_RANGE) / (WARN_RANGE - CRIT_RANGE));

    // Radial vignette — red glow from edges
    const vg = ctx.createRadialGradient(W / 2, H / 2, Math.min(W, H) * 0.3,
                                        W / 2, H / 2, Math.max(W, H) * 0.75);
    vg.addColorStop(0, 'rgba(255,30,30,0)');
    vg.addColorStop(1, `rgba(255,30,30,${intensity})`);
    ctx.fillStyle = vg;
    ctx.fillRect(0, 0, W, H);
  }

  // ── 6. HUD frame + distance labels ──────────────────────────────────────
  drawHUDFrame(W, H, { points, minDist, minX });
}

// ── HUD chrome: corner brackets, FOV ticks, distance readouts ───────────────
function drawHUDFrame(W, H, data) {
  const cx = W / 2;
  ctx.save();

  // Corner brackets
  const cLen = Math.min(20, W * 0.06);
  ctx.strokeStyle = 'rgba(0,212,255,0.35)';
  ctx.lineWidth = 1;
  drawCorner(ctx, 1, 1, cLen);
  drawCorner(ctx, W - 1, 1, cLen, true, false);
  drawCorner(ctx, 1, H - 1, cLen, false, true);
  drawCorner(ctx, W - 1, H - 1, cLen, true, true);

  // Centre cross-hair tick marks
  const tickLen = 6;
  ctx.strokeStyle = 'rgba(0,212,255,0.25)';
  ctx.beginPath();
  ctx.moveTo(cx - tickLen, H / 2); ctx.lineTo(cx + tickLen, H / 2);
  ctx.moveTo(cx, H / 2 - tickLen); ctx.lineTo(cx, H / 2 + tickLen);
  ctx.stroke();

  // FOV label
  ctx.font = '9px "Share Tech Mono", monospace';
  ctx.fillStyle = 'rgba(0,212,255,0.4)';
  ctx.textAlign = 'center';
  ctx.fillText(`FOV ${Math.round(CAMERA_HFOV * 180 / Math.PI)}°`, cx, 12);

  if (!data) { ctx.restore(); return; }

  const { points, minDist, minX } = data;

  // Nearest-object callout
  if (isFinite(minDist)) {
    const col   = depthColour(minDist);
    const label = minDist.toFixed(2) + 'm';
    const ly    = H * 0.18;

    // Line from label to scan point
    ctx.strokeStyle = rgba(col, 0.6);
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(minX, H / 2);
    ctx.lineTo(minX, ly + 6);
    ctx.stroke();
    ctx.setLineDash([]);

    // Label background
    const tw = ctx.measureText(label).width + 8;
    ctx.fillStyle = 'rgba(8,17,26,0.7)';
    ctx.fillRect(minX - tw / 2, ly - 8, tw, 14);
    ctx.strokeStyle = rgba(col, 0.7);
    ctx.strokeRect(minX - tw / 2, ly - 8, tw, 14);

    // Label text
    ctx.fillStyle = rgba(col, 1);
    ctx.font = 'bold 10px "Share Tech Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText(label, minX, ly + 3);

    // "NEAREST" tag
    ctx.font = '7px "Share Tech Mono", monospace';
    ctx.fillStyle = rgba(col, 0.6);
    ctx.fillText('NEAREST', minX, ly - 11);
  }

  // Bottom distance scale
  ctx.font = '8px "Share Tech Mono", monospace';
  ctx.fillStyle = 'rgba(0,212,255,0.3)';
  ctx.textAlign = 'center';
  const scaleY = H - 6;
  for (let m = 1; m <= MAX_RANGE; m++) {
    // Find approximate pixel position for that distance at angle=0
    // (just label along the bottom as distance references)
    const frac = m / MAX_RANGE;
    ctx.fillText(m + 'm', W * 0.05 + frac * W * 0.15, scaleY);
  }

  // Side labels: L / R
  ctx.fillStyle = 'rgba(0,212,255,0.2)';
  ctx.font = '8px "Share Tech Mono", monospace';
  ctx.textAlign = 'left';
  ctx.fillText('L', 4, H / 2 + 3);
  ctx.textAlign = 'right';
  ctx.fillText('R', W - 4, H / 2 + 3);

  ctx.restore();
}

function drawCorner(ctx, x, y, len, flipX = false, flipY = false) {
  const dx = flipX ? -1 : 1;
  const dy = flipY ? -1 : 1;
  ctx.beginPath();
  ctx.moveTo(x + dx * len, y);
  ctx.lineTo(x, y);
  ctx.lineTo(x, y + dy * len);
  ctx.stroke();
}
