/**
 * aio-graphs.js — Rolling 60-second sparkline telemetry charts.
 * Renders vx, vy, wz (from odometry) and battery voltage on canvas elements.
 * Data arrives via callback hooks exported by aio-status.js — no duplicate
 * ROS subscriptions.
 */

import { onOdomData, onBatteryData } from './aio-status.js';

// ── Configuration ────────────────────────────────────────────────────────────
const WINDOW_SEC     = 60;      // visible time window (seconds)
const VEL_MAX_POINTS = 300;     // ~5 Hz × 60 s
const BATT_MAX_POINTS = 60;     // ~1 Hz × 60 s
const LINE_WIDTH     = 2;
const FILL_ALPHA     = 0.10;
const Y_PAD_FACTOR   = 0.10;    // 10 % padding above/below data range
const VEL_COLOR      = '#00D4FF';
const BATT_COLOR     = '#00E676';
const GRID_COLOR     = 'rgba(255,255,255,0.08)';
const ZERO_COLOR     = 'rgba(255,255,255,0.18)';
const NO_DATA_COLOR  = 'rgba(255,255,255,0.25)';
const TOOLTIP_BG     = 'rgba(0,0,0,0.80)';
const TOOLTIP_FG     = '#e0e0e0';
const FONT           = '11px Share Tech Mono, monospace';

// ── Circular buffer ──────────────────────────────────────────────────────────
class RingBuffer {
  constructor(capacity) {
    this.cap   = capacity;
    this.buf   = new Array(capacity);
    this.head  = 0;   // next write index
    this.count = 0;
  }

  push(entry) {
    this.buf[this.head] = entry;
    this.head = (this.head + 1) % this.cap;
    if (this.count < this.cap) this.count++;
  }

  /** Iterate oldest → newest. */
  forEach(fn) {
    if (this.count === 0) return;
    const start = (this.head - this.count + this.cap) % this.cap;
    for (let i = 0; i < this.count; i++) {
      fn(this.buf[(start + i) % this.cap], i);
    }
  }

  newest() {
    if (this.count === 0) return null;
    return this.buf[(this.head - 1 + this.cap) % this.cap];
  }
}

// ── Per-metric state ─────────────────────────────────────────────────────────
function makeSeries(canvasId, capacity, color, unit, decimals) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) {
    console.warn(`aio-graphs: canvas #${canvasId} not found`);
    return null;
  }
  // Find the sibling label inside the same .sparkline-card
  const card  = canvas.closest('.sparkline-card');
  const label = card?.querySelector('.sparkline-label');

  return {
    canvas,
    ctx: canvas.getContext('2d'),
    buf: new RingBuffer(capacity),
    color,
    unit,
    decimals,
    label,
    baseLabelText: label ? label.textContent.trim() : '',
    hoverIdx: -1,  // index into visible slice for tooltip
  };
}

// ── Module state ─────────────────────────────────────────────────────────────
let series    = [];  // array of series objects (vx, vy, wz, batt)
let rafId     = null;
let resizeObs = null;

// ── Sizing helper ────────────────────────────────────────────────────────────
function sizeCanvas(s) {
  const rect = s.canvas.parentElement.getBoundingClientRect();
  const dpr  = window.devicePixelRatio || 1;
  const w    = Math.round(rect.width);
  const h    = Math.round(rect.height);
  if (s.canvas.width !== w * dpr || s.canvas.height !== h * dpr) {
    s.canvas.width  = w * dpr;
    s.canvas.height = h * dpr;
    s.canvas.style.width  = w + 'px';
    s.canvas.style.height = h + 'px';
    s.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  return { w, h };
}

// ── Render one sparkline ─────────────────────────────────────────────────────
function drawSeries(s, now) {
  const { w, h } = sizeCanvas(s);
  const ctx = s.ctx;

  ctx.clearRect(0, 0, w, h);

  // No data — centred message
  if (s.buf.count === 0) {
    ctx.fillStyle = NO_DATA_COLOR;
    ctx.font = FONT;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('NO DATA', w / 2, h / 2);
    updateLabel(s, null);
    return;
  }

  // Collect visible points
  const tMin = now - WINDOW_SEC * 1000;
  const pts  = [];
  let vMin = Infinity, vMax = -Infinity;

  s.buf.forEach((entry) => {
    if (entry.time >= tMin) {
      pts.push(entry);
      if (entry.value < vMin) vMin = entry.value;
      if (entry.value > vMax) vMax = entry.value;
    }
  });

  if (pts.length === 0) {
    ctx.fillStyle = NO_DATA_COLOR;
    ctx.font = FONT;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('NO DATA', w / 2, h / 2);
    updateLabel(s, null);
    return;
  }

  // Y-axis auto-scale with padding
  const isVelocity = s.unit === 'm/s' || s.unit === 'r/s';
  let range = vMax - vMin || 0.01;

  // For velocity, include 0 when range crosses it
  if (isVelocity) {
    if (vMin > 0) vMin = 0;
    if (vMax < 0) vMax = 0;
    range = vMax - vMin || 0.01;
  }

  const pad = range * Y_PAD_FACTOR;
  vMin -= pad;
  vMax += pad;
  range = vMax - vMin;

  const mapX = (t) => ((t - tMin) / (WINDOW_SEC * 1000)) * w;
  const mapY = (v) => h - ((v - vMin) / range) * h;

  // Horizontal zero line for velocity
  if (isVelocity && vMin < 0 && vMax > 0) {
    const y0 = mapY(0);
    ctx.strokeStyle = ZERO_COLOR;
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, y0);
    ctx.lineTo(w, y0);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Build path
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const x = mapX(pts[i].time);
    const y = mapY(pts[i].value);
    if (i === 0) ctx.moveTo(x, y);
    else         ctx.lineTo(x, y);
  }

  // Stroke
  ctx.strokeStyle = s.color;
  ctx.lineWidth   = LINE_WIDTH;
  ctx.lineJoin    = 'round';
  ctx.stroke();

  // Fill under the line
  const lastPt = pts[pts.length - 1];
  ctx.lineTo(mapX(lastPt.time), h);
  ctx.lineTo(mapX(pts[0].time), h);
  ctx.closePath();
  ctx.fillStyle = s.color.replace(')', `, ${FILL_ALPHA})`).replace('rgb', 'rgba');
  // For hex colors, convert
  ctx.fillStyle = hexAlpha(s.color, FILL_ALPHA);
  ctx.fill();

  // Tooltip on hover
  if (s.hoverIdx >= 0 && s.hoverIdx < pts.length) {
    const p  = pts[s.hoverIdx];
    const px = mapX(p.time);
    const py = mapY(p.value);

    // Crosshair
    ctx.strokeStyle = 'rgba(255,255,255,0.3)';
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 3]);
    ctx.beginPath();
    ctx.moveTo(px, 0); ctx.lineTo(px, h);
    ctx.stroke();
    ctx.setLineDash([]);

    // Dot
    ctx.beginPath();
    ctx.arc(px, py, 3, 0, Math.PI * 2);
    ctx.fillStyle = s.color;
    ctx.fill();

    // Value label
    const txt = p.value.toFixed(s.decimals) + ' ' + s.unit;
    ctx.font      = FONT;
    ctx.textAlign = px > w / 2 ? 'right' : 'left';
    ctx.textBaseline = py > h / 2 ? 'bottom' : 'top';
    const tx = px + (px > w / 2 ? -6 : 6);
    const ty = py + (py > h / 2 ? -6 : 6);

    // Background
    const m = ctx.measureText(txt);
    const pad2 = 3;
    const bx = ctx.textAlign === 'right' ? tx - m.width - pad2 : tx - pad2;
    ctx.fillStyle = TOOLTIP_BG;
    ctx.fillRect(bx, ty - (ctx.textBaseline === 'bottom' ? 14 : 0) - pad2,
                 m.width + pad2 * 2, 14 + pad2 * 2);

    ctx.fillStyle = TOOLTIP_FG;
    ctx.fillText(txt, tx, ty);
  }

  // Store pts on series for hover lookup
  s._visPts = pts;
  s._mapX   = mapX;

  // Update label with latest value
  updateLabel(s, lastPt.value);
}

// ── Label update ─────────────────────────────────────────────────────────────
function updateLabel(s, value) {
  if (!s.label) return;
  if (value === null || value === undefined) {
    s.label.textContent = s.baseLabelText;
  } else {
    s.label.textContent = `${s.baseLabelText}  ${value.toFixed(s.decimals)}${s.unit}`;
  }
}

// ── Hex color → rgba with alpha ──────────────────────────────────────────────
function hexAlpha(hex, alpha) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  return `rgba(${r},${g},${b},${alpha})`;
}

// ── Hover logic ──────────────────────────────────────────────────────────────
function attachHover(s) {
  s.canvas.addEventListener('mousemove', (e) => {
    if (!s._visPts || s._visPts.length === 0) { s.hoverIdx = -1; return; }
    const rect = s.canvas.getBoundingClientRect();
    const mx   = e.clientX - rect.left;
    let best = -1, bestD = Infinity;
    for (let i = 0; i < s._visPts.length; i++) {
      const px = s._mapX(s._visPts[i].time);
      const d  = Math.abs(px - mx);
      if (d < bestD) { bestD = d; best = i; }
    }
    s.hoverIdx = bestD < 30 ? best : -1;
  });

  s.canvas.addEventListener('mouseleave', () => {
    s.hoverIdx = -1;
  });
}

// ── RAF loop ─────────────────────────────────────────────────────────────────
function renderLoop() {
  const now = performance.now();
  for (const s of series) {
    if (s) drawSeries(s, now);
  }
  rafId = requestAnimationFrame(renderLoop);
}

// ── Public init ──────────────────────────────────────────────────────────────
export function initGraphs() {
  const vx   = makeSeries('spark-vx',   VEL_MAX_POINTS,  VEL_COLOR,  'm/s', 3);
  const vy   = makeSeries('spark-vy',   VEL_MAX_POINTS,  VEL_COLOR,  'm/s', 3);
  const wz   = makeSeries('spark-wz',   VEL_MAX_POINTS,  VEL_COLOR,  'r/s', 3);
  const batt = makeSeries('spark-batt', BATT_MAX_POINTS, BATT_COLOR, 'V',   1);

  series = [vx, vy, wz, batt].filter(Boolean);

  if (series.length === 0) {
    console.warn('aio-graphs: no sparkline canvases found');
    return;
  }

  // ResizeObserver to keep canvases correctly sized
  resizeObs = new ResizeObserver(() => {
    for (const s of series) sizeCanvas(s);
  });
  for (const s of series) {
    resizeObs.observe(s.canvas.parentElement);
  }

  // Attach hover handlers
  for (const s of series) attachHover(s);

  // Odom callback — ~10 Hz from rosbridge throttle, we store all of them
  onOdomData((_pos, vel, _yaw) => {
    const t = performance.now();
    if (vx)  vx.buf.push({ time: t, value: vel.linear.x });
    if (vy)  vy.buf.push({ time: t, value: vel.linear.y });
    if (wz)  wz.buf.push({ time: t, value: vel.angular.z });
  });

  // Battery callback — ~1 Hz
  onBatteryData((voltage, _pct) => {
    if (batt) batt.buf.push({ time: performance.now(), value: voltage });
  });

  // Start render loop
  rafId = requestAnimationFrame(renderLoop);
}
