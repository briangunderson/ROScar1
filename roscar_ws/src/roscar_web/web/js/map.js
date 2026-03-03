/**
 * map.js — OccupancyGrid renderer + robot pose overlay + click-to-navigate.
 * Subscribes to /map (OccupancyGrid) and /odometry/filtered (robot pose).
 * Supports pan (drag), zoom (buttons), and sends Nav2 goals via action.
 */

import { onAppEvent, HOST, PORTS, toast } from './app.js';

let getRos;
let mapSub   = null;
let poseSub  = null;
let mapData  = null;    // {width, height, resolution, origin, data[]}
let robotPos = null;    // {x, y, yaw}

// View transform (canvas pixels per map pixel)
let viewScale  = 1.0;
let viewOffset = { x: 0, y: 0 };
let dragging   = false;
let dragStart  = { x: 0, y: 0, ox: 0, oy: 0 };

let navGoalMode = false;
let goalActionClient = null;
let goalHandle      = null;

export function initMap(getRosFn) {
  getRos = getRosFn;
  setupControls();
  onAppEvent((ev, tab) => {
    if (ev === 'connected') { subscribeMap(); subscribePose(); setupGoalClient(); }
    if (ev === 'tabchange' && tab === 'map') { sizeCanvas(); drawMap(); }
  });
  window.addEventListener('resize', () => { sizeCanvas(); drawMap(); });
}

// ── Subscriptions ──────────────────────────────────────────────────────────
function subscribeMap() {
  const ros = getRos(); if (!ros) return;
  if (mapSub) { try { mapSub.unsubscribe(); } catch(_) {} }
  mapSub = new ROSLIB.Topic({
    ros, name: '/map', messageType: 'nav_msgs/OccupancyGrid',
    throttle_rate: 2000,  // 0.5 Hz — maps update slowly
  });
  mapSub.subscribe((msg) => {
    mapData = {
      width: msg.info.width, height: msg.info.height,
      resolution: msg.info.resolution,
      origin: msg.info.origin.position,
      data: msg.data,
    };
    setEl('map-size-disp', `${msg.info.width}×${msg.info.height}`);
    setEl('map-res-disp',  `${(msg.info.resolution * 100).toFixed(0)}cm`);
    fitMapToCanvas();
    drawMap();
  });
}

function subscribePose() {
  const ros = getRos(); if (!ros) return;
  if (poseSub) { try { poseSub.unsubscribe(); } catch(_) {} }
  poseSub = new ROSLIB.Topic({
    ros, name: '/odometry/filtered', messageType: 'nav_msgs/Odometry',
    throttle_rate: 200,
  });
  poseSub.subscribe((msg) => {
    const q = msg.pose.pose.orientation;
    const p = msg.pose.pose.position;
    robotPos = {
      x: p.x, y: p.y,
      yaw: Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)),
    };
    drawMap();
  });
}

function setupGoalClient() {
  const ros = getRos(); if (!ros) return;
  goalActionClient = new ROSLIB.ActionClient({
    ros, serverName: '/navigate_to_pose',
    actionName: 'nav2_msgs/action/NavigateToPose',
  });
}

// ── Canvas helpers ─────────────────────────────────────────────────────────
function getCanvas() { return document.getElementById('map-canvas'); }

function sizeCanvas() {
  const c = getCanvas(); if (!c) return;
  const rect = c.getBoundingClientRect();
  c.width  = rect.width;
  c.height = rect.height;
}

function fitMapToCanvas() {
  if (!mapData) return;
  const c = getCanvas(); if (!c) return;
  // After 90° CCW rotation: map height spans screen X, map width spans screen Y
  const scaleX = c.width  / mapData.height;
  const scaleY = c.height / mapData.width;
  viewScale = Math.min(scaleX, scaleY) * 0.9;
  viewOffset.x = (c.width  - mapData.height * viewScale) / 2;
  viewOffset.y = (c.height - mapData.width  * viewScale) / 2;
}

// ── Drawing ────────────────────────────────────────────────────────────────
function drawMap() {
  const c = getCanvas(); if (!c) return;
  const ctx = c.getContext('2d');
  ctx.clearRect(0, 0, c.width, c.height);

  if (!mapData) {
    ctx.fillStyle = 'rgba(0,212,255,0.15)';
    ctx.font = '600 13px Rajdhani, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('NO MAP DATA — launch SLAM or NAV mode', c.width/2, c.height/2);
    return;
  }

  // Render OccupancyGrid as ImageData
  const imgData = ctx.createImageData(mapData.width, mapData.height);
  const d = imgData.data;
  for (let i = 0; i < mapData.data.length; i++) {
    const v = mapData.data[i];
    const base = i * 4;
    if (v === -1) {       // unknown
      d[base]   = 13; d[base+1] = 34; d[base+2] = 51; d[base+3] = 255;
    } else if (v === 0) { // free
      d[base]   = 8;  d[base+1] = 17; d[base+2] = 26; d[base+3] = 255;
    } else {              // occupied (0-100)
      const bright = Math.floor(180 * v / 100);
      d[base]   = bright; d[base+1] = bright; d[base+2] = bright; d[base+3] = 255;
    }
  }

  // Draw via offscreen canvas for scaling
  const offscreen = document.createElement('canvas');
  offscreen.width  = mapData.width;
  offscreen.height = mapData.height;
  offscreen.getContext('2d').putImageData(imgData, 0, 0);

  ctx.save();
  ctx.translate(viewOffset.x, viewOffset.y);
  ctx.scale(viewScale, viewScale);
  // 90° CCW rotation: forward (ROS +X) → screen UP, left (ROS +Y) → screen LEFT
  ctx.translate(mapData.height, mapData.width);
  ctx.rotate(Math.PI / 2);
  ctx.scale(-1, 1);
  ctx.drawImage(offscreen, 0, 0);
  ctx.restore();

  // Robot pose overlay
  if (robotPos && mapData) {
    drawRobotPose(ctx);
  }
}

function drawRobotPose(ctx) {
  // Convert world coords to canvas coords (90° CCW rotated view)
  const ox  = mapData.origin.x;
  const oy  = mapData.origin.y;
  const res = mapData.resolution;
  const w   = mapData.width;
  const h   = mapData.height;

  // Map pixel coords
  const mpx = (robotPos.x - ox) / res;
  const mpy = (robotPos.y - oy) / res;

  // Screen coords: forward(+mpx)→UP, left(+mpy)→LEFT
  const sx = viewOffset.x + (h - mpy) * viewScale;
  const sy = viewOffset.y + (w - mpx) * viewScale;

  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-robotPos.yaw);

  // Arrow shape
  ctx.fillStyle = '#FF8C00';
  ctx.strokeStyle = '#FF8C00';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, -10);
  ctx.lineTo(6, 6);
  ctx.lineTo(0, 3);
  ctx.lineTo(-6, 6);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // Position ring
  ctx.save();
  ctx.strokeStyle = 'rgba(255,140,0,0.35)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(sx, sy, 14, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();
}

// ── Controls ───────────────────────────────────────────────────────────────
function setupControls() {
  document.getElementById('map-zoom-in').addEventListener('click', () => {
    viewScale *= 1.25; drawMap();
  });
  document.getElementById('map-zoom-out').addEventListener('click', () => {
    viewScale = Math.max(0.5, viewScale / 1.25); drawMap();
  });
  document.getElementById('map-reset').addEventListener('click', () => {
    fitMapToCanvas(); drawMap();
  });
  document.getElementById('cancel-goal-btn').addEventListener('click', cancelGoal);

  const c = getCanvas();
  if (!c) return;

  // Pan
  c.addEventListener('mousedown', (e) => {
    if (navGoalMode) return;
    dragging = true;
    dragStart = { x: e.clientX, y: e.clientY, ox: viewOffset.x, oy: viewOffset.y };
  });
  c.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    viewOffset.x = dragStart.ox + (e.clientX - dragStart.x);
    viewOffset.y = dragStart.oy + (e.clientY - dragStart.y);
    drawMap();
  });
  c.addEventListener('mouseup', () => { dragging = false; });
  c.addEventListener('mouseleave', () => { dragging = false; });

  // Scroll zoom
  c.addEventListener('wheel', (e) => {
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.1 : 0.9;
    viewScale = Math.max(0.5, viewScale * factor);
    drawMap();
  }, { passive: false });

  // Nav goal on click
  c.addEventListener('click', (e) => {
    if (!navGoalMode || !mapData) return;
    const rect = c.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    sendNavGoal(cx, cy);
  });
}

function sendNavGoal(canvasX, canvasY) {
  if (!goalActionClient) { toast('Nav2 not connected', 'err'); return; }

  // Canvas → map pixel → world (inverse of 90° CCW rotated view)
  const mpx = mapData.width  - (canvasY - viewOffset.y) / viewScale;
  const mpy = mapData.height - (canvasX - viewOffset.x) / viewScale;
  const wx  = mapData.origin.x + mpx * mapData.resolution;
  const wy  = mapData.origin.y + mpy * mapData.resolution;

  const goal = new ROSLIB.ActionGoal({
    goal: {
      pose: {
        header: { frame_id: 'map' },
        pose: {
          position: { x: wx, y: wy, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
    },
  });

  goalHandle = goalActionClient.sendGoal(
    goal,
    (result) => {
      toast('Goal reached!', 'ok');
      setNavGoalMode(false);
    },
    null,
    (feedback) => { /* could show progress */ },
  );

  toast(`Nav goal: (${wx.toFixed(2)}, ${wy.toFixed(2)})`, 'ok');
  setNavGoalMode(false);
}

function cancelGoal() {
  if (goalHandle) { try { goalHandle.cancel(); } catch(_) {} goalHandle = null; }
  setNavGoalMode(false);
  toast('Goal cancelled');
}

function setNavGoalMode(on) {
  navGoalMode = on;
  const hint = document.getElementById('nav-goal-hint');
  const btn  = document.getElementById('cancel-goal-btn');
  if (hint) hint.classList.toggle('hidden', !on);
  if (btn)  btn.classList.toggle('hidden', !on);
}

function setEl(id, val) {
  const el = document.getElementById(id); if (el) el.textContent = val;
}

// Expose goal mode toggle (called from modes.js when Nav mode active)
export function enableNavGoalMode() { setNavGoalMode(true); }
