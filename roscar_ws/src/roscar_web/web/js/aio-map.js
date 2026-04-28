/**
 * aio-map.js — OccupancyGrid renderer + robot pose overlay + click-to-navigate.
 * Always active (no tab gating). Subscribes to /map for the grid, uses TFClient
 * (map→odom→base_footprint) for SLAM-corrected pose, falls back to /odometry/filtered.
 * Supports pan (drag), zoom (buttons + scroll), Nav2 goal via action client.
 *
 * Two view modes:
 *   FREE   — manual pan/zoom, robot arrow moves on the map
 *   LOCKED — robot fixed at canvas center facing up, map rotates/pans underneath
 */

import { onAppEvent, toast } from './aio-app.js';

let getRos;
let mapSub   = null;
let poseSub  = null;
let tfSub = null;       // /tf subscriber (replaces TFClient which needs tf2_web_republisher)
let mapToOdom = null;   // {tx, ty, yaw} — latest map→odom transform from slam_toolbox
let lastTFTime = 0;     // timestamp of last map→odom TF update
const TF_STALE_MS = 5000; // ignore map→odom if stale
let mapData  = null;    // {width, height, resolution, origin, data[]}
let robotPos = null;    // {x, y, yaw}

// View transform (canvas pixels per map pixel)
let viewScale  = 1.0;
let viewOffset = { x: 0, y: 0 };
let dragging   = false;
let dragStart  = { x: 0, y: 0, ox: 0, oy: 0 };

// Robot-locked mode: robot at center, map rotates with heading
let robotLocked = false;

let navGoalMode = false;
let goalActionClient = null;
let goalHandle      = null;

// AMCL initial-pose seeding: when initPoseMode is true, a click publishes
// to /initialpose instead of sending a nav goal. Click-and-drag sets
// orientation (drag direction = robot heading). Mutually exclusive with
// navGoalMode.
let initPoseMode = false;
let initialPoseTopic = null;
let initPoseDrag = null;  // {x0, y0, x1, y1} canvas coords during drag

// Landmark markers from /landmark/known_markers
let landmarkSub = null;
let knownMarkers = [];  // [{id, x, y, yaw, visible}]

let rafId = null;
let needsDraw = true;

// ── Public API ──────────────────────────────────────────────────────────────
export function initMap(getRosFn) {
  getRos = getRosFn;
  setupControls();
  setupResizeObserver();
  onAppEvent((ev) => {
    if (ev === 'connected') { subscribeMap(); subscribePose(); subscribeTF(); subscribeLandmarks(); }
  });
  startRAFLoop();
}

export function enableNavGoalMode() { setNavGoalMode(true); }

/** Clear local map data so the canvas shows the "NO MAP DATA" placeholder. */
export function clearMap() {
  mapData = null;
  robotPos = null;
  mapToOdom = null;
  lastTFTime = 0;
  needsDraw = true;
}

// ── Subscriptions ───────────────────────────────────────────────────────────
function subscribeMap() {
  const ros = getRos(); if (!ros) return;
  if (mapSub) { try { mapSub.unsubscribe(); } catch (_) {} }
  mapSub = new ROSLIB.Topic({
    ros, name: '/map', messageType: 'nav_msgs/OccupancyGrid',
    throttle_rate: 1000,
  });
  let prevW = 0, prevH = 0;
  let prevOriginX = null, prevOriginY = null;
  mapSub.subscribe((msg) => {
    const w = msg.info.width, h = msg.info.height;
    const ox = msg.info.origin.position.x, oy = msg.info.origin.position.y;
    // Copy the data array so we always hold a fresh snapshot
    mapData = {
      width: w, height: h,
      resolution: msg.info.resolution,
      origin: msg.info.origin.position,
      data: Array.from(msg.data),
    };
    setEl('map-size-disp', `${w}\u00d7${h}`);
    setEl('map-res-disp',  `${(msg.info.resolution * 100).toFixed(0)}cm`);
    // Re-fit view when map dimensions or origin change (grid expanded by SLAM)
    if (w !== prevW || h !== prevH || ox !== prevOriginX || oy !== prevOriginY) {
      if (!robotLocked) fitMapToCanvas();
      prevW = w; prevH = h; prevOriginX = ox; prevOriginY = oy;
    }
    needsDraw = true;
  });
}

function subscribePose() {
  const ros = getRos(); if (!ros) return;
  if (poseSub) { try { poseSub.unsubscribe(); } catch (_) {} }
  poseSub = new ROSLIB.Topic({
    ros, name: '/odometry/filtered', messageType: 'nav_msgs/Odometry',
    throttle_rate: 200,
  });
  poseSub.subscribe((msg) => {
    const q = msg.pose.pose.orientation;
    const p = msg.pose.pose.position;
    const odomYaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

    // Apply map→odom transform if available and fresh
    if (mapToOdom && (Date.now() - lastTFTime) < TF_STALE_MS) {
      // Transform odom-frame pose into map frame:
      // map_pos = R(map→odom_yaw) * odom_pos + map→odom_translation
      const cos = Math.cos(mapToOdom.yaw);
      const sin = Math.sin(mapToOdom.yaw);
      robotPos = {
        x: cos * p.x - sin * p.y + mapToOdom.tx,
        y: sin * p.x + cos * p.y + mapToOdom.ty,
        yaw: odomYaw + mapToOdom.yaw,
      };
    } else {
      // No map→odom TF (teleop-only mode): use odom frame directly
      robotPos = { x: p.x, y: p.y, yaw: odomYaw };
    }
    needsDraw = true;
  });
}

function subscribeTF() {
  // Subscribe directly to /tf and extract the map→odom transform.
  // This avoids needing tf2_web_republisher (which has Jazzy compat issues).
  const ros = getRos(); if (!ros) return;
  if (tfSub) { try { tfSub.unsubscribe(); } catch (_) {} }
  tfSub = new ROSLIB.Topic({
    ros, name: '/tf', messageType: 'tf2_msgs/TFMessage',
    throttle_rate: 100,  // 10Hz is plenty for map→odom
  });
  tfSub.subscribe((msg) => {
    for (const t of msg.transforms) {
      if (t.header.frame_id === 'map' && t.child_frame_id === 'odom') {
        const tr = t.transform.translation;
        const q = t.transform.rotation;
        const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        mapToOdom = { tx: tr.x, ty: tr.y, yaw };
        lastTFTime = Date.now();
        needsDraw = true;
        break;
      }
    }
  });
}

function subscribeLandmarks() {
  const ros = getRos(); if (!ros) return;
  if (landmarkSub) { try { landmarkSub.unsubscribe(); } catch (_) {} }
  landmarkSub = new ROSLIB.Topic({
    ros, name: '/landmark/known_markers', messageType: 'std_msgs/String',
    throttle_rate: 2000,
  });
  landmarkSub.subscribe((msg) => {
    try {
      knownMarkers = JSON.parse(msg.data);
      needsDraw = true;
    } catch (_) {}
  });
}

function setupGoalClient() {
  const ros = getRos(); if (!ros) return;
  // Bundled roslibjs (web/js/lib/roslib.min.js) exposes ActionClient + Goal
  // (legacy API). Use those — ROSLIB.Action / .ActionGoal don't exist here.
  goalActionClient = new ROSLIB.ActionClient({
    ros,
    serverName: '/navigate_to_pose',
    actionName: 'nav2_msgs/action/NavigateToPose',
  });
}

function setupInitialPoseTopic() {
  const ros = getRos(); if (!ros) return;
  initialPoseTopic = new ROSLIB.Topic({
    ros, name: '/initialpose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped',
  });
}

// ── Canvas helpers ──────────────────────────────────────────────────────────
function getCanvas() { return document.getElementById('map-canvas'); }

function sizeCanvas() {
  const c = getCanvas(); if (!c) return;
  const rect = c.getBoundingClientRect();
  c.width  = rect.width;
  c.height = rect.height;
  needsDraw = true;
}

function setupResizeObserver() {
  const c = getCanvas(); if (!c) return;
  if (typeof ResizeObserver !== 'undefined') {
    new ResizeObserver(() => { sizeCanvas(); }).observe(c);
  } else {
    window.addEventListener('resize', () => { sizeCanvas(); });
  }
  // Initial size
  sizeCanvas();
}

function fitMapToCanvas() {
  if (!mapData) return;
  const c = getCanvas(); if (!c) return;
  // After 90deg CCW rotation: map height spans screen X, map width spans screen Y
  const scaleX = c.width  / mapData.height;
  const scaleY = c.height / mapData.width;
  viewScale = Math.min(scaleX, scaleY) * 0.9;
  viewOffset.x = (c.width  - mapData.height * viewScale) / 2;
  viewOffset.y = (c.height - mapData.width  * viewScale) / 2;
}

// ── Robot-to-screen helpers (shared by draw + nav goal) ─────────────────────
/** Map-pixel coords for the robot in the raw OccupancyGrid image. */
function robotMapPixel() {
  if (!robotPos || !mapData) return null;
  return {
    col: (robotPos.x - mapData.origin.x) / mapData.resolution,
    row: (robotPos.y - mapData.origin.y) / mapData.resolution,
  };
}

/**
 * In locked mode, compute viewOffset so the robot (after the 90° CCW map
 * rotation + scale) sits exactly at canvas center.  Called every frame.
 */
function computeLockedOffset(c) {
  const rp = robotMapPixel();
  if (!rp) return;
  // In the 90° CCW rotated + scaled space (before viewOffset):
  //   screenX_local = (h - row) * viewScale
  //   screenY_local = (w - col) * viewScale
  const ccx = c.width  / 2;
  const ccy = c.height / 2;
  viewOffset.x = ccx - (mapData.height - rp.row) * viewScale;
  viewOffset.y = ccy - (mapData.width  - rp.col) * viewScale;
}

// ── RAF loop (always running) ───────────────────────────────────────────────
function startRAFLoop() {
  function frame() {
    if (needsDraw) { drawMap(); needsDraw = false; }
    rafId = requestAnimationFrame(frame);
  }
  rafId = requestAnimationFrame(frame);
}

// ── Drawing ─────────────────────────────────────────────────────────────────
function drawMap() {
  const c = getCanvas(); if (!c) return;
  const ctx = c.getContext('2d');
  ctx.clearRect(0, 0, c.width, c.height);

  if (!mapData) {
    ctx.fillStyle = 'rgba(0,212,255,0.15)';
    ctx.font = '600 13px Rajdhani, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('NO MAP DATA \u2014 launch SLAM or NAV mode', c.width / 2, c.height / 2);
    return;
  }

  // In locked mode, recompute viewOffset each frame to track the robot
  if (robotLocked) computeLockedOffset(c);

  // Render OccupancyGrid as ImageData
  const imgData = ctx.createImageData(mapData.width, mapData.height);
  const d = imgData.data;
  for (let i = 0; i < mapData.data.length; i++) {
    const v = mapData.data[i];
    const base = i * 4;
    if (v === -1) {       // unknown
      d[base] = 13; d[base + 1] = 34; d[base + 2] = 51; d[base + 3] = 255;
    } else if (v === 0) { // free
      d[base] = 8;  d[base + 1] = 17; d[base + 2] = 26; d[base + 3] = 255;
    } else {              // occupied (1-100)
      const bright = Math.floor(180 * v / 100);
      d[base] = bright; d[base + 1] = bright; d[base + 2] = bright; d[base + 3] = 255;
    }
  }

  // Draw via offscreen canvas for scaling
  const offscreen = document.createElement('canvas');
  offscreen.width  = mapData.width;
  offscreen.height = mapData.height;
  offscreen.getContext('2d').putImageData(imgData, 0, 0);

  ctx.save();

  // In locked mode: rotate entire scene around canvas center by +yaw
  // so the robot heading (which draws at -yaw) nets to 0 = pointing up
  if (robotLocked && robotPos) {
    const ccx = c.width / 2, ccy = c.height / 2;
    ctx.translate(ccx, ccy);
    ctx.rotate(robotPos.yaw);
    ctx.translate(-ccx, -ccy);
  }

  ctx.translate(viewOffset.x, viewOffset.y);
  ctx.scale(viewScale, viewScale);
  // 90deg CCW rotation: forward (ROS +X) -> screen UP, left (ROS +Y) -> screen LEFT
  ctx.translate(mapData.height, mapData.width);
  ctx.rotate(Math.PI / 2);
  ctx.scale(-1, 1);
  ctx.drawImage(offscreen, 0, 0);
  ctx.restore();

  // Robot pose overlay
  if (robotPos && mapData) {
    if (robotLocked) {
      drawRobotPoseLocked(ctx, c);
    } else {
      drawRobotPose(ctx);
    }
  }

  // Landmark markers
  if (knownMarkers.length > 0 && mapData) {
    if (robotLocked && robotPos) {
      drawLandmarksLocked(ctx, c);
    } else {
      drawLandmarks(ctx);
    }
  }

  // Init-pose drag indicator (arrow from click point to current cursor)
  if (initPoseDrag) {
    const { x0, y0, x1, y1 } = initPoseDrag;
    const dlen = Math.hypot(x1 - x0, y1 - y0);
    ctx.save();
    ctx.strokeStyle = '#00ffaa';
    ctx.fillStyle   = '#00ffaa';
    ctx.lineWidth   = 2;
    // Origin dot
    ctx.beginPath();
    ctx.arc(x0, y0, 6, 0, Math.PI * 2);
    ctx.fill();
    if (dlen >= 10) {
      // Arrow shaft
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
      // Arrowhead
      const a = Math.atan2(y1 - y0, x1 - x0);
      const ah = 12;
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x1 - ah * Math.cos(a - 0.4), y1 - ah * Math.sin(a - 0.4));
      ctx.lineTo(x1 - ah * Math.cos(a + 0.4), y1 - ah * Math.sin(a + 0.4));
      ctx.closePath();
      ctx.fill();
    }
    ctx.restore();
  }
}

/** Free mode: robot drawn at computed screen position, arrow rotated by heading. */
function drawRobotPose(ctx) {
  const ox  = mapData.origin.x;
  const oy  = mapData.origin.y;
  const res = mapData.resolution;
  const w   = mapData.width;
  const h   = mapData.height;

  const mpx = (robotPos.x - ox) / res;
  const mpy = (robotPos.y - oy) / res;

  // Skip drawing if robot is wildly outside the map grid (bad TF or odom data)
  const margin = 50; // pixels of grace beyond map edges
  if (mpx < -margin || mpx > w + margin || mpy < -margin || mpy > h + margin) {
    return;
  }

  const sx = viewOffset.x + (h - mpy) * viewScale;
  const sy = viewOffset.y + (w - mpx) * viewScale;

  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-robotPos.yaw);
  drawArrow(ctx);
  ctx.restore();

  drawRing(ctx, sx, sy);
}

/** Locked mode: robot fixed at canvas center, arrow always points up. */
function drawRobotPoseLocked(ctx, c) {
  const ccx = c.width / 2, ccy = c.height / 2;

  ctx.save();
  ctx.translate(ccx, ccy);
  // No rotation — arrow already points up
  drawArrow(ctx);
  ctx.restore();

  drawRing(ctx, ccx, ccy);
}

/** Shared arrow shape (points up from local origin). */
function drawArrow(ctx) {
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
}

/** Shared position ring. */
function drawRing(ctx, sx, sy) {
  ctx.save();
  ctx.strokeStyle = 'rgba(255,140,0,0.35)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(sx, sy, 14, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();
}

// ── Landmark marker drawing ──────────────────────────────────────────────────
/** Convert a map-frame (x, y) to screen coords in free mode. */
function mapToScreen(mx, my) {
  if (!mapData) return null;
  const col = (mx - mapData.origin.x) / mapData.resolution;
  const row = (my - mapData.origin.y) / mapData.resolution;
  // Same transform as robot pose: 90° CCW rotated view
  const sx = viewOffset.x + (mapData.height - row) * viewScale;
  const sy = viewOffset.y + (mapData.width  - col) * viewScale;
  return { x: sx, y: sy };
}

/** Draw all known landmark markers in free mode. */
function drawLandmarks(ctx) {
  for (const m of knownMarkers) {
    const sp = mapToScreen(m.x, m.y);
    if (!sp) continue;
    drawMarkerIcon(ctx, sp.x, sp.y, m.id, m.visible);
  }
}

/** Draw all known landmark markers in locked mode. */
function drawLandmarksLocked(ctx, c) {
  const ccx = c.width / 2, ccy = c.height / 2;
  for (const m of knownMarkers) {
    const sp = mapToScreen(m.x, m.y);
    if (!sp) continue;
    // Apply the locked-mode rotation around canvas center
    const dx = sp.x - ccx, dy = sp.y - ccy;
    const cosA = Math.cos(robotPos.yaw), sinA = Math.sin(robotPos.yaw);
    const rx = dx * cosA - dy * sinA + ccx;
    const ry = dx * sinA + dy * cosA + ccy;
    drawMarkerIcon(ctx, rx, ry, m.id, m.visible);
  }
}

/** Draw a single landmark marker icon: diamond with ID label. */
function drawMarkerIcon(ctx, sx, sy, id, visible) {
  const size = 7;
  const color = visible ? '#00ff88' : '#00d4ff';
  const alpha = visible ? 1.0 : 0.6;

  ctx.save();
  ctx.globalAlpha = alpha;

  // Diamond shape
  ctx.beginPath();
  ctx.moveTo(sx, sy - size);
  ctx.lineTo(sx + size, sy);
  ctx.lineTo(sx, sy + size);
  ctx.lineTo(sx - size, sy);
  ctx.closePath();
  ctx.fillStyle = visible ? 'rgba(0,255,136,0.25)' : 'rgba(0,212,255,0.15)';
  ctx.fill();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // ID label
  ctx.fillStyle = color;
  ctx.font = '600 9px Rajdhani, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'bottom';
  ctx.fillText(`A${id}`, sx, sy - size - 2);

  ctx.restore();
}

// ── Zoom helper (zoom toward a canvas point) ───────────────────────────────
function zoomAtPoint(factor, cx, cy) {
  if (robotLocked) {
    // In locked mode just change scale — centering is recomputed each frame
    viewScale = Math.max(0.5, viewScale * factor);
    needsDraw = true;
    return;
  }
  // World point under (cx, cy) before zoom
  const wx = (cx - viewOffset.x) / viewScale;
  const wy = (cy - viewOffset.y) / viewScale;
  // Apply new scale (clamp to minimum)
  viewScale = Math.max(0.5, viewScale * factor);
  // Adjust offset so the same world point stays under (cx, cy)
  viewOffset.x = cx - wx * viewScale;
  viewOffset.y = cy - wy * viewScale;
  needsDraw = true;
}

// ── Lock toggle ─────────────────────────────────────────────────────────────
function toggleLock() {
  robotLocked = !robotLocked;
  const btn = document.getElementById('map-lock');
  if (btn) {
    btn.textContent = robotLocked ? 'L' : 'L';
    btn.title = robotLocked ? 'Unlock map (free pan)' : 'Lock to robot (heading up)';
    btn.classList.toggle('active', robotLocked);
  }
  // When unlocking, preserve current view (viewOffset is already set)
  needsDraw = true;
}

// ── Controls ────────────────────────────────────────────────────────────────
function setupControls() {
  document.getElementById('map-zoom-in').addEventListener('click', () => {
    const cv = getCanvas();
    zoomAtPoint(1.25, cv ? cv.width / 2 : 0, cv ? cv.height / 2 : 0);
  });
  document.getElementById('map-zoom-out').addEventListener('click', () => {
    const cv = getCanvas();
    zoomAtPoint(1 / 1.25, cv ? cv.width / 2 : 0, cv ? cv.height / 2 : 0);
  });
  document.getElementById('map-reset').addEventListener('click', () => {
    if (robotLocked) toggleLock(); // exit locked mode
    fitMapToCanvas(); needsDraw = true;
  });
  document.getElementById('map-lock').addEventListener('click', toggleLock);
  document.getElementById('cancel-goal-btn').addEventListener('click', cancelGoal);
  const initBtn = document.getElementById('map-init-pose');
  if (initBtn) initBtn.addEventListener('click', () => setInitPoseMode(!initPoseMode));

  const c = getCanvas();
  if (!c) return;

  // Track press start so mouseup can decide what action to take. We deliberately
  // do NOT rely on the synthesized `click` event — it can be flaky when the
  // cursor moves a few pixels between press and release.
  let navPress = null;  // { x, y } canvas coords at mousedown for nav-goal
  c.addEventListener('mousedown', (e) => {
    const rect = c.getBoundingClientRect();
    const cx = e.clientX - rect.left, cy = e.clientY - rect.top;
    if (initPoseMode || e.shiftKey) {
      initPoseDrag = { x0: cx, y0: cy, x1: cx, y1: cy, shiftStart: e.shiftKey };
      needsDraw = true;
      return;
    }
    if (navGoalMode) {
      navPress = { x: cx, y: cy };
      return;
    }
    if (robotLocked) return;
    dragging = true;
    dragStart = { x: e.clientX, y: e.clientY, ox: viewOffset.x, oy: viewOffset.y };
  });
  c.addEventListener('mousemove', (e) => {
    if (initPoseDrag) {
      const rect = c.getBoundingClientRect();
      initPoseDrag.x1 = e.clientX - rect.left;
      initPoseDrag.y1 = e.clientY - rect.top;
      needsDraw = true;
      return;
    }
    if (!dragging) return;
    viewOffset.x = dragStart.ox + (e.clientX - dragStart.x);
    viewOffset.y = dragStart.oy + (e.clientY - dragStart.y);
    needsDraw = true;
  });
  c.addEventListener('mouseup', (e) => {
    if (initPoseDrag) {
      const dx = initPoseDrag.x1 - initPoseDrag.x0;
      const dy = initPoseDrag.y1 - initPoseDrag.y0;
      const dragLen = Math.hypot(dx, dy);
      if (dragLen < 10) {
        sendInitialPose(initPoseDrag.x0, initPoseDrag.y0);
      } else {
        sendInitialPoseDrag(initPoseDrag.x0, initPoseDrag.y0,
                             initPoseDrag.x1, initPoseDrag.y1);
      }
      initPoseDrag = null;
      needsDraw = true;
      return;
    }
    if (navPress) {
      const rect = c.getBoundingClientRect();
      const cx = e.clientX - rect.left, cy = e.clientY - rect.top;
      // Tolerate a few px of mouse jitter between mousedown and mouseup.
      if (Math.hypot(cx - navPress.x, cy - navPress.y) < 12 && mapData) {
        sendNavGoal(navPress.x, navPress.y);
      }
      navPress = null;
      return;
    }
    dragging = false;
  });
  c.addEventListener('mouseleave', () => {
    initPoseDrag = null;
    navPress = null;
    dragging = false;
    needsDraw = true;
  });

  // Touch pan (disabled when locked)
  let touchStart = null;
  c.addEventListener('touchstart', (e) => {
    if (navGoalMode || robotLocked || e.touches.length !== 1) return;
    const t = e.touches[0];
    touchStart = { x: t.clientX, y: t.clientY, ox: viewOffset.x, oy: viewOffset.y };
  }, { passive: true });
  c.addEventListener('touchmove', (e) => {
    if (!touchStart || e.touches.length !== 1) return;
    const t = e.touches[0];
    viewOffset.x = touchStart.ox + (t.clientX - touchStart.x);
    viewOffset.y = touchStart.oy + (t.clientY - touchStart.y);
    needsDraw = true;
  }, { passive: true });
  c.addEventListener('touchend', () => { touchStart = null; });

  // Scroll zoom (toward cursor in free mode, centered in locked mode)
  c.addEventListener('wheel', (e) => {
    e.preventDefault();
    const rect = c.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const factor = e.deltaY < 0 ? 1.1 : 0.9;
    zoomAtPoint(factor, cx, cy);
  }, { passive: false });

  // (No `click` handler — nav goals fire from mouseup so they survive small
  // mouse jitter between press and release. Init pose drag also handled in
  // mouseup above.)
}

// ── Nav2 goal ───────────────────────────────────────────────────────────────
function sendNavGoal(canvasX, canvasY) {
  if (!goalActionClient) { toast('Nav2 not connected', 'err'); return; }

  let ux = canvasX, uy = canvasY;

  // In locked mode, undo the scene rotation around canvas center
  if (robotLocked && robotPos) {
    const c = getCanvas();
    const ccx = c.width / 2, ccy = c.height / 2;
    const dx = canvasX - ccx, dy = canvasY - ccy;
    const cosA = Math.cos(-robotPos.yaw), sinA = Math.sin(-robotPos.yaw);
    ux = dx * cosA - dy * sinA + ccx;
    uy = dx * sinA + dy * cosA + ccy;
  }

  // Unrotated canvas → map pixel → world (inverse of 90° CCW rotated view)
  const mpx = mapData.width  - (uy - viewOffset.y) / viewScale;
  const mpy = mapData.height - (ux - viewOffset.x) / viewScale;
  const wx  = mapData.origin.x + mpx * mapData.resolution;
  const wy  = mapData.origin.y + mpy * mapData.resolution;

  // Legacy ROSLIB.Goal API: construct Goal bound to client, attach event
  // handlers, then send().
  goalHandle = new ROSLIB.Goal({
    actionClient: goalActionClient,
    goalMessage: {
      pose: {
        header: { frame_id: 'map' },
        pose: {
          position: { x: wx, y: wy, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
    },
  });
  goalHandle.on('result', () => {
    goalHandle = null;
    toast('Goal reached!', 'ok');
    updateGoalUI();
  });
  // No-op feedback handler (could surface ETA / distance remaining later).
  goalHandle.on('feedback', () => {});
  goalHandle.send();

  toast(`Nav goal: (${wx.toFixed(2)}, ${wy.toFixed(2)})`, 'ok');
  // Exit goal-picking mode but keep the cancel button visible.
  navGoalMode = false;
  updateGoalUI();
}

function cancelGoal() {
  if (goalHandle) {
    try { goalHandle.cancel(); } catch (_) {}
    goalHandle = null;
  }
  toast('Goal cancelled');
  navGoalMode = true;
  updateGoalUI();
}

function setNavGoalMode(on) {
  navGoalMode = on;
  if (on && !goalActionClient) setupGoalClient();
  // Mutually exclusive with init-pose mode
  if (on) initPoseMode = false;
  updateGoalUI();
}

// ── AMCL initial pose ──────────────────────────────────────────────────────
/** Plain-click init-pose: position only, yaw from current TF (best guess). */
function sendInitialPose(canvasX, canvasY) {
  if (!initialPoseTopic) setupInitialPoseTopic();
  if (!initialPoseTopic || !mapData) { toast('rosbridge not connected', 'err'); return; }
  const p = canvasToWorld(canvasX, canvasY);
  const yaw = (robotPos && typeof robotPos.yaw === 'number') ? robotPos.yaw : 0;
  publishInitialPose(p.x, p.y, yaw);
  toast(`Initial pose: (${p.x.toFixed(2)}, ${p.y.toFixed(2)}) ` +
        `yaw=${(yaw * 180 / Math.PI).toFixed(0)}° (from TF — drag to set explicitly)`, 'ok');
  setInitPoseMode(false);
}

/** Convert canvas (cx,cy) to world (wx,wy) using the same inverse as sendNavGoal. */
function canvasToWorld(cx, cy) {
  let ux = cx, uy = cy;
  if (robotLocked && robotPos) {
    const c = getCanvas();
    const ccx = c.width / 2, ccy = c.height / 2;
    const dx = cx - ccx, dy = cy - ccy;
    const cosA = Math.cos(-robotPos.yaw), sinA = Math.sin(-robotPos.yaw);
    ux = dx * cosA - dy * sinA + ccx;
    uy = dx * sinA + dy * cosA + ccy;
  }
  const mpx = mapData.width  - (uy - viewOffset.y) / viewScale;
  const mpy = mapData.height - (ux - viewOffset.x) / viewScale;
  return {
    x: mapData.origin.x + mpx * mapData.resolution,
    y: mapData.origin.y + mpy * mapData.resolution,
  };
}

/** Click-and-drag init-pose: position from start, yaw from drag direction. */
function sendInitialPoseDrag(cx0, cy0, cx1, cy1) {
  if (!initialPoseTopic) setupInitialPoseTopic();
  if (!initialPoseTopic || !mapData) { toast('rosbridge not connected', 'err'); return; }
  const p0 = canvasToWorld(cx0, cy0);
  const p1 = canvasToWorld(cx1, cy1);
  const yaw = Math.atan2(p1.y - p0.y, p1.x - p0.x);
  publishInitialPose(p0.x, p0.y, yaw);
  toast(`Initial pose: (${p0.x.toFixed(2)}, ${p0.y.toFixed(2)}) ` +
        `yaw=${(yaw * 180 / Math.PI).toFixed(0)}°`, 'ok');
  setInitPoseMode(false);
}

/** Shared publish helper. */
function publishInitialPose(wx, wy, yaw) {
  const qz = Math.sin(yaw / 2.0);
  const qw = Math.cos(yaw / 2.0);
  // Tighter covariance when user explicitly set yaw via drag — give AMCL
  // less room to wander; loose enough that scan match still refines.
  const cov = new Array(36).fill(0);
  cov[0]  = 0.10 * 0.10;
  cov[7]  = 0.10 * 0.10;
  cov[35] = (Math.PI / 12) * (Math.PI / 12);
  initialPoseTopic.publish(new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: {
      pose: {
        position: { x: wx, y: wy, z: 0 },
        orientation: { x: 0, y: 0, z: qz, w: qw },
      },
      covariance: cov,
    },
  }));
}

function setInitPoseMode(on) {
  initPoseMode = on;
  if (on && !initialPoseTopic) setupInitialPoseTopic();
  if (on) navGoalMode = false;  // mutually exclusive
  updateGoalUI();
}

/** Update hint, cancel button visibility, button states, and cursor. */
function updateGoalUI() {
  const hint = document.getElementById('nav-goal-hint');
  const btn  = document.getElementById('cancel-goal-btn');
  const initBtn = document.getElementById('map-init-pose');
  const c    = getCanvas();
  if (hint) {
    if (initPoseMode) {
      hint.textContent = 'CLICK + DRAG TO SET INITIAL POSE (drag = robot heading)';
      hint.classList.remove('hidden');
    } else if (navGoalMode && !goalHandle) {
      hint.textContent = 'TAP MAP TO SET GOAL';
      hint.classList.remove('hidden');
    } else {
      hint.classList.add('hidden');
    }
  }
  if (btn)  btn.classList.toggle('hidden', !goalHandle);
  if (initBtn) initBtn.classList.toggle('active', initPoseMode);
  if (c) c.style.cursor = (navGoalMode || initPoseMode) ? 'crosshair' : '';
}

function setEl(id, val) {
  const el = document.getElementById(id); if (el) el.textContent = val;
}
