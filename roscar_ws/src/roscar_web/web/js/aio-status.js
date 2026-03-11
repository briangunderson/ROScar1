/**
 * aio-status.js — Status readouts, mode switching, and map save.
 * Combines status.js + modes.js for the AIO dashboard.
 * Always active (no tab gating).
 */

import { getRos, isConnected, onAppEvent, toast, HOST } from './aio-app.js';

// ── Constants ───────────────────────────────────────────────────────────────
const BATT_MIN = 10.0;
const BATT_MAX = 12.6;
const R2D      = 180 / Math.PI;

// ── State ───────────────────────────────────────────────────────────────────
const subs        = [];
let setModeSvc    = null;
let saveMapSvc    = null;
let currentMode   = 'idle';
let navGoalCallback = null;

// ── Data callbacks for graphs module ────────────────────────────────────────
const odomCbs = [];
const battCbs = [];
export function onOdomData(cb)    { odomCbs.push(cb); }
export function onBatteryData(cb) { battCbs.push(cb); }

export function onNavModeChange(cb) { navGoalCallback = cb; }

// ── Helpers ─────────────────────────────────────────────────────────────────
const fmt = (v, d = 3) => (v >= 0 ? '+' : '') + v.toFixed(d);

function setEl(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function showMsg(el, text, type = '') {
  if (!el) return;
  el.textContent = text;
  el.className = 'mode-status-msg' + (type ? ' ' + type : '');
}

// ── Init ────────────────────────────────────────────────────────────────────
export function initStatus() {
  setupModeButtons();
  setupMapSave();
  onAppEvent((ev) => {
    if (ev === 'connected') {
      subscribeAll();
      setupServices();
    }
  });
}

// ── ROS Subscriptions ───────────────────────────────────────────────────────
function subscribeAll() {
  const ros = getRos();
  if (!ros) return;

  // Clean up previous subs
  subs.forEach(s => { try { s.unsubscribe(); } catch (_) {} });
  subs.length = 0;

  // Filtered odometry (position, velocity)
  const odomSub = new ROSLIB.Topic({
    ros,
    name: '/odometry/filtered',
    messageType: 'nav_msgs/Odometry',
    throttle_rate: 100,
  });
  odomSub.subscribe(handleOdom);
  subs.push(odomSub);

  // IMU orientation
  const imuSub = new ROSLIB.Topic({
    ros,
    name: '/imu/data',
    messageType: 'sensor_msgs/Imu',
    throttle_rate: 200,
  });
  imuSub.subscribe(handleImu);
  subs.push(imuSub);

  // Battery voltage
  const battSub = new ROSLIB.Topic({
    ros,
    name: '/battery_voltage',
    messageType: 'std_msgs/Float32',
    throttle_rate: 1000,
  });
  battSub.subscribe(handleBattery);
  subs.push(battSub);
}

// ── Odometry Handler ────────────────────────────────────────────────────────
function handleOdom(msg) {
  const pos = msg.pose.pose.position;
  const vel = msg.twist.twist;

  const q   = msg.pose.pose.orientation;
  const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z));

  setEl('st-x',   fmt(pos.x, 3));
  setEl('st-y',   fmt(pos.y, 3));
  setEl('st-yaw', fmt(yaw * R2D, 1));
  setEl('st-vx',  fmt(vel.linear.x, 3));
  setEl('st-vy',  fmt(vel.linear.y, 3));
  setEl('st-wz',  fmt(vel.angular.z, 3));

  odomCbs.forEach(cb => {
    try { cb(pos, vel, yaw); } catch (e) { console.error(e); }
  });
}

// ── IMU Handler ─────────────────────────────────────────────────────────────
function handleImu(msg) {
  const q     = msg.orientation;
  const roll  = Math.atan2(2 * (q.w * q.x + q.y * q.z),
                           1 - 2 * (q.x * q.x + q.y * q.y));
  const pitch = Math.asin(Math.max(-1, Math.min(1,
                           2 * (q.w * q.y - q.z * q.x))));
  const yaw   = Math.atan2(2 * (q.w * q.z + q.x * q.y),
                           1 - 2 * (q.y * q.y + q.z * q.z));

  setEl('st-roll',    fmt(roll  * R2D, 1));
  setEl('st-pitch',   fmt(pitch * R2D, 1));
  setEl('st-imu-yaw', fmt(yaw   * R2D, 1));
}

// ── Battery Handler ─────────────────────────────────────────────────────────
function handleBattery(msg) {
  const v   = msg.data;
  const pct = Math.max(0, Math.min(100,
                (v - BATT_MIN) / (BATT_MAX - BATT_MIN) * 100));
  const color = pct > 40 ? 'var(--green)'
              : pct > 20 ? 'var(--amber)'
              :            'var(--red)';

  // Header mini bar
  const hBar = document.getElementById('battery-bar');
  if (hBar) {
    hBar.style.width      = pct + '%';
    hBar.style.background = color;
  }
  setEl('battery-voltage', v.toFixed(1) + 'V');

  // Status panel gauge
  const bFill = document.getElementById('batt-fill');
  if (bFill) {
    bFill.style.width      = pct + '%';
    bFill.style.background = color;
  }
  setEl('batt-volts', v.toFixed(1) + ' V');
  setEl('batt-pct',   Math.round(pct) + '%');

  battCbs.forEach(cb => {
    try { cb(v, pct); } catch (e) { console.error(e); }
  });
}

// ── Services ────────────────────────────────────────────────────────────────
function setupServices() {
  const ros = getRos();
  if (!ros) return;

  setModeSvc = new ROSLIB.Service({
    ros, name: '/web/set_mode', serviceType: 'roscar_interfaces/SetMode',
  });
  saveMapSvc = new ROSLIB.Service({
    ros, name: '/web/save_map', serviceType: 'roscar_interfaces/SaveMap',
  });
}

// ── Mode Buttons ────────────────────────────────────────────────────────────
function setupModeButtons() {
  const navMapSelect = document.getElementById('nav-map-select');
  const applyBtn     = document.getElementById('apply-mode-btn');
  const statusMsg    = document.getElementById('mode-status-msg');

  document.querySelectorAll('#panel-status .mode-btn-sm[data-mode]').forEach(btn => {
    btn.addEventListener('click', () => {
      const mode = btn.dataset.mode;

      if (mode === 'navigation') {
        if (navMapSelect) navMapSelect.classList.remove('hidden');
      } else {
        if (navMapSelect) navMapSelect.classList.add('hidden');
        requestSetMode(mode, '', statusMsg);
      }

      // Optimistic active state
      document.querySelectorAll('#panel-status .mode-btn-sm[data-mode]').forEach(b =>
        b.classList.remove('active'));
      btn.classList.add('active');
    });
  });

  if (applyBtn) {
    applyBtn.addEventListener('click', () => {
      const mapPath = document.getElementById('nav-map-path')?.value.trim();
      if (!mapPath) {
        showMsg(statusMsg, 'Map path required', 'err');
        return;
      }
      requestSetMode('navigation', mapPath, statusMsg);
      if (navMapSelect) navMapSelect.classList.add('hidden');
    });
  }
}

function requestSetMode(mode, mapPath, statusEl) {
  if (!setModeSvc) {
    showMsg(statusEl, 'Not connected to launch_manager', 'err');
    toast('launch_manager not reachable', 'err');
    return;
  }

  showMsg(statusEl, `Switching to ${mode.toUpperCase()}...`);

  const req = new ROSLIB.ServiceRequest({ mode, map_path: mapPath });
  setModeSvc.callService(req, (resp) => {
    if (resp.success) {
      currentMode = mode;
      setEl('mode-display', mode.toUpperCase());
      showMsg(statusEl, resp.message || 'OK', 'ok');
      toast(`Mode: ${mode.toUpperCase()}`, 'ok');

      // Enable nav goal clicking when in nav or slam_nav mode
      if ((mode === 'navigation' || mode === 'slam_nav') && navGoalCallback) {
        navGoalCallback(true);
      }
    } else {
      showMsg(statusEl, 'FAILED: ' + (resp.message || 'unknown'), 'err');
      toast('Mode switch failed', 'err');
      // Revert active button
      document.querySelectorAll('#panel-status .mode-btn-sm[data-mode]').forEach(b => {
        b.classList.toggle('active', b.dataset.mode === currentMode);
      });
    }
  });
}

// ── Map Save ────────────────────────────────────────────────────────────────
function setupMapSave() {
  const saveBtn = document.getElementById('save-map-btn');
  const nameEl  = document.getElementById('map-save-name');
  const msgEl   = document.getElementById('save-map-msg');

  if (!saveBtn) return;

  saveBtn.addEventListener('click', () => {
    const mapName = nameEl?.value.trim() || 'my_map';

    if (!saveMapSvc) {
      showMsg(msgEl, 'Not connected', 'err');
      return;
    }

    showMsg(msgEl, 'Saving...');
    const req = new ROSLIB.ServiceRequest({ map_name: mapName });
    saveMapSvc.callService(req, (resp) => {
      if (resp.success) {
        showMsg(msgEl, `Saved: ${resp.path}`, 'ok');
        toast('Map saved!', 'ok');
      } else {
        showMsg(msgEl, 'Failed: ' + (resp.message || 'unknown'), 'err');
        toast('Map save failed', 'err');
      }
    });
  });
}
