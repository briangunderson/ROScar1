/**
 * modes.js — Robot mode switching and map save via ROS2 services.
 * Calls /web/set_mode (roscar_interfaces/SetMode) and
 *       /web/save_map  (roscar_interfaces/SaveMap).
 */

import { onAppEvent } from './app.js';
import { enableNavGoalMode } from './map.js';

let getRos;
let toastFn;
let setModeSvc  = null;
let saveMapSvc  = null;
let currentMode = 'idle';

export function initModes(getRosFn, toastFnArg) {
  getRos  = getRosFn;
  toastFn = toastFnArg;
  setupModeButtons();
  setupMapSave();
  onAppEvent((ev) => {
    if (ev === 'connected') setupServices();
  });
}

// ── Services ──────────────────────────────────────────────────────────────
function setupServices() {
  const ros = getRos(); if (!ros) return;

  setModeSvc = new ROSLIB.Service({
    ros, name: '/web/set_mode', serviceType: 'roscar_interfaces/SetMode',
  });
  saveMapSvc = new ROSLIB.Service({
    ros, name: '/web/save_map', serviceType: 'roscar_interfaces/SaveMap',
  });
}

// ── Mode Buttons ──────────────────────────────────────────────────────────
function setupModeButtons() {
  const navMapSelect = document.getElementById('nav-map-select');
  const applyBtn     = document.getElementById('apply-mode-btn');
  const statusMsg    = document.getElementById('mode-status-msg');

  document.querySelectorAll('.mode-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      const mode = btn.dataset.mode;

      // Show map path input only for navigation mode
      if (mode === 'navigation') {
        navMapSelect.classList.remove('hidden');
      } else {
        navMapSelect.classList.add('hidden');
        requestSetMode(mode, '', statusMsg);
      }

      // Update button active state optimistically
      document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
    });
  });

  applyBtn.addEventListener('click', () => {
    const mapPath = document.getElementById('nav-map-path').value.trim();
    if (!mapPath) {
      showMsg(statusMsg, 'Map path required', 'err');
      return;
    }
    requestSetMode('navigation', mapPath, statusMsg);
    navMapSelect.classList.add('hidden');
  });
}

function requestSetMode(mode, mapPath, statusEl) {
  if (!setModeSvc) {
    showMsg(statusEl, 'Not connected to launch_manager', 'err');
    toastFn('launch_manager not reachable', 'err');
    return;
  }

  showMsg(statusEl, `Switching to ${mode.toUpperCase()}…`);

  // use_depth defaults true — the D435i is the sole camera now and is
  // expected in every mode (teleop/slam/navigation/slam_nav).
  const req = new ROSLIB.ServiceRequest({
    mode,
    map_path: mapPath,
    use_depth: true,
  });
  setModeSvc.callService(req, (resp) => {
    if (resp.success) {
      currentMode = mode;
      document.getElementById('mode-display').textContent = mode.toUpperCase();
      showMsg(statusEl, resp.message || 'OK', 'ok');
      toastFn(`Mode: ${mode.toUpperCase()}`, 'ok');

      // Enable nav goal clicking on map tab when in nav/slam_nav mode
      if (mode === 'navigation' || mode === 'slam_nav') {
        enableNavGoalMode();
      }
    } else {
      showMsg(statusEl, 'FAILED: ' + (resp.message || 'unknown'), 'err');
      toastFn('Mode switch failed', 'err');
      // Revert active button
      document.querySelectorAll('.mode-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.mode === currentMode);
      });
    }
  });
}

// ── Map Save ──────────────────────────────────────────────────────────────
function setupMapSave() {
  const saveBtn = document.getElementById('save-map-btn');
  const nameEl  = document.getElementById('map-save-name');
  const msgEl   = document.getElementById('save-map-msg');

  saveBtn.addEventListener('click', () => {
    const mapName = nameEl.value.trim() || 'my_map';

    if (!saveMapSvc) {
      showMsg(msgEl, 'Not connected', 'err');
      return;
    }

    showMsg(msgEl, 'Saving…');
    const req = new ROSLIB.ServiceRequest({ map_name: mapName });
    saveMapSvc.callService(req, (resp) => {
      if (resp.success) {
        showMsg(msgEl, `Saved → ${resp.path}`, 'ok');
        toastFn('Map saved!', 'ok');
      } else {
        showMsg(msgEl, 'Failed: ' + (resp.message || 'unknown'), 'err');
        toastFn('Map save failed', 'err');
      }
    });
  });
}

// ── Helpers ───────────────────────────────────────────────────────────────
function showMsg(el, text, type = '') {
  if (!el) return;
  el.textContent = text;
  el.className = 'mode-status-msg' + (type ? ' ' + type : '');
}
