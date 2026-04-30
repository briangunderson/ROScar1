/**
 * aio-nav-tuning.js — Live DWB controller tuning sliders.
 *
 * Pushes parameter updates to /controller_server via the
 * rcl_interfaces/srv/SetParameters service. Values persist in
 * localStorage and are re-applied on connect, so a fresh nav-mode
 * launch picks up the user's preferences automatically.
 *
 * Strafing efficiency note: mecanum strafing burns roughly twice the
 * motor work per meter compared to forward driving. Defaulting Max
 * strafe to 0.20 m/s (vs 0.80 forward) discourages DWB from picking
 * strafe-heavy trajectories when a forward-arc would do.
 */

import { onAppEvent, toast } from './aio-app.js';

let getRos;
let setParamsSvc = null;

// Slider definitions: param name → { domId, default, type, format }
// type 2 = integer, type 3 = double (rcl_interfaces/msg/ParameterType)
const PARAMS = [
  { dom: 'tune-vx',         param: 'FollowPath.max_vel_x', type: 3, def: 0.8,  fmt: v => v.toFixed(2) },
  { dom: 'tune-vy',         param: 'FollowPath.max_vel_y', type: 3, def: 0.2,  fmt: v => v.toFixed(2),
    // mirror sign onto min_vel_y (so strafe is symmetric)
    mirror: { param: 'FollowPath.min_vel_y', type: 3, transform: v => -v } },
  { dom: 'tune-vy-samples', param: 'FollowPath.vy_samples', type: 2, def: 3,  fmt: v => `${v|0}` },
  { dom: 'tune-twirl',      param: 'FollowPath.Twirling.scale', type: 3, def: 10, fmt: v => `${v|0}` },
];
const STORAGE_KEY = 'roscar_nav_tuning_v1';

export function initNavTuning(getRosFn) {
  getRos = getRosFn;
  // Load persisted values + wire up the slider DOM
  const saved = loadFromStorage();
  for (const p of PARAMS) {
    const slider = document.getElementById(p.dom);
    const valEl  = document.getElementById(p.dom + '-val');
    if (!slider) continue;
    if (saved[p.param] != null) slider.value = String(saved[p.param]);
    if (valEl) valEl.textContent = p.fmt(parseFloat(slider.value));
    // Live label update on every drag, but only PUSH the param when the
    // user lets go (change event) — set_parameters is non-trivially heavy
    // and we don't want to flood the controller during a drag.
    slider.addEventListener('input', () => {
      if (valEl) valEl.textContent = p.fmt(parseFloat(slider.value));
    });
    slider.addEventListener('change', () => pushParam(p, slider.value));
  }
  const resetBtn = document.getElementById('nav-tune-reset');
  if (resetBtn) resetBtn.addEventListener('click', resetDefaults);

  onAppEvent((ev) => {
    if (ev === 'connected') {
      ensureService();
      // Re-push every current slider value so a fresh nav launch picks up
      // the user's preferences without any manual interaction.
      for (const p of PARAMS) {
        const s = document.getElementById(p.dom);
        if (s) pushParam(p, s.value);
      }
    }
  });
}

function ensureService() {
  if (setParamsSvc) return;
  const ros = getRos(); if (!ros) return;
  setParamsSvc = new ROSLIB.Service({
    ros,
    name: '/controller_server/set_parameters',
    serviceType: 'rcl_interfaces/srv/SetParameters',
  });
}

/** Build a Parameter request entry of the given type. */
function paramValue(type, raw) {
  const v = type === 2 ? Math.round(parseFloat(raw)) : parseFloat(raw);
  // ParameterValue: type byte + matching field
  if (type === 2) return { type: 2, integer_value: v };
  if (type === 3) return { type: 3, double_value: v };
  return { type: 0 };
}

function pushParam(p, raw) {
  ensureService();
  if (!setParamsSvc) {
    setMsg(`(rosbridge not connected — saved locally)`, 'warn');
    saveToStorage();
    return;
  }
  const params = [{ name: p.param, value: paramValue(p.type, raw) }];
  if (p.mirror) {
    const mv = p.mirror.transform(parseFloat(raw));
    params.push({ name: p.mirror.param, value: paramValue(p.mirror.type, mv) });
  }
  setParamsSvc.callService(
    new ROSLIB.ServiceRequest({ parameters: params }),
    (resp) => {
      const allOk = (resp.results || []).every(r => r.successful);
      const label = p.fmt(parseFloat(raw));
      if (allOk) {
        setMsg(`${p.param} = ${label}`, 'ok');
        saveToStorage();
      } else {
        const why = (resp.results || []).find(r => !r.successful)?.reason || 'rejected';
        setMsg(`${p.param} ${why}`, 'err');
      }
    },
    (err) => {
      // controller_server might not be running (idle / teleop / slam mode).
      // Save to localStorage so the value re-applies on next nav launch.
      setMsg(`${p.param} — controller_server not reachable, saved for next nav launch`, 'warn');
      saveToStorage();
    },
  );
}

function resetDefaults() {
  for (const p of PARAMS) {
    const s = document.getElementById(p.dom);
    const v = document.getElementById(p.dom + '-val');
    if (!s) continue;
    s.value = String(p.def);
    if (v) v.textContent = p.fmt(p.def);
    pushParam(p, p.def);
  }
}

function saveToStorage() {
  const obj = {};
  for (const p of PARAMS) {
    const s = document.getElementById(p.dom);
    if (s) obj[p.param] = parseFloat(s.value);
  }
  try { localStorage.setItem(STORAGE_KEY, JSON.stringify(obj)); } catch (_) {}
}

function loadFromStorage() {
  try { return JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}'); }
  catch (_) { return {}; }
}

function setMsg(text, level) {
  const el = document.getElementById('nav-tune-msg');
  if (!el) return;
  el.textContent = text;
  el.className = 'status-msg ' + (level || '');
}
