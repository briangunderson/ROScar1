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
// Tracked from aio-status's mode poll. We only push to /controller_server
// when nav is actually running — otherwise the service doesn't exist yet
// and rosbridge logs InvalidServiceException noise on every push attempt.
let navActive = false;

// Slider definitions: param name → { domId, default, type, format }
// type 2 = integer, type 3 = double (rcl_interfaces/msg/ParameterType)
const PARAMS = [
  // 2026-05-03: default 0.8 → 0.4 after couch collision. See nav2_params.yaml
  // and the collision PR for the math: at 0.65 m/s (the user's tuned-up
  // value at the time) worst-case stop distance was ~50 cm vs D435i's
  // 20 cm range_min. New default of 0.4 m/s gives ~26 cm stop distance,
  // well inside detection range. User can tune up via the slider when
  // the environment is open.
  { dom: 'tune-vx',         param: 'FollowPath.max_vel_x', type: 3, def: 0.4,  fmt: v => v.toFixed(2) },
  { dom: 'tune-vy',         param: 'FollowPath.max_vel_y', type: 3, def: 0.2,  fmt: v => v.toFixed(2),
    // mirror sign onto min_vel_y (so strafe is symmetric)
    mirror: { param: 'FollowPath.min_vel_y', type: 3, transform: v => -v } },
  { dom: 'tune-vy-samples', param: 'FollowPath.vy_samples', type: 2, def: 3,  fmt: v => `${v|0}` },
  { dom: 'tune-twirl',      param: 'FollowPath.Twirling.scale', type: 3, def: 10, fmt: v => `${v|0}` },
];
// v1 → v2 (2026-05-03): bumped to invalidate stored slider values from
// before the post-couch-collision speed reduction. v1 users had ~0.65
// m/s saved; without this bump the slider would re-push the unsafe
// value on every nav launch, defeating the YAML default change.
const STORAGE_KEY = 'roscar_nav_tuning_v2';

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
    if (ev === 'connected') ensureService();
  });
}

/** Called by aio-status when the active mode is/becomes nav-capable.
 * On nav-mode entry we push every saved slider value once so a fresh nav
 * launch picks up the user's tuning automatically. Outside nav mode we
 * skip pushes — controller_server doesn't exist yet and rosbridge would
 * log InvalidServiceException noise on every attempt. */
export function setNavTuningActive(on) {
  const wasActive = navActive;
  navActive = !!on;
  if (navActive && !wasActive) {
    // Mode just flipped to nav. Re-apply the user's saved tuning.
    // Small delay so the controller_server lifecycle activate completes
    // before we hit it with set_parameters calls.
    setTimeout(() => {
      if (!navActive) return;  // bailed back to non-nav mode in the gap
      for (const p of PARAMS) {
        const s = document.getElementById(p.dom);
        if (s) pushParam(p, s.value);
      }
    }, 4000);
  }
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
  // Always persist the slider's current value, regardless of whether we
  // can reach controller_server right now.
  saveToStorage();
  if (!navActive) {
    setMsg(`${p.param} = ${p.fmt(parseFloat(raw))} (saved — applies on next nav launch)`, 'warn');
    return;
  }
  ensureService();
  if (!setParamsSvc) {
    setMsg(`(rosbridge not connected — saved locally)`, 'warn');
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
