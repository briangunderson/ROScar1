/**
 * teleop.js — Holonomic virtual joystick + keyboard teleoperation
 * Left joy: vx/vy (translate).  Right joy: wz (rotate).
 * Keyboard: W/A/S/D translate, Q/E rotate.
 */

import { isConnected, onAppEvent } from './app.js';

// Speed limits (updated by sliders)
let maxLinear  = 0.3;
let maxAngular = 1.0;

// Current commanded velocities from joysticks / keyboard
const joy    = { vx: 0, vy: 0, wz: 0 };  // from nipplejs
const keys   = { vx: 0, vy: 0, wz: 0 };  // from keyboard

let pubInterval = null;
let getPub = null;   // injected from app.js

// ── Init ──────────────────────────────────────────────────────────────────
export function initTeleop(getPubFn) {
  getPub = getPubFn;
  setupSliders();
  setupJoysticks();
  setupKeyboard();
  // Publish loop at 10 Hz
  pubInterval = setInterval(publishVelocity, 100);
  // Register tab-change handler here (avoids circular-import TDZ at module load)
  onAppEvent((ev) => {
    if (ev === 'tabchange') {
      joy.vx = 0; joy.vy = 0; joy.wz = 0;
      keys.vx = 0; keys.vy = 0; keys.wz = 0;
    }
  });
}

// ── Sliders ───────────────────────────────────────────────────────────────
function setupSliders() {
  const linearSlider  = document.getElementById('spd-linear');
  const angularSlider = document.getElementById('spd-angular');
  const linearVal     = document.getElementById('spd-linear-val');
  const angularVal    = document.getElementById('spd-angular-val');

  linearSlider.addEventListener('input', () => {
    maxLinear = parseFloat(linearSlider.value);
    linearVal.textContent = maxLinear.toFixed(2) + ' m/s';
  });
  angularSlider.addEventListener('input', () => {
    maxAngular = parseFloat(angularSlider.value);
    angularVal.textContent = maxAngular.toFixed(1) + ' rad/s';
  });
}

// ── Nipplejs Joysticks ────────────────────────────────────────────────────
function setupJoysticks() {
  const opts = (zone) => ({
    zone,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#00D4FF',
    size: 100,
    lockX: false,
    lockY: false,
    dynamicPage: true,
  });

  // Left joystick: translate (vx forward/back, vy strafe left/right)
  const joyT = nipplejs.create(opts(document.getElementById('joy-translate')));
  joyT.on('move', (_, data) => {
    if (!data.vector) return;
    const angle = data.angle.radian;
    const force = Math.min(data.force, 1.0);
    // nipplejs: angle 0=right, π/2=up. ROS: vx=forward, vy=left
    joy.vx =  force * Math.sin(angle)  * maxLinear;
    joy.vy = -force * Math.cos(angle)  * maxLinear;
  });
  joyT.on('end', () => { joy.vx = 0; joy.vy = 0; updateDisp(); });

  // Right joystick: rotate (wz, use only left/right axis)
  const joyR = nipplejs.create(opts(document.getElementById('joy-rotate')));
  joyR.on('move', (_, data) => {
    if (!data.vector) return;
    const angle = data.angle.radian;
    const force = Math.min(data.force, 1.0);
    // Only use horizontal component for rotation
    joy.wz = -force * Math.cos(angle) * maxAngular;
  });
  joyR.on('end', () => { joy.wz = 0; updateDisp(); });
}

// ── Keyboard ──────────────────────────────────────────────────────────────
const KEY_MAP = {
  KeyW: 'fwd',  KeyS: 'back',
  KeyA: 'left', KeyD: 'right',
  KeyQ: 'ccw',  KeyE: 'cw',
};
const held = new Set();

function setupKeyboard() {
  document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.repeat) return;
    if (KEY_MAP[e.code]) { held.add(e.code); updateKeys(); }
  });
  document.addEventListener('keyup', (e) => {
    if (KEY_MAP[e.code]) { held.delete(e.code); updateKeys(); }
  });
  // Clear on focus loss
  window.addEventListener('blur', () => { held.clear(); updateKeys(); });
}

function updateKeys() {
  keys.vx = (held.has('KeyW') ? 1 : 0) - (held.has('KeyS') ? 1 : 0);
  keys.vy = (held.has('KeyA') ? 1 : 0) - (held.has('KeyD') ? 1 : 0);
  keys.wz = (held.has('KeyQ') ? 1 : 0) - (held.has('KeyE') ? 1 : 0);
}

// ── Publish ───────────────────────────────────────────────────────────────
function publishVelocity() {
  if (!isConnected()) return;
  const pub = getPub();
  if (!pub) return;

  // Joystick takes priority over keyboard when active
  const vx = joy.vx !== 0 ? joy.vx : keys.vx * maxLinear;
  const vy = joy.vy !== 0 ? joy.vy : keys.vy * maxLinear;
  const wz = joy.wz !== 0 ? joy.wz : keys.wz * maxAngular;

  pub.publish(new ROSLIB.Message({
    linear:  { x: vx, y: vy, z: 0 },
    angular: { x: 0,  y: 0,  z: wz },
  }));

  updateDisp(vx, vy, wz);
}

function updateDisp(vx, vy, wz) {
  const fmt = (n) => (n >= 0 ? '+' : '') + (n ?? 0).toFixed(2);
  document.getElementById('vx-disp').textContent = fmt(vx);
  document.getElementById('vy-disp').textContent = fmt(vy);
  document.getElementById('wz-disp').textContent = fmt(wz);
}

