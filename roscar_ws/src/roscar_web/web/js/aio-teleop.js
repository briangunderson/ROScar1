/**
 * aio-teleop.js — Holonomic teleoperation: joysticks + keyboard + gamepad
 * Left joy: vx/vy (translate).  Right joy: wz (rotate).
 * Keyboard: W/A/S/D translate, Q/E rotate.
 * Gamepad: left stick translate, right stick rotate.
 * Priority: SpaceMouse > gamepad > joystick > keyboard.
 */

import { isConnected } from './aio-app.js';
import { getSpaceMouseVelocity } from './aio-spacemouse.js';

// Speed limits (updated by sliders)
let maxLinear  = 0.3;
let maxAngular = 1.0;

// Per-source velocity state
const joy = { vx: 0, vy: 0, wz: 0 };
const keys = { vx: 0, vy: 0, wz: 0 };
const pad = { vx: 0, vy: 0, wz: 0 };

let getPub = null;
let activeSource = 'KB';  // 'KB', 'JOY', 'PAD'

const DEADZONE = 0.15;

// ── Init ─────────────────────────────────────────────────────────────────
export function initTeleop(getPubFn) {
  getPub = getPubFn;
  setupSliders();
  setupJoysticks();
  setupKeyboard();
  setupGamepad();
  // Publish loop at 10 Hz
  setInterval(publishVelocity, 100);
}

// ── Sliders ──────────────────────────────────────────────────────────────
function setupSliders() {
  const linSlider = document.getElementById('spd-linear');
  const angSlider = document.getElementById('spd-angular');
  const linVal    = document.getElementById('spd-linear-val');
  const angVal    = document.getElementById('spd-angular-val');

  linSlider.addEventListener('input', () => {
    maxLinear = parseFloat(linSlider.value);
    linVal.textContent = maxLinear.toFixed(2);
  });
  angSlider.addEventListener('input', () => {
    maxAngular = parseFloat(angSlider.value);
    angVal.textContent = maxAngular.toFixed(1);
  });
}

// ── Nipplejs Joysticks ───────────────────────────────────────────────────
function setupJoysticks() {
  const opts = (zone) => ({
    zone,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#00D4FF',
    size: 100,
  });

  // Left joystick: translate (vx forward/back, vy strafe)
  const joyT = nipplejs.create(opts(document.getElementById('joy-translate')));
  joyT.on('move', (_, data) => {
    if (!data.vector) return;
    const angle = data.angle.radian;
    const force = Math.min(data.force, 1.0);
    // nipplejs: angle 0=right, pi/2=up. ROS: vx=forward, vy=left
    joy.vx =  force * Math.sin(angle) * maxLinear;
    joy.vy = -force * Math.cos(angle) * maxLinear;
  });
  joyT.on('end', () => { joy.vx = 0; joy.vy = 0; });

  // Right joystick: rotate + drive (wz from X-axis, vx from Y-axis)
  const joyR = nipplejs.create(opts(document.getElementById('joy-rotate')));
  joyR.on('move', (_, data) => {
    if (!data.vector) return;
    const angle = data.angle.radian;
    const force = Math.min(data.force, 1.0);
    joy.wz = -force * Math.cos(angle) * maxAngular;
    joy.vx = force * Math.sin(angle) * maxLinear;
  });
  joyR.on('end', () => { joy.wz = 0; joy.vx = 0; });
}

// ── Keyboard ─────────────────────────────────────────────────────────────
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
  window.addEventListener('blur', () => { held.clear(); updateKeys(); });
}

function updateKeys() {
  keys.vx = (held.has('KeyW') ? 1 : 0) - (held.has('KeyS') ? 1 : 0);
  keys.vy = (held.has('KeyA') ? 1 : 0) - (held.has('KeyD') ? 1 : 0);
  keys.wz = (held.has('KeyQ') ? 1 : 0) - (held.has('KeyE') ? 1 : 0);
}

// ── Gamepad ──────────────────────────────────────────────────────────────
let gamepadIndex = null;

function setupGamepad() {
  window.addEventListener('gamepadconnected', (e) => {
    gamepadIndex = e.gamepad.index;
    const indicator = document.getElementById('gamepad-indicator');
    if (indicator) {
      indicator.classList.remove('hidden');
      const nameEl = indicator.querySelector('.gamepad-label');
      if (nameEl) {
        const name = e.gamepad.id.length > 20
          ? e.gamepad.id.substring(0, 20) + '…'
          : e.gamepad.id;
        nameEl.textContent = name;
      }
    }
  });

  window.addEventListener('gamepaddisconnected', (e) => {
    if (e.gamepad.index === gamepadIndex) {
      gamepadIndex = null;
      pad.vx = 0; pad.vy = 0; pad.wz = 0;
      const indicator = document.getElementById('gamepad-indicator');
      if (indicator) indicator.classList.add('hidden');
    }
  });

  // Poll gamepad state in RAF loop
  requestAnimationFrame(pollGamepad);
}

function applyDeadzone(val) {
  return Math.abs(val) < DEADZONE ? 0 : val;
}

function pollGamepad() {
  if (gamepadIndex !== null) {
    const gamepads = navigator.getGamepads();
    const gp = gamepads[gamepadIndex];
    if (gp) {
      // Standard mapping: left stick axes[0]=X, axes[1]=Y; right stick axes[2]=X, axes[3]=Y
      const lx = applyDeadzone(gp.axes[0] || 0);  // left/right -> vy (strafe)
      const ly = applyDeadzone(gp.axes[1] || 0);  // up/down -> vx (forward, inverted)
      const rx = applyDeadzone(gp.axes.length > 2 ? (gp.axes[2] || 0) : 0); // right stick X -> wz

      pad.vx = -ly * maxLinear;   // invert: stick up = negative axis = forward
      pad.vy = -lx * maxLinear;   // invert: stick left = negative axis = strafe left (positive vy)
      pad.wz = -rx * maxAngular;  // invert: stick left = negative axis = rotate CCW (positive wz)
    }
  }
  requestAnimationFrame(pollGamepad);
}

// ── Publish ──────────────────────────────────────────────────────────────
function publishVelocity() {
  if (!isConnected()) return;
  const pub = getPub();
  if (!pub) return;

  let vx = 0, vy = 0, wz = 0;

  // Priority: SpaceMouse > Gamepad > Joystick > Keyboard
  const smVel = getSpaceMouseVelocity();
  const padActive = pad.vx !== 0 || pad.vy !== 0 || pad.wz !== 0;
  const joyActive = joy.vx !== 0 || joy.vy !== 0 || joy.wz !== 0;
  const kbActive  = keys.vx !== 0 || keys.vy !== 0 || keys.wz !== 0;

  if (smVel) {
    vx = smVel.vx; vy = smVel.vy; wz = smVel.wz;
    setSource('SM');
  } else if (padActive) {
    vx = pad.vx; vy = pad.vy; wz = pad.wz;
    setSource('PAD');
  } else if (joyActive) {
    vx = joy.vx; vy = joy.vy; wz = joy.wz;
    setSource('JOY');
  } else if (kbActive) {
    vx = keys.vx * maxLinear; vy = keys.vy * maxLinear; wz = keys.wz * maxAngular;
    setSource('KB');
  } else {
    setSource('--');
  }

  // Dim speed sliders when SpaceMouse is active source
  document.querySelectorAll('.drive .slider-row').forEach(
    row => row.classList.toggle('sm-dimmed', smVel !== null)
  );

  pub.publish(new ROSLIB.Message({
    linear:  { x: vx, y: vy, z: 0 },
    angular: { x: 0,  y: 0,  z: wz },
  }));

  updateDisp(vx, vy, wz);
}

function setSource(src) {
  if (src !== activeSource) {
    activeSource = src;
    const el = document.getElementById('input-source');
    if (el) el.textContent = src;
  }
}

function updateDisp(vx, vy, wz) {
  const fmt = (n) => (n >= 0 ? '+' : '') + (n ?? 0).toFixed(2);
  const vxEl = document.getElementById('vx-disp');
  const vyEl = document.getElementById('vy-disp');
  const wzEl = document.getElementById('wz-disp');
  if (vxEl) vxEl.textContent = fmt(vx);
  if (vyEl) vyEl.textContent = fmt(vy);
  if (wzEl) wzEl.textContent = fmt(wz);
}
