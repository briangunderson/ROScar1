/**
 * aio-app.js — AIO dashboard entry point, ROS connection, E-STOP
 * All panels active simultaneously (no tab switching).
 * Loaded as <script type="module"> from aio.html
 */

import { initTeleop }      from './aio-teleop.js';
import { initCamera }      from './aio-camera.js';
import { initStatus, onNavModeChange } from './aio-status.js';
import { initLidar }       from './aio-lidar.js';
import { initMap, enableNavGoalMode }  from './aio-map.js';
import { initGraphs }       from './aio-graphs.js';
import { initDiagnostics }  from './aio-diagnostics.js';
import { initTF }           from './aio-tf.js';

// ── Connection config ────────────────────────────────────────────────────
export const HOST  = window.location.hostname || 'localhost';
export const PORTS = { rosbridge: 9090, video: 8080, http: 8888 };

let ros       = null;
let connected = false;

export function getRos()      { return ros; }
export function isConnected() { return connected; }

// ── Module notification bus ──────────────────────────────────────────────
const moduleCallbacks = [];
export function onAppEvent(cb) { moduleCallbacks.push(cb); }

function notifyModules(event, data) {
  moduleCallbacks.forEach(cb => {
    try { cb(event, data); } catch (e) { console.error(e); }
  });
}

// ── Toast ────────────────────────────────────────────────────────────────
let toastTimer = null;
export function toast(msg, type = '') {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className = 'toast show' + (type ? ' ' + type : '');
  if (toastTimer) clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.className = 'toast'; }, 2800);
}

// ── E-STOP ───────────────────────────────────────────────────────────────
let cmdVelPub = null;
const ZERO_TWIST = {
  linear:  { x: 0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0 },
};

function getOrCreateCmdVelPub() {
  if (!cmdVelPub && ros) {
    cmdVelPub = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });
  }
  return cmdVelPub;
}

function estop() {
  const pub = getOrCreateCmdVelPub();
  if (pub) {
    pub.publish(new ROSLIB.Message(ZERO_TWIST));
    pub.publish(new ROSLIB.Message(ZERO_TWIST)); // send twice for safety
  }
  toast('E-STOP: HALT', 'err');
}

document.getElementById('estop-btn').addEventListener('click', estop);
document.addEventListener('keydown', (e) => {
  if (e.code === 'Space' && e.target.tagName !== 'INPUT') {
    e.preventDefault();
    estop();
  }
});

// ── ROS Connection ───────────────────────────────────────────────────────
function connect() {
  if (ros) { try { ros.close(); } catch (_) {} }

  ros = new ROSLIB.Ros({ url: `ws://${HOST}:${PORTS.rosbridge}` });

  ros.on('connection', () => {
    connected = true;
    setConnUI(true);
    toast('Connected to rosbridge', 'ok');
    notifyModules('connected');
  });

  ros.on('error', (e) => {
    console.warn('rosbridge error', e);
    setConnUI(false);
  });

  ros.on('close', () => {
    connected = false;
    cmdVelPub = null;
    setConnUI(false);
    setTimeout(connect, 3000);
  });
}

function setConnUI(ok) {
  const el = document.getElementById('conn-indicator');
  el.className = 'conn-indicator ' + (ok ? 'connected' : 'disconnected');
  el.querySelector('.conn-label').textContent = ok ? 'CONNECTED' : 'DISCONNECTED';
}

// ── Init all modules ─────────────────────────────────────────────────────
initTeleop(getOrCreateCmdVelPub);
initCamera();
initStatus();
initLidar(getRos);
initMap(getRos);
onNavModeChange(enableNavGoalMode);
initGraphs();
initDiagnostics(getRos);
initTF(getRos);

connect();
