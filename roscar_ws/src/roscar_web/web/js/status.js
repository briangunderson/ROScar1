/**
 * status.js — Subscribe to odometry, IMU, battery topics and update status tab.
 * Also updates the header battery bar.
 */

import { onAppEvent } from './app.js';

// Voltage range for battery percentage (adjust per battery spec)
const BATT_MIN = 10.0;
const BATT_MAX = 12.6;

let getRos;
const subs = [];

export function initStatus(getRosFn) {
  getRos = getRosFn;
  onAppEvent((ev) => {
    if (ev === 'connected') subscribeAll();
  });
  // Register tab-change handler here (avoids circular-import TDZ at module load)
  onAppEvent((ev, tab) => {
    if (ev === 'tabchange' && tab === 'status') refreshNodeList();
  });
}

function subscribeAll() {
  const ros = getRos();
  if (!ros) return;

  // Clean up previous subs
  subs.forEach(s => { try { s.unsubscribe(); } catch(_) {} });
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

// ── Odometry ───────────────────────────────────────────────────────────────
function handleOdom(msg) {
  const pos = msg.pose.pose.position;
  const vel = msg.twist.twist;

  // Yaw from quaternion
  const q = msg.pose.pose.orientation;
  const yaw = Math.atan2(
    2*(q.w*q.z + q.x*q.y),
    1 - 2*(q.y*q.y + q.z*q.z)
  );

  setEl('st-x',  fmtSigned(pos.x, 3) );
  setEl('st-y',  fmtSigned(pos.y, 3) );
  setEl('st-yaw', fmtSigned(yaw * 180 / Math.PI, 2));
  setEl('st-vx', fmtSigned(vel.linear.x, 3));
  setEl('st-vy', fmtSigned(vel.linear.y, 3));
  setEl('st-wz', fmtSigned(vel.angular.z, 3));
}

// ── IMU ────────────────────────────────────────────────────────────────────
function handleImu(msg) {
  const q = msg.orientation;
  // Roll and pitch from quaternion
  const roll  = Math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2*(q.w*q.y - q.z*q.x))));
  const yaw   = Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));

  const r2d = 180 / Math.PI;
  setEl('st-roll',    fmtSigned(roll  * r2d, 1));
  setEl('st-pitch',   fmtSigned(pitch * r2d, 1));
  setEl('st-imu-yaw', fmtSigned(yaw   * r2d, 1));
}

// ── Battery ────────────────────────────────────────────────────────────────
function handleBattery(msg) {
  const v   = msg.data;
  const pct = Math.max(0, Math.min(100, (v - BATT_MIN) / (BATT_MAX - BATT_MIN) * 100));
  const color = pct > 40 ? 'var(--green)' : pct > 20 ? 'var(--amber)' : 'var(--red)';

  // Header mini bar
  const hBar = document.getElementById('battery-bar');
  hBar.style.width = pct + '%';
  hBar.style.background = color;
  document.getElementById('battery-voltage').textContent = v.toFixed(1) + 'V';

  // Status tab big gauge
  const bFill = document.getElementById('batt-fill');
  if (bFill) {
    bFill.style.width = pct + '%';
    bFill.style.background = color;
    setEl('batt-volts', v.toFixed(1) + ' V');
    setEl('batt-pct',   Math.round(pct) + '%');
  }
}

// ── Helpers ────────────────────────────────────────────────────────────────
function setEl(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function fmtSigned(n, decimals) {
  return (n >= 0 ? '+' : '') + n.toFixed(decimals);
}

function refreshNodeList() {
  const ros = getRos();
  if (!ros) return;
  ros.getNodes((nodes) => {
    const el = document.getElementById('nodes-list');
    if (!el) return;
    if (!nodes || nodes.length === 0) { el.textContent = 'No nodes found'; return; }
    el.textContent = nodes.sort().join('\n');
  });
}
