/**
 * aio-cv.js — Computer vision overlay + feed toggle.
 * Subscribes to /detections for object detection results,
 * provides toggle between raw and annotated camera feeds.
 */

import { HOST, PORTS, onAppEvent } from './aio-app.js';

let getRos;
let detectSub = null;
let cvFeedActive = false;  // false = /image_raw, true = /image_annotated
let lastDetections = [];   // [{cls: 'person', count: 2}, ...]

const PANEL = '#panel-camera';

export function initCV(getRosFn) {
  getRos = getRosFn;
  setupCVToggle();
  onAppEvent((ev) => {
    if (ev === 'connected') subscribeDetections();
  });
}

/** Switch camera feed between raw and annotated. */
export function setCVFeed(on) {
  cvFeedActive = on;
  updateFeedToggleUI();
  // Trigger camera stream restart with new topic
  const event = new CustomEvent('cv-feed-change', { detail: { annotated: on } });
  window.dispatchEvent(event);
}

// ── Subscriptions ────────────────────────────────────────────────────────────
function subscribeDetections() {
  const ros = getRos(); if (!ros) return;
  if (detectSub) { try { detectSub.unsubscribe(); } catch (_) {} }
  detectSub = new ROSLIB.Topic({
    ros,
    name: '/detections',
    messageType: 'vision_msgs/Detection2DArray',
    throttle_rate: 500,  // 2 Hz — UI doesn't need faster
  });
  detectSub.subscribe((msg) => {
    lastDetections = summarizeDetections(msg.detections || []);
    updateDetectionOverlay();
  });
}

function summarizeDetections(detections) {
  // Group by class and track the closest distance for each class.
  // Class name lives at det.results[0].hypothesis.class_id; det.id is
  // a tracker handle (not the class). Distance, when available, is
  // det.results[0].pose.pose.position.z (depth in meters from the
  // camera optical frame). 0.0 means "unknown" — D435i not running or
  // the bbox center had no valid depth return.
  const groups = {};
  for (const det of detections) {
    const hyp = (det.results && det.results[0]) || null;
    const name = (hyp && hyp.hypothesis && hyp.hypothesis.class_id)
      || det.id
      || 'unknown';
    const z = hyp && hyp.pose && hyp.pose.pose && hyp.pose.pose.position
      ? Number(hyp.pose.pose.position.z) || 0
      : 0;
    if (!groups[name]) groups[name] = { count: 0, minDist: 0 };
    groups[name].count += 1;
    if (z > 0 && (groups[name].minDist === 0 || z < groups[name].minDist)) {
      groups[name].minDist = z;
    }
  }
  return Object.entries(groups)
    .map(([cls, info]) => ({ cls, count: info.count, minDist: info.minDist }))
    .sort((a, b) => b.count - a.count);
}

// ── UI ───────────────────────────────────────────────────────────────────────
function setupCVToggle() {
  const btn = document.getElementById('cv-feed-toggle');
  if (btn) {
    btn.addEventListener('click', () => {
      setCVFeed(!cvFeedActive);
    });
  }
}

function updateFeedToggleUI() {
  const btn = document.getElementById('cv-feed-toggle');
  if (btn) {
    btn.classList.toggle('active', cvFeedActive);
    btn.textContent = cvFeedActive ? 'CV' : 'RAW';
    btn.title = cvFeedActive ? 'Showing annotated feed' : 'Showing raw feed';
  }
}

function updateDetectionOverlay() {
  const el = document.getElementById('cv-detections');
  if (!el) return;
  if (lastDetections.length === 0) {
    el.textContent = '';
    el.style.display = 'none';
    return;
  }
  el.style.display = '';
  // Format: "2 person 1.4m · 1 chair · 1 cup 0.6m"
  // Distance shown only when > 0 (D435i sees the object); count alone
  // when depth is unknown.
  el.textContent = lastDetections
    .map(d => {
      const dist = d.minDist > 0 ? ` ${d.minDist.toFixed(1)}m` : '';
      return `${d.count} ${d.cls}${dist}`;
    })
    .join(' · ');
}

/** Get the current CV feed topic (used by aio-camera.js). */
export function getCVTopic() {
  return cvFeedActive ? '/image_annotated' : '/image_raw';
}
