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
    messageType: 'vision_msgs/msg/Detection2DArray',
    throttle_rate: 500,  // 2 Hz — UI doesn't need faster
  });
  detectSub.subscribe((msg) => {
    lastDetections = summarizeDetections(msg.detections || []);
    updateDetectionOverlay();
  });
}

function summarizeDetections(detections) {
  const counts = {};
  for (const det of detections) {
    // det.id contains class name (set by yolo_detector_node)
    const name = det.id || 'unknown';
    counts[name] = (counts[name] || 0) + 1;
  }
  return Object.entries(counts)
    .map(([cls, count]) => ({ cls, count }))
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
  el.textContent = lastDetections
    .map(d => `${d.count} ${d.cls}`)
    .join(' · ');
}

/** Get the current CV feed topic (used by aio-camera.js). */
export function getCVTopic() {
  return cvFeedActive ? '/image_annotated' : '/image_raw';
}
