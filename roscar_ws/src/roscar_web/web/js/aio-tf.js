/**
 * aio-tf.js — TF tree visualizer.
 * Subscribes to /tf and /tf_static, renders a horizontal node diagram.
 */

import { onAppEvent } from './aio-app.js';

// ── State ────────────────────────────────────────────────────────────────────
const frames = new Map();   // frame_name -> { parent, isStatic, lastSeen }
let lastStructureKey = '';   // serialised structure for change detection
let stalenessTimer = null;
let getRos = null;
const subs = [];

// ── Subscriptions ────────────────────────────────────────────────────────────
function subscribe() {
  const ros = getRos();
  if (!ros) return;

  unsub();

  const tfTopic = new ROSLIB.Topic({
    ros,
    name: '/tf',
    messageType: 'tf2_msgs/TFMessage',
    throttle_rate: 1000,
  });
  tfTopic.subscribe((msg) => handleTFMsg(msg, false));
  subs.push(tfTopic);

  const tfStaticTopic = new ROSLIB.Topic({
    ros,
    name: '/tf_static',
    messageType: 'tf2_msgs/TFMessage',
    throttle_rate: 0,
  });
  tfStaticTopic.subscribe((msg) => handleTFMsg(msg, true));
  subs.push(tfStaticTopic);

  // Update staleness styling every 2s
  if (stalenessTimer) clearInterval(stalenessTimer);
  stalenessTimer = setInterval(updateStaleness, 2000);
}

function unsub() {
  subs.forEach((s) => { try { s.unsubscribe(); } catch (_) {} });
  subs.length = 0;
}

// ── Message handling ─────────────────────────────────────────────────────────
function handleTFMsg(msg, isStatic) {
  if (!msg.transforms) return;
  const now = Date.now();
  let changed = false;

  for (const t of msg.transforms) {
    const parent = t.header.frame_id;
    const child  = t.child_frame_id;
    const prev   = frames.get(child);

    if (!prev || prev.parent !== parent || prev.isStatic !== isStatic) {
      changed = true;
    }
    frames.set(child, { parent, isStatic, lastSeen: now });

    // Ensure parent exists in the map (as a root if not already a child)
    if (!frames.has(parent)) {
      frames.set(parent, { parent: null, isStatic: false, lastSeen: now });
      changed = true;
    }
  }

  if (changed) {
    scheduleRebuild();
  }
}

// ── Tree rendering (debounced) ───────────────────────────────────────────────
let rebuildTimeout = null;

function scheduleRebuild() {
  if (rebuildTimeout) return;
  rebuildTimeout = setTimeout(() => {
    rebuildTimeout = null;
    rebuildTree();
  }, 200);
}

function rebuildTree() {
  const container = document.getElementById('tf-tree');
  if (!container) return;

  // Build structure key to avoid unnecessary DOM rebuilds
  const key = buildStructureKey();
  if (key === lastStructureKey) return;
  lastStructureKey = key;

  container.innerHTML = '';

  // Find root nodes: frames whose parent is null or whose parent is not itself a child
  const roots = [];
  for (const [name, info] of frames) {
    if (info.parent === null) {
      roots.push(name);
    }
  }

  // If no explicit roots found (all frames are children), find topmost parents
  if (roots.length === 0) {
    const childSet = new Set(frames.keys());
    for (const [, info] of frames) {
      if (info.parent) childSet.delete(info.parent);
    }
    // Remaining in childSet are not parents of anything; find frames whose parent is not in frames
    for (const [name, info] of frames) {
      if (info.parent && !frames.has(info.parent)) {
        roots.push(name);
      }
    }
  }

  if (roots.length === 0 && frames.size > 0) {
    // Fallback: pick all frames that have no parent entry in the map
    for (const [name, info] of frames) {
      if (!info.parent) roots.push(name);
    }
  }

  roots.sort();
  for (const root of roots) {
    container.appendChild(renderBranch(root));
  }
}

function buildStructureKey() {
  const parts = [];
  const sorted = [...frames.entries()].sort((a, b) => a[0].localeCompare(b[0]));
  for (const [name, info] of sorted) {
    parts.push(`${name}:${info.parent || ''}:${info.isStatic ? 1 : 0}`);
  }
  return parts.join('|');
}

function getChildren(parentName) {
  const children = [];
  for (const [name, info] of frames) {
    if (info.parent === parentName) {
      children.push(name);
    }
  }
  return children.sort();
}

function renderBranch(frameName) {
  const info = frames.get(frameName);
  const children = getChildren(frameName);

  // Single chain (no branching): render as horizontal row
  const row = document.createElement('div');
  row.className = 'tf-row';

  row.appendChild(makeNode(frameName, info));

  if (children.length === 1) {
    row.appendChild(makeArrow());
    row.appendChild(renderBranch(children[0]));
  } else if (children.length > 1) {
    row.appendChild(makeArrow());
    const col = document.createElement('div');
    col.className = 'tf-col';
    for (const child of children) {
      col.appendChild(renderBranch(child));
    }
    row.appendChild(col);
  }

  return row;
}

function makeNode(name, info) {
  const el = document.createElement('div');
  el.className = 'tf-node';
  el.textContent = name;
  el.dataset.frame = name;

  if (info && info.isStatic) el.classList.add('tf-static');

  applyActivityClass(el, info);
  return el;
}

function makeArrow() {
  const el = document.createElement('span');
  el.className = 'tf-arrow';
  el.textContent = '\u2192';
  return el;
}

// ── Staleness updates (no DOM rebuild) ───────────────────────────────────────
function updateStaleness() {
  const container = document.getElementById('tf-tree');
  if (!container) return;

  const nodes = container.querySelectorAll('.tf-node');
  for (const el of nodes) {
    const name = el.dataset.frame;
    const info = frames.get(name);
    applyActivityClass(el, info);
  }
}

function applyActivityClass(el, info) {
  el.classList.remove('tf-active', 'tf-stale');
  if (!info) return;

  if (info.isStatic) {
    // Static frames are always considered active
    el.classList.add('tf-active');
  } else if (Date.now() - info.lastSeen < 5000) {
    el.classList.add('tf-active');
  } else {
    el.classList.add('tf-stale');
  }
}

// ── Init ─────────────────────────────────────────────────────────────────────
export function initTF(getRosFn) {
  getRos = getRosFn;
  onAppEvent((ev) => {
    if (ev === 'connected') subscribe();
  });
}
