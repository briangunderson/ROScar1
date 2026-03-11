/**
 * aio-diagnostics.js — /rosout log viewer for AIO dashboard
 * Subscribes to /rosout, displays colour-coded log entries with severity filtering.
 */

import { onAppEvent } from './aio-app.js';

const MAX_ENTRIES = 500;

const LEVEL = { DEBUG: 10, INFO: 20, WARN: 30, ERROR: 40, FATAL: 50 };

const LEVEL_NAME = {
  [LEVEL.DEBUG]: 'debug',
  [LEVEL.INFO]:  'info',
  [LEVEL.WARN]:  'warn',
  [LEVEL.ERROR]: 'error',
  [LEVEL.FATAL]: 'fatal',
};

const LEVEL_COLOR = {
  [LEVEL.DEBUG]: 'var(--text-dim)',
  [LEVEL.INFO]:  'var(--primary)',
  [LEVEL.WARN]:  'var(--amber)',
  [LEVEL.ERROR]: 'var(--red)',
  [LEVEL.FATAL]: 'var(--red)',
};

// Minimum level for each filter button
const FILTER_MIN = {
  all:  LEVEL.DEBUG,
  info: LEVEL.INFO,
  warn: LEVEL.WARN,
  err:  LEVEL.ERROR,
};

let getRos       = null;
let rosoutSub    = null;
let logContainer = null;
let entries      = [];      // all stored entries (max MAX_ENTRIES)
let activeFilter = 'all';
let autoScroll   = true;

// ── Formatting ──────────────────────────────────────────────────────────────

function formatTime(stamp) {
  const d = new Date(stamp.sec * 1000);
  const hh = String(d.getHours()).padStart(2, '0');
  const mm = String(d.getMinutes()).padStart(2, '0');
  const ss = String(d.getSeconds()).padStart(2, '0');
  return `${hh}:${mm}:${ss}`;
}

function levelTag(level) {
  if (level >= LEVEL.FATAL) return 'fatal';
  if (level >= LEVEL.ERROR) return 'error';
  if (level >= LEVEL.WARN)  return 'warn';
  if (level >= LEVEL.INFO)  return 'info';
  return 'debug';
}

// ── Entry creation ──────────────────────────────────────────────────────────

function createEntryEl(entry) {
  const div = document.createElement('div');
  div.className = 'log-entry';
  div.dataset.level = levelTag(entry.level);
  div.style.color = LEVEL_COLOR[entry.level] || LEVEL_COLOR[LEVEL.DEBUG];
  div.textContent = `[${entry.time}] ${entry.name}: ${entry.msg}`;

  // apply current filter visibility
  const minLevel = FILTER_MIN[activeFilter] || LEVEL.DEBUG;
  if (entry.level < minLevel) {
    div.style.display = 'none';
  }

  return div;
}

// ── Filtering ───────────────────────────────────────────────────────────────

function applyFilter(severity) {
  activeFilter = severity;
  const minLevel = FILTER_MIN[severity] || LEVEL.DEBUG;

  const children = logContainer.children;
  for (let i = 0; i < children.length; i++) {
    const el = children[i];
    const tag = el.dataset.level;
    // map tag back to numeric level for comparison
    const numLevel = tag === 'fatal' ? LEVEL.FATAL
                   : tag === 'error' ? LEVEL.ERROR
                   : tag === 'warn'  ? LEVEL.WARN
                   : tag === 'info'  ? LEVEL.INFO
                   : LEVEL.DEBUG;
    el.style.display = numLevel >= minLevel ? '' : 'none';
  }

  // scroll to bottom after filter change
  scrollToBottom();
}

// ── Auto-scroll ─────────────────────────────────────────────────────────────

function scrollToBottom() {
  const parent = logContainer.parentElement;
  if (parent) {
    parent.scrollTop = parent.scrollHeight;
  }
}

function checkAutoScroll() {
  const parent = logContainer.parentElement;
  if (!parent) return;
  const threshold = 10;
  autoScroll = (parent.scrollTop + parent.clientHeight >= parent.scrollHeight - threshold);
}

// ── ROS subscription ────────────────────────────────────────────────────────

function subscribe() {
  const ros = getRos();
  if (!ros) return;

  // clean up previous subscription
  if (rosoutSub) {
    try { rosoutSub.unsubscribe(); } catch (_) {}
    rosoutSub = null;
  }

  rosoutSub = new ROSLIB.Topic({
    ros,
    name: '/rosout',
    messageType: 'rcl_interfaces/Log',
  });

  rosoutSub.subscribe((msg) => {
    const entry = {
      level: msg.level,
      time:  formatTime(msg.stamp),
      name:  msg.name || '??',
      msg:   msg.msg  || '',
    };

    // FIFO cap
    if (entries.length >= MAX_ENTRIES) {
      entries.shift();
      if (logContainer.firstChild) {
        logContainer.removeChild(logContainer.firstChild);
      }
    }

    entries.push(entry);
    const el = createEntryEl(entry);
    logContainer.appendChild(el);

    if (autoScroll) {
      scrollToBottom();
    }
  });
}

// ── Init ────────────────────────────────────────────────────────────────────

export function initDiagnostics(getRosFn) {
  getRos = getRosFn;
  logContainer = document.getElementById('log-container');

  // Filter button listeners
  const filterBtns = document.querySelectorAll('#panel-diag .filter-btn[data-severity]');
  filterBtns.forEach((btn) => {
    btn.addEventListener('click', () => {
      filterBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      applyFilter(btn.dataset.severity);
    });
  });

  // Track user scroll to pause/resume auto-scroll
  const scrollParent = logContainer.parentElement;
  if (scrollParent) {
    scrollParent.addEventListener('scroll', checkAutoScroll);
  }

  // Subscribe on connection
  onAppEvent((event) => {
    if (event === 'connected') {
      subscribe();
    }
  });
}
