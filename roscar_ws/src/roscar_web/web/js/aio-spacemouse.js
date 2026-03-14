/**
 * aio-spacemouse.js — WebHID SpaceMouse driver module
 *
 * Handles device pairing, HID report parsing, signal processing, and
 * settings persistence for 3Dconnexion SpaceMouse devices.
 *
 * Exports:
 *   initSpaceMouse(callbacks)   — startup: checks WebHID, loads settings, auto-reconnects
 *   requestSpaceMouse()         — opens browser HID picker for pairing, returns boolean
 *   isSpaceMouseConnected()     — boolean: device opened
 *   isSpaceMouseActive()        — boolean: connected AND report within 200ms
 *   isSpaceMouseSleeping()      — boolean: connected AND no report for >5s
 *   getSpaceMouseVelocity()     — {vx, vy, wz} or null (processed, m/s + rad/s)
 */

// ── HID Protocol Constants ────────────────────────────────────────────────

/** Vendor IDs for SpaceMouse devices */
const VENDOR_IDS = [0x256F, 0x046D];

const HID_FILTERS = VENDOR_IDS.map(vendorId => ({ vendorId }));

/** Report IDs */
const REPORT_TRANSLATION = 1;
const REPORT_ROTATION    = 2;
const REPORT_BUTTONS     = 3;

/** Raw axis full-deflection magnitude */
const AXIS_MAX = 350;

// ── Staleness Thresholds ─────────────────────────────────────────────────

const STALE_MS = 200;   // no report → getSpaceMouseVelocity() returns null
const SLEEP_MS = 5000;  // no report → isSpaceMouseSleeping() returns true

// ── Settings Keys + Defaults ─────────────────────────────────────────────

const SETTINGS_DEFAULTS = {
  sm_deadzone:    5,
  sm_curve:       'exponential',
  sm_exponent:    2.0,
  sm_max_linear:  0.3,
  sm_max_angular: 1.0,
  sm_invert_x:    false,
  sm_invert_y:    false,
  sm_invert_rz:   false,
};

// ── Module State ─────────────────────────────────────────────────────────

let device        = null;   // HIDDevice | null
let lastReportAt  = 0;      // performance.now() of last inputreport

/** Latest raw axis values (counts, pre-processing) */
const raw = {
  cx:  0,   // translation X  (push right)
  cy:  0,   // translation Y  (push forward)
  crz: 0,   // rotation Rz    (twist CW)
};

/** Active settings (loaded from localStorage) */
let settings = { ...SETTINGS_DEFAULTS };

/** Callbacks registered by initSpaceMouse */
let _estop            = null;
let _onConnectionChange = null;

/** Button state for rising-edge E-STOP detection */
let _buttonsWerePressed = false;

// ── Public API ───────────────────────────────────────────────────────────

/**
 * Call once at startup. Checks WebHID availability, loads settings,
 * wires settings UI, registers disconnect handler, and auto-reconnects
 * to any previously paired device.
 *
 * @param {{ estop: Function, onConnectionChange: Function }} callbacks
 */
export function initSpaceMouse(callbacks) {
  _estop             = callbacks?.estop             ?? null;
  _onConnectionChange = callbacks?.onConnectionChange ?? null;

  if (!navigator.hid) {
    console.warn('[SpaceMouse] WebHID not available in this browser/context.');
    const btn = document.getElementById('sm-connect-btn');
    if (btn) btn.style.display = 'none';
    return;
  }

  loadSettings();
  setupSettingsUI();

  // Register disconnect handler
  navigator.hid.addEventListener('disconnect', ({ device: dev }) => {
    if (device && dev.vendorId === device.vendorId && dev.productId === device.productId) {
      _handleDisconnect();
    }
  });

  // Auto-reconnect to previously granted device
  navigator.hid.getDevices().then(devices => {
    const sm = devices.find(d => VENDOR_IDS.includes(d.vendorId));
    if (sm) {
      connectDevice(sm).catch(e => {
        console.warn('[SpaceMouse] Auto-reconnect failed:', e);
      });
    }
  });
}

/**
 * Opens the browser HID device picker for the user to select a SpaceMouse.
 * @returns {Promise<boolean>} true if a device was selected and opened
 */
export async function requestSpaceMouse() {
  if (!navigator.hid) return false;
  try {
    const devices = await navigator.hid.requestDevice({ filters: HID_FILTERS });
    if (!devices || devices.length === 0) return false;
    await connectDevice(devices[0]);
    return true;
  } catch (e) {
    console.warn('[SpaceMouse] requestDevice failed:', e);
    return false;
  }
}

/** @returns {boolean} true if a HIDDevice is currently open */
export function isSpaceMouseConnected() {
  return device !== null && device.opened;
}

/** @returns {boolean} connected AND a report was received within STALE_MS */
export function isSpaceMouseActive() {
  if (!isSpaceMouseConnected()) return false;
  return (performance.now() - lastReportAt) < STALE_MS;
}

/** @returns {boolean} connected AND no report for more than SLEEP_MS */
export function isSpaceMouseSleeping() {
  if (!isSpaceMouseConnected()) return false;
  if (lastReportAt === 0) return true;
  return (performance.now() - lastReportAt) > SLEEP_MS;
}

/**
 * Returns fully-processed robot velocities from the SpaceMouse.
 * Returns null if: not connected, no recent report (stale), or all axes in deadzone.
 * @returns {{ vx: number, vy: number, wz: number } | null}
 */
export function getSpaceMouseVelocity() {
  if (!isSpaceMouseActive()) return null;

  const { sm_deadzone, sm_max_linear, sm_max_angular } = settings;
  const deadzoneCount = sm_deadzone;  // deadzone is in raw counts

  // Apply deadzone, normalize, curve, scale, invert for each axis
  const vx  = processAxis(raw.cy,  deadzoneCount) *  sm_max_linear;  // forward
  const vyR = processAxis(raw.cx,  deadzoneCount) * -sm_max_linear;   // right push → -vy (ROS left-positive)
  const wzR = processAxis(raw.crz, deadzoneCount) * -sm_max_angular;  // CW twist → -wz (ROS CCW-positive)

  // Apply inversion toggles (sm_invert_x = fwd/back = vx, sm_invert_y = strafe = vy)
  const vxOut  = settings.sm_invert_x  ? -vx  : vx;
  const vyOut  = settings.sm_invert_y  ? -vyR : vyR;
  const wzOut  = settings.sm_invert_rz ? -wzR : wzR;

  // Return null if everything is zero after processing
  if (vxOut === 0 && vyOut === 0 && wzOut === 0) return null;

  return { vx: vxOut, vy: vyOut, wz: wzOut };
}

// ── Internal: Connection Management ──────────────────────────────────────

async function connectDevice(dev) {
  if (!dev.opened) {
    await dev.open();
  }
  device       = dev;
  lastReportAt = 0;

  dev.addEventListener('inputreport', _handleInputReport);

  _showSettingsPanel(true);
  if (_onConnectionChange) _onConnectionChange(true, dev.productName || dev.vendorId.toString(16));
  console.info(`[SpaceMouse] Connected: ${dev.productName} (${dev.vendorId.toString(16)}:${dev.productId.toString(16)})`);
}

function _handleDisconnect() {
  console.info('[SpaceMouse] Disconnected.');
  if (device) {
    device.removeEventListener('inputreport', _handleInputReport);
    device = null;
  }
  raw.cx = 0; raw.cy = 0; raw.crz = 0;
  lastReportAt = 0;
  _buttonsWerePressed = false;
  _showSettingsPanel(false);
  if (_onConnectionChange) _onConnectionChange(false, null);
}

// ── Internal: HID Report Parsing ─────────────────────────────────────────

function _handleInputReport(event) {
  const { reportId, data } = event;
  lastReportAt = performance.now();

  if (reportId === REPORT_TRANSLATION && data.byteLength >= 12) {
    // 12-byte combined report (SpaceMouse Wireless): TX, TY, TZ, RX, RY, RZ (Int16LE each)
    raw.cx  = data.getInt16(0, true);   // X: right push
    raw.cy  = -data.getInt16(2, true);  // Y: forward push (HID forward = -Y)
    raw.crz = data.getInt16(10, true);  // Rz at offset 10: twist CW
  } else if (reportId === REPORT_TRANSLATION && data.byteLength >= 6) {
    // 6-byte translation-only report (SpaceMouse Pro/Enterprise)
    raw.cx = data.getInt16(0, true);
    raw.cy = -data.getInt16(2, true);
  } else if (reportId === REPORT_ROTATION) {
    if (data.byteLength >= 6) {
      // Full 6-byte report (SpaceMouse Pro, Enterprise): Rx, Ry, Rz
      raw.crz = data.getInt16(4, true); // Rz at offset 4
    } else if (data.byteLength >= 2) {
      // Short 2-byte report (SpaceMouse Compact, Wireless): Rz only
      raw.crz = data.getInt16(0, true); // Rz at offset 0
    }
  } else if (reportId === REPORT_BUTTONS) {
    // Detect rising edge: fire E-STOP only on 0→non-zero transition
    let anyPressed = false;
    for (let i = 0; i < data.byteLength; i++) {
      if (data.getUint8(i) !== 0) { anyPressed = true; break; }
    }
    if (anyPressed && !_buttonsWerePressed && _estop) {
      _estop();
    }
    _buttonsWerePressed = anyPressed;
  }
}

// ── Internal: Signal Processing ──────────────────────────────────────────

/**
 * Process a raw axis count through the full pipeline:
 *   deadzone → normalize → sensitivity curve → output [-1, +1]
 *
 * @param {number} rawCount  Raw HID count value
 * @param {number} deadzone  Deadzone in raw counts
 * @returns {number}  Processed value in [-1, +1]
 */
function processAxis(rawCount, deadzone) {
  // 1. Apply deadzone
  const abs = Math.abs(rawCount);
  if (abs <= deadzone) return 0;

  // 2. Rescale post-deadzone range [deadzone, AXIS_MAX] to [0, 1], then normalize to [-1, +1]
  const sign = rawCount < 0 ? -1 : 1;
  const norm = Math.min(1, (abs - deadzone) / (AXIS_MAX - deadzone));

  // 3. Apply sensitivity curve
  return applyCurve(sign * norm);
}

/**
 * Apply the currently selected sensitivity curve.
 * @param {number} x  Normalized input in [-1, +1]
 * @returns {number}  Curved output in [-1, +1]
 */
function applyCurve(x) {
  const { sm_curve, sm_exponent: e } = settings;
  const sign = x < 0 ? -1 : 1;
  const abs  = Math.abs(x);

  switch (sm_curve) {
    case 'linear':
      return x;

    case 'exponential':
      // output = sign(x) * |x|^e
      return sign * Math.pow(abs, e);

    case 's-curve':
      // output = sign(x) * (3|x|^e - 2|x|^(e+1))
      return sign * (3 * Math.pow(abs, e) - 2 * Math.pow(abs, e + 1));

    default:
      return x;
  }
}

// ── Internal: Settings Persistence ───────────────────────────────────────

function loadSettings() {
  const s = { ...SETTINGS_DEFAULTS };
  try {
    for (const key of Object.keys(SETTINGS_DEFAULTS)) {
      const stored = localStorage.getItem(key);
      if (stored === null) continue;
      const def = SETTINGS_DEFAULTS[key];
      if (typeof def === 'boolean') {
        s[key] = stored === 'true';
      } else if (typeof def === 'number') {
        const n = parseFloat(stored);
        if (!isNaN(n)) s[key] = n;
      } else {
        s[key] = stored;
      }
    }
  } catch (e) {
    console.warn('[SpaceMouse] Could not read localStorage:', e);
  }
  settings = s;
}

function saveSetting(key, value) {
  settings[key] = value;
  try {
    localStorage.setItem(key, String(value));
  } catch (e) {
    console.warn('[SpaceMouse] Could not write localStorage:', e);
  }
}

// ── Internal: Settings UI ─────────────────────────────────────────────────

function setupSettingsUI() {
  // ── Curve selector buttons ────────────────────────────────────────────
  const curveBtns = document.querySelectorAll('.sm-curve-btns .btn-sm');
  curveBtns.forEach(btn => {
    const curve = btn.dataset.curve;
    if (!curve) return;
    // Set initial active state
    if (curve === settings.sm_curve) btn.classList.add('active');
    btn.addEventListener('click', () => {
      curveBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      saveSetting('sm_curve', curve);
    });
  });

  // ── Sliders ──────────────────────────────────────────────────────────
  _bindSlider('sm-exponent',    'sm_exponent',    2,  v => v.toFixed(1));
  _bindSlider('sm-deadzone',    'sm_deadzone',    0,  v => String(Math.round(v)));
  _bindSlider('sm-max-linear',  'sm_max_linear',  2,  v => v.toFixed(2) + ' m/s');
  _bindSlider('sm-max-angular', 'sm_max_angular', 1,  v => v.toFixed(1) + ' rad/s');

  // ── Inversion checkboxes ─────────────────────────────────────────────
  _bindCheckbox('sm-invert-x',  'sm_invert_x');
  _bindCheckbox('sm-invert-y',  'sm_invert_y');
  _bindCheckbox('sm-invert-rz', 'sm_invert_rz');

}

/**
 * Bind a range slider to a settings key.
 * Looks for the slider by id and a display span by id + '-val'.
 */
function _bindSlider(id, key, decimals, formatFn) {
  const slider = document.getElementById(id);
  const display = document.getElementById(id + '-val');
  if (!slider) return;

  // Set initial value from settings
  slider.value = settings[key];
  if (display) display.textContent = formatFn(settings[key]);

  slider.addEventListener('input', () => {
    const val = parseFloat(slider.value);
    if (!isNaN(val)) {
      saveSetting(key, val);
      if (display) display.textContent = formatFn(val);
    }
  });
}

/**
 * Bind a checkbox to a boolean settings key.
 */
function _bindCheckbox(id, key) {
  const cb = document.getElementById(id);
  if (!cb) return;
  cb.checked = settings[key];
  cb.addEventListener('change', () => {
    saveSetting(key, cb.checked);
  });
}

/**
 * Show or hide the SpaceMouse settings panel.
 */
function _showSettingsPanel(visible) {
  const panel = document.getElementById('sm-settings');
  if (!panel) return;
  panel.style.display = visible ? '' : 'none';
}
