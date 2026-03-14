# SpaceMouse Teleop Integration — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add proportional 3Dconnexion SpaceMouse Wireless control to the AIO web dashboard via Chrome WebHID API, plus fix the right virtual joystick to support forward/back.

**Architecture:** New `aio-spacemouse.js` ES module handles WebHID connection, HID report parsing, signal processing (deadzone/curve/scaling), and settings UI. Integrates into existing teleop priority system via a polled getter function. No ROS-side changes needed — all browser-side.

**Tech Stack:** WebHID API (Chrome 89+), ES modules, localStorage, existing rosbridge/roslibjs stack.

**Spec:** `docs/superpowers/specs/2026-03-13-spacemouse-teleop-design.md`

**Secure Context Prerequisite:** User must add `http://192.168.1.170:8888` to `chrome://flags/#unsafely-treat-insecure-origin-as-secure` and relaunch Chrome. Already done.

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `web/js/aio-spacemouse.js` | CREATE | WebHID connection, HID report parsing, signal processing, settings UI, button E-STOP |
| `web/js/aio-teleop.js` | MODIFY | Add SpaceMouse to priority system, fix right joystick Y-axis |
| `web/js/aio-app.js` | MODIFY | Import + initialize SpaceMouse module, pass estop callback |
| `web/aio.html` | MODIFY | Add Connect button, SM indicator, settings panel markup |
| `web/css/aio.css` | MODIFY | SpaceMouse settings panel styles |

All paths relative to `roscar_ws/src/roscar_web/`.

---

## Chunk 1: Core SpaceMouse Module

### Task 1: Create `aio-spacemouse.js` — Connection Management

**Files:**
- Create: `web/js/aio-spacemouse.js`

This is the core module. We build it incrementally, starting with WebHID connection lifecycle.

- [ ] **Step 1: Create module skeleton with exports and state**

Create `web/js/aio-spacemouse.js`:

```javascript
/* aio-spacemouse.js – WebHID 3Dconnexion SpaceMouse driver for AIO dashboard */

// ── Vendor/product filters for WebHID device picker ──
const HID_FILTERS = [
  { vendorId: 0x256F },   // 3Dconnexion
  { vendorId: 0x046D },   // Logitech (legacy 3Dconnexion devices)
];

// ── Module state ──
let device = null;            // HIDDevice instance
let callbacks = {};           // { estop, onConnectionChange }
let lastReportTime = 0;       // timestamp of last HID input report
const STALE_MS = 200;         // staleness timeout — return null if no report in this window
const SLEEP_MS = 5000;        // show "sleeping" if no report in this window

// Raw axis state (from HID reports, updated async ~100Hz)
let rawTranslation = { x: 0, y: 0, z: 0 };
let rawRotation    = { rx: 0, ry: 0, rz: 0 };

// ── Settings (loaded from localStorage) ──
const DEFAULTS = {
  sm_deadzone:    5,
  sm_curve:       'exponential',
  sm_exponent:    2.0,
  sm_max_linear:  0.3,
  sm_max_angular: 1.0,
  sm_invert_x:    false,
  sm_invert_y:    false,
  sm_invert_rz:   false,
};
let settings = { ...DEFAULTS };

function loadSettings() {
  for (const [key, def] of Object.entries(DEFAULTS)) {
    const stored = localStorage.getItem(key);
    if (stored !== null) {
      if (typeof def === 'boolean')      settings[key] = stored === 'true';
      else if (typeof def === 'number')  settings[key] = Number(stored);
      else                               settings[key] = stored;
    } else {
      settings[key] = def;
    }
  }
}

function saveSetting(key, value) {
  settings[key] = value;
  localStorage.setItem(key, String(value));
}

// ── Public API ──

export function isSpaceMouseConnected() {
  return device !== null;
}

export function isSpaceMouseActive() {
  return device !== null && (Date.now() - lastReportTime) < STALE_MS;
}

export function isSpaceMouseSleeping() {
  return device !== null && (Date.now() - lastReportTime) > SLEEP_MS;
}

export function getSpaceMouseVelocity() {
  // Implemented in Task 2
  return null;
}

export async function initSpaceMouse(cbs) {
  callbacks = cbs || {};
  loadSettings();

  if (!navigator.hid) {
    console.warn('[SpaceMouse] WebHID not available. Requires secure context (HTTPS or chrome://flags insecure origin allowlist).');
    const btn = document.getElementById('sm-connect-btn');
    if (btn) btn.style.display = 'none';
    return;
  }

  // Listen for device disconnection
  navigator.hid.addEventListener('disconnect', (e) => {
    if (device && e.device === device) {
      console.warn('[SpaceMouse] Device disconnected.');
      device = null;
      rawTranslation = { x: 0, y: 0, z: 0 };
      rawRotation = { rx: 0, ry: 0, rz: 0 };
      if (callbacks.onConnectionChange) callbacks.onConnectionChange(false);
      const settingsEl = document.getElementById('sm-settings');
      if (settingsEl) settingsEl.style.display = 'none';
    }
  });

  // Try to auto-reconnect to a previously paired device
  try {
    const devices = await navigator.hid.getDevices();
    const sm = devices.find(d =>
      HID_FILTERS.some(f => f.vendorId === d.vendorId)
    );
    if (sm) {
      await connectDevice(sm);
    }
  } catch (err) {
    console.warn('[SpaceMouse] Auto-reconnect failed:', err);
  }
}

export async function requestSpaceMouse() {
  if (!navigator.hid) {
    console.warn('[SpaceMouse] WebHID not available.');
    return false;
  }
  try {
    const [selected] = await navigator.hid.requestDevice({ filters: HID_FILTERS });
    if (selected) {
      await connectDevice(selected);
      return true;
    }
  } catch (err) {
    console.error('[SpaceMouse] Pairing failed:', err);
  }
  return false;
}

// ── Connection lifecycle ──

async function connectDevice(dev) {
  if (!dev.opened) {
    await dev.open();
  }
  device = dev;
  lastReportTime = Date.now();
  device.addEventListener('inputreport', onInputReport);
  console.log(`[SpaceMouse] Connected: ${dev.productName} (vendor:0x${dev.vendorId.toString(16)} product:0x${dev.productId.toString(16)})`);
  if (callbacks.onConnectionChange) callbacks.onConnectionChange(true);
}

function onInputReport(e) {
  lastReportTime = Date.now();
  const { reportId, data } = e;

  if (reportId === 1) {
    // Translation: 3x Int16LE
    rawTranslation.x = data.getInt16(0, true);
    rawTranslation.y = data.getInt16(2, true);
    rawTranslation.z = data.getInt16(4, true);
  } else if (reportId === 2) {
    // Rotation: 3x Int16LE
    rawRotation.rx = data.getInt16(0, true);
    rawRotation.ry = data.getInt16(2, true);
    rawRotation.rz = data.getInt16(4, true);
  } else if (reportId === 3) {
    // Buttons: any non-zero byte in the report = E-STOP
    let anyPressed = false;
    for (let i = 0; i < data.byteLength; i++) {
      if (data.getUint8(i) !== 0) { anyPressed = true; break; }
    }
    if (anyPressed && callbacks.estop) {
      callbacks.estop();
    }
  }
}
```

- [ ] **Step 2: Verify file loads without errors**

Open browser console, check for import errors. The module won't be imported yet (we do that in Task 4), but verify syntax by temporarily adding to the browser console:
```javascript
import('./js/aio-spacemouse.js').then(m => console.log('SpaceMouse module loaded:', Object.keys(m)));
```

- [ ] **Step 3: Commit**

```bash
git add web/js/aio-spacemouse.js
git commit -m "feat(spacemouse): add WebHID connection and HID report parsing module"
```

---

### Task 2: Signal Processing — Deadzone, Curves, Scaling

**Files:**
- Modify: `web/js/aio-spacemouse.js`

Implement the `getSpaceMouseVelocity()` function with the full signal processing pipeline.

- [ ] **Step 1: Add signal processing functions**

Replace the placeholder `getSpaceMouseVelocity()` in `aio-spacemouse.js` and add helper functions. Insert these just before the existing `getSpaceMouseVelocity` export:

```javascript
// ── Signal processing ──

const AXIS_MAX = 350;  // approximate max raw value at full puck deflection

function applyDeadzone(raw, deadzone) {
  return Math.abs(raw) < deadzone ? 0 : raw;
}

function normalize(raw) {
  // Clamp to [-1, +1] based on known axis range
  return Math.max(-1, Math.min(1, raw / AXIS_MAX));
}

function applyCurve(normalized, curve, exponent) {
  const abs = Math.abs(normalized);
  const sign = Math.sign(normalized);
  switch (curve) {
    case 'exponential':
      return sign * Math.pow(abs, exponent);
    case 's-curve':
      // Tunable smoothstep: 3t^e - 2t^(e+1)
      return sign * (3 * Math.pow(abs, exponent) - 2 * Math.pow(abs, exponent + 1));
    case 'linear':
    default:
      return normalized;
  }
}
```

Then replace the `getSpaceMouseVelocity` export:

```javascript
export function getSpaceMouseVelocity() {
  if (!device) return null;
  if ((Date.now() - lastReportTime) > STALE_MS) return null;

  const { sm_deadzone, sm_curve, sm_exponent,
          sm_max_linear, sm_max_angular,
          sm_invert_x, sm_invert_y, sm_invert_rz } = settings;

  // Apply deadzone to raw values
  const tx = applyDeadzone(rawTranslation.x, sm_deadzone);
  const ty = applyDeadzone(rawTranslation.y, sm_deadzone);
  const rz = applyDeadzone(rawRotation.rz,   sm_deadzone);

  // If all zero after deadzone, return null (no active input)
  if (tx === 0 && ty === 0 && rz === 0) return null;

  // Normalize to [-1, +1]
  const nx = normalize(tx);
  const ny = normalize(ty);
  const nrz = normalize(rz);

  // Apply sensitivity curve
  const cx = applyCurve(nx, sm_curve, sm_exponent);
  const cy = applyCurve(ny, sm_curve, sm_exponent);
  const crz = applyCurve(nrz, sm_curve, sm_exponent);

  // Map to robot velocities with inversion and scaling
  // SpaceMouse Y (push forward) → robot vx (drive forward)
  // SpaceMouse X (push right)   → robot vy (strafe — negate for ROS left-positive)
  // SpaceMouse Rz (twist CW)    → robot wz (rotate — negate for ROS CCW-positive)
  const vx = (sm_invert_x ? -1 : 1) * cy * sm_max_linear;
  const vy = (sm_invert_y ? -1 : 1) * -cx * sm_max_linear;
  const wz = (sm_invert_rz ? -1 : 1) * -crz * sm_max_angular;

  return { vx, vy, wz };
}
```

**NOTE on axis mapping:** SpaceMouse Translation-Y = push puck forward = positive → maps to robot vx (forward). SpaceMouse Translation-X = push puck right = positive → maps to robot -vy (ROS left-positive). SpaceMouse Rotation-Rz = twist CW = positive → maps to robot -wz (ROS CCW-positive). These sign conventions may need tweaking during hardware testing — that's what the inversion toggles are for.

- [ ] **Step 2: Manually test signal processing in browser console**

After module is imported (Task 4), test with:
```javascript
// Simulate raw HID data:
// With SpaceMouse connected, push puck and check console for:
// [SpaceMouse] vx=0.15, vy=-0.08, wz=0.00
```

- [ ] **Step 3: Commit**

```bash
git add web/js/aio-spacemouse.js
git commit -m "feat(spacemouse): add signal processing pipeline (deadzone, curves, scaling)"
```

---

### Task 3: Right Virtual Joystick Fix

**Files:**
- Modify: `web/js/aio-teleop.js` (lines 76–82, the right joystick handler)

- [ ] **Step 1: Modify right joystick move handler**

In `aio-teleop.js`, find the right joystick `move` handler (around line 76–82). Current code:

```javascript
  rJoy.on('move', (_e, d) => {
    const f = Math.min(d.force, 1);
    const a = d.angle.radian;
    joy.wz = f * -Math.cos(a) * maxAngular;
  });
  rJoy.on('end', () => { joy.wz = 0; });
```

Replace with:

```javascript
  rJoy.on('move', (_e, d) => {
    const f = Math.min(d.force, 1);
    const a = d.angle.radian;
    joy.wz = f * -Math.cos(a) * maxAngular;
    joy.vx = f * Math.sin(a) * maxLinear;
  });
  rJoy.on('end', () => { joy.wz = 0; joy.vx = 0; });
```

This adds Y-axis → vx (forward/back) to the right joystick. The `sin(angle)` component maps vertical joystick displacement to forward/back velocity. The left joystick's vx/vy mapping is unchanged.

**Note:** When both joysticks are active (right sending vx + wz, left sending vx + vy), the priority system picks "joystick" as the source and the `joy` object accumulates all values. Since both write to `joy.vx`, the right joystick's vx value will be the one published if the right joystick is the last one moved. This is acceptable — users will typically use one joystick at a time or left for strafe + right for drive+turn.

- [ ] **Step 2: Test right joystick in browser**

Open AIO dashboard, use right virtual joystick:
- Push up → robot drives forward (vx positive)
- Push down → robot drives backward (vx negative)
- Push left → robot rotates CCW (wz positive)
- Diagonal → combined drive + rotate

- [ ] **Step 3: Commit**

```bash
git add web/js/aio-teleop.js
git commit -m "fix(teleop): add forward/back axis to right virtual joystick"
```

---

### Task 4: Integrate SpaceMouse into App and Teleop

**Files:**
- Modify: `web/js/aio-app.js` (lines 7–14 imports, lines 113–124 init sequence)
- Modify: `web/js/aio-teleop.js` (lines 165–198 publishVelocity)

- [ ] **Step 1: Add SpaceMouse import and init in aio-app.js**

In `aio-app.js`, add import at top (after line 14):

```javascript
import { initSpaceMouse, requestSpaceMouse, isSpaceMouseConnected } from './aio-spacemouse.js';
```

In the module initialization sequence (around lines 114–122), add SpaceMouse init. The init calls are bare function calls (no try/catch wrapper). Add the `initSpaceMouse()` call after the last existing `init*()` call (e.g., after `initTF(getRos);`):

```javascript
    initSpaceMouse({
      estop,
      onConnectionChange: (connected) => {
        const btn = document.getElementById('sm-connect-btn');
        const indicator = document.getElementById('sm-status');
        if (btn) btn.textContent = connected ? 'SM: Connected' : 'Connect SpaceMouse';
        if (btn) btn.classList.toggle('active', connected);
        if (indicator) indicator.textContent = connected ? 'SM' : '';
      },
    });
```

Also add a click handler for the connect button. Add after the E-STOP listeners (after line 79):

```javascript
  // SpaceMouse connect button
  const smBtn = document.getElementById('sm-connect-btn');
  if (smBtn) {
    smBtn.addEventListener('click', async () => {
      if (!isSpaceMouseConnected()) {
        await requestSpaceMouse();
      }
    });
  }
```

- [ ] **Step 2: Add SpaceMouse priority in publishVelocity()**

In `aio-teleop.js`, add import at top (after line 7):

```javascript
import { getSpaceMouseVelocity } from './aio-spacemouse.js';
```

In `publishVelocity()` (around line 171–190), insert SpaceMouse check BEFORE the existing gamepad check. Find the priority block and replace:

```javascript
  // ── Priority: SpaceMouse > Gamepad > Joystick > Keyboard ──
  let vx = 0, vy = 0, wz = 0;
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
```

This replaces the existing priority logic (lines ~171–190) with the new 4-source priority.

- [ ] **Step 3: Commit**

```bash
git add web/js/aio-app.js web/js/aio-teleop.js
git commit -m "feat(spacemouse): integrate into app init and teleop priority system"
```

---

## Chunk 2: UI — HTML, CSS, Settings Panel

### Task 5: Add SpaceMouse UI Elements to HTML

**Files:**
- Modify: `web/aio.html` (lines 169–209, DRIVE panel)

- [ ] **Step 1: Add Connect button and SM indicator to DRIVE panel header**

In `aio.html`, find the DRIVE panel header. The panel starts around line 169:

```html
    <div class="panel drive" style="grid-area:drive">
      <div class="panel-hdr">DRIVE
```

The existing header already has `<span id="input-source" class="panel-title-info mono">KB</span>`. Add the SM indicator span and Connect button AFTER the existing input-source span. Find:

```html
        <span id="input-source" class="panel-title-info mono">KB</span>
```

Replace with:

```html
        <span id="input-source" class="panel-title-info mono">KB</span>
        <span id="sm-status" class="sm-tag"></span>
        <button id="sm-connect-btn" class="sm-connect-btn">Connect SpaceMouse</button>
```

- [ ] **Step 2: Add SpaceMouse settings panel**

In `aio.html`, add a collapsible settings section inside the DRIVE panel. Place it AFTER the closing `</div>` of `panel-body-drive` (the 3-column joystick/slider row) and BEFORE the panel's final closing `</div>`. This way it appears below the joystick/slider row, spanning the full panel width.

```html
        <!-- SpaceMouse settings (hidden until connected) -->
        <div id="sm-settings" class="sm-settings" style="display:none">
          <details>
            <summary class="sm-settings-title">SpaceMouse Settings</summary>
            <div class="sm-settings-body">
              <div class="sm-row">
                <span class="sm-label">Curve</span>
                <div class="sm-curve-btns">
                  <button data-curve="linear" class="btn-sm">Linear</button>
                  <button data-curve="exponential" class="btn-sm active">Expo</button>
                  <button data-curve="s-curve" class="btn-sm">S-Curve</button>
                </div>
              </div>
              <div class="sm-row">
                <span class="sm-label">Exponent</span>
                <input type="range" id="sm-exponent" min="1" max="3" step="0.1" value="2.0">
                <span id="sm-exponent-val" class="sm-val">2.0</span>
              </div>
              <div class="sm-row">
                <span class="sm-label">Deadzone</span>
                <input type="range" id="sm-deadzone" min="0" max="50" step="1" value="5">
                <span id="sm-deadzone-val" class="sm-val">5</span>
              </div>
              <div class="sm-row">
                <span class="sm-label">Linear</span>
                <input type="range" id="sm-max-linear" min="0.1" max="1.0" step="0.05" value="0.3">
                <span id="sm-max-linear-val" class="sm-val">0.3</span>
              </div>
              <div class="sm-row">
                <span class="sm-label">Angular</span>
                <input type="range" id="sm-max-angular" min="0.1" max="3.0" step="0.1" value="1.0">
                <span id="sm-max-angular-val" class="sm-val">1.0</span>
              </div>
              <div class="sm-row">
                <span class="sm-label">Invert</span>
                <label class="sm-check"><input type="checkbox" id="sm-invert-x"> Fwd/Back</label>
                <label class="sm-check"><input type="checkbox" id="sm-invert-y"> Strafe</label>
                <label class="sm-check"><input type="checkbox" id="sm-invert-rz"> Rotate</label>
              </div>
            </div>
          </details>
        </div>
```

- [ ] **Step 3: Commit**

```bash
git add web/aio.html
git commit -m "feat(spacemouse): add connect button and settings panel HTML"
```

---

### Task 6: Add SpaceMouse CSS Styles

**Files:**
- Modify: `web/css/aio.css`

- [ ] **Step 1: Add SpaceMouse styles**

Append to the end of `aio.css` (before any final comments):

```css
/* ── SpaceMouse ── */
.sm-connect-btn {
  margin-left: auto;
  padding: 2px 8px;
  font: 9px var(--mono);
  background: var(--bg2);
  color: var(--text-dim);
  border: 1px solid var(--border);
  border-radius: 3px;
  cursor: pointer;
  transition: all .15s;
}
.sm-connect-btn:hover { border-color: var(--primary); color: var(--primary); }
.sm-connect-btn.active {
  background: rgba(0, 212, 255, 0.1);
  border-color: var(--primary);
  color: var(--primary);
  cursor: default;
}
.sm-tag {
  color: var(--primary);
  font: 10px/1 var(--mono);
  margin-left: 4px;
}

.sm-settings {
  padding: 0 8px 4px;
}
.sm-settings summary.sm-settings-title {
  font: 9px var(--mono);
  color: var(--text-dim);
  cursor: pointer;
  padding: 4px 0;
  user-select: none;
}
.sm-settings summary.sm-settings-title:hover { color: var(--primary); }
.sm-settings-body {
  display: flex;
  flex-direction: column;
  gap: 4px;
  padding: 4px 0;
}
.sm-row {
  display: flex;
  align-items: center;
  gap: 6px;
  font: 9px var(--mono);
}
.sm-label {
  min-width: 55px;
  color: var(--text-dim);
}
.sm-val {
  min-width: 28px;
  text-align: right;
  color: var(--primary);
  font: 10px var(--mono);
}
.sm-row input[type="range"] { flex: 1; }
.sm-curve-btns {
  display: flex;
  gap: 3px;
}
.sm-curve-btns .btn-sm {
  padding: 1px 6px;
  font: 8px var(--mono);
  background: var(--bg2);
  color: var(--text-dim);
  border: 1px solid var(--border);
  border-radius: 2px;
  cursor: pointer;
}
.sm-curve-btns .btn-sm:hover { border-color: var(--text); }
.sm-curve-btns .btn-sm.active {
  background: rgba(0, 212, 255, 0.15);
  border-color: var(--primary);
  color: var(--primary);
}
.sm-check {
  font: 9px var(--mono);
  color: var(--text-dim);
  display: flex;
  align-items: center;
  gap: 3px;
  cursor: pointer;
}
.sm-check input[type="checkbox"] {
  accent-color: var(--primary);
  width: 12px;
  height: 12px;
}

/* Dim speed sliders when SpaceMouse is active source (still interactive for adjustment) */
.drive .slider-row.sm-dimmed {
  opacity: 0.3;
}
```

- [ ] **Step 2: Verify styles render correctly**

Open the AIO dashboard, inspect the DRIVE panel. The Connect button should appear in the panel header. Settings panel is hidden until SpaceMouse connects.

- [ ] **Step 3: Commit**

```bash
git add web/css/aio.css
git commit -m "feat(spacemouse): add CSS styles for connect button and settings panel"
```

---

### Task 7: Wire Settings UI to SpaceMouse Module

**Files:**
- Modify: `web/js/aio-spacemouse.js`

Add settings UI binding logic to the SpaceMouse module. This connects the HTML sliders/buttons to the settings state.

- [ ] **Step 1: Add setupSettingsUI function**

Add this function to `aio-spacemouse.js`, and call it from `initSpaceMouse()` after `loadSettings()`:

```javascript
function setupSettingsUI() {
  // Curve buttons
  document.querySelectorAll('.sm-curve-btns .btn-sm').forEach(btn => {
    if (btn.dataset.curve === settings.sm_curve) btn.classList.add('active');
    btn.addEventListener('click', () => {
      document.querySelectorAll('.sm-curve-btns .btn-sm').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      saveSetting('sm_curve', btn.dataset.curve);
    });
  });

  // Sliders
  const sliders = [
    { id: 'sm-exponent',    key: 'sm_exponent',    valId: 'sm-exponent-val',    fmt: v => Number(v).toFixed(1) },
    { id: 'sm-deadzone',    key: 'sm_deadzone',    valId: 'sm-deadzone-val',    fmt: v => String(v) },
    { id: 'sm-max-linear',  key: 'sm_max_linear',  valId: 'sm-max-linear-val',  fmt: v => Number(v).toFixed(2) },
    { id: 'sm-max-angular', key: 'sm_max_angular', valId: 'sm-max-angular-val', fmt: v => Number(v).toFixed(1) },
  ];
  for (const { id, key, valId, fmt } of sliders) {
    const el = document.getElementById(id);
    const valEl = document.getElementById(valId);
    if (!el) continue;
    el.value = settings[key];
    if (valEl) valEl.textContent = fmt(settings[key]);
    el.addEventListener('input', () => {
      const v = key === 'sm_deadzone' ? parseInt(el.value) : parseFloat(el.value);
      saveSetting(key, v);
      if (valEl) valEl.textContent = fmt(v);
    });
  }

  // Checkboxes
  const checks = [
    { id: 'sm-invert-x',  key: 'sm_invert_x' },
    { id: 'sm-invert-y',  key: 'sm_invert_y' },
    { id: 'sm-invert-rz', key: 'sm_invert_rz' },
  ];
  for (const { id, key } of checks) {
    const el = document.getElementById(id);
    if (!el) continue;
    el.checked = settings[key];
    el.addEventListener('change', () => saveSetting(key, el.checked));
  }
}
```

Then in `initSpaceMouse()`, add after `loadSettings()`:

```javascript
  setupSettingsUI();
```

- [ ] **Step 2: Add settings panel show/hide logic**

In the `onConnectionChange` callback (already wired in aio-app.js from Task 4), the panel visibility is handled. But also add visibility toggle inside the SpaceMouse module's `connectDevice` function. After the `callbacks.onConnectionChange(true)` line, add:

```javascript
  const settingsEl = document.getElementById('sm-settings');
  if (settingsEl) settingsEl.style.display = '';
```

- [ ] **Step 3: Add slider dimming when SM is active**

In `aio-teleop.js`, inside the `publishVelocity()` function, after the `setSource('SM')` line, add slider dimming:

```javascript
  // Dim speed sliders when SpaceMouse is active
  const sliderRows = document.querySelectorAll('.drive .slider-row');
  sliderRows.forEach(row => row.classList.toggle('sm-dimmed', smVel !== null));
```

Wait — this should run every cycle, not just when SM is active. Place it after the priority block, before publishing:

```javascript
  // Dim speed sliders when SpaceMouse is active source
  document.querySelectorAll('.drive .slider-row').forEach(
    row => row.classList.toggle('sm-dimmed', smVel !== null)
  );
```

- [ ] **Step 4: Commit**

```bash
git add web/js/aio-spacemouse.js web/js/aio-teleop.js
git commit -m "feat(spacemouse): wire settings UI sliders, curves, and inversion toggles"
```

---

## Chunk 3: Testing & Polish

### Task 8: Hardware Testing and Axis Calibration

This task is manual — connect the real SpaceMouse and verify everything works.

- [ ] **Step 1: Deploy to Pi**

The web files are symlinked from the git repo to `~/roscar_ws`, so changes should be live. Restart the web service to pick up any cached files:

```bash
ssh brian@192.168.1.170 "sudo systemctl restart roscar-web"
```

- [ ] **Step 2: Open AIO dashboard and connect SpaceMouse**

1. Open `http://192.168.1.170:8888/aio.html` in Chrome (with the secure context flag already set)
2. Click "Connect SpaceMouse" button
3. Select the SpaceMouse Wireless from the browser picker
4. Verify "SM: Connected" appears in the DRIVE panel header
5. Open browser console — look for `[SpaceMouse] Connected: ...` log

- [ ] **Step 3: Test axes and fix inversions**

Push the SpaceMouse puck in each direction and verify:
- Push forward → robot drives forward (vx positive in display)
- Push right → robot strafes right (vy negative in display)
- Twist CW → robot rotates CW (wz negative in display)

If any axis is inverted, use the Invert checkboxes in the settings panel. If the axis MAPPING is wrong (e.g., push forward produces strafe), the axis mapping constants in `getSpaceMouseVelocity()` need adjustment.

- [ ] **Step 4: Test deadzone — verify instant stop on release**

Push the puck, then release. The robot should stop within ~200ms (the driver's deceleration ramp provides the gradual stop). The velocity display should show 0.00/0.00/0.00 as soon as you release. If it keeps showing non-zero values after release, increase the deadzone setting.

- [ ] **Step 5: Test sensitivity curves**

Switch between Linear, Expo, and S-Curve. With Expo at exponent 2.0:
- Gentle push → very slow movement (fine positioning)
- Full push → max speed

Adjust exponent and max speed sliders to taste.

- [ ] **Step 6: Test E-STOP buttons**

Press either SpaceMouse button. Robot should stop immediately and red "E-STOP: HALT" toast should appear.

- [ ] **Step 7: Test auto-reconnect**

Refresh the browser page. The SpaceMouse should reconnect automatically without the picker dialog. Verify "SM: Connected" appears after page load.

- [ ] **Step 8: Commit any axis calibration fixes**

```bash
git add -A
git commit -m "fix(spacemouse): calibrate axis mapping and sign conventions from hardware testing"
```

---

### Task 9: Final Verification and Cleanup

- [ ] **Step 1: Verify all input sources still work**

Test each input source independently:
1. Keyboard (WASD + QE) → verify KB indicator and robot motion
2. Left virtual joystick → verify JOY indicator and translate motion
3. Right virtual joystick → verify JOY indicator and drive+rotate motion (NEW)
4. SpaceMouse → verify SM indicator and proportional motion
5. Priority: use SpaceMouse + keyboard simultaneously → SM should win

- [ ] **Step 2: Verify settings persist across refresh**

Change several settings (curve, deadzone, max speeds, inversions), refresh the page, verify all settings are restored from localStorage.

- [ ] **Step 3: Verify sleep/wake indicator**

Leave SpaceMouse idle for >5 seconds. Check that the UI reflects sleeping state (if implemented — may defer this to a follow-up if not critical).

- [ ] **Step 4: Update CLAUDE.md with SpaceMouse documentation**

Add a section to the project CLAUDE.md documenting the SpaceMouse feature:
- WebHID setup requirements (Chrome flag)
- Supported devices (vendor IDs)
- Settings and tuning parameters
- Known limitations

- [ ] **Step 5: Final commit**

```bash
git add -A
git commit -m "docs: update CLAUDE.md with SpaceMouse teleop documentation"
```
