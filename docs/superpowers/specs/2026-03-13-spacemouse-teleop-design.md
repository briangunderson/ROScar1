# SpaceMouse Teleop Integration — Design Spec

**Date**: 2026-03-13
**Branch**: `feature/spacemouse`
**Status**: Approved

## Summary

Add 3Dconnexion SpaceMouse Wireless support to the AIO web dashboard as a proportional 6DOF input device for mecanum robot teleop. Uses the Chrome WebHID API to read raw HID reports, bypassing the 3Dconnexion driver software entirely. Also fixes the right virtual joystick to support forward/back + rotation simultaneously.

## Problem

The SpaceMouse Wireless currently controls the robot through the 3Dconnexion driver translating puck movements into keyboard events. This produces:
- Binary on/off control (no proportional input)
- Glitchy key-repeat behavior
- Continued driving after releasing the puck (keyboard events linger)

The SpaceMouse is a natural fit for mecanum control — its 6DOF puck maps directly to holonomic motion (vx, vy, wz).

## Approach: WebHID API

Chrome's WebHID API allows direct access to HID devices from JavaScript. This gives us:
- Raw proportional axis data (signed 16-bit integers)
- Clean zero readings at rest (no drift)
- No dependency on 3Dconnexion driver configuration
- Works alongside the driver (WebHID claims the device for the browser tab only)
- One-time pairing dialog, then Chrome remembers the device

### Secure Context Requirement

WebHID requires a [secure context](https://developer.mozilla.org/en-US/docs/Web/Security/Secure_Contexts) (HTTPS or localhost). The dashboard is served over plain HTTP from the Pi (port 8888). Since the SpaceMouse is plugged into the dev PC (not the Pi), the browser accessing the dashboard is on a remote origin — `http://192.168.1.170:8888/` — which is NOT a secure context.

**Solution**: The user must add the Pi's URL to Chrome's insecure origin allowlist:
1. Navigate to `chrome://flags/#unsafely-treat-insecure-origin-as-secure`
2. Add `http://192.168.1.170:8888` to the text field
3. Relaunch Chrome

This is a one-time setup, appropriate for a development/hobbyist robot. The `initSpaceMouse()` function will check `navigator.hid` availability and gracefully hide the Connect button if WebHID is unavailable, with a console warning explaining the secure context requirement.

### HID Protocol

3Dconnexion SpaceMouse devices are standard HID multi-axis controllers:
- **Usage Page**: 0x01 (Generic Desktop)
- **Usage ID**: 0x08 (Multi-axis Controller)
- **Vendor IDs**: `0x256F` (3Dconnexion) and `0x046D` (Logitech legacy)
- **SpaceMouse Wireless Product IDs**: `0xC62E` (cabled), `0xC62F` (wireless receiver)

HID input reports:

| Report ID | Content | Format |
|-----------|---------|--------|
| 1 | Translation (X, Y, Z) | 3x Int16LE (6 bytes) |
| 2 | Rotation (Rx, Ry, Rz) | 3x Int16LE (6 bytes) |
| 3 | Buttons | Bitmask |

Values are signed 16-bit, range approximately ±350 at full deflection. At rest: `[0, 0, 0]`.

## Axis Mapping

| SpaceMouse Axis | Robot Motion | ROS Field | Notes |
|-----------------|-------------|-----------|-------|
| Translation Y (push fwd/back) | Drive forward/back | `linear.x` (vx) | Primary driving axis |
| Translation X (push left/right) | Strafe left/right | `linear.y` (vy) | Mecanum strafing |
| Rotation Rz (twist CW/CCW) | Rotate | `angular.z` (wz) | Yaw control |
| Translation Z (push up/down) | Unused | — | Available for future use |
| Rotation Rx (tilt fwd/back) | Unused | — | Available for future use |
| Rotation Ry (tilt left/right) | Unused | — | Available for future use |

Axis directions will be validated on hardware and inverted if needed via configurable toggles.

## Tunable Parameters

All stored in `localStorage`, adjustable from UI, with sensible defaults:

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `sm_deadzone` | integer | 5 | 0–50 | Counts below this threshold treated as zero |
| `sm_curve` | enum | `exponential` | linear/exponential/s-curve | Sensitivity response curve |
| `sm_exponent` | float | 2.0 | 1.0–3.0 | Exponent for exponential/s-curve |
| `sm_max_linear` | float | 0.3 | 0.1–1.0 | Max linear velocity (m/s) |
| `sm_max_angular` | float | 1.0 | 0.1–3.0 | Max angular velocity (rad/s) |
| `sm_invert_x` | boolean | false | — | Invert drive (fwd/back) axis |
| `sm_invert_y` | boolean | false | — | Invert strafe (left/right) axis |
| `sm_invert_rz` | boolean | false | — | Invert rotation axis |

### Sensitivity Curves

- **Linear**: `output = input` (proportional, no shaping)
- **Exponential**: `output = sign(input) * |input|^exponent` (fine control at low deflection, full speed at max)
- **S-Curve**: `output = sign(input) * (3 * |input|^e - 2 * |input|^(e+1))` where `e = exponent`. At e=2 this is standard smoothstep. Higher exponents increase the low-speed fine-control region.

Input is normalized to [-1.0, +1.0] before curve application, then scaled by max speed.

### Speed Limit Interaction

The SpaceMouse has its own `sm_max_linear` and `sm_max_angular` parameters, independent of the existing dashboard speed sliders. This is intentional: SpaceMouse users typically want different max speeds than joystick/keyboard users (the SpaceMouse's fine proportional control makes higher max speeds safe). The existing sliders continue to govern gamepad/joystick/keyboard input. When SpaceMouse is the active input source, the sliders are dimmed to indicate they are not in effect.

## Input Priority System

Updated priority order (highest first):

1. **SpaceMouse** (WebHID — any axis non-zero after deadzone)
2. **Gamepad** (Gamepad API — any axis non-zero)
3. **Virtual Joysticks** (nipplejs — if active)
4. **Keyboard** (WASD + QE)

The 10Hz `publishVelocity()` loop checks each source in priority order and publishes the first one with active input. If no source has input, publishes zero velocity.

## Input Source Indicator

Existing indicators: **KB** / **JOY** / **PAD**

Added: **SM** — shown in cyan when SpaceMouse is the active input source.

## E-STOP via SpaceMouse Buttons

The SpaceMouse Wireless has 2 buttons (Report ID 3, 4-byte LE bitmask: bit 0 = left, bit 1 = right). Mapping:
- **Either button press (any non-zero bitmask)** → calls the app-level `estop()` function via callback
- Matches physical intuition: slap/press the puck in a panic to stop
- Integrates with the existing E-STOP mechanism in `aio-app.js` (publishes zero twist + shows toast)

## New File: `aio-spacemouse.js`

ES module responsible for the complete SpaceMouse lifecycle:

### Exports
- `initSpaceMouse(callbacks)` — called by `aio-app.js` at startup. `callbacks` must include `{ estop: Function, onConnectionChange: Function }`. The module does not need the `ros` object — it does not publish to ROS directly.
- `getSpaceMouseVelocity()` → `{vx, vy, wz}` or `null` — returns fully processed velocities (deadzone + curve + scaling already applied, in m/s and rad/s). Returns `null` if SpaceMouse is not connected or all axes are within deadzone. Polled by teleop module at 10Hz.
- `isSpaceMouseConnected()` → boolean

### Internal Responsibilities
1. **Connection management**: Pair via WebHID, auto-reconnect on page load if previously paired
2. **HID report parsing**: Decode report IDs 1/2/3 into translation, rotation, buttons
3. **Signal processing**: Apply deadzone → normalize → apply sensitivity curve → scale by max speed
4. **Settings UI**: Render collapsible settings panel, bind to localStorage
5. **Button handling**: Map button presses to E-STOP

### Connection Flow
1. User clicks "Connect SpaceMouse" button
2. `navigator.hid.requestDevice({filters: [{vendorId: 0x256F}, {vendorId: 0x046D}]})` — shows browser picker
3. User selects device, browser remembers for future sessions
4. `device.open()` → `device.addEventListener('inputreport', onInputReport)`
5. On subsequent page loads: `navigator.hid.getDevices()` retrieves previously paired devices automatically (no picker needed)

### Data Flow
```
HID inputreport event (async, ~100Hz from device)
  → parse report ID 1 (translation) or 2 (rotation) or 3 (buttons)
  → store latest raw axis values + timestamp in module state
  → buttons: if any non-zero, call callbacks.estop()

getSpaceMouseVelocity() (called by teleop at 10Hz)
  → if not connected or last report > 200ms ago → return null (staleness timeout)
  → apply deadzone to raw values → normalize to [-1, +1] → apply curve → scale by max speed
  → return {vx, vy, wz} in m/s and rad/s (ready to publish)

publishVelocity() (10Hz timer in aio-teleop.js)
  → calls getSpaceMouseVelocity()
  → if non-null, publish as cmd_vel (highest priority)
  → else fall through to gamepad → joystick → keyboard
```

### Staleness Timeout

If no HID input report has been received in the last 200ms, `getSpaceMouseVelocity()` returns `null` regardless of stored values. This handles:
- Puck release timing edge cases (last non-zero report before zero report)
- Device disconnection mid-use without a clean zero report
- SpaceMouse entering sleep mode after inactivity

### Sleep/Wake Handling

The SpaceMouse Wireless enters sleep mode after ~5 minutes of inactivity. On wake (user touches puck), it resumes sending HID reports. The `inputreport` listener remains bound across sleep/wake cycles since the HID connection handle persists. The staleness timeout ensures no stale velocity is published while asleep. The UI shows a "sleeping" indicator if the device is paired but no reports have been received for >5 seconds.

## Right Virtual Joystick Fix

**Current behavior**: Right nipplejs joystick only maps X-axis → wz (rotation).

**New behavior**: Right joystick maps both axes:
- X-axis → wz (rotation) — unchanged
- Y-axis → vx (forward/back) — new

This allows single-thumb drive+turn, which is more natural for mobile control. The left joystick remains translate-only (vx + vy).

**Change location**: `aio-teleop.js`, in the right joystick `move` event handler.

## UI Layout

### DRIVE Panel Header
```
[DRIVE]  KB  JOY  PAD  SM    [Connect SpaceMouse]  [E-STOP]
```

### SpaceMouse Settings (collapsible, only shown when SM connected)
```
▶ SpaceMouse Settings
  Curve:    [Linear] [Expo ✓] [S-Curve]
  Exponent: ──●────── 2.0
  Deadzone: ●──────── 5
  Linear:   ───●───── 0.3 m/s
  Angular:  ───●───── 1.0 rad/s
  Invert:   [ ] Fwd/Back  [ ] Strafe  [ ] Rotate
```

Styled consistently with existing AIO dashboard (dark theme, cyan accents, monospace labels).

## Files Changed

| File | Change |
|------|--------|
| `web/js/aio-spacemouse.js` | **NEW** — WebHID SpaceMouse module |
| `web/js/aio-teleop.js` | Add SpaceMouse priority, fix right joystick Y-axis |
| `web/js/aio-app.js` | Import and initialize SpaceMouse module |
| `web/aio.html` | Add Connect button, SM indicator, settings panel markup |
| `web/css/aio.css` | Styles for SpaceMouse settings panel |

## Testing Plan

1. **Connection**: Pair SpaceMouse via browser picker, verify HID reports arrive
2. **Axes**: Push puck in all 6 directions, verify correct mapping (console log raw + processed values)
3. **Deadzone**: Release puck, verify robot stops instantly (zero velocity published)
4. **Sensitivity curves**: Switch between linear/expo/s-curve, verify feel difference
5. **Settings persistence**: Change settings, refresh page, verify they persist
6. **Auto-reconnect**: Pair once, refresh page, verify SpaceMouse reconnects without picker
7. **Priority**: Use SpaceMouse + keyboard simultaneously, verify SM takes priority
8. **E-STOP**: Press SpaceMouse button, verify immediate zero velocity
9. **Right joystick fix**: Verify right nipplejs Y-axis controls forward/back
10. **No regression**: Verify keyboard and left joystick still work correctly

## References

- [nytamin/spacemouse](https://github.com/nytamin/spacemouse) — Node.js + WebHID SpaceMouse library
- [larsgk/webhid-space](https://github.com/larsgk/webhid-space) — WebHID SpaceMouse demo
- [3Dconnexion HID spec](https://3dconnexion.com/wp-content/uploads/2020/02/HW-Spec-3DX-700048_Rev001_USB.pdf) — SpaceMouse Module USB Product Specification
- [WebHID API](https://developer.chrome.com/docs/capabilities/hid) — Chrome WebHID documentation
- [SpaceMouse raw data blog post](https://vielzutun.ch/wordpress/?p=8416) — Reverse-engineered HID protocol details
