# AIO Mission Control Dashboard Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a desktop mission-control AIO dashboard (`aio.html`) that shows all robot panels simultaneously — camera, map, joysticks, status, lidar, graphs, diagnostics, TF tree, and gamepad — on a single CSS Grid page.

**Architecture:** Vanilla JS + CSS Grid. New `aio-*.js` modules with same ROS patterns as existing dashboard but without tab-switching lifecycle. All panels active at once. Shared vendored libs (roslib.min.js, nipplejs.min.js). Served as separate page alongside existing index.html.

**Tech Stack:** HTML5, CSS Grid, vanilla ES modules, Canvas API, Browser Gamepad API, roslibjs, nipplejs

---

### Task 1: Create aio.html skeleton + CSS Grid layout

**Files:**
- Create: `roscar_ws/src/roscar_web/web/aio.html`
- Create: `roscar_ws/src/roscar_web/web/css/aio.css`

**Step 1: Create aio.html**

The HTML defines 6 grid panels, a header bar, E-STOP, and toast container. All panels are visible simultaneously (no tabs).

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ROScar1 Mission Control</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link href="https://fonts.googleapis.com/css2?family=Rajdhani:wght@400;500;600;700&family=Share+Tech+Mono&display=swap" rel="stylesheet">
  <link rel="stylesheet" href="css/aio.css">
</head>
<body>
  <!-- HEADER -->
  <header id="aio-header">
    <div class="header-left">
      <div class="logo"><span class="logo-bracket">[</span>ROScar1<span class="logo-bracket">]</span> <span class="aio-tag">AIO</span></div>
      <div id="conn-indicator" class="conn-indicator disconnected">
        <span class="conn-dot"></span>
        <span class="conn-label">DISCONNECTED</span>
      </div>
    </div>
    <div class="header-center">
      <div class="mode-badge">
        <span class="mode-label">MODE</span>
        <span id="mode-display" class="mode-value">IDLE</span>
      </div>
    </div>
    <div class="header-right">
      <div id="gamepad-indicator" class="gamepad-indicator hidden">
        <span class="gamepad-icon">&#x1F3AE;</span>
        <span id="gamepad-name" class="gamepad-name mono">--</span>
      </div>
      <div class="battery-display">
        <span class="battery-label">BATT</span>
        <div class="battery-bar-wrap">
          <div id="battery-bar" class="battery-bar" style="width:0%"></div>
        </div>
        <span id="battery-voltage" class="battery-voltage mono">--.-V</span>
      </div>
    </div>
  </header>

  <!-- MAIN GRID -->
  <main id="aio-grid">
    <!-- Row 1, Col 1: Camera -->
    <section class="panel" id="panel-camera">
      <div class="panel-title">CAMERA</div>
      <div class="panel-body camera-body">
        <div class="camera-frame">
          <img id="camera-img" alt="Camera feed" draggable="false">
          <div id="camera-overlay" class="camera-overlay"><span>CAMERA OFFLINE</span></div>
        </div>
        <div class="camera-ctrls">
          <div class="ctrl-group">
            <span class="ctrl-label">QUAL</span>
            <button class="sm-btn" data-quality="20">L</button>
            <button class="sm-btn active" data-quality="50">M</button>
            <button class="sm-btn" data-quality="80">H</button>
          </div>
          <div class="ctrl-group">
            <span class="ctrl-label">RES</span>
            <button class="sm-btn" data-res="320x240">320</button>
            <button class="sm-btn active" data-res="640x480">640</button>
          </div>
        </div>
      </div>
    </section>

    <!-- Row 1, Col 2: Map -->
    <section class="panel" id="panel-map">
      <div class="panel-title">
        MAP
        <span id="map-size-disp" class="mono panel-stat">--x--</span>
        <span id="map-res-disp" class="mono panel-stat">--cm</span>
        <span class="panel-actions">
          <button class="sm-btn" id="map-zoom-in" title="Zoom In">+</button>
          <button class="sm-btn" id="map-zoom-out" title="Zoom Out">-</button>
          <button class="sm-btn" id="map-reset" title="Reset">R</button>
        </span>
      </div>
      <div class="panel-body map-body">
        <canvas id="map-canvas"></canvas>
        <div id="nav-goal-hint" class="nav-hint hidden">CLICK MAP TO SET NAV GOAL</div>
        <button id="cancel-goal-btn" class="action-btn danger hidden">CANCEL GOAL</button>
      </div>
    </section>

    <!-- Row 1, Col 3: Status + Settings -->
    <section class="panel" id="panel-status">
      <div class="panel-title">STATUS</div>
      <div class="panel-body status-body">
        <!-- Position -->
        <div class="stat-group">
          <span class="stat-head">POSITION</span>
          <div class="stat-row">
            <span class="stat-item"><span class="stat-key">X</span><span id="st-x" class="mono">+0.000</span><span class="stat-u">m</span></span>
            <span class="stat-item"><span class="stat-key">Y</span><span id="st-y" class="mono">+0.000</span><span class="stat-u">m</span></span>
            <span class="stat-item"><span class="stat-key">&theta;</span><span id="st-yaw" class="mono">+0.0</span><span class="stat-u">&deg;</span></span>
          </div>
        </div>
        <!-- Velocity -->
        <div class="stat-group">
          <span class="stat-head">VELOCITY</span>
          <div class="stat-row">
            <span class="stat-item"><span class="stat-key">VX</span><span id="st-vx" class="mono">+0.000</span><span class="stat-u">m/s</span></span>
            <span class="stat-item"><span class="stat-key">VY</span><span id="st-vy" class="mono">+0.000</span><span class="stat-u">m/s</span></span>
            <span class="stat-item"><span class="stat-key">WZ</span><span id="st-wz" class="mono">+0.000</span><span class="stat-u">r/s</span></span>
          </div>
        </div>
        <!-- IMU -->
        <div class="stat-group">
          <span class="stat-head">ORIENTATION</span>
          <div class="stat-row">
            <span class="stat-item"><span class="stat-key">R</span><span id="st-roll" class="mono">+0.0</span><span class="stat-u">&deg;</span></span>
            <span class="stat-item"><span class="stat-key">P</span><span id="st-pitch" class="mono">+0.0</span><span class="stat-u">&deg;</span></span>
            <span class="stat-item"><span class="stat-key">Y</span><span id="st-imu-yaw" class="mono">+0.0</span><span class="stat-u">&deg;</span></span>
          </div>
        </div>
        <!-- Battery -->
        <div class="stat-group">
          <span class="stat-head">BATTERY</span>
          <div class="batt-row">
            <div class="batt-bar-mini"><div id="batt-fill" class="batt-fill-mini"></div></div>
            <span id="batt-volts" class="mono">--.-V</span>
            <span id="batt-pct" class="mono batt-pct-txt">--%</span>
          </div>
        </div>
        <!-- Mode Buttons -->
        <div class="stat-group">
          <span class="stat-head">MODE</span>
          <div class="mode-row">
            <button class="mode-btn-sm active" data-mode="idle">IDLE</button>
            <button class="mode-btn-sm" data-mode="teleop">TELEOP</button>
            <button class="mode-btn-sm" data-mode="slam">SLAM</button>
            <button class="mode-btn-sm" data-mode="navigation">NAV</button>
            <button class="mode-btn-sm" data-mode="slam_nav">S+N</button>
          </div>
          <div id="nav-map-select" class="nav-map-row hidden">
            <input type="text" id="nav-map-path" class="text-input-sm mono" placeholder="/home/brian/maps/my_map.yaml">
            <button id="apply-mode-btn" class="sm-btn primary">GO</button>
          </div>
          <div id="mode-status-msg" class="status-msg"></div>
        </div>
        <!-- Map Save -->
        <div class="stat-group">
          <span class="stat-head">SAVE MAP</span>
          <div class="save-row">
            <input type="text" id="map-save-name" class="text-input-sm mono" placeholder="my_map">
            <button id="save-map-btn" class="sm-btn">SAVE</button>
          </div>
          <div id="save-map-msg" class="status-msg"></div>
        </div>
      </div>
    </section>

    <!-- Row 2, Col 1: Drive Controls -->
    <section class="panel" id="panel-drive">
      <div class="panel-title">DRIVE <span id="input-source" class="input-source">KEYBOARD</span></div>
      <div class="panel-body drive-body">
        <div class="joy-col">
          <div class="joy-mini-header">
            TRANSLATE
            <span class="vel-chip">VX <span id="vx-disp" class="mono">+0.00</span></span>
            <span class="vel-chip">VY <span id="vy-disp" class="mono">+0.00</span></span>
          </div>
          <div class="joy-zone" id="joy-translate"></div>
        </div>
        <div class="drive-mid">
          <div class="slider-row">
            <span class="ctrl-label">LIN</span>
            <input type="range" id="spd-linear" min="0.1" max="1.0" step="0.05" value="0.3">
            <span id="spd-linear-val" class="mono">0.30</span>
          </div>
          <div class="slider-row">
            <span class="ctrl-label">ANG</span>
            <input type="range" id="spd-angular" min="0.1" max="3.0" step="0.1" value="1.0">
            <span id="spd-angular-val" class="mono">1.00</span>
          </div>
          <div class="kbd-hints-mini">
            <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> <kbd>Q</kbd><kbd>E</kbd>
          </div>
          <!-- Lidar mini-radar -->
          <div class="lidar-zone" id="lidar-container">
            <canvas id="lidar-canvas"></canvas>
          </div>
        </div>
        <div class="joy-col">
          <div class="joy-mini-header">
            ROTATE
            <span class="vel-chip">WZ <span id="wz-disp" class="mono">+0.00</span></span>
          </div>
          <div class="joy-zone" id="joy-rotate"></div>
        </div>
      </div>
    </section>

    <!-- Row 2, Col 2: Graphs + TF Tree -->
    <section class="panel" id="panel-graphs">
      <div class="panel-title">TELEMETRY</div>
      <div class="panel-body graphs-body">
        <div class="sparkline-grid">
          <div class="spark-card">
            <span class="spark-label">VX <span id="graph-vx-val" class="mono spark-val">0.00</span> <span class="spark-unit">m/s</span></span>
            <canvas id="graph-vx" class="spark-canvas"></canvas>
          </div>
          <div class="spark-card">
            <span class="spark-label">VY <span id="graph-vy-val" class="mono spark-val">0.00</span> <span class="spark-unit">m/s</span></span>
            <canvas id="graph-vy" class="spark-canvas"></canvas>
          </div>
          <div class="spark-card">
            <span class="spark-label">WZ <span id="graph-wz-val" class="mono spark-val">0.00</span> <span class="spark-unit">r/s</span></span>
            <canvas id="graph-wz" class="spark-canvas"></canvas>
          </div>
          <div class="spark-card">
            <span class="spark-label">BATT <span id="graph-bat-val" class="mono spark-val">-.-</span> <span class="spark-unit">V</span></span>
            <canvas id="graph-bat" class="spark-canvas"></canvas>
          </div>
        </div>
        <div class="tf-section">
          <div class="panel-subtitle">TF TREE</div>
          <div id="tf-tree" class="tf-tree-container"></div>
        </div>
      </div>
    </section>

    <!-- Row 2, Col 3: Diagnostics -->
    <section class="panel" id="panel-diag">
      <div class="panel-title">
        DIAGNOSTICS
        <span class="diag-filters">
          <button class="sm-btn active" data-level="all">ALL</button>
          <button class="sm-btn" data-level="info">INFO</button>
          <button class="sm-btn" data-level="warn">WARN</button>
          <button class="sm-btn" data-level="error">ERR</button>
        </span>
      </div>
      <div class="panel-body diag-body">
        <div id="log-container" class="log-container mono"></div>
      </div>
    </section>
  </main>

  <!-- E-STOP -->
  <button id="estop-btn" title="Emergency Stop (Spacebar)">
    <span class="estop-icon">&#9632;</span>
    <span class="estop-label">STOP</span>
  </button>

  <!-- TOAST -->
  <div id="toast" class="toast"></div>

  <!-- SCRIPTS -->
  <script src="js/lib/roslib.min.js"></script>
  <script src="js/lib/nipplejs.min.js"></script>
  <script type="module" src="js/aio-app.js"></script>
</body>
</html>
```

**Step 2: Create aio.css**

Full CSS Grid layout with industrial theme. Reuses CSS custom properties from existing dashboard but standalone (no import of style.css).

The CSS is large — approximately 400 lines. Key structure:
- `:root` variables matching existing theme
- Header bar styles (reused from existing)
- `#aio-grid` CSS Grid: `grid-template-columns: 1fr 1.5fr 1fr; grid-template-rows: 1.2fr 1fr;`
- `.panel` base styles: dark bg, border, flex column
- Panel-specific layouts for each of the 6 panels
- Compact stat displays, mini buttons, sparkline canvases
- Log viewer with severity colors
- TF tree node/arrow rendering
- Gamepad indicator
- E-STOP and toast (same as existing)
- Scanline CRT overlay

**Step 3: Verify HTML loads**

Open `http://<host>:8888/aio.html` in browser. Should show empty grid panels with correct layout and theme.

**Step 4: Commit**

```bash
git add roscar_ws/src/roscar_web/web/aio.html roscar_ws/src/roscar_web/web/css/aio.css
git commit -m "feat(aio): add HTML skeleton and CSS Grid layout"
```

---

### Task 2: Create aio-app.js — entry point, ROS connection, E-STOP

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-app.js`

**Step 1: Write aio-app.js**

Same pattern as existing `app.js` but without tab switching. Imports all aio modules. Manages:
- ROS connection via roslib (`ws://{HOST}:9090`)
- Auto-reconnect on close (3s delay)
- Connection UI updates
- E-STOP button + spacebar handler
- `/cmd_vel` publisher factory
- Toast notification function
- Module notification bus (`notifyModules('connected')`)
- Init calls to all aio modules

```javascript
// Key exports:
export const HOST = window.location.hostname || 'localhost';
export const PORTS = { rosbridge: 9090, video: 8080, http: 8888 };
export function getRos() { return ros; }
export function isConnected() { return connected; }
export function onAppEvent(cb) { moduleCallbacks.push(cb); }
export function toast(msg, type) { /* ... */ }

// Init all modules (no tab gating):
initTeleop(getOrCreateCmdVelPub);
initCamera();
initStatus(getRos);
initLidar(getRos);
initMap(getRos);
initModes(getRos, toast);
initGraphs(getRos);
initDiagnostics(getRos);
initTF(getRos);
connect();
```

**Step 2: Verify module loads without errors**

Open browser console at `aio.html`. Should connect to rosbridge (or show reconnect loop if robot offline).

**Step 3: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-app.js
git commit -m "feat(aio): add app entry point with ROS connection and E-STOP"
```

---

### Task 3: Create aio-teleop.js — joysticks, keyboard, gamepad

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-teleop.js`

**Step 1: Write aio-teleop.js**

Combines joystick + keyboard control from existing teleop.js, plus new gamepad support.

Key behavior:
- Two nipplejs joysticks in `joy-translate` and `joy-rotate` zones
- Keyboard WASD+QE (same mapping as existing)
- **NEW: Gamepad API** — `navigator.getGamepads()` polled in RAF loop
  - Left stick (axes 0,1) → vy, vx (note: axis 1 is inverted for forward)
  - Right stick (axis 2 or 3) → wz
  - Deadzone: 0.15
  - Updates `#input-source` indicator: "KEYBOARD", "JOYSTICK", or "GAMEPAD"
  - Updates `#gamepad-indicator` in header with connected gamepad name
- Priority: gamepad > joystick > keyboard
- Speed sliders update maxLinear and maxAngular
- 10 Hz publish loop merging all input sources
- Publishes to `/cmd_vel` via getPub callback

**DOM IDs:** `joy-translate`, `joy-rotate`, `spd-linear`, `spd-linear-val`, `spd-angular`, `spd-angular-val`, `vx-disp`, `vy-disp`, `wz-disp`, `input-source`, `gamepad-indicator`, `gamepad-name`

**Step 2: Verify joysticks render and publish**

Test with `ros2 topic echo /cmd_vel` on robot while moving joysticks.

**Step 3: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-teleop.js
git commit -m "feat(aio): add teleop with joystick, keyboard, and gamepad support"
```

---

### Task 4: Create aio-camera.js — MJPEG stream

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-camera.js`

**Step 1: Write aio-camera.js**

Same pattern as existing camera.js but always active (no tab gating).

- MJPEG stream from `http://{HOST}:8080/stream?topic=/image_raw&quality=...&width=...&height=...&type=mjpeg`
- Quality buttons (L/M/H), resolution buttons (320/640)
- 5s timeout → "CAMERA OFFLINE" overlay
- Stream starts on page load, restarts on quality/resolution change

**DOM IDs:** `camera-img`, `camera-overlay`, `[data-quality]`, `[data-res]` within `#panel-camera`

**Step 2: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-camera.js
git commit -m "feat(aio): add MJPEG camera stream panel"
```

---

### Task 5: Create aio-status.js — readouts, mode switching, map save

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-status.js`

**Step 1: Write aio-status.js**

Combines status.js + modes.js functionality into one module:

**Subscriptions:**
- `/odometry/filtered` (100ms throttle) → position, velocity, yaw
- `/imu/data` (200ms throttle) → roll, pitch, yaw
- `/battery_voltage` (1000ms throttle) → gauge + header bar

**Services:**
- `/web/set_mode` (roscar_interfaces/SetMode) — mode buttons
- `/web/save_map` (roscar_interfaces/SaveMap) — map save

**Mode buttons:** Compact row of 5 buttons. Navigation shows map path input.

**DOM IDs:** `st-x`, `st-y`, `st-yaw`, `st-vx`, `st-vy`, `st-wz`, `st-roll`, `st-pitch`, `st-imu-yaw`, `battery-bar`, `battery-voltage`, `batt-fill`, `batt-volts`, `batt-pct`, `mode-display`, `mode-status-msg`, `nav-map-select`, `nav-map-path`, `apply-mode-btn`, `map-save-name`, `save-map-btn`, `save-map-msg`, `.mode-btn-sm`

**Also exports:** `onOdomData(cb)` and `onBatteryData(cb)` — callbacks for graphs module to receive data without duplicate subscriptions.

**Step 2: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-status.js
git commit -m "feat(aio): add status readouts, mode switching, and map save"
```

---

### Task 6: Create aio-map.js — OccupancyGrid + nav goals

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-map.js`

**Step 1: Write aio-map.js**

Same logic as existing map.js:
- Subscribe `/map` (2000ms throttle) → store grid data
- Subscribe `/odometry/filtered` (200ms throttle) → robot pose
- Canvas rendering with 90° CCW rotation convention
- Pan (drag), zoom (scroll + buttons), reset
- Click-to-navigate via `/navigate_to_pose` action client
- Cancel goal button
- Always active (no tab gating), RAF render loop

**DOM IDs:** `map-canvas`, `map-zoom-in`, `map-zoom-out`, `map-reset`, `map-size-disp`, `map-res-disp`, `nav-goal-hint`, `cancel-goal-btn`

**Exports:** `enableNavGoalMode()` — called by aio-status.js after mode switch to nav/slam_nav

**Step 2: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-map.js
git commit -m "feat(aio): add OccupancyGrid map with click-to-navigate"
```

---

### Task 7: Create aio-lidar.js — mini radar

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-lidar.js`

**Step 1: Write aio-lidar.js**

Subscribes to `/scan` (500ms throttle). Renders polar scan on a canvas inside `#lidar-container`.

Unlike existing lidar.js (which overlays on joystick zone), this has its own dedicated canvas in the drive panel's center column. Same rendering logic: grid circles, cyan laser points, orange robot center dot.

**DOM IDs:** `lidar-canvas`, `lidar-container`

**Step 2: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-lidar.js
git commit -m "feat(aio): add lidar mini-radar panel"
```

---

### Task 8: Create aio-graphs.js — sparkline time-series

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-graphs.js`

**Step 1: Write aio-graphs.js**

NEW feature. Rolling 60s sparkline charts for vx, vy, wz, battery.

Implementation:
- Circular buffer per metric: `Float32Array(300)` (60s at 5Hz for velocity, 60 entries at 1Hz for battery)
- Receives data from aio-status.js callbacks (`onOdomData`, `onBatteryData`)
- Canvas rendering per sparkline:
  - Background: dark
  - Line: cyan (velocity) or green (battery)
  - Y-axis auto-scales to min/max in buffer (with padding)
  - Current value displayed in label
- Single RAF loop renders all 4 canvases
- Hover: show value at cursor x-position (optional enhancement)

**DOM IDs:** `graph-vx`, `graph-vy`, `graph-wz`, `graph-bat`, `graph-vx-val`, `graph-vy-val`, `graph-wz-val`, `graph-bat-val`

**Step 2: Verify sparklines render with mock data**

Push velocity commands and watch graphs update.

**Step 3: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-graphs.js
git commit -m "feat(aio): add sparkline telemetry graphs"
```

---

### Task 9: Create aio-diagnostics.js — /rosout log viewer

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-diagnostics.js`

**Step 1: Write aio-diagnostics.js**

NEW feature. Subscribes to `/rosout` (rcl_interfaces/msg/Log). Displays scrolling log.

Implementation:
- Subscribe `/rosout` (no throttle — want all messages)
- Message fields: `level` (10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL), `msg`, `name` (node), `stamp`
- Severity filter buttons (ALL, INFO, WARN, ERR) — filter stored messages
- Log entry format: `[HH:MM:SS] /node_name: message`
- Color coding: DEBUG=`var(--text-dim)`, INFO=`var(--primary)`, WARN=`var(--amber)`, ERROR/FATAL=`var(--red)`
- Max 500 entries retained (FIFO)
- Auto-scroll to bottom unless user scrolled up (detect via `scrollTop + clientHeight < scrollHeight - 10`)
- Scroll-to-bottom button appears when paused

**DOM IDs:** `log-container`, `[data-level]` filter buttons within `#panel-diag`

**Step 2: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-diagnostics.js
git commit -m "feat(aio): add /rosout diagnostics log viewer"
```

---

### Task 10: Create aio-tf.js — TF tree visualizer

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-tf.js`

**Step 1: Write aio-tf.js**

NEW feature. Renders the TF tree as a simple horizontal node diagram.

Implementation:
- Subscribe `/tf` (tf2_msgs/TFMessage, 1000ms throttle)
- Subscribe `/tf_static` (tf2_msgs/TFMessage, latched/once)
- Build adjacency map: `parent -> [children]`, track last-seen timestamp per frame
- Render as HTML divs (not canvas):
  - Each frame = box with frame name
  - Arrows (CSS borders/pseudo-elements) connecting parent → child
  - Static frames get a distinct style (dashed border)
  - Stale frames (>5s since update for non-static) get dim color
- Expected tree for ROScar1:
  ```
  map → odom → base_footprint → base_link → laser
                                           → camera_link
  ```
- Rebuilds DOM on tree structure change (debounced 1s)

**DOM IDs:** `tf-tree`

**Step 2: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-tf.js
git commit -m "feat(aio): add TF tree visualizer"
```

---

### Task 11: Integration testing and polish

**Files:**
- Modify: `roscar_ws/src/roscar_web/web/aio.html` (if needed)
- Modify: `roscar_ws/src/roscar_web/web/css/aio.css` (if needed)

**Step 1: Test all panels simultaneously**

Open `http://<robot-ip>:8888/aio.html` with robot in SLAM mode. Verify:
- Camera stream (or offline indicator)
- Map rendering with robot pose
- Joystick control works
- Lidar radar updates
- Status readouts update
- Mode switching works
- Graph sparklines animate
- Diagnostics log shows /rosout messages
- TF tree renders correct hierarchy
- Gamepad detected (if connected)
- E-STOP works (button + spacebar)

**Step 2: Fix layout issues**

Adjust CSS for any overflow, sizing, or alignment issues found during testing.

**Step 3: Commit**

```bash
git add -A
git commit -m "fix(aio): polish layout and integration fixes"
```

---

### Task 12: Update CLAUDE.md and final commit

**Files:**
- Modify: `CLAUDE.md`

**Step 1: Add AIO dashboard section to CLAUDE.md**

Add entry under Web Dashboard section documenting:
- Access URL: `http://<robot-ip>:8888/aio.html`
- Panel layout description
- New features (graphs, diagnostics, TF tree, gamepad)
- File list for AIO-specific files

**Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add AIO dashboard documentation to CLAUDE.md"
```
