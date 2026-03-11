# AIO Mission Control Dashboard Design

**Date**: 2026-03-11
**Status**: Approved

## Overview

A desktop mission-control style all-in-one dashboard for ROScar1. Served as a separate page (`aio.html`) alongside the existing tabbed dashboard. All panels visible simultaneously on a CSS Grid layout. Vanilla JS, no build tools, no frameworks.

## Layout

3-column, 2-row CSS Grid:

```
┌────────────────────┬─────────────────────┬───────────────────────┐
│  CAMERA FEED       │  MAP                │  STATUS + SETTINGS    │
│  (MJPEG stream)    │  (OccupancyGrid +   │  Position/Vel/IMU     │
│                    │   pose + nav goals)  │  Battery gauge        │
│                    │                     │  Mode buttons + save  │
├────────────────────┼─────────────────────┼───────────────────────┤
│  DRIVE CONTROLS    │  GRAPHS + TF TREE   │  DIAGNOSTICS          │
│  Joysticks + sliders│  Sparkline charts   │  /rosout log viewer   │
│  Lidar mini-radar  │  TF tree visualizer │  Severity filter      │
│  Gamepad indicator │                     │                       │
└────────────────────┴─────────────────────┴───────────────────────┘
```

Column widths: 1fr / 1.5fr / 1fr. Row heights: 1.2fr / 1fr.

## Core Features

All functionality from existing tabbed dashboard, running simultaneously:
- Camera MJPEG stream with quality/resolution controls
- OccupancyGrid map with pan/zoom, robot pose, click-to-navigate
- Dual nipplejs joysticks (translate + rotate) + keyboard WASD+QE
- Speed limit sliders (linear + angular)
- Lidar mini-radar polar plot
- Position/velocity/orientation readouts
- Battery gauge (voltage + percentage + color coding)
- Mode switching (IDLE/TELEOP/SLAM/NAV/SLAM+NAV)
- Map save with custom name
- E-STOP button + spacebar shortcut
- Connection status + reconnect
- Toast notifications

## Bonus Features

### 1. Live Sparkline Graphs
- Rolling 60s canvas-drawn sparklines for vx, vy, wz, battery
- Circular buffer + canvas path. No charting library.
- Hover to see value at cursor position

### 2. Diagnostics Log Viewer
- Subscribe to `/rosout` (rcl_interfaces/msg/Log)
- Scrolling console, severity color coded (DEBUG=gray, INFO=cyan, WARN=amber, ERROR=red)
- Severity filter buttons
- Auto-scroll with pause-on-scroll-up
- Max 500 lines retained

### 3. TF Tree Visualizer
- Subscribe to `/tf` and `/tf_static`
- Build tree structure, render as horizontal node graph (boxes + arrows)
- Shows last-updated timestamp per transform
- Expected tree: map → odom → base_footprint → base_link → {laser, camera_link}

### 4. Gamepad Support
- Browser Gamepad API (`navigator.getGamepads()`)
- Left stick = vx/vy, right stick = wz
- Deadzone filtering (0.15 default)
- Visual indicator: gamepad name + axis bars
- Priority: gamepad > joystick > keyboard
- Standard mapping works with Xbox, PS, Logitech F710

## File Structure

New files only (no changes to existing dashboard):

```
roscar_ws/src/roscar_web/web/
├── aio.html
├── css/aio.css
└── js/
    ├── aio-app.js         # Entry: ROS, E-STOP, gamepad loop, init
    ├── aio-teleop.js      # Joysticks + keyboard + gamepad → /cmd_vel
    ├── aio-camera.js      # MJPEG stream panel
    ├── aio-map.js         # OccupancyGrid + nav goals
    ├── aio-status.js      # Readouts + mode switching + map save
    ├── aio-lidar.js       # Mini-radar
    ├── aio-graphs.js      # Sparkline charts
    ├── aio-diagnostics.js # /rosout log viewer
    └── aio-tf.js          # TF tree visualizer
```

Separate modules from existing dashboard because existing modules are coupled to tab-switching lifecycle. AIO has all panels active simultaneously.

## ROS Subscriptions & Throttle Rates

| Topic | Throttle | Panels |
|-------|----------|--------|
| `/scan` | 2 Hz | Lidar radar |
| `/map` | 0.5 Hz | Map |
| `/odometry/filtered` | 5 Hz | Status, Map, Graphs |
| `/imu/data` | 5 Hz | Status |
| `/battery_voltage` | 1 Hz | Status, Graphs |
| `/rosout` | unthrottled | Diagnostics |
| `/tf` | 1 Hz | TF tree |
| `/tf_static` | once (latched) | TF tree |
| `/cmd_vel` (publish) | 10 Hz | Teleop |

## Technical Decisions

- **Vanilla JS + CSS Grid**: Matches existing codebase, zero build deps
- **Shared vendored libs**: roslib.min.js, nipplejs.min.js (already present)
- **Separate page**: aio.html served alongside index.html, zero risk to existing
- **Aggressive throttling**: All panels active = more subscriptions; throttle keeps bandwidth manageable
- **Same industrial theme**: Dark background, cyan accents, Rajdhani/Share Tech Mono fonts
