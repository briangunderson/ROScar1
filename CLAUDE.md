# ROScar1 - 4WD Mecanum Robot

## Project Overview
A ROS2 Jazzy workspace for a 4WD Mecanum wheel robot built on:
- **Raspberry Pi 5** (Ubuntu 24.04 Server) running ROS2
- **YB-ERF01-V3.0** (Yahboom STM32F103RCT6 motor driver board) connected via USB serial
- **Intel RealSense D435i** depth camera (USB 3.0 SuperSpeed) — provides both depth pointcloud (for Nav2 voxel_layer 3D obstacles) AND color stream (for ArUco, YOLO, dashboard). Replaces the previous Logitech webcam.
- **RPLIDAR C1** (Slamtec) for 2D laser scanning, connected via USB
- **4x DC encoder motors** with mecanum wheels

**Status**: Milestones 1-3 fully verified. Milestone 4 SLAM + Nav2 verified on hardware (slam_toolbox publishing /map, Nav2 accepting and completing navigation goals). Web dashboard implemented (roscar_web package + roscar_interfaces).

## Architecture
The STM32 board handles PID motor control and mecanum inverse kinematics internally.
Our ROS2 layer is a thin bridge:

```
cmd_vel -> roscar_driver -> Rosmaster_Lib -> USB Serial -> STM32 -> Motors
                          <- encoder/IMU data <- STM32 <- Sensors
         -> odom_raw, imu/data_raw, battery_voltage
```

Full pipeline with EKF (verified working):
```
teleop_twist_keyboard -> /cmd_vel -> roscar_driver -> /odom_raw, /imu/data_raw
                                                       |             |
                                                       v             v
                                                   ekf_node    imu_filter_madgwick
                                                       |             |
                                                       +------+------+
                                                              v
                                                      /odometry/filtered

Key: EKF owns odom->base_footprint TF (driver publish_odom_tf=false)
     EKF fuses velocity from odom + differential yaw from IMU
     Madgwick converts raw accel+gyro -> orientation (az=+9.81 when flat)
```

## Critical Files

| File | Purpose |
|------|---------|
| `roscar_ws/src/roscar_driver/roscar_driver/driver_node.py` | Core node: cmd_vel->motors, sensors->topics |
| `roscar_ws/src/roscar_driver/roscar_driver/mecanum_kinematics.py` | Odometry integration (FK) |
| `roscar_ws/src/roscar_driver/config/driver_params.yaml` | Serial port, speed limits, frame names |
| `roscar_ws/src/roscar_description/urdf/roscar.urdf.xacro` | Robot model (measured 2026-03-16) |
| `roscar_ws/src/roscar_bringup/launch/robot.launch.py` | Full robot bringup |
| `roscar_ws/src/roscar_bringup/launch/teleop.launch.py` | Teleop + full robot |
| `roscar_ws/src/roscar_bringup/config/ekf.yaml` | EKF sensor fusion config |
| `roscar_ws/src/roscar_bringup/config/laser_filter.yaml` | Laser range filter (drops chassis reflections < 0.15m) |
| `roscar_ws/src/roscar_bringup/config/slam_toolbox.yaml` | slam_toolbox online_async config |
| `roscar_ws/src/roscar_bringup/config/camera_calibration.yaml` | Logitech webcam intrinsics (046d:0825, plumb_bob) |
| `roscar_ws/src/roscar_bringup/config/nav2_params.yaml` | Nav2 full stack config (DWB holonomic) |
| `roscar_ws/src/roscar_bringup/launch/slam.launch.py` | SLAM mapping (teleop + slam_toolbox) |
| `roscar_ws/src/roscar_bringup/launch/navigation.launch.py` | Nav2 on saved map |
| `roscar_ws/src/roscar_bringup/launch/slam_nav.launch.py` | SLAM + Nav2 combined |
| `roscar_ws/src/roscar_bringup/launch/depth_camera.launch.py` | Intel RealSense D435i driver launch |
| `roscar_ws/src/roscar_bringup/config/realsense_params.yaml` | D435i driver config (depth+color@15Hz, pointcloud) |
| `roscar_ws/src/roscar_web/launch/web.launch.py` | Web dashboard (rosbridge+video+http) |
| `roscar_ws/src/roscar_web/roscar_web/launch_manager_node.py` | Mode switching service node |
| `roscar_ws/src/roscar_web/roscar_web/http_server_node.py` | Static file server node |
| `scripts/roscar-recovery.py` | Standalone recovery HTTP server (port 9999) |
| `roscar_ws/src/roscar_driver/roscar_driver/landmark_localizer_node.py` | ArUco marker-based pose correction |
| `roscar_ws/src/roscar_driver/config/landmark_params.yaml` | Landmark localizer config |
| `scripts/setup_rpi.sh` | RPi5 initial setup script |
| `docs/chassis/fusion360/roscar_v2_chassis.py` | Fusion 360 parametric chassis model script (rev13 — STEP imports for frame, simple placeholders for electronics) |
| `docs/chassis/models/` | 3D models: 3030 extrusion, brackets, T-nuts (STEP/IGES/F3D) |
| `tasks/chassis-v2-handoff.md` | Chassis v2 session handoff — design decisions, status, next steps |
| `tasks/chassis-v2-cut-sheet.md` | Printable saw-ready cut list: 13 pieces with length + label for each |

## Packages

| Package | Type | Description |
|---------|------|-------------|
| `roscar_driver` | ament_python | Hardware bridge: Rosmaster_Lib <-> ROS2 topics |
| `roscar_description` | ament_cmake | URDF model, rviz config |
| `roscar_bringup` | ament_cmake | Launch files, EKF/IMU filter configs |
| `roscar_interfaces` | ament_cmake | Custom .srv: SetMode, SaveMap, GetStatus |
| `roscar_web` | ament_python | Web dashboard: backend nodes + static frontend |
| `roscar_cv` | ament_python | CV nodes: ArUco + YOLO (runs on remote GPU PC, not Pi) |

## Key Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | Twist | teleop | Velocity commands |
| `/odom_raw` | Odometry | roscar_driver | Wheel odometry (integrated) |
| `/vel_raw` | Twist | roscar_driver | Raw measured body velocities |
| `/imu/data_raw` | Imu | roscar_driver | Raw accel + gyro |
| `/imu/data` | Imu | imu_filter_madgwick | Filtered IMU with orientation |
| `/imu/mag` | MagneticField | roscar_driver | Magnetometer |
| `/battery_voltage` | Float32 | roscar_driver | Battery level |
| `/scan_raw` | LaserScan | sllidar_node | RPLIDAR C1 raw laser scan (includes chassis reflections) |
| `/scan` | LaserScan | laser_filter | Filtered scan (chassis reflections < 0.15m removed) |
| `/odometry/filtered` | Odometry | ekf_node | Fused odometry (wheels+IMU) |
| `/image_raw` | Image | realsense2_camera | Color feed (640×480@30, REMAPPED from /camera/camera/color/image_raw) |
| `/camera_info` | CameraInfo | realsense2_camera | Factory intrinsics (REMAPPED from /camera/camera/color/camera_info) |
| `/map` | OccupancyGrid | slam_toolbox | SLAM-generated map |
| `/plan` | Path | planner_server | Global navigation path |
| `/local_plan` | Path | controller_server | Local trajectory |
| `/aruco/markers` | MarkerArray | aruco_detector_node | Detected ArUco marker visualizations |
| `/aruco/image` | Image | aruco_detector_node | Debug view with marker outlines |
| `/camera/camera/depth/image_rect_raw` | Image | realsense2_camera | D435i depth image (16-bit, 848×480@15) |
| `/camera/camera/depth/color/points` | PointCloud2 | realsense2_camera | D435i pointcloud aligned to color (15Hz) |
| `/local_costmap/voxel_marked_cloud` | PointCloud2 | nav2_costmap_2d | Voxel_layer 3D obstacle markers |
| `/detections` | Detection2DArray | yolo_detector_node | YOLO detection results |
| `/image_annotated` | Image | yolo_detector_node | Camera feed with bounding boxes |

## Rosmaster_Lib API (Key Methods)
- `Rosmaster(car_type=1, com='/dev/roscar_board', delay=0.002)` - Constructor
- `create_receive_threading()` - Start background serial receiver (MUST call)
- `set_car_motion(vx, vy, wz)` - Send velocity command (m/s, m/s, rad/s)
- `get_motion_data()` -> (vx, vy, vz) - Measured velocities
- `get_accelerometer_data()` -> (ax, ay, az) - m/s^2
- `get_gyroscope_data()` -> (gx, gy, gz) - rad/s
- `get_magnetometer_data()` -> (mx, my, mz)
- `get_battery_voltage()` -> float
- `get_motor_encoder()` -> (m1, m2, m3, m4) - Raw encoder counts
- `reset_car_state()` - Stop motors, lights off, buzzer off

### Car Type Constants
| Value | Constant | Description |
|-------|----------|-------------|
| 0x01 | CARTYPE_X3 | 4WD Mecanum (our robot) |
| 0x02 | CARTYPE_X3_PLUS | 4WD Mecanum Plus variant |
| 0x04 | CARTYPE_X1 | 2WD differential drive |
| 0x05 | CARTYPE_R2 | Ackermann steering |

### Serial Protocol
- USART1 via CH340 USB, 115200 baud, 8N1
- Wire encoding: velocities × 1000 (mm/s and mrad/s on wire, int16)
- Firmware velocity limits (X3 mecanum): vx ±1.0 m/s, vy ±1.0 m/s, wz ±5.0 rad/s
- Auto-report: 4 data packets (speed, IMU, encoder, etc.) every 10ms each (~40ms full cycle)

### IMU Sensor
- **V3.0 board uses ICM20948** (not MPU9250). Library auto-detects.
- ICM20948 conversion: firmware pre-scales, library divides by 1/1000.0 (all axes)
- MPU9250 conversion: gyro 1/3754.9, accel 1/1671.84
- MPU9250 library negates gy, gz internally; ICM20948 has no library-layer negation
- I2C on GPIO bit-bang: PB15 (SDA), PB13 (SCL), addr 0x68

### Encoder Specs
- 1320 counts per revolution (30:1 gear × 11 PPR × 4x quadrature)
- 16-bit counter, read every 10ms (100Hz PID loop)
- Motors: M1=front-left (TIM2), M2=rear-left (TIM4), M3=front-right (TIM5), M4=rear-right (TIM3)

### Mecanum Wheel Installation (ABBA pattern)
- Viewed from front: FL=A, FR=B, RL=B, RR=A (A/B are mirror-image 45° rollers)
- Wheel dimensions (radius, L_x, L_y) NOT documented — embedded in firmware, physical measurement required

## SLAM + Navigation

### Design Choices
- **slam_toolbox online_async**: RPi5-friendly async processing
- **DWB controller**: Holonomic support (min_vel_y for mecanum strafing)
- **SmacPlanner2D**: No min turning radius (holonomic)
- **AMCL OmniMotionModel**: Required for mecanum localization
- **RPi5-tuned**: controller 10Hz, local costmap 5Hz, global costmap 1Hz
- **Costmap inflation**: 0.08m inflation_radius, cost_scaling_factor 3.0 (tight for indoor use)
- **Collision monitor**: PolygonStop disabled (robot operates in tight spaces)
- **Landmark localizer**: ArUco marker-based pose correction (auto-learn mode)

### Landmark Localizer (ArUco Pose Correction)
Addresses SLAM drift in featureless corridors. When the robot sees an ArUco marker
whose map-frame position is known, it computes the drift and calls EKF's `/set_pose`
service to correct the robot's estimated position.

**Node**: `landmark_localizer` (roscar_driver package, runs on Pi)
**Config**: `roscar_driver/config/landmark_params.yaml`
**Persistence**: Learned marker positions saved to `~/roscar_ws/learned_markers.yaml`
**Dashboard**: Markers shown as diamonds on AIO map tile (green=visible, cyan=known)

**How it works**:
1. ArUco detector (on dev PC) publishes `/aruco/markers` (MarkerArray, optical frame coords)
2. Landmark localizer (on Pi) subscribes directly to `/aruco/markers` (bypasses cross-machine /tf)
3. Looks up `map → camera_optical_frame` TF (local Pi TF tree handles optical→ROS conversion)
4. Chains transforms: `map→optical × optical→marker` to get marker position in map frame
5. First sighting: records marker's map-frame position (auto-learn)
6. Subsequent sightings: compares observed vs. known position
7. If drift > 0.3m and robot is stationary: calls EKF `/set_pose` to correct pose

**Key design decisions**:
- Uses `camera_optical_frame` (not `camera_link`) for TF lookup — OpenCV's solvePnP returns
  tvec in optical convention (x=right, y=down, z=forward). The URDF's optical joint
  (`rpy=-π/2, 0, -π/2`) handles the coordinate conversion automatically.
- Subscribes to `/aruco/markers` topic directly instead of using TF for ArUco frames —
  CycloneDDS unicast has DDS discovery saturation issues with 20+ participants on `/tf`.
- Uses EKF `/set_pose` service (not `/initialpose`) — slam_toolbox online_async mode
  has zero subscribers on `/initialpose`.
- `load_learned` parameter (default=False): prevents loading stale markers from previous
  SLAM sessions (each SLAM restart creates a new map frame origin).

**Safety guards**:
- Only corrects when robot velocity < 0.05 m/s (stationary)
- Rate-limited to one correction per 10 seconds
- Ignores markers > 3m away (unreliable pose at distance)

**Services**:
- `/landmark/clear_markers` (std_srvs/Trigger): Clears all learned markers and deletes persistence file
- Called by dashboard reset button before SLAM restart

**Dashboard integration** (aio-map.js):
- Subscribes to `/landmark/known_markers` (std_msgs/String, JSON, published at 2Hz)
- `mapToScreen()` converts map-frame positions to canvas coordinates
- Diamond icons: green (currently visible), cyan (known but not visible)
- Works in both free and locked (robot-centered) view modes

**Marker setup**: Print ArUco markers from DICT_4X4_50 at 15cm size (chev.me/arucogen).
Place on walls in featureless areas (hallways, open rooms). The localizer auto-learns
positions on first sighting — no manual measurement needed.

**slam_toolbox tuning** (anti-drift, applied to both slam_toolbox.yaml and nav2_params.yaml):
| Parameter | Original | Tuned | Rationale |
|-----------|----------|-------|-----------|
| `max_laser_range` | 12.0 | 8.0 | Trim noisy long-range returns |
| `minimum_time_interval` | 0.5 | 0.3 | 50% faster scan processing |
| `minimum_travel_distance` | 0.3 | 0.2 | 33% tighter matching |
| `minimum_travel_heading` | 0.3 | 0.2 | 33% tighter |
| `loop_search_maximum_distance` | 3.0 | 4.0 | Wider loop closure search |

**Known limitation**: `correlation_search_space_dimension` > 0.5 crashes slam_toolbox
with "unable to get pointer in probability search" (OOB in internal probability grid).
Must stay at 0.5 (default).

### cmd_vel Pipeline (when Nav2 is running)
```
controller_server → /cmd_vel → velocity_smoother (20Hz) → /cmd_vel_smoothed → collision_monitor → /cmd_vel → driver
                                                                                                  ↑
teleop (joystick/keyboard) ──────────────────────────────────────────────────────────────────────────┘
```
**Known issue**: Both teleop and collision_monitor publish to `/cmd_vel`. When Nav2 has a failed goal and enters recovery behaviors, the recovery commands override teleop. Cancel the Nav2 goal to regain manual control. Future fix: add twist_mux for priority-based arbitration.

### Lifecycle Management (IMPORTANT)
In ROS2 Jazzy, `async_slam_toolbox_node` is a **lifecycle node**. It starts in "unconfigured" state and will NOT process scans or publish /map until explicitly transitioned through configure → activate. The `slam.launch.py` uses a `nav2_lifecycle_manager` with `autostart: True` and `bond_timeout: 0.0` (slam_toolbox doesn't implement the Nav2 bond interface). The `slam_nav.launch.py` and `navigation.launch.py` get lifecycle management from nav2_bringup's built-in lifecycle manager.

### Jazzy nav2_bringup Gotchas (CRITICAL)
Jazzy's `nav2_bringup` `bringup_launch.py` includes **collision_monitor**, **docking_server**, and **route_server** by default. ALL must configure+activate successfully or the lifecycle manager aborts the entire navigation stack. The `nav2_params.yaml` MUST include configs for all three, even though we don't use docking or routing:
- **collision_monitor**: Needs `observation_sources: ["scan"]` and polygon definitions
- **docking_server**: Needs `dock_plugins: ['simple_charging_dock']` with plugin config
- **route_server**: Needs `operations` list with plugin entries
- When `slam:=True`, slam_toolbox params must be in nav2_params.yaml (under `slam_toolbox:` namespace), NOT in a separate file

### Three Launch Modes
1. **SLAM mapping** (`slam.launch.py`): Drive with teleop to build map (8 nodes: 7 robot + lifecycle_manager)
2. **Navigation** (`navigation.launch.py map:=<path>`): Navigate on saved map
3. **SLAM + Nav2** (`slam_nav.launch.py`): Map and navigate simultaneously

### Map Workflow
```bash
# 1. Build a map
ros2 launch roscar_bringup slam.launch.py
# (drive around with teleop in another terminal)

# 2. Save the map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 3. Navigate on saved map
ros2 launch roscar_bringup navigation.launch.py map:=$HOME/maps/my_map.yaml
# (use rviz2: 2D Pose Estimate -> 2D Goal Pose)
```

### Nav2 Velocity Limits
| Axis | Min | Max | Unit |
|------|-----|-----|------|
| vx | -0.2 | 0.5 | m/s |
| vy | -0.3 | 0.3 | m/s (strafing) |
| wz | -2.0 | 2.0 | rad/s |

## Depth Camera (Intel RealSense D435i)

The D435i is the robot's sole camera — it replaced the Logitech webcam.
Provides **both** a depth pointcloud (for Nav2's 3D voxel_layer local costmap)
**and** a color stream (for ArUco, YOLO, dashboard). One sensor, two jobs.

### Hardware
- **Bus:** USB 3.0 SuperSpeed (REQUIRED — USB 2.0 caps depth at 6 FPS and
  causes MIPI errors under load). Verify with `dmesg | grep -i SuperSpeed`.
- **Cable:** must be a genuine USB 3.0-rated USB-C to USB-A. Charging-only
  USB-C cables force USB 2.0 fallback.
- **Firmware:** 5.17.0.10 known-good (update via Intel RealSense Viewer on
  Windows if needed).
- **Mount:** front-center of upper deck, tilted ~10° down. URDF placeholder
  pose at `d435i_x=0.09, d435i_z=0.07, d435i_pitch=0.175` — **TODO: measure
  after physical mount.**

### Launch
```bash
# Depth camera only (standalone):
ros2 launch roscar_bringup depth_camera.launch.py

# Full robot stack with depth enabled:
ros2 launch roscar_bringup robot.launch.py use_depth:=true

# Default is use_depth:=false so existing flows are unchanged.
```

### Streams
- **Depth:** 848×480 Z16 @ 15 Hz, topic `/camera/camera/depth/image_rect_raw`
- **Color:** 640×480 RGB8 @ 30 Hz, topic `/image_raw` (REMAPPED from
  `/camera/camera/color/image_raw` so existing consumers — dashboard MJPEG
  stream, ArUco, YOLO — work with zero code changes)
- **Camera info:** topic `/camera_info` (REMAPPED — factory intrinsics)
- **Pointcloud:** aligned to color @ 15 Hz, topic `/camera/camera/depth/color/points`
- Decimation filter ×2 halves pointcloud density for Pi5 CPU savings.
- Driver uses ~28% of one core on Pi5 with this config.

### Nav2 Integration
Local costmap gets a `voxel_layer` plugin with `d435i_depth` observation
source subscribed to `/camera/camera/depth/color/points`:
- `z_resolution: 0.1`, `z_voxels: 16` → 1.6m vertical range
- `min_obstacle_height: 0.05` → ignores ground-plane noise
- `obstacle_max_range: 3.0` → matches D435i useful depth range
- Global costmap stays lidar-only (we don't want transient 3D noise in the
  persistent map).

### IMU Handling
D435i has a built-in BMI055 IMU (accel + gyro). **It is disabled** in the
driver config (`enable_gyro: false`, `enable_accel: false`). The STM32's
ICM20948 stays the primary IMU for EKF. Enabling D435i IMU fusion requires
deliberate extrinsic calibration and is explicitly out of scope.

### TF Tree
The URDF's `realsense2_description` xacro owns the camera's internal
frames (`camera_link`, `camera_depth_frame`, `camera_color_frame`,
`camera_accel_frame`, `camera_gyro_frame`, and all `*_optical_frame`
variants). Driver has `publish_tf: false` to avoid duplicate broadcasts.

### Param-Name Gotcha (aarch64 NEON)
`realsense2_camera` autodetects CPU SIMD features and dynamically renames
the pointcloud param namespace. On Pi5 (aarch64) it's `pointcloud__neon_.*`
not `pointcloud.*`. YAML config uses the NEON-specific names; if ported to
another architecture (x86 WSL2), update accordingly.

### Camera Consolidation (Logitech removed)
The Logitech webcam was retired when the D435i replaced it. History:
1. First pass kept both cameras, renaming Logitech's URDF frames to
   `webcam_link` / `webcam_optical_frame` to free `camera_link` for the
   D435i's `realsense2_description` xacro.
2. Second pass removed the Logitech entirely — ArUco and YOLO now consume
   the D435i's color stream, which is factory-calibrated and global-shutter
   (better for marker detection than the Logitech's rolling shutter).
3. The D435i driver launch remaps `/camera/camera/color/image_raw` →
   `/image_raw` and `/camera/camera/color/camera_info` → `/camera_info` so
   existing consumers (dashboard MJPEG, ArUco, YOLO) needed no changes.
4. ArUco detector publishes marker TFs under `camera_color_optical_frame`
   (the D435i's factory optical frame); landmark localizer looks up
   `map → camera_color_optical_frame` accordingly.
5. `use_camera:=false` is the default in `robot.launch.py` now (v4l2_camera
   only runs if you physically reattach the Logitech and set the flag).

## Web Dashboard (roscar_web)

### Architecture
- **rosbridge_server** port 9090: WebSocket bridge to all ROS2 topics/services
- **web_video_server** port 8080: MJPEG camera stream (`/image_raw`)
- **http_server_node** port 8888: serves static frontend files from `web/`
- **launch_manager_node**: ROS2 services to start/stop robot launch modes via subprocess

### Web Nodes (launched by web.launch.py)
| Node | Package | Purpose |
|------|---------|---------|
| rosbridge_websocket | rosbridge_server | WebSocket bridge to ROS2 topics/services/actions |
| rosapi | rosapi | Provides /rosapi/* services (list nodes, topics, etc.) |
| web_video_server | web_video_server | MJPEG camera streams over HTTP |
| launch_manager | roscar_web | Mode switching service (start/stop robot launches) |
| http_server | roscar_web | Serves dashboard static files |

### Launch (auto-start on boot)
The web stack starts automatically via systemd:
```bash
# Service: roscar-web.service (enabled, starts on boot)
sudo systemctl status roscar-web   # check status
sudo systemctl restart roscar-web  # restart
sudo journalctl -u roscar-web -f   # follow logs

# Manual install (already done on Pi):
sudo cp ~/ROScar1/scripts/roscar-web.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable roscar-web

# Access from any device on the same network:
http://<robot-ip>:8888/
```

### Dashboard Tabs
| Tab | Function |
|-----|---------|
| DRIVE | Dual virtual joystick (translate vx/vy + drive/rotate wz), keyboard WASD+QE, SpaceMouse, gamepad |
| CAMERA | MJPEG live feed, quality/resolution selectors |
| MAP | OccupancyGrid renderer, pan/zoom, click-to-navigate (Nav2 goal) |
| STATUS | Position, velocity, orientation, battery gauge, active nodes |
| SETTINGS | Mode switch (idle/teleop/slam/navigation/slam_nav), map save |

### Services (provided by launch_manager_node)
| Service | Type | Description |
|---------|------|-------------|
| `/web/set_mode` | `roscar_interfaces/SetMode` | Switch robot launch mode |
| `/web/save_map` | `roscar_interfaces/SaveMap` | Save current SLAM map |
| `/web/get_status` | `roscar_interfaces/GetStatus` | Query current mode/nodes |
| `/web/list_maps` | `roscar_interfaces/ListMaps` | List saved maps in ~/maps/ |

### Web Frontend Files
```
roscar_ws/src/roscar_web/web/
├── index.html          # Tabbed single-page app (original)
├── aio.html            # All-in-one mission control dashboard
├── css/
│   ├── style.css       # Original tabbed theme
│   └── aio.css         # AIO dashboard standalone CSS
└── js/
    ├── lib/roslib.min.js      # vendored roslibjs
    ├── lib/nipplejs.min.js    # vendored nipplejs joystick
    ├── app.js          # Original: ROS connection, tabs, E-STOP
    ├── teleop.js       # Original: Joystick + keyboard -> /cmd_vel
    ├── camera.js       # Original: MJPEG stream
    ├── status.js       # Original: odometry/IMU/battery
    ├── lidar.js        # Original: /scan mini radar
    ├── map.js          # Original: OccupancyGrid + nav goals
    ├── modes.js        # Original: Mode switching + map save
    ├── aio-app.js      # AIO entry: ROS connection, E-STOP, module init
    ├── aio-teleop.js   # AIO: dual joystick + WASD + gamepad + SpaceMouse priority
    ├── aio-spacemouse.js   # AIO: WebHID SpaceMouse driver (connection, parsing, signal processing, settings)
    ├── aio-camera.js   # AIO: MJPEG always-on stream
    ├── aio-status.js   # AIO: status + mode switching combined
    ├── aio-lidar.js    # AIO: lidar mini-radar
    ├── aio-map.js      # AIO: OccupancyGrid + nav goals
    ├── aio-graphs.js   # AIO: rolling sparkline charts (vx/vy/wz/battery)
    ├── aio-diagnostics.js  # AIO: /rosout log viewer
    ├── aio-tf.js       # AIO: TF tree visualizer
    ├── aio-cv.js       # AIO: CV feed toggle + detection overlay
    └── aio-lidar-cam.js # AIO: sci-fi HUD lidar depth overlay on camera feed
```

### AIO Dashboard (aio.html)
All-in-one mission control page showing everything simultaneously (no tab switching).
Access at `http://<robot-ip>:8888/aio.html` — coexists with original tabbed UI at `/`.

**Layout**: CSS Grid, 3 columns x 2 rows:
| Camera + Lidar | Map (1.5x wide) | Status + Modes |
|----------------|------------------|----------------|
| Drive (joysticks) | Graphs (sparklines) | Diagnostics |

**Features beyond original UI**:
- **SpaceMouse teleop**: Chrome WebHID driver for 3Dconnexion SpaceMouse Wireless — proportional 6DOF input with configurable deadzone, sensitivity curves (linear/expo/s-curve), axis inversion, and independent speed limits. See [SpaceMouse Teleop](#spacemouse-teleop-webhid) section below.
- **Gamepad support**: Browser Gamepad API, standard mapping, 0.15 deadzone, priority over joystick/keyboard
- **Input priority system**: SpaceMouse > Gamepad > Virtual Joystick > Keyboard. First active source wins each 10Hz publish cycle.
- **Right joystick drive+rotate**: Right virtual joystick maps both axes (Y→vx forward/back, X→wz rotation) for single-thumb control
- **Sparkline graphs**: Rolling 60s canvas charts for vx, vy, wz, battery with hover tooltips
- **Diagnostics log viewer**: `/rosout` subscriber, severity filtering (ALL/INFO/WARN/ERR), 500-entry FIFO, auto-scroll
- **TF tree visualizer**: `/tf` + `/tf_static` subscribers, HTML tree with staleness detection
- **CV overlay**: Toggle between raw and annotated camera feed, shows detection counts from YOLO/ArUco
- **Lidar depth overlay**: Sci-fi HUD overlay on camera feed — projects /scan points within camera FOV as depth-coloured bars, proximity warning vignette, nearest-object callout, sweep animation. Toggle via DEPTH button in camera controls.

**Module architecture**: Each `aio-*.js` is an ES module. `aio-app.js` is the entry point that creates the ROS connection, E-STOP handler, and initializes all modules. Data sharing uses callbacks (`onOdomData`, `onBatteryData`) to avoid duplicate subscriptions.

### Install Dependencies (in addition to SLAM deps)
```bash
sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-web-video-server
```

### SpaceMouse Teleop (WebHID)

Proportional 6DOF teleop using a 3Dconnexion SpaceMouse Wireless via Chrome's WebHID API. Bypasses the 3Dconnexion desktop driver entirely — reads raw HID reports for clean proportional control.

**Supported devices**: Any 3Dconnexion SpaceMouse (vendor IDs `0x256F`, `0x046D`). Tested with SpaceMouse Wireless (cabled and via USB receiver).

**Browser setup (one-time)**:
WebHID requires a secure context. Since the dashboard is served over plain HTTP:
1. Navigate to `chrome://flags/#unsafely-treat-insecure-origin-as-secure`
2. Add `http://<robot-ip>:8888` (e.g., `http://192.168.1.170:8888`)
3. Relaunch Chrome

If WebHID is unavailable, the Connect button is hidden and a console warning is logged.

**Connection flow**:
1. Click "Connect SpaceMouse" in the DRIVE panel header
2. Select device from Chrome's HID picker (one-time — Chrome remembers the pairing)
3. On subsequent page loads, auto-reconnects without the picker

**Axis mapping** (SpaceMouse → ROS cmd_vel):
| SpaceMouse | Robot Motion | ROS Field |
|------------|-------------|-----------|
| Push forward/back (Trans Y) | Drive fwd/back | `linear.x` (vx) |
| Push left/right (Trans X) | Strafe | `linear.y` (vy) |
| Twist CW/CCW (Rot Rz) | Rotate | `angular.z` (wz) |

**E-STOP**: Any SpaceMouse button press triggers the app-level E-STOP (zero twist + toast).

**Settings** (collapsible panel, shown when connected, persisted in localStorage):

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `sm_deadzone` | 5 | 0–50 | Raw count threshold (below = zero) |
| `sm_curve` | exponential | linear/expo/s-curve | Sensitivity response curve |
| `sm_exponent` | 2.0 | 1.0–3.0 | Curve exponent (higher = more fine control at low deflection) |
| `sm_max_linear` | 0.3 m/s | 0.1–1.0 | Max linear velocity |
| `sm_max_angular` | 1.0 rad/s | 0.1–3.0 | Max angular velocity |
| `sm_invert_x/y/rz` | false | — | Axis inversion toggles |

SpaceMouse has its own speed limits, independent of the dashboard speed sliders (sliders dim when SM is active).

**Signal pipeline**: raw HID (±350 counts) → deadzone → normalize [-1,+1] → sensitivity curve → scale by max speed → output m/s and rad/s.

**Staleness**: If no HID report for 200ms, velocity returns null (puck released or device sleeping). Sleep indicator shown after 5s of inactivity.

## Recovery Service (port 9999)

Standalone admin page for restarting the web stack when the AIO dashboard is unresponsive. Zero ROS2 dependencies — pure Python stdlib `http.server`.

### Architecture
- **Separate systemd service** (`roscar-recovery.service`): independent of `roscar-web.service`
- **Single script** (`scripts/roscar-recovery.py`): inline HTML, no external files
- **Passwordless sudo**: sudoers rule allows `brian` to start/stop/restart `roscar-web` only

### Access
```
http://<robot-ip>:9999/
```

### Endpoints
| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | Recovery page (status + Start/Restart/Stop buttons) |
| GET | `/status` | JSON: service state, uptime, hostname |
| POST | `/start` | Start roscar-web service |
| POST | `/stop` | Stop roscar-web service |
| POST | `/restart` | Restart roscar-web service |

### Systemd
```bash
sudo systemctl status roscar-recovery   # check status
sudo systemctl restart roscar-recovery  # restart recovery service itself
sudo journalctl -u roscar-recovery -f   # follow logs
```

### Install (already in setup_rpi.sh)
```bash
sudo cp scripts/roscar-recovery.service /etc/systemd/system/
sudo cp scripts/roscar-recovery-sudoers /etc/sudoers.d/roscar-recovery
sudo chmod 440 /etc/sudoers.d/roscar-recovery
chmod +x scripts/roscar-recovery.py
sudo systemctl daemon-reload
sudo systemctl enable roscar-recovery
sudo systemctl start roscar-recovery
```

## Computer Vision (roscar_cv)

### Architecture
CV processing runs on the dev PC (RTX 4070 via WSL2), NOT on the Pi. Images flow from Pi → PC over CycloneDDS, get processed, and annotated images flow back.

```
RPi5 (Robot)                          Windows PC (WSL2)
┌─────────────┐    CycloneDDS     ┌──────────────────────────┐
│ v4l2_camera  │───/image_raw────>│ aruco_detector_node      │
│              │                   │   -> /aruco/markers      │
│              │                   │   -> /aruco/image        │
│              │                   │   -> TF (marker poses)   │
│              │                   │                          │
│              │                   │ yolo_detector_node       │
│              │                   │   -> /detections         │
│              │                   │   -> /image_annotated    │
│ web_video    │<──/image_annotated│ GPU: RTX 4070 (CUDA)    │
│ _server      │                   └──────────────────────────┘
└─────────────┘
```

### Launch (on WSL2 dev PC)
```bash
source ~/roscar_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
ros2 launch roscar_cv cv.launch.py                    # both nodes
ros2 launch roscar_cv cv.launch.py use_aruco:=false   # YOLO only
ros2 launch roscar_cv cv.launch.py yolo_model:=yolov8s.pt  # larger model
```

### Dependencies (WSL2)
```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-vision-msgs ros-jazzy-image-transport
pip3 install opencv-contrib-python ultralytics
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

### Dashboard Integration
- **Feed toggle**: RAW/CV button in camera panel header switches between `/image_raw` and `/image_annotated`
- **Detection overlay**: Shows object counts from `/detections` (e.g., "2 person · 1 chair")
- **CV stream routing**: Raw feeds from Pi's web_video_server (port 8080), annotated feeds from WSL2's web_video_server (port 8081)
- **Access**: `http://<pi-ip>:8888/aio.html?cv_host=<wsl2-ip>` (cv_host persisted in localStorage)
- Module: `aio-cv.js` — imported and initialized by `aio-app.js`

### Gotchas
- **vision_msgs v4 (Jazzy)**: Use `hyp.hypothesis.class_id` and `hyp.hypothesis.score`, NOT `hyp.id`/`hyp.score`
- **Parameter typing**: `declare_parameter('classes', [])` crashes — use `rclpy.Parameter.Type.INTEGER_ARRAY`
- **CycloneDDS localhost peer**: Required for web_video_server to discover YOLO's `/image_annotated` topic

## Build & Run

```bash
# On RPi5:
cd ~/roscar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Teleop (keyboard):
ros2 launch roscar_bringup teleop.launch.py

# Camera:
ros2 launch roscar_bringup camera.launch.py

# SLAM mapping:
ros2 launch roscar_bringup slam.launch.py

# Navigation on saved map:
ros2 launch roscar_bringup navigation.launch.py map:=$HOME/maps/my_map.yaml

# SLAM + Nav2 combined:
ros2 launch roscar_bringup slam_nav.launch.py

# Visualize from dev machine:
rviz2 -d roscar_ws/src/roscar_description/rviz/roscar.rviz
```

## WSL2 Visualization (Dev Machine)

### Setup
ROS2 Jazzy Desktop is installed in WSL2 (Ubuntu 24.04) with WSLg for GUI forwarding.
CycloneDDS unicast peer discovery bridges the Pi <-> WSL2 network.

### Prerequisites
- WSL2 mirrored networking: `networkingMode=mirrored` in `C:\Users\brian\.wslconfig`
- ROS2 Jazzy Desktop + CycloneDDS installed (run `scripts/install_ros2_wsl.sh` in WSL terminal)
- Windows Firewall: UDP ports 7400-7500 open (inbound + outbound)
- Hyper-V Firewall: may need a UDP allow rule for DDS data transport

### CycloneDDS Config
Both Pi (`~/cyclonedds.xml`) and WSL2 (`~/cyclonedds.xml`) use unicast peer discovery:
- Pi peers: `localhost` + WSL2's LAN IP (e.g., 192.168.1.194)
- WSL2 peers: `localhost` + Pi IP (192.168.1.170)
- `AllowMulticast=false` (multicast doesn't reliably cross WSL2 boundary)
- **CRITICAL**: WSL2 config MUST include `<Interfaces><NetworkInterface address="192.168.1.194" /></Interfaces>` (or whichever interface has the LAN IP — check with `ip -4 addr show`). Docker Desktop creates bridge networks that CycloneDDS may bind to instead, causing discovery to silently fail. Interface name can shift between `eth0`/`eth1` after Windows/WSL updates.
- **CRITICAL**: WSL2 peers list MUST include `localhost` — without it, local processes (e.g., YOLO publisher and web_video_server) cannot discover each other's DDS topics when `AllowMulticast=false`.

### CRITICAL: ros2 daemon
The ros2 daemon caches DDS discovery. If started before CycloneDDS env vars are set, it uses FastDDS and cross-machine discovery fails. Kill daemon with `pkill -f ros2.*daemon` and use `--no-daemon` for testing. `ros2 daemon stop` can itself hang if the daemon is in a bad state.

### CRITICAL: WSL2 Distros
Multiple WSL2 distros may exist (Ubuntu, Ubuntu-20.04, Ubuntu-24.04). ROS2 Jazzy is in `Ubuntu-24.04`, NOT the default `Ubuntu`. Always use `wsl -d Ubuntu-24.04` for ROS2 commands.

### Running rviz2
```bash
# In WSL terminal (env vars should be in .bashrc):
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
rviz2 -d /mnt/d/localrepos/ROScar1/roscar_ws/src/roscar_description/rviz/roscar.rviz
```

### rviz Config Displays
| Display | Topic | Default |
|---------|-------|---------|
| RobotModel | /robot_description | on |
| TF | /tf | on |
| LaserScan | /scan | on |
| SLAM Map | /map | on |
| Odometry (filtered) | /odometry/filtered | on |
| Global Plan | /plan | on |
| Local Plan | /local_plan | on |
| Camera | /image_raw | off |

### Useful CLI from WSL2
```bash
ros2 topic list --no-daemon         # List all Pi topics
ros2 topic echo /scan --once --no-daemon  # Check lidar
ros2 topic hz /odometry/filtered --no-daemon  # Odom rate
ros2 run rqt_graph rqt_graph        # Node graph
ros2 run rqt_tf_tree rqt_tf_tree    # TF tree
```

## Hardware Notes

### RPLIDAR C1
- Package: `sllidar_ros2` (build from source, cloned by setup_rpi.sh)
- Serial: 460800 baud, CP210x USB chip (10c4:ea60)
- udev symlink: `/dev/rplidar`
- TF frame: `laser`
- Publishes: `/scan` (sensor_msgs/LaserScan)
- Launch arg: `use_lidar:=true|false` in robot.launch.py

## Hardware Orientation
The board is mounted 180-deg rotated on the chassis. Motor wiring has been physically
corrected so all 4 motor ports match their board labels (M1=FL, M2=RL, M3=FR, M4=RR)
with correct polarity (+PWM = forward). No motor command or odometry sign corrections needed.

### Motor Wire Color Codes (DO NOT LOSE)
Physical cable colors → motor ports on the YB-ERF01-V3.0. Use these any time
you re-terminate a connector, trace a loose wire, or re-route after chassis work.

| Wire color | Port | Position       |
|-----------|------|----------------|
| Purple    | M1   | Front Left     |
| Orange    | M2   | Rear Left      |
| Yellow    | M3   | Front Right    |
| Green     | M4   | Rear Right     |

Mnemonic: **P.O.Y.G.** (P-O-Y-G) from front-left around the chassis counter-clockwise
(front-left → rear-left → front-right → rear-right), matching M1→M2→M3→M4.

IMU/magnetometer sensors are still on the rotated board and need software correction:

| Data | Correction |
|------|-----------|
| cmd_vel | `set_car_motion(vx, vy, wz)` — no corrections (wiring matches board labels) |
| Wheel odom | none — STM32 FK produces correct velocities directly |
| Accelerometer | negate ALL: ax, ay, az (180-deg rotation + gravity sign convention) |
| Gyroscope | negate ALL: gx, gy, gz (board firmware reports gz inverted) |
| Magnetometer | negate mx, my; keep mz |

## Driver Features

### Gyro Bias Calibration
On startup, `driver_node.py` samples ~200 gyroscope readings over 1 second while stationary, computes the mean bias, and subtracts it from all subsequent readings. Reduces yaw drift from ~0.11°/s to ~0.0015°/s (73x improvement).

### Yaw Trim
`yaw_trim` parameter compensates for mechanical pull when driving straight. Applied proportionally to forward speed: `wz += yaw_trim * vx`. Tune live: `ros2 param set /roscar_driver yaw_trim -0.03`. Negative = correct leftward pull.

### Velocity Smoothing (Deceleration Limiter)
The robot is top-heavy with a short wheelbase and tips over on abrupt stops. The driver limits deceleration rate so the robot slows down gently:
- `max_decel_linear`: 1.0 m/s² (default) — linear deceleration limit
- `max_decel_angular`: 3.0 rad/s² (default) — angular deceleration limit
- Acceleration is unlimited (instant response when speeding up)
- A 50Hz ramp timer smoothly moves current velocity toward the target
- The watchdog sets target to zero; the ramp handles the smooth stop
- Tune live: `ros2 param set /roscar_driver max_decel_linear 0.5`

### IMU → EKF Integration
The EKF uses `imu/data_raw` directly (bypassing Madgwick filter) for angular velocity fusion. The driver sets an identity orientation quaternion (0,0,0,1) on imu/data_raw so robot_localization's frame transform is a no-op. This avoids a subtle bug where negating gyro gz corrects the angular velocity sign but breaks Madgwick's internal orientation quaternion, which robot_localization uses for frame transforms even when orientation fusion is disabled.

## TODO
- [x] Measure actual robot dimensions and update URDF (measured 2026-03-16: 96.5mm wheelbase, 205mm track, 39.7mm wheel radius, 20.5mm ground clearance, 2.2kg)
- [x] Deploy SLAM to RPi5 and test mapping (verified: /map publishing, lifecycle auto-activate working)
- [x] WSL2 rviz2 visualization working (CycloneDDS unicast, all topics visible)
- [x] Nav2 slam_nav mode verified on hardware (all lifecycle nodes active, collision_monitor + docking_server + route_server configured)
- [x] Laser filter added to remove chassis self-reflections from RPLIDAR scan (< 0.15m)
- [x] Nav2 click-to-navigate verified (2D Goal Pose → planner → DWB controller → goal reached)
- [ ] Save first map and test navigation-only mode (navigation.launch.py)
- [ ] Verify holonomic motion (strafing) during navigation
- [ ] Tighten xy_goal_tolerance (currently 0.25m — robot declares success 25cm from target)
- [ ] Tune yaw_trim parameter for straight driving
- [x] Camera calibration for undistorted images (plumb_bob, fx=924, fy=920, cx=357, cy=253)
- [ ] Fine-tune EKF covariances under dynamic conditions
- [ ] Update Nav2 robot footprint after measuring real dimensions
- [ ] Tune Nav2 velocity/acceleration limits based on real-world testing
- [ ] Implement wireless gamepad control (Logitech F710 — USB dongle on RPi5, joy + teleop_twist_joy packages)
- [x] Fix camera feed (io_method=read fixes YUYV mmap failure; YUYV→RGB conversion works but slow)
- [ ] Optimize camera: switch to MJPG pixel format for faster streaming (needs cv_bridge MJPG support)
- [x] Landmark localizer: fix marker map positions (use camera_optical_frame TF, not manual optical→camera_link conversion)
- [x] Landmark localizer: add `/landmark/clear_markers` service for dashboard reset button
- [x] Landmark localizer: add `load_learned` parameter (False for SLAM, prevents stale markers from old sessions)
- [x] Dashboard: add ArUco marker diamonds on map tile (aio-map.js)
- [x] Dashboard: fix reset button crash (add 6s shutdown delay, clear markers before restart)
- [ ] Verify landmark drift correction end-to-end (markers learn correctly, correction needs real-world testing)
- [ ] Add landmark_localizer to navigation.launch.py with `load_learned: true`
- [ ] **Chassis v2**: Design and build new 3030 aluminum extrusion chassis (spec: `docs/superpowers/specs/2026-04-04-extrusion-chassis-design.md`)

## Chassis v2 Design (Parallel Development)

A new two-deck aluminum extrusion chassis is being designed to replace the current metal plate. **This work is isolated from the main codebase** — no existing files are modified until the physical build is complete.

**Design spec:** `docs/superpowers/specs/2026-04-04-extrusion-chassis-design.md`
**3D scan of motor bracket:** [Polycam](https://poly.cam/capture/47BED671-7E69-465D-95AC-22A8A67D7DB1)

### Key Design Decisions
- **3030 extrusion** as primary structural material (matches existing inventory)
- **Two-deck sandwich** — lower deck for battery/motors/board, upper deck for RPi5/camera
- **Lidar on a short mast** (120mm) at center-rear of upper deck
- **Motors hang BELOW frame** on L-brackets (confirmed from robot photos 2026-04-05) — NOT inside the frame
- **3D-printed motor brackets** (PETG, parametric) clamp to frame exterior for adjustable track/wheelbase
- **3-way corner brackets** at all 8 frame-to-post joints (no L-brackets needed for primary structure)
- **Frame: ~250×250mm square** from 3030 extrusion
- **Estimated total weight: ~4.9kg** (vs current 3.0kg — heavier but much more stable)
- **CoG at ~29% of total height** (vs current ~50-60% — major stability improvement)

### Fusion 360 Model
- **Script:** `docs/chassis/fusion360/roscar_v2_chassis.py` (rev8b — needs testing)
- **Current state:** Assembled model with colors, T-slot profiles, cylindrical wheels/motors, motor mounting below frame
- **CRITICAL:** All geometry must use XY construction plane ONLY — XZ/YZ planes have axis mapping bugs
- **Session handoff:** `tasks/chassis-v2-handoff.md` has full context for continuing this work

### Known Risks
- STM32 firmware has hardcoded mecanum geometry — changing track/wheelbase means odometry errors until firmware is reflashed
- Motor board IMU axis corrections may need updating if board orientation changes
- Laser filter threshold may need adjustment for new lidar height
