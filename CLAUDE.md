# ROScar1 - 4WD Mecanum Robot

## Project Overview
A ROS2 Jazzy workspace for a 4WD Mecanum wheel robot built on:
- **Raspberry Pi 5** (Ubuntu 24.04 Server) running ROS2
- **YB-ERF01-V3.0** (Yahboom STM32F103RCT6 motor driver board) connected via USB serial
- **Logitech webcam** for vision
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
| `roscar_ws/src/roscar_description/urdf/roscar.urdf.xacro` | Robot model (PLACEHOLDER dimensions!) |
| `roscar_ws/src/roscar_bringup/launch/robot.launch.py` | Full robot bringup |
| `roscar_ws/src/roscar_bringup/launch/teleop.launch.py` | Teleop + full robot |
| `roscar_ws/src/roscar_bringup/config/ekf.yaml` | EKF sensor fusion config |
| `roscar_ws/src/roscar_bringup/config/laser_filter.yaml` | Laser range filter (drops chassis reflections < 0.15m) |
| `roscar_ws/src/roscar_bringup/config/slam_toolbox.yaml` | slam_toolbox online_async config |
| `roscar_ws/src/roscar_bringup/config/nav2_params.yaml` | Nav2 full stack config (DWB holonomic) |
| `roscar_ws/src/roscar_bringup/launch/slam.launch.py` | SLAM mapping (teleop + slam_toolbox) |
| `roscar_ws/src/roscar_bringup/launch/navigation.launch.py` | Nav2 on saved map |
| `roscar_ws/src/roscar_bringup/launch/slam_nav.launch.py` | SLAM + Nav2 combined |
| `roscar_ws/src/roscar_web/launch/web.launch.py` | Web dashboard (rosbridge+video+http) |
| `roscar_ws/src/roscar_web/roscar_web/launch_manager_node.py` | Mode switching service node |
| `roscar_ws/src/roscar_web/roscar_web/http_server_node.py` | Static file server node |
| `scripts/setup_rpi.sh` | RPi5 initial setup script |

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
| `/image_raw` | Image | v4l2_camera | Camera feed (640x480, 30fps) |
| `/camera_info` | CameraInfo | v4l2_camera | Camera calibration info |
| `/map` | OccupancyGrid | slam_toolbox | SLAM-generated map |
| `/plan` | Path | planner_server | Global navigation path |
| `/local_plan` | Path | controller_server | Local trajectory |
| `/aruco/markers` | MarkerArray | aruco_detector_node | Detected ArUco marker visualizations |
| `/aruco/image` | Image | aruco_detector_node | Debug view with marker outlines |
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
| DRIVE | Dual virtual joystick (translate vx/vy + rotate wz), keyboard WASD+QE |
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
    ├── aio-teleop.js   # AIO: dual joystick + WASD + gamepad
    ├── aio-camera.js   # AIO: MJPEG always-on stream
    ├── aio-status.js   # AIO: status + mode switching combined
    ├── aio-lidar.js    # AIO: lidar mini-radar
    ├── aio-map.js      # AIO: OccupancyGrid + nav goals
    ├── aio-graphs.js   # AIO: rolling sparkline charts (vx/vy/wz/battery)
    ├── aio-diagnostics.js  # AIO: /rosout log viewer
    ├── aio-tf.js       # AIO: TF tree visualizer
    └── aio-cv.js       # AIO: CV feed toggle + detection overlay
```

### AIO Dashboard (aio.html)
All-in-one mission control page showing everything simultaneously (no tab switching).
Access at `http://<robot-ip>:8888/aio.html` — coexists with original tabbed UI at `/`.

**Layout**: CSS Grid, 3 columns x 2 rows:
| Camera + Lidar | Map (1.5x wide) | Status + Modes |
|----------------|------------------|----------------|
| Drive (joysticks) | Graphs (sparklines) | Diagnostics |

**Features beyond original UI**:
- **Gamepad support**: Browser Gamepad API, standard mapping, 0.15 deadzone, priority over joystick/keyboard
- **Sparkline graphs**: Rolling 60s canvas charts for vx, vy, wz, battery with hover tooltips
- **Diagnostics log viewer**: `/rosout` subscriber, severity filtering (ALL/INFO/WARN/ERR), 500-entry FIFO, auto-scroll
- **TF tree visualizer**: `/tf` + `/tf_static` subscribers, HTML tree with staleness detection

**Module architecture**: Each `aio-*.js` is an ES module. `aio-app.js` is the entry point that creates the ROS connection, E-STOP handler, and initializes all modules. Data sharing uses callbacks (`onOdomData`, `onBatteryData`) to avoid duplicate subscriptions.

### Install Dependencies (in addition to SLAM deps)
```bash
sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-web-video-server
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
- Module: `aio-cv.js` — imported and initialized by `aio-app.js`

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
- **CRITICAL**: WSL2 config MUST include `<Interfaces><NetworkInterface name="eth0" /></Interfaces>` (or whichever interface has the LAN IP — check with `ip -4 addr show`). Docker Desktop creates bridge networks that CycloneDDS may bind to instead, causing discovery to silently fail. Interface name can shift between `eth0`/`eth1` after Windows/WSL updates.

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
- [ ] Measure actual robot dimensions and update URDF (current values are PLACEHOLDERS)
- [x] Deploy SLAM to RPi5 and test mapping (verified: /map publishing, lifecycle auto-activate working)
- [x] WSL2 rviz2 visualization working (CycloneDDS unicast, all topics visible)
- [x] Nav2 slam_nav mode verified on hardware (all lifecycle nodes active, collision_monitor + docking_server + route_server configured)
- [x] Laser filter added to remove chassis self-reflections from RPLIDAR scan (< 0.15m)
- [x] Nav2 click-to-navigate verified (2D Goal Pose → planner → DWB controller → goal reached)
- [ ] Save first map and test navigation-only mode (navigation.launch.py)
- [ ] Verify holonomic motion (strafing) during navigation
- [ ] Tighten xy_goal_tolerance (currently 0.25m — robot declares success 25cm from target)
- [ ] Tune yaw_trim parameter for straight driving
- [ ] Camera calibration for undistorted images
- [ ] Fine-tune EKF covariances under dynamic conditions
- [ ] Update Nav2 robot footprint after measuring real dimensions
- [ ] Tune Nav2 velocity/acceleration limits based on real-world testing
- [ ] Implement wireless gamepad control (Logitech F710 — USB dongle on RPi5, joy + teleop_twist_joy packages)
- [ ] Fix camera feed (web_video_server not streaming — may need video encoding tweaks for Logitech webcam)
