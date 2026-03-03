# ROScar1 - 4WD Mecanum Robot

## Project Overview
A ROS2 Jazzy workspace for a 4WD Mecanum wheel robot built on:
- **Raspberry Pi 5** (Ubuntu 24.04 Server) running ROS2
- **YB-ERF01-V3.0** (Yahboom STM32F103RCT6 motor driver board) connected via USB serial
- **Logitech webcam** for vision
- **RPLIDAR C1** (Slamtec) for 2D laser scanning, connected via USB
- **4x DC encoder motors** with mecanum wheels

**Status**: Milestones 1-3 fully verified. Milestone 4 SLAM mapping verified on hardware (slam_toolbox publishing /map). Nav2 navigation pending testing. Web dashboard implemented (roscar_web package + roscar_interfaces).

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
| `/scan` | LaserScan | sllidar_node | RPLIDAR C1 laser scan |
| `/odometry/filtered` | Odometry | ekf_node | Fused odometry (wheels+IMU) |
| `/image_raw` | Image | v4l2_camera | Camera feed (640x480, 30fps) |
| `/camera_info` | CameraInfo | v4l2_camera | Camera calibration info |
| `/map` | OccupancyGrid | slam_toolbox | SLAM-generated map |
| `/plan` | Path | planner_server | Global navigation path |
| `/local_plan` | Path | controller_server | Local trajectory |

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
- `car_type=1` for mecanum (X3)

## SLAM + Navigation

### Design Choices
- **slam_toolbox online_async**: RPi5-friendly async processing
- **DWB controller**: Holonomic support (min_vel_y for mecanum strafing)
- **SmacPlanner2D**: No min turning radius (holonomic)
- **AMCL OmniMotionModel**: Required for mecanum localization
- **RPi5-tuned**: controller 10Hz, local costmap 5Hz, global costmap 1Hz

### Lifecycle Management (IMPORTANT)
In ROS2 Jazzy, `async_slam_toolbox_node` is a **lifecycle node**. It starts in "unconfigured" state and will NOT process scans or publish /map until explicitly transitioned through configure → activate. The `slam.launch.py` uses a `nav2_lifecycle_manager` with `autostart: True` and `bond_timeout: 0.0` (slam_toolbox doesn't implement the Nav2 bond interface). The `slam_nav.launch.py` and `navigation.launch.py` get lifecycle management from nav2_bringup's built-in lifecycle manager.

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

### Launch
```bash
# On RPi5 (starts web stack only; select mode from browser):
ros2 launch roscar_web web.launch.py

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
├── index.html          # Single-page app
├── css/style.css       # Industrial mission-control theme
└── js/
    ├── lib/roslib.min.js      # vendored roslibjs
    ├── lib/nipplejs.min.js    # vendored nipplejs joystick
    ├── app.js          # Entry: ROS connection, tabs, E-STOP
    ├── teleop.js       # Joystick + keyboard -> /cmd_vel
    ├── camera.js       # MJPEG stream from web_video_server
    ├── status.js       # Subscribe odometry/IMU/battery
    ├── lidar.js        # /scan mini radar on drive tab
    ├── map.js          # OccupancyGrid canvas + nav goals
    └── modes.js        # Mode switching + map save services
```

### Install Dependencies (in addition to SLAM deps)
```bash
sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-web-video-server
```

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

## Hardware Notes

### RPLIDAR C1
- Package: `sllidar_ros2` (build from source, cloned by setup_rpi.sh)
- Serial: 460800 baud, CP210x USB chip (10c4:ea60)
- udev symlink: `/dev/rplidar`
- TF frame: `laser`
- Publishes: `/scan` (sensor_msgs/LaserScan)
- Launch arg: `use_lidar:=true|false` in robot.launch.py

## Hardware Orientation (CRITICAL)
The board is mounted 180-deg rotated and motor L/R ports are swapped.
All sensor/command data is corrected at the hardware boundary in driver_node.py:

| Data | Correction |
|------|-----------|
| cmd_vel | `set_car_motion(-vx, -vy, -wz)` |
| Wheel odom | negate vx, vy, vz from `get_motion_data()` |
| Accelerometer | negate ALL: ax, ay, az (180-deg + gravity sign convention) |
| Gyroscope | negate gx, gy ONLY; keep gz (Z-axis unchanged by yaw rotation) |
| Magnetometer | negate mx, my; keep mz |

## TODO
- [ ] Measure actual robot dimensions and update URDF (current values are PLACEHOLDERS)
- [x] Deploy SLAM to RPi5 and test mapping (verified: /map publishing, lifecycle auto-activate working)
- [ ] Save first map and test Nav2 navigation
- [ ] Verify holonomic motion (strafing) during navigation
- [ ] Camera calibration for undistorted images
- [ ] Fine-tune EKF covariances under dynamic conditions
- [ ] Update Nav2 robot footprint after measuring real dimensions
- [ ] Tune Nav2 velocity/acceleration limits based on real-world testing
