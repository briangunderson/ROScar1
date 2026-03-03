# ROScar1 - Lessons Learned

## Rosmaster_Lib
- car_type=1 for mecanum (X3 type)
- set_car_motion(vx, vy, vz) handles mecanum IK internally on the STM32
- get_motion_data() returns (vx, vy, vz) measured velocities
- Background thread auto-parses serial data every 10ms per packet type
- Serial: 115200 baud, 8N1, no flow control
- Must call create_receive_threading() before reading sensor data
- NOT on PyPI — install via: `pip install --break-system-packages git+https://github.com/RobLibs/Rosmaster_Lib@V3.3.9`
- Official Yahboom repo (ROS-robot-expansion-board) has PDFs only, no source code

## Deployment
- Ubuntu 24.04 on RPi5 needs `--break-system-packages` for pip installs (PEP 668)
- pip/pip3 not installed by default on Ubuntu 24.04 Server — need `apt install python3-pip`
- SSH + sudo requires NOPASSWD config for remote automation (`/etc/sudoers.d/brian-nopasswd`)
- teleop_twist_keyboard needs its own terminal with stdin — can't capture keys when launched inside a launch file. Run robot bringup and teleop in separate terminals.

## Hardware Orientation
- Board's X-axis (forward) points opposite to the physical front (where camera/lidar are)
- Motor L/R ports are also swapped relative to the board's expectations
- Net result: negate ALL components (vx, vy, wz) at the hardware boundary in driver_node.py
- Commands: `set_car_motion(-vx, -vy, -wz)`
- Odometry: negate vx, vy, vz from `get_motion_data()`
- IMU accelerometer: negate ALL three axes (ax, ay, az) — 180-deg yaw flips x,y; Rosmaster_Lib reports gravity as -Z but ROS expects +9.81 when flat, so negate az too
- IMU gyroscope: negate gx, gy only; gz is UNCHANGED by 180-deg yaw rotation (Z-axis direction preserved). Do NOT negate gz — it must agree with wheel encoder angular velocity or the EKF goes to NaN
- Magnetometer: negate mx, my; keep mz

## ROS2 Launch Files
- v4l2_camera's `image_size` parameter expects `integer_array` type — LaunchConfiguration substitutions resolve to strings, causing InvalidParameterTypeException
- Fix: use `OpaqueFunction` to call `.perform(context)` and cast to `int()` before passing to Node parameters
- General rule: any ROS2 parameter expecting int/float arrays will fail with LaunchConfiguration strings — always use OpaqueFunction for type conversion

## Camera (Logitech 046d:0825)
- Works with `v4l2_camera` package, YUYV @ 640x480, auto-converts to RGB8
- Publishes /image_raw at 30fps and /camera_info
- No calibration file yet — camera_calibration_parsers warns but runs fine
- YUYV→RGB8 conversion is "possibly slow" per v4l2_camera — consider MJPG output_encoding for better performance

## Sensor Fusion (EKF + Madgwick)
- imu_filter_madgwick: `fixed_frame` must be `odom`, NOT `base_link` — Madgwick needs a non-rotating reference
- Only ONE node should publish odom->base_footprint TF. With EKF running, set driver's `publish_odom_tf: false`
- EKF: don't fuse both position AND velocity from wheel odom — position is just integrated velocity, so they're not independent. Use velocity only from odom, let EKF integrate.
- EKF frequency 20Hz is sufficient for RPi5; 30Hz causes "failed to meet update rate" startup errors
- When debugging, watch out for STALE PROCESSES from previous launches. Multiple driver nodes publishing on the same topic causes confusing data. Always `kill -9` by PID before relaunching.
- The `pgrep` command is not installed by default on Ubuntu 24.04 Server — use `killall` or `ps aux | grep` + `kill` instead

## SLAM (slam_toolbox)
- In ROS2 Jazzy, `async_slam_toolbox_node` is a **lifecycle node** — starts in "unconfigured" state, will NOT process scans or publish /map until configure+activate transitions
- The `use_lifecycle_manager: false` parameter in slam_toolbox.yaml does NOT make the node auto-activate — it just means slam_toolbox won't launch its own internal lifecycle manager
- Fix: use `nav2_lifecycle_manager` node alongside slam_toolbox with `autostart: True` and `node_names: ['slam_toolbox']`
- Must set `bond_timeout: 0.0` because slam_toolbox doesn't implement the Nav2 bond heartbeat interface — otherwise lifecycle_manager logs a spurious error after 4s
- The `slam_nav.launch.py` and `navigation.launch.py` don't need this because they use `nav2_bringup`'s `bringup_launch.py` which includes its own lifecycle manager with `autostart: true`
- When deploying new files via `colcon build --symlink-install` for ament_cmake packages, must do a CLEAN rebuild (delete build/ and install/ for that package) — incremental builds don't pick up new files

## ES Module Circular Imports (roscar_web JS)
- `app.js` (module) has circular imports with teleop.js, status.js, lidar.js, map.js, modes.js
- `export function foo()` declarations in app.js ARE hoisted/available even to circular importers
- BUT `const moduleCallbacks = []` is NOT initialized until app.js's module body executes
- Calling `onAppEvent(cb)` at MODULE LEVEL in any sub-module (outside a function) will throw
  `ReferenceError: Cannot access 'moduleCallbacks' before initialization`
- This silently crashes the entire module graph — page renders HTML/CSS but JS does nothing
- **Rule**: NEVER call `onAppEvent()` at module level. Always register inside an `init*()` function.
- Sub-module init functions (initTeleop, initStatus, etc.) are called AFTER app.js fully evaluates,
  so `moduleCallbacks` is already initialized — safe to call `onAppEvent()` inside them.

## udev
- CH340 (1a86:7523) → /dev/roscar_board (motor board)
- CP210x (10c4:ea60) → /dev/rplidar (RPLIDAR C1)
- Both symlinks confirmed working on first boot after setup_rpi.sh
