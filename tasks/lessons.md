# ROScar1 - Lessons Learned

## Rosmaster_Lib
- car_type=1 for mecanum (X3 type)
- set_car_motion(vx, vy, vz) handles mecanum IK internally on the STM32
- get_motion_data() returns (vx, vy, vz) measured velocities
- Background thread auto-parses serial data every 10ms per packet type
- Serial: 115200 baud, 8N1, no flow control, wire encoding = values × 1000 (int16)
- Must call create_receive_threading() before reading sensor data
- NOT on PyPI — install via: `pip install --break-system-packages git+https://github.com/RobLibs/Rosmaster_Lib@V3.3.9`
- Official Yahboom repo (ROS-robot-expansion-board) has PDFs only, no source code
- Encoder: 1320 CPR (30:1 gear × 11 PPR × 4x quadrature), read every 10ms
- **V3.0 board has ICM20948 IMU** (not MPU9250). Library auto-detects sensor.
- MPU9250 path: library internally negates gy and gz before returning to caller
- ICM20948 path: library does NO axis negation (firmware handles orientation)
- This means our driver's gyro negation (`gx,gy,gz = -gx,-gy,-gz`) has different net effects depending on which sensor is present — test empirically
- Wheel dimensions (radius, separation) are NOT in any public documentation — embedded in firmware functions `Motion_Get_Circle_MM()` and `Motion_Get_APB()`. Must measure physically.

## Deployment
- Ubuntu 24.04 on RPi5 needs `--break-system-packages` for pip installs (PEP 668)
- pip/pip3 not installed by default on Ubuntu 24.04 Server — need `apt install python3-pip`
- SSH + sudo requires NOPASSWD config for remote automation (`/etc/sudoers.d/brian-nopasswd`)
- teleop_twist_keyboard needs its own terminal with stdin — can't capture keys when launched inside a launch file. Run robot bringup and teleop in separate terminals.

## Hardware Orientation
- Board's X-axis (forward) points opposite to the physical front (where camera/lidar are)
- Motor L/R ports are also swapped relative to the board's expectations
- Commands: `set_car_motion(-vx, -vy, -wz)` — negate all three at hardware boundary
- Wheel odom: negate vx, vy ONLY; keep vz (Z-axis angular velocity is unchanged by 180° yaw rotation)
- IMU accelerometer: negate ALL three axes (ax, ay, az) — 180-deg yaw flips x,y; Rosmaster_Lib reports gravity as -Z but ROS expects +9.81 when flat, so negate az too
- IMU gyroscope: negate ALL three axes (gx, gy, gz) — board firmware reports gz with inverted sign (empirically confirmed: left turn gives negative gz from board)
- Magnetometer: negate mx, my; keep mz
- **LESSON**: The original code negated wheel odom vz — this broke yaw tracking. Z-axis rotation direction is NOT changed by a 180° yaw rotation, so vz should NOT be negated.
- **LESSON**: Negating gyro gz gives correct angular_velocity sign but BREAKS Madgwick filter. Madgwick uses gz internally for orientation tracking — negating it makes the orientation quaternion rotate the wrong way. Solution: bypass Madgwick entirely and use imu/data_raw for the EKF.

## robot_localization (EKF) IMU Gotcha
- Even when IMU orientation fusion is DISABLED in the config (imu0_config yaw=false), robot_localization STILL uses the IMU's orientation quaternion to TRANSFORM angular velocity from sensor frame to body frame
- A wrong/invalid orientation quaternion (like 0,0,0,0 or Madgwick's inverted quaternion) will flip the angular velocity sign silently
- Solution: set identity quaternion (0,0,0,1) on imu/data_raw to mean "sensor frame == body frame", then EKF can use angular velocity directly
- The EKF now uses imu/data_raw (not imu/data from Madgwick) for vyaw fusion

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

## Nav2 bringup in Jazzy (CRITICAL)
- `nav2_bringup`'s `bringup_launch.py` in Jazzy includes **collision_monitor**, **docking_server**, and **route_server** by default — these are new in Jazzy and not present in older Nav2 versions
- ALL nodes managed by `lifecycle_manager_navigation` MUST successfully configure+activate, or the lifecycle manager **aborts all of them** — a single failure kills the entire navigation stack
- collision_monitor requires `observation_sources` parameter — without it: "parameter 'observation_sources' is not initialized" → lifecycle abort
- docking_server requires `dock_plugins` with valid plugin entries — without it: "Charging dock plugins not given!" → lifecycle abort
- **Rule**: When using nav2_bringup's bringup_launch.py, the nav2_params.yaml MUST include configs for ALL nodes that the lifecycle manager manages, including collision_monitor, docking_server, and route_server
- Alternative: build your own launch file that only includes the Nav2 nodes you need (avoids configuring unused nodes)
- When using slam_nav mode (slam:=True), slam_toolbox params must be in the nav2_params.yaml file (under `slam_toolbox:` namespace) — nav2_bringup's slam_launch.py reads from the main params_file, NOT from a separate slam_toolbox.yaml
- The planner_server logs an ERROR about inflation radius being too small — this is a warning, not fatal. It still works.
- Local costmap logs TF timeout errors during startup while EKF hasn't published odom→base_footprint yet — these resolve automatically after a few seconds

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

## roslibjs + ROS2 rosbridge
- ROSLIB.ActionClient uses ROS1-style message naming (`NavigateToPoseGoal`) — doesn't exist in ROS2
- rosbridge_suite for ROS2 errors on the bad advertise, and the connection drops
- If ActionClient is created on every 'connected' event, it causes a reconnect loop (connect → error → close → 3s → reconnect)
- This prevents ALL other functionality (e.g., /cmd_vel never gets established)
- **Rule**: defer ActionClient creation to when it's actually needed (e.g., user enters nav goal mode), not on connection
- TODO: investigate newer roslibjs versions that support ROS2 actions natively, or use ROSLIB.Service for /navigate_to_pose

## Deployment: Git Repo vs Workspace on Pi
- Git repo lives at `~/ROScar1/` on the Pi
- `roscar_web` and `roscar_interfaces` are COPIED into `~/roscar_ws/src/` (not symlinked)
- `colcon build --symlink-install` symlinks install → build → src (the workspace copy)
- `git pull` updates `~/ROScar1/` but NOT `~/roscar_ws/src/roscar_web/`
- Must manually `cp` changed files from `~/ROScar1/...` to `~/roscar_ws/src/...` after pull
- `chmod +x` on scripts causes local git changes — use `git checkout --` before pull

## systemd for ROS2
- ROS2 `setup.bash` uses unbound variables (e.g., `AMENT_TRACE_SETUP_FILES`) — don't use `set -u` in launcher scripts
- Use `set -eo pipefail` (not `-euo`) for ROS2 launcher scripts

## rviz2 QoS for Cross-Machine Visualization
- slam_toolbox publishes /map with RELIABLE + TRANSIENT_LOCAL QoS
- rviz2 Map display MAY default to BEST_EFFORT or VOLATILE depending on platform
- **Always specify explicit QoS in rviz config files** when subscribing to cross-machine topics
- Map display needs: `Durability Policy: Transient Local`, `Reliability Policy: Reliable`
- LaserScan needs: `Reliability Policy: Best Effort` (sensor_default QoS)
- robot_description needs: `Durability Policy: Transient Local` (latched topic)
- rviz2 with `Fixed Frame: map` requires slam_toolbox or AMCL to publish the map→odom TF — won't work in teleop-only mode

## CycloneDDS on WSL2 with Docker Installed (CRITICAL)
- Docker Desktop installs bridge networks (docker0, br-*) inside WSL2
- CycloneDDS without explicit `<Interfaces>` may bind to docker0 or loopback instead of eth1
- **Symptom**: `ros2 topic list` hangs/times out even though ping to remote host works
- **Fix**: Add `<Interfaces><NetworkInterface name="eth0" /></Interfaces>` in `<General>` section of cyclonedds.xml
- **NOTE**: The interface name can change between `eth0` and `eth1` after Windows/WSL updates or Docker Desktop changes. Check with `ip -4 addr show` to find which interface has the LAN IP.
- The ros2 daemon caches DDS state — if started before env vars are set, it uses wrong RMW. Kill daemon and use `--no-daemon` for testing.
- `ros2 daemon stop` can itself hang if the daemon was started with wrong config — use `pkill -f ros2.*daemon` instead
- WSL2 has multiple distros (Ubuntu, Ubuntu-20.04, Ubuntu-24.04) — ROS2 Jazzy is only in Ubuntu-24.04, use `wsl -d Ubuntu-24.04`

## Velocity Smoothing
- Top-heavy mecanum robots with short wheelbases tip over on abrupt stops
- Implemented deceleration limiter in driver: 50Hz ramp timer limits decel to 1.0 m/s² (linear) and 3.0 rad/s² (angular)
- Acceleration is unlimited (instant response when speeding up)
- Watchdog now sets TARGET to 0; the ramp timer handles the gradual stop
- Tunable via parameters: max_decel_linear, max_decel_angular

## Lidar Self-Reflections (RPLIDAR C1)
- RPLIDAR C1 mounted on middle platform sees chassis/cables/battery at 0.02-0.09m range (~150 points per scan)
- These near-field reflections cause TWO problems:
  1. **costmap obstacle_layer**: lethal cells at robot position → "Start occupied" planner error
  2. **slam_toolbox**: bakes reflections into the MAP itself (static_layer) → permanent ghost obstacles at every position the robot visits
- `obstacle_min_range` on costmap obstacle layers does NOT fix problem #2 — it only affects the obstacle_layer, not the static_layer/map
- **Fix**: Use `laser_filters` package to filter at source before any subscriber sees the data
  - sllidar publishes to `/scan_raw`, laser_filter_node filters and republishes as `/scan`
  - `LaserScanRangeFilter` with `lower_threshold: 0.15` drops all returns < 15cm
- ROS2 laser_filters YAML format: use `node_name: ros__parameters: filter1:` with numbered keys, NOT YAML sequences (list with `-` syntax)
- The wrong YAML format ("Sequences can only be values and not keys in params") causes the node to crash immediately on startup

## Nav2 Progress/Goal Checking
- Default `required_movement_radius: 0.5` with `movement_time_allowance: 10.0` is too aggressive for slow robots
- A mecanum robot navigating at ~0.05 m/s needs ~10s to move 0.5m — barely meets the deadline, often aborts
- Relaxed to `required_movement_radius: 0.15`, `movement_time_allowance: 20.0`
- Default `xy_goal_tolerance: 0.25` means the robot is "at the goal" when within 25cm — may need tightening for precision
- Goal at (0.30, 0.00) with map extent [−1.65, 0.25] causes "Goal Coordinates outside bounds" — the goal must be within the SLAM map

## Nav2 Collision Monitor (CRITICAL)
- Nav2 velocity pipeline: controller → `/cmd_vel_nav` → velocity_smoother → `/cmd_vel_smoothed` → collision_monitor → `/cmd_vel`
- The collision_monitor sits LAST in the pipeline and can silently zero ALL velocity commands
- A PolygonStop zone matching the robot footprint exactly (0.15m from center) with `min_points: 4` triggers on nearby wall returns
- **Symptom**: controller publishes good velocities, smoother passes them through, but `/cmd_vel` is ALL zeros — robot won't move
- The laser_filter drops returns < 0.15m, but returns at 0.15-0.20m still fall inside a tight polygon
- **Fix**: Expand PolygonStop beyond footprint (0.20m) with high `min_points: 8`, add PolygonSlow (0.30m) for gradual deceleration
- **Debugging tip**: `ros2 topic echo /cmd_vel_nav` vs `/cmd_vel` — if nav has velocities but cmd_vel is zero, collision_monitor is the culprit

## udev
- CH340 (1a86:7523) → /dev/roscar_board (motor board)
- CP210x (10c4:ea60) → /dev/rplidar (RPLIDAR C1)
- Both symlinks confirmed working on first boot after setup_rpi.sh
