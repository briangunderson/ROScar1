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
- Board is mounted 180° rotated on the chassis
- **Motor wiring physically corrected** (2025-03-11): All 4 motor connectors rewired so ports match physical positions with correct polarity:
  - M1=FL forward, M2=RL forward, M3=FR forward, M4=RR forward (all for +PWM)
  - Required: reversing polarity on all 4 motor connectors + swapping M2↔M4 cables on the board
- **Motor commands**: `set_car_motion(vx, vy, wz)` — NO sign corrections needed
- **Wheel odom readback**: use raw values from `get_motion_data()` — NO negation needed
- **LESSON**: Left and right H-bridges on YB-ERF01-V3.0 have DIFFERENT polarity conventions. Moving a cable from one side to the other flips its direction.
- **LESSON**: Theoretical analysis of 180° rotation + L/R swap effects is error-prone. Always verify empirically with individual `set_motor()` tests on each port.
- **LESSON**: When troubleshooting motor direction, test individual motors with raw PWM (`set_motor`) to map physical wiring before attempting software corrections.
- IMU accelerometer: negate ALL three axes (ax, ay, az) — 180-deg yaw flips x,y; Rosmaster_Lib reports gravity as -Z but ROS expects +9.81 when flat, so negate az too
- IMU gyroscope: negate ALL three axes (gx, gy, gz) — board firmware reports gz with inverted sign (empirically confirmed: left turn gives negative gz from board)
- Magnetometer: negate mx, my; keep mz
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
- RPi5 ISP occupies /dev/video0-37 — USB webcams get high numbers. Use udev symlink `/dev/webcam`
- **web_video_server does NOT URL-decode query parameter values** — `encodeURIComponent('/image_raw')` → `%2Fimage_raw` which doesn't match any topic. Server sends HTTP 200 + multipart headers but closes with no frames. Browser `<img>` tag fails silently (no onload, no onerror), showing black rectangle.
- **Rule**: Never `encodeURIComponent()` ROS topic names in web_video_server URLs. ROS topic chars (`/`, `_`, alphanumeric) are all valid in URL query parameters without encoding.

## Sensor Fusion (EKF + Madgwick)
- imu_filter_madgwick: `fixed_frame` must be `odom`, NOT `base_link` — Madgwick needs a non-rotating reference
- Only ONE node should publish odom->base_footprint TF. With EKF running, set driver's `publish_odom_tf: false`
- EKF: don't fuse both position AND velocity from wheel odom — position is just integrated velocity, so they're not independent. Use velocity only from odom, let EKF integrate.
- EKF frequency 20Hz is sufficient for RPi5; 30Hz causes "failed to meet update rate" startup errors
- When debugging, watch out for STALE PROCESSES from previous launches. Multiple driver nodes publishing on the same topic causes confusing data. Always `kill -9` by PID before relaunching.
- **CRITICAL: Duplicate EKF nodes cause state explosion**. Two EKF nodes reading the same sensor topics and publishing to `/odometry/filtered` creates a feedback/divergence spiral. Observed: angular.z reaching 55 MILLION rad/s despite sane inputs. Root cause in one case: two `ros2 launch` commands ran in backgrounded SSH sessions, each spawning their own EKF + driver + IMU nodes. The launch processes died but child nodes survived as orphans. Fix: kill ALL orphaned ROS nodes (`ps aux | grep` each node type), then start a single clean launch.
- **LESSON**: When launching ROS2 via SSH background (`nohup ... &`), killing the launch process does NOT kill child nodes — they become orphans. Always check for orphaned processes before relaunching: `ps aux | grep -E 'ekf|driver|imu|sllidar|v4l2|joint_state'`
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

## Python __pycache__ Gotcha (CRITICAL)
- After SCP'ing updated `.py` files, Python may still load the OLD code from `__pycache__/*.pyc` if the `.pyc` has a newer timestamp than the `.py`
- `colcon build --symlink-install` doesn't clear `__pycache__` — stale bytecache persists across builds
- **Fix**: Always clear `__pycache__` after deploying changed Python files: `find ~/roscar_ws -path "*/__pycache__" -name "*.pyc" -delete`
- Or `touch` the `.py` file to ensure its timestamp is newer than any `.pyc`
- **Rule**: After SCP + colcon build, delete `__pycache__` for any package with changed Python files before restarting nodes

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

## Nav2 Collision Monitor + cmd_vel Pipeline (CRITICAL)
- Nav2 Jazzy velocity pipeline: controller → `/cmd_vel` → velocity_smoother → `/cmd_vel_smoothed` → collision_monitor → `/cmd_vel`
- This IS a feedback loop by design — velocity_smoother subscribes to `/cmd_vel` (from controller) and collision_monitor publishes back to `/cmd_vel`
- The collision_monitor sits LAST in the pipeline and can silently zero ALL velocity commands
- **cmd_vel conflict**: When Nav2 + teleop both run (slam_nav mode), the controller, velocity_smoother (via collision_monitor), AND teleop all publish to `/cmd_vel`. The driver gets a mix of messages.
- **Symptom**: When Nav2 goal fails → recovery behaviors (spin, backup, wait) run continuously, overriding teleop commands. Drive page "stops working" because recovery commands override joystick.
- **Root cause of "Start occupied"**: costmap inflation_radius was too aggressive (0.15m) + PLACEHOLDER footprint (0.25×0.28m) = effective zone ~0.55×0.58m. Any wall within ~28cm of robot → planner thinks start is inside obstacle.
- **Fix**: Reduced inflation_radius to 0.08m, cost_scaling_factor to 3.0. Effective zone now ~0.41×0.44m.
- PolygonStop disabled (robot operates in tight spaces, costmap handles avoidance via planner)
- **Debugging tip**: `ros2 topic echo /cmd_vel_smoothed` vs `/cmd_vel` — if smoothed has velocities but cmd_vel is zero, collision_monitor is the culprit
- **Future**: Add `twist_mux` for proper teleop/Nav2 coexistence with priority-based cmd_vel arbitration

## Nav2 Cancel Goal (Web Dashboard)
- roslibjs ActionClient.sendGoal() returns a goalHandle with .cancel() method
- The cancel button must remain visible while a goal is ACTIVE (goalHandle !== null), not just while in goal-picking mode (navGoalMode)
- Separating navGoalMode (clicking map to pick target) from goalHandle (goal is running) prevents losing the cancel button after sending a goal

## rosapi Node (CRITICAL for Web Dashboard)
- roslibjs's `ros.getNodes()` requires the `rosapi` node to be running — it calls the `/rosapi/nodes` service
- Without rosapi, the STATUS tab's node list shows nothing (silent failure)
- **Fix**: Add rosapi_node to web.launch.py alongside rosbridge_server
- rosapi is part of `rosbridge_suite` but is a SEPARATE node that must be explicitly launched

## Motor Diagnostics
- A dead encoder on one mecanum wheel is nearly invisible during manual teleop (human compensates) but completely breaks Nav2 autonomous navigation
- The STM32 PID loop uses encoder feedback — dead encoder means PID saturates output (motor runs uncontrolled) and `get_motion_data()` returns wrong velocities
- Two compounding failures: (1) wheel odom is systematically wrong → EKF position drifts, (2) actual robot motion doesn't match commanded velocities → controller can't follow paths
- `scripts/motor_test.py` tests all 4 motors independently: commands forward/strafe/rotate and reads encoder deltas per motor
- Motor mapping: M1=front-left, M2=rear-left, M3=front-right, M4=rear-right
- To isolate motor vs board vs wiring: swap wires between ports and re-test. If the dead reading follows the motor, it's the motor/encoder. If it stays on the same port, it's the board.

## udev
- CH340 (1a86:7523) → /dev/roscar_board (motor board)
- CP210x (10c4:ea60) → /dev/rplidar (RPLIDAR C1)
- Both symlinks confirmed working on first boot after setup_rpi.sh

## ArUco Marker Coordinate Frames (CRITICAL)
- OpenCV's `estimatePoseSingleMarkers` returns `tvec` in **optical frame** convention: x=right, y=down, z=forward (depth)
- The ArUco detector node publishes this as `frame_id: camera_link` but the coordinates are NOT in camera_link convention
- **WRONG approach**: manually convert optical→camera_link (`cam_x=opt_z, cam_y=-opt_x, cam_z=-opt_y`). This is error-prone and placed markers behind/to-side of the robot on the map.
- **CORRECT approach**: look up `map → camera_optical_frame` TF and use the raw tvec directly. The URDF defines `camera_optical_frame` with `rpy=(-π/2, 0, -π/2)` relative to `camera_link`, so the TF tree handles the coordinate conversion automatically.
- **Rule**: When OpenCV returns camera-frame coordinates, use `camera_optical_frame` for TF lookups, never `camera_link`.

## CycloneDDS Participant Limits
- Default `MaxAutoParticipantIndex` is ~10 — too low when 20+ ROS2 nodes are running (SLAM + Nav2 + web stack)
- Cross-machine `/tf` topic delivery fails silently when participant slots are exhausted
- **Fix**: Set `MaxAutoParticipantIndex` to 120 in cyclonedds.xml
- Even with this fix, `/tf` is unreliable cross-machine under heavy load — prefer direct topic subscription for cross-machine data

## slam_toolbox + Pose Correction
- `async_slam_toolbox_node` in online_async mode has **zero subscribers** on `/initialpose` — cannot use it for pose corrections
- **Fix**: Use EKF's `/set_pose` service (`robot_localization/srv/SetPose`) instead — resets the odom→base_footprint transform
- slam_toolbox re-matches scans on next cycle, adjusting map→odom accordingly

## SLAM Session-Specific Markers
- Each SLAM restart creates a **new map frame origin** — learned marker positions from previous sessions have wrong coordinates
- Persistence file (`learned_markers.yaml`) must NOT be loaded on SLAM startup
- **Fix**: `load_learned` parameter (default=False) — only load in navigation mode (fixed map)
- Dashboard reset button: clear markers (service call) → idle → wait 6s → restart SLAM

## SLAM Restart Race Condition
- Old SLAM nodes take ~5s to terminate after SIGINT (SIGINT → 5s timeout → SIGTERM)
- Starting new SLAM immediately causes node conflicts (duplicate publishers, TF confusion)
- **Fix**: Add 6s delay between idle and SLAM restart (setTimeout in dashboard JS)

## Landmark Localizer vs SLAM (CRITICAL)
- Landmark localizer and slam_toolbox online_async are **fundamentally incompatible**
- slam_toolbox's map optimization (loop closures, scan pose adjustments) shifts the map frame over time
- Learned marker positions become stale after map optimization — correction teleports robot to wrong position
- SLAM then fights the correction, creating a feedback loop: marker corrects → SLAM adjusts → marker corrects to stale position → repeat
- Observed: 4.98m "drift correction" with 105° yaw change — not drift correction, it's teleportation
- **Rule**: NEVER run landmark localizer during SLAM mapping. Only use on a FIXED map (navigation mode with `load_learned: true`)
- Landmark localizer belongs in `navigation.launch.py`, NOT `slam.launch.py`

## Intel RealSense D435i (2026-04-21)
- **USB 3.0 is NOT optional** — D435i on USB 2.0 caps at 6 FPS depth, triggers MIPI
  errors under any real load. Verify with `dmesg | grep -i SuperSpeed` (NOT just "blue
  USB port" — a charging-only USB-C cable forces USB 2.0 fallback even on a blue port).
- **`lsusb -t` bus speed is the authoritative check**: 480M = USB 2.0, 5000M = USB 3.0.
  The "Device USB type:" line in realsense2_camera_node's startup log also reports
  USB version ("3.2" = USB 3.x).
- **Original box cable is USB 3.0-rated** (look for "SuperSpeed" or "SS" on the connector
  or jacket). Random USB-C cables from Amazon are often charging-only — they have the
  4 USB 2.0 pins but not the 8 SuperSpeed pins, so connection succeeds but falls back.
- **realsense2_description `sensor_d435i` macro signature (jazzy)**: only accepts
  `parent`, `*origin`, `name`, `use_nominal_extrinsics`. Older/other-distro versions
  have `topics_ns`, `add_plug`, `use_mesh` — those DON'T exist on jazzy and cause
  "Invalid parameter" errors. Always `grep -A2 "xacro:macro name=\"sensor_d435i\""`
  the actual macro file in `/opt/ros/<distro>/share/realsense2_description/urdf/`.
- **realsense2_camera dynamic param namespace rename**: on aarch64 (Pi5), the
  pointcloud param namespace is `pointcloud__neon_.*` not `pointcloud.*`. On x86 with
  SSSE3 it's `pointcloud__ssse3_.*`. The driver rewrites at startup based on detected
  CPU SIMD features. If a YAML parameter "silently has no effect", `ros2 param list`
  the live node and look for the `__<simd>_` variant.
- **Depth profile param is `depth_module.depth_profile`** not `depth_module.profile`.
  Color is `rgb_camera.color_profile` not `rgb_camera.profile`.
- **publish_tf: false** when URDF owns the camera TF tree via `realsense2_description`'s
  xacro. Leaving it true causes duplicate static broadcasts + "extrapolation into the
  future" spam in /tf.
- **URDF name collision**: `realsense2_description`'s sensor_d435i macro always uses
  `camera_link` as its body frame. Any existing `camera_link` (e.g. Logitech webcam)
  MUST be renamed first. Ripple changes through: URDF, camera.launch.py `frame_id`
  params, aruco_detector_node.py's `header.frame_id`, landmark_localizer_node.py's TF
  lookup frame, camera_calibration.yaml (if it has a frame_id), any rviz config.
- **CPU budget on Pi5** for realsense2_camera with depth+color@15 + decimation×2:
  ~28% of one core. Well within budget.
- **D435i replaces the Logitech webcam outright** — its color camera is factory
  calibrated (no checkerboard), global shutter (less motion blur than Logitech's
  rolling shutter), and already in the budget. Rather than maintain two camera
  frames + two URDF links + two sets of CV pipelines, use topic remaps in
  `depth_camera.launch.py` to publish D435i color as `/image_raw` and
  `/camera_info`. Consumers don't know or care. Keep `use_camera:=false` as
  default in `robot.launch.py` so the legacy v4l2_camera node doesn't spawn
  and complain about a missing `/dev/video0`.
- **ArUco frame restored to `camera_color_optical_frame`** (realsense2_description's
  factory name) once the Logitech was retired. Don't invent frame names if a
  library-provided one already fits.
- **Pi5 + D435i + RPLIDAR C1 USB power contention**. With both devices attached,
  the RPLIDAR's CP2102N bridge reports `cp210x ttyUSB1: failed set request 0x12
  status: -110` (ETIMEDOUT on `CP210X_PURGE`) and every `sllidar_node` start
  fails with `SL_RESULT_OPERATION_TIMEOUT`. Unplug the D435i and the lidar
  works; plug it back and the lidar breaks again. Root cause: Pi5 defaults to
  a conservative 600 mA per USB port — the D435i pulls ~700 mA on USB 3.0 and
  the combined draw destabilises the CP2102N's USB control endpoint. Fix: set
  `usb_max_current_enable=1` in `/boot/firmware/config.txt` (now in
  `setup_rpi.sh`). With the flag on, both devices coexist. The flag is harmless
  — it only raises the allowed ceiling; it doesn't force extra draw. Verified
  on a Pi5 with the official 27W PSU.
- **Dashboard-issued mode switches must go via the `/web/set_mode` service.**
  Never start a robot launch via SSH while the dashboard's launch_manager is
  in a non-idle mode — you'll end up with TWO `ros2 launch` processes each
  spawning their own `realsense2_camera_node`, which fight over the D435i's
  V4L2 device (`xioctl(VIDIOC_S_FMT) failed, errno=16 EBUSY`) and the launch
  respawn cycle makes `/rosout` unreadable. Either set dashboard to `idle`
  first, or use `ros2 service call /web/set_mode` to launch through the
  dashboard. Also: when SSH-ing to a non-interactive shell, `.bashrc` isn't
  sourced, so you MUST `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
  `export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml` explicitly before any
  `ros2 service call` or `rclpy` client — otherwise DDS discovery can't find
  local services and calls time out mysteriously.
- **mjpeg_grabber legacy node** in `roscar_cv` was bridging the Pi's
  mjpg-streamer (port 8081, no longer running) into `/image_raw`. After the
  D435i became the canonical source for `/image_raw` (via a remap in
  `depth_camera.launch.py`), mjpeg_grabber still ran from `cv.launch.py`,
  failed to connect, and became a SECOND publisher on `/image_raw` —
  confusing `web_video_server` enough that the AIO dashboard's CAMERA panel
  went offline. Removed from canonical `cv.launch.py`. If the WSL2 source
  tree diverges (you added nodes there that aren't in the canonical repo),
  they can re-appear; sync Windows → WSL2 after every canonical change.
- **`<img>.onerror` is NOT a reliable "MJPEG stream is dead" signal in
  Chrome.** Transient multipart-boundary glitches trip onerror repeatedly
  even on a healthy stream. `<img>.onload` fires exactly ONCE per
  `<img>.src` assignment (initial decode), not per-frame. So you can't
  build a "stream alive" watchdog from either event. The AIO camera panel
  previously flipped the OFFLINE overlay on every onerror → 3s retry
  reconnect → brief frame → onerror again → flicker. Fix: show OFFLINE
  until first onload, hide forever after, ignore onerror. If the stream
  genuinely dies the browser keeps showing the last frame, which is
  strictly better than a flickering black↔OFFLINE cycle. For a proper
  "stream is live" signal use a canvas + `drawImage` loop and sample for
  change, or poll `/snapshot` — both overkill for a status panel.
- **Don't tie the MJPEG camera to the rosbridge 'connected' event.** The
  browser's MJPEG `<img>` is plain HTTP from `web_video_server`. It has
  no dependency on rosbridge. If initCamera only calls startStream inside
  the rosbridge onConnect callback, a rosbridge hiccup leaves the panel
  stuck OFFLINE even though the stream is serving fine.
- **ES module TDZ on `let` in circular imports on Chrome** (Pi5 + Chrome
  139 observed). Pattern: `aio-app.js` imports `aio-teleop.js`, which
  imports back `aio-app.js`. A module-scope `let foo = null;` in
  aio-teleop.js threw "Cannot access 'foo' before initialization" at
  the first assignment inside `export function initTeleop()` — even
  though the let statement is lexically before the function and should
  have run first. Switching `let foo = null` → `var foo = null` hoists
  the binding to the module's start (as `undefined`) and dodges TDZ
  entirely. Semantics are the same for subsequent reads/writes. Worth
  reaching for if you hit an otherwise-unexplainable TDZ in a module
  with circular imports.
- **realsense2_camera D435i on Pi5: `ERROR [ds-options.cpp:93] Asic
  Temperature value is not valid!`** repeating once per second → depth
  topics (`/camera/camera/depth/image_rect_raw`, `/camera/camera/depth/
  color/points`) have publishers but never deliver a message. Color
  stream still works. Seems to coincide with USB instability or D435i
  firmware state getting wedged mid-session. Workaround: physical
  unplug/replug of the D435i USB cable (same family as the RPLIDAR
  `SL_RESULT_OPERATION_TIMEOUT` pattern). Software resets don't recover
  it. Verify with `sudo journalctl -u roscar-web --since '2 min ago' |
  grep -i asic`.
- **launch_manager `/web/set_mode idle` does NOT survive systemd restart
  in a useful way.** If you restart `roscar-web.service`, the child
  `ros2 launch` subprocess (the robot stack) is killed via cgroup by
  systemd, but the launch_manager starts fresh in `idle` mode instead
  of remembering the last mode. That's why the dashboard shows "IDLE"
  and the robot stack is gone after a service restart — not a bug in
  the stack, just how the service lifecycle works. Workaround: the
  dashboard's mode-sync poll will correct the UI, and the user can
  click SLAM again. A persistent-mode feature would need to write the
  last mode to disk.
- **ES modules + `?v=...` cache-bust is a TRAP if children import the
  bare module name.** Adding `js/aio-app.js?v=2026-04-23` on the
  `<script type="module">` tag and having child modules import
  `./aio-app.js` (no query) creates TWO distinct module URLs and
  Chrome evaluates the whole graph twice. Every `init*` at module
  top-level runs twice, every `addEventListener` attaches twice, every
  rosbridge subscription is doubled. User-visible: click handlers
  appear to do nothing because the second handler immediately undoes
  the first. Fix: don't put `?v=` on the script tag if children use
  bare imports. Rely on `Cache-Control: no-store` headers (which
  `http_server_node.py` now sends on every response) + Ctrl+Shift+R
  when a stale asset really needs bypassing. If you NEED a cache-bust,
  either (a) version-stamp EVERY import across every module, or
  (b) rename the files.
- **`<img>.onerror` is NOT a reliable "MJPEG stream is dead" signal in
  Chrome** (full rule — see the earlier AIO camera entry). Paired
  lesson: the watchdog-based design I tried first (check staleness by
  observing `onload` timestamps) also doesn't work because `onload`
  fires ONCE per `src` assignment on MJPEG streams, not per frame.
  Pragmatic policy is: hide OFFLINE overlay on first `onload`, never
  show it again until a mode/quality/resolution change tears down the
  stream. Dead streams show the last frame (which is fine — strictly
  better than a flickering OFFLINE/black cycle).
- **Motor wire color codes (keep forever)**: Purple=M1=Front Left,
  Orange=M2=Rear Left, Yellow=M3=Front Right, Green=M4=Rear Right.
  Same table is in CLAUDE.md under Hardware Orientation. Reach for this
  any time you're tracing a loose wire, re-terminating a connector, or
  wondering which port a motor should be plugged into — the mecanum
  kinematics on the STM32 assume this exact mapping and any swap
  produces wrong-direction wheels.
- **realsense2_camera silent depth-stream wedge on Pi5 when depth +
  color BOTH run at native resolutions.** Driver diagnostics show
  `camera: depth` → "Events since startup: 14, Actual frequency: 0,
  message: No events recorded" while `camera: color` stays at target
  — i.e. the sensor is producing depth frames internally but they're
  not reaching DDS subscribers. Isolated by disabling color: depth
  streams perfectly at 14.97 Hz alone. On this Pi5 + D435i combo the
  working compromise is depth at 640×480×15 + color at 424×240×15 +
  `enable_sync: false`. Depth then reaches subscribers at ~5 Hz (not
  the internal 15 Hz — downstream frame drops somewhere in the
  driver→DDS path) with pointcloud at ~9 Hz. Good enough for the
  Nav2 voxel_layer. Also `initial_reset: true` cleared an earlier
  "Asic Temperature value is not valid!" wedge and is a good default.
