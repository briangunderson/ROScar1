# ROScar1 - Task Tracker

## Milestone 1: Teleop Ready ✅
- [x] Create workspace structure
- [x] roscar_driver package (driver node, kinematics, config, launch)
- [x] roscar_description package (URDF, launch, rviz)
- [x] roscar_bringup package (launch files, teleop config, EKF config)
- [x] Setup scripts (RPi setup, udev rules)
- [x] CLAUDE.md project documentation
- [x] Deploy to RPi5 and test
- [x] Rosmaster_Lib v3.3.9 installed (pip from github.com/Roblibs/Rosmaster_Lib)
- [x] All 6 nodes running: driver, robot_state_publisher, joint_state_publisher, sllidar, imu_filter, ekf
- [x] Teleop driving confirmed working

## Milestone 2: Camera Integration ✅
- [x] v4l2_camera launch (fixed image_size param type — OpaqueFunction for int cast)
- [x] Camera link in URDF (camera_link + camera_optical_frame already in xacro)
- [x] Test image streaming (/image_raw at 30fps, /camera_info publishing)

## Milestone 3: Sensor Fusion ✅
- [x] Fix IMU sign conventions (negate all accel axes, negate only gx/gy for gyro)
- [x] Fix conflicting odom TFs (driver publish_odom_tf=false, EKF owns TF)
- [x] EKF config: velocity-only from odom, 20Hz, differential IMU yaw
- [x] IMU Madgwick filter: fixed_frame=odom, gain=0.1, orientation correct (flat=identity)
- [x] Verified: 0 NaN errors, position stable at origin when stationary, all topics 20Hz

## Milestone 4: SLAM + Navigation
- [x] slam_toolbox config (online_async, 5cm resolution, RPLIDAR C1 tuned)
- [x] Nav2 config (DWB holonomic controller, SmacPlanner2D, AMCL OmniMotionModel, RPi5-tuned)
- [x] slam.launch.py (SLAM-only mapping with teleop)
- [x] navigation.launch.py (saved map + AMCL + Nav2)
- [x] slam_nav.launch.py (SLAM + Nav2 combined for explore-and-navigate)
- [x] Package deps updated (slam_toolbox, navigation2, nav2_bringup)
- [x] setup_rpi.sh updated with new apt packages
- [x] Deploy to RPi5 and test SLAM mapping (lifecycle fix: nav2_lifecycle_manager + bond_timeout=0.0)
- [x] SLAM verified: /map publishing (74×135 cells @ 5cm), map->odom TF, all 8 nodes running
- [ ] Save first map and test Nav2 navigation
- [ ] Verify holonomic motion (strafing) during navigation

## Milestone 5: D435i Depth Camera Integration (in progress)
Plan: `docs/superpowers/plans/2026-04-21-d435i-depth-camera-integration.md`

- [x] USB 3.0 SuperSpeed connection verified (new cable required)
- [x] Install ros-jazzy-realsense2-{camera,description} + pointcloud_to_laserscan on Pi
- [x] URDF: rename camera_link→webcam_link, add sensor_d435i xacro from realsense2_description
- [x] Update dependent files: camera.launch.py, aruco_detector_node.py, landmark_localizer_node.py
- [x] realsense_params.yaml: 848×480@15 depth, 640×480@15 color, aligned pointcloud, IMU disabled
- [x] depth_camera.launch.py (standalone) + robot.launch.py use_depth:=true flag (default false)
- [x] Nav2 local costmap: voxel_layer with d435i_depth observation source
- [x] CLAUDE.md: Depth Camera section, topics table, critical files
- [ ] **Physical mount**: measure actual D435i mount pose, update URDF placeholders
  (d435i_x, d435i_y, d435i_z, d435i_pitch)
- [ ] **End-to-end test**: restart live SLAM stack with use_depth:=true, place physical
  obstacle, verify voxel_marked_cloud lights up and local costmap inflates
- [ ] **Dashboard depth stream toggle** (deferred): add DEPTH mode to aio-camera.js
  that routes to web_video_server's depth stream
- [ ] **D435i IMU fusion** (deferred, explicit non-goal): would require extrinsic
  calibration vs STM32 IMU position
- [x] **Logitech retirement**: remove webcam from URDF, default use_camera:=false,
  remap D435i color to /image_raw so dashboard+ArUco+YOLO work unchanged,
  bump color FPS 15→30, revert aruco/landmark frames back to
  camera_color_optical_frame

### Review (2026-04-21)
- **What landed**: full driver + URDF + launch + Nav2 voxel_layer on branch
  `feat/d435i-depth-camera`. Driver verified at 15Hz on USB 3.0, 28% CPU on one core.
- **Hardware gotcha found**: original USB-C cable was charging-only, forced USB 2.0
  fallback + MIPI errors. New cable fixed it. Documented in CLAUDE.md.
- **URDF gotcha found**: realsense2_description's `sensor_d435i` macro signature is
  `(parent, *origin, name, use_nominal_extrinsics)` — earlier `topics_ns/add_plug/use_mesh`
  params I expected don't exist in the jazzy version. Simplified instantiation works.
- **Param-name gotcha found**: realsense2_camera dynamically renames the pointcloud
  param namespace based on CPU SIMD features. On Pi5 aarch64 it's `pointcloud__neon_.*`
  not `pointcloud.*`. Documented in YAML comments and CLAUDE.md.
- **Not verified end-to-end**: voxel_layer integration not yet tested with a physical
  obstacle and Nav2 running — requires restarting the live stack with use_depth:=true.
