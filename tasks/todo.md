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

## Milestone 3: Sensor Fusion
- [ ] robot_localization EKF config
- [ ] IMU Madgwick filter config
- [ ] Tune parameters

## Milestone 4: SLAM + Navigation
- [ ] SLAM Toolbox config
- [ ] Nav2 config
- [ ] Navigation launch files
