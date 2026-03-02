# ROScar1 - 4WD Mecanum Robot

## Project Overview
A ROS2 Jazzy workspace for a 4WD Mecanum wheel robot built on:
- **Raspberry Pi 5** (Ubuntu 24.04 Server) running ROS2
- **YB-ERF01-V3.0** (Yahboom STM32F103RCT6 motor driver board) connected via USB serial
- **Logitech webcam** for vision
- **RPLIDAR C1** (Slamtec) for 2D laser scanning, connected via USB
- **4x DC encoder motors** with mecanum wheels

**Status**: Milestone 1 (teleop-ready skeleton) complete. Untested on hardware.

## Architecture
The STM32 board handles PID motor control and mecanum inverse kinematics internally.
Our ROS2 layer is a thin bridge:

```
cmd_vel -> roscar_driver -> Rosmaster_Lib -> USB Serial -> STM32 -> Motors
                          <- encoder/IMU data <- STM32 <- Sensors
         -> odom_raw, imu/data_raw, battery_voltage
```

Full pipeline with EKF:
```
teleop_twist_keyboard -> /cmd_vel -> roscar_driver -> /odom_raw, /imu/data_raw
                                                       |             |
                                                       v             v
                                                   ekf_node    imu_filter_madgwick
                                                       |             |
                                                       +------+------+
                                                              v
                                                      /odometry/filtered
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
| `scripts/setup_rpi.sh` | RPi5 initial setup script |

## Packages

| Package | Type | Description |
|---------|------|-------------|
| `roscar_driver` | ament_python | Hardware bridge: Rosmaster_Lib <-> ROS2 topics |
| `roscar_description` | ament_cmake | URDF model, rviz config |
| `roscar_bringup` | ament_cmake | Launch files, EKF/IMU filter configs |

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

## TODO
- [ ] Measure actual robot dimensions and update URDF
- [ ] Install Rosmaster_Lib on RPi5 and test serial connection
- [ ] Test teleop and tune speed limits
- [ ] Tune EKF covariance parameters
- [ ] Add SLAM (slam_toolbox) and Nav2 configuration
- [ ] Verify udev rules match actual board USB IDs
