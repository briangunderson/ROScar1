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

## udev
- CH340 (1a86:7523) → /dev/roscar_board (motor board)
- CP210x (10c4:ea60) → /dev/rplidar (RPLIDAR C1)
- Both symlinks confirmed working on first boot after setup_rpi.sh
