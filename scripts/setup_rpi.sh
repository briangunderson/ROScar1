#!/usr/bin/env bash
#
# ROScar1 Raspberry Pi 5 Setup Script
#
# Prerequisites: Ubuntu 24.04 Server installed on the RPi5
# Run as: sudo bash setup_rpi.sh
#
set -euo pipefail

echo "========================================="
echo " ROScar1 - Raspberry Pi 5 Setup"
echo "========================================="

# ---- 1. System updates ----
echo "[1/6] Updating system packages..."
apt update && apt upgrade -y

# ---- 2. Install ROS2 Jazzy ----
echo "[2/6] Installing ROS2 Jazzy Jalisco..."

# Add ROS2 apt repository
apt install -y software-properties-common curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update

# Install ROS2 base (no GUI) + dev tools
apt install -y \
    ros-jazzy-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep

# ---- 3. Install ROS2 packages ----
echo "[3/6] Installing ROS2 dependencies..."
apt install -y \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-filter-madgwick \
    ros-jazzy-v4l2-camera \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    python3-serial

# ---- 4. Initialize rosdep ----
echo "[4/6] Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
sudo -u "${SUDO_USER:-$USER}" rosdep update

# ---- 5. Install udev rules ----
echo "[5/6] Installing udev rules..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cp "${SCRIPT_DIR}/udev/99-roscar.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
echo "  -> YB-ERF01-V3.0 will be available at /dev/roscar_board"
echo "  -> RPLIDAR C1 will be available at /dev/rplidar"

# ---- 6. Clone sllidar_ros2 (RPLIDAR C1 driver) ----
echo "[6/7] Cloning sllidar_ros2 driver..."
WORKSPACE_SRC="/home/${SUDO_USER:-$USER}/roscar_ws/src"
if [ ! -d "${WORKSPACE_SRC}/sllidar_ros2" ]; then
    sudo -u "${SUDO_USER:-$USER}" git clone \
        https://github.com/Slamtec/sllidar_ros2.git \
        "${WORKSPACE_SRC}/sllidar_ros2"
    echo "  -> sllidar_ros2 cloned to ${WORKSPACE_SRC}/sllidar_ros2"
else
    echo "  -> sllidar_ros2 already exists, skipping"
fi

# ---- 7. Install Rosmaster_Lib ----
echo "[7/7] Installing Rosmaster_Lib..."
# The Rosmaster_Lib Python package is distributed by Yahboom.
# Download from: https://github.com/YahboomTechnology/ROS-robot-expansion-board
# Or from the Yahboom product page downloads section.
#
# If you have the py_install zip:
#   cd ~/py_install_V3.3.1/Rosmaster_Lib
#   sudo python3 setup.py install
#
echo "  -> NOTE: You must install Rosmaster_Lib manually."
echo "     Download from Yahboom, then run:"
echo "       cd <path-to>/Rosmaster_Lib"
echo "       sudo python3 setup.py install"

# ---- Shell setup ----
BASHRC="/home/${SUDO_USER:-$USER}/.bashrc"
if ! grep -q "ros/jazzy" "$BASHRC" 2>/dev/null; then
    echo "" >> "$BASHRC"
    echo "# ROS2 Jazzy" >> "$BASHRC"
    echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
    echo "# Source ROScar1 workspace (after building)" >> "$BASHRC"
    echo "# source ~/roscar_ws/install/setup.bash" >> "$BASHRC"
fi

echo ""
echo "========================================="
echo " Setup complete!"
echo ""
echo " Next steps:"
echo "   1. Install Rosmaster_Lib (see note above)"
echo "   2. Clone this repo: git clone <repo-url> ~/roscar_ws"
echo "      (or symlink roscar_ws/src to your workspace)"
echo "   3. Build: cd ~/roscar_ws && colcon build"
echo "   4. Source: source install/setup.bash"
echo "   5. Launch: ros2 launch roscar_bringup teleop.launch.py"
echo "========================================="
