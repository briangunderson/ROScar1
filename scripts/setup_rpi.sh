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
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-web-video-server \
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

# ---- 5b. Enable full USB power budget ----
# Pi5 defaults to a conservative 600 mA per USB port. With the D435i depth
# camera AND the RPLIDAR C1 both attached, the CP2102N bridge chip in the
# RPLIDAR times out on control requests (status: -110 / ETIMEDOUT) when
# running at the default budget. Enabling usb_max_current_enable=1 lifts
# the limit to the PSU's full capacity (~1.6 A with the official Pi PSU)
# and lets both peripherals coexist.
#
# Empirically: without this + with both devices attached, RPLIDAR throws
# SL_RESULT_OPERATION_TIMEOUT on every sllidar_node start. With this set,
# both work simultaneously on a Pi5 official PSU.
CONFIG_TXT=/boot/firmware/config.txt
if ! grep -qE "^usb_max_current_enable=" "$CONFIG_TXT"; then
    echo "[5b] Adding usb_max_current_enable=1 to $CONFIG_TXT..."
    sed -i '/^\[all\]/a usb_max_current_enable=1' "$CONFIG_TXT"
    echo "  -> USB full-power budget enabled (takes effect on next reboot)"
else
    echo "[5b] usb_max_current_enable already set in $CONFIG_TXT"
fi

# ---- 6. Clone sllidar_ros2 (RPLIDAR C1 driver) ----
echo "[6/8] Cloning sllidar_ros2 driver..."
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
echo "[7/8] Installing Rosmaster_Lib..."
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

# ---- 8. Install systemd services ----
echo "[8/8] Installing systemd services..."

# Web dashboard service
cp "${SCRIPT_DIR}/roscar-web.service" /etc/systemd/system/
chmod +x "${SCRIPT_DIR}/roscar-web.sh"
echo "  -> Installed roscar-web.service"

# Recovery service (standalone admin page on port 9999)
cp "${SCRIPT_DIR}/roscar-recovery.service" /etc/systemd/system/
chmod +x "${SCRIPT_DIR}/roscar-recovery.py"
cp "${SCRIPT_DIR}/roscar-recovery-sudoers" /etc/sudoers.d/roscar-recovery
chmod 440 /etc/sudoers.d/roscar-recovery
visudo -c  # validate sudoers syntax
echo "  -> Installed roscar-recovery.service + sudoers"

systemctl daemon-reload
systemctl enable roscar-web roscar-recovery
echo "  -> Services enabled (will start on next boot)"
echo "  -> To start now: systemctl start roscar-web roscar-recovery"

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
