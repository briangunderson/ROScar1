#!/bin/bash
# ============================================================
# Install ROS2 Jazzy + visualization tools in WSL2 Ubuntu 24.04
# Run this inside your WSL terminal (not from Windows):
#   bash /mnt/d/localrepos/ROScar1/scripts/install_ros2_wsl.sh
# ============================================================
set -eo pipefail

echo "============================================"
echo " ROS2 Jazzy Install for ROScar1 Development"
echo "============================================"
echo ""

# --- Step 1: Add ROS2 apt repository (skip if already configured) ---
echo "[1/5] Adding ROS2 Jazzy repository..."
if ls /etc/apt/sources.list.d/*ros* >/dev/null 2>&1; then
    # Remove any duplicate .list file if DEB822 .sources already exists
    if [ -f /etc/apt/sources.list.d/ros2.sources ] && [ -f /etc/apt/sources.list.d/ros2.list ]; then
        echo "  Removing duplicate ros2.list (ros2.sources already exists)..."
        sudo rm /etc/apt/sources.list.d/ros2.list
    fi
    echo "  ROS2 repo already configured, skipping."
else
    sudo apt-get install -y -qq software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi
sudo apt-get update -qq
echo "  Done."

# --- Step 2: Install ROS2 desktop (rviz2, rqt, etc.) ---
echo ""
echo "[2/5] Installing ros-jazzy-desktop (~2GB, this takes a while)..."
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-desktop
echo "  Done."

# --- Step 3: Extra visualization packages + CycloneDDS ---
echo ""
echo "[3/5] Installing extra packages..."
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rqt-tf-tree \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-plot \
    2>/dev/null || echo "  (some optional packages not available, skipping)"

# PlotJuggler (may not be in apt, try anyway)
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-jazzy-plotjuggler-ros \
    2>/dev/null || echo "  PlotJuggler not in apt, install via snap or skip."
echo "  Done."

# --- Step 4: Configure shell ---
echo ""
echo "[4/5] Configuring shell..."
if ! grep -q '/opt/ros/jazzy/setup.bash' ~/.bashrc; then
    echo '' >> ~/.bashrc
    echo '# ROS2 Jazzy' >> ~/.bashrc
    echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
    echo "  Added 'source /opt/ros/jazzy/setup.bash' to ~/.bashrc"
else
    echo "  ~/.bashrc already sources ROS2."
fi

# --- Step 5: Configure DDS for network discovery ---
echo ""
echo "[5/5] Configuring CycloneDDS for cross-network discovery..."
CYCLONE_CFG="$HOME/cyclonedds.xml"
cat > "$CYCLONE_CFG" << 'XML'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <!-- Allow discovery across WSL2 <-> LAN (Pi) -->
      <AllowMulticast>true</AllowMulticast>
      <EnableMulticastLoopback>true</EnableMulticastLoopback>
    </General>
    <Discovery>
      <Peers>
        <!-- ROScar1 Pi5 - update IP if it changes -->
        <Peer address="192.168.1.170"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
XML

# Add CycloneDDS config to bashrc
if ! grep -q 'CYCLONEDDS_URI' ~/.bashrc; then
    echo '' >> ~/.bashrc
    echo '# CycloneDDS config for cross-network ROS2 discovery (WSL2 <-> Pi)' >> ~/.bashrc
    echo "export CYCLONEDDS_URI=file://$CYCLONE_CFG" >> ~/.bashrc
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
    echo "  Added CycloneDDS config to ~/.bashrc"
else
    echo "  CycloneDDS already configured in ~/.bashrc."
fi

echo ""
echo "============================================"
echo " INSTALL COMPLETE"
echo "============================================"
echo ""
echo "Next steps:"
echo "  1. Close and reopen your WSL terminal (or run: source ~/.bashrc)"
echo "  2. Test rviz2:  rviz2"
echo "  3. Test with ROScar1:  rviz2 -d /mnt/d/localrepos/ROScar1/roscar_ws/src/roscar_description/rviz/roscar.rviz"
echo "  4. Check topics from Pi:  ros2 topic list"
echo ""
echo "If ros2 topic list doesn't show Pi topics, you may need to also"
echo "configure CycloneDDS on the Pi (same peer discovery approach)."
echo ""
