#!/usr/bin/env bash
#
# Launcher for the ROScar1 CV stack (ArUco + YOLO + web_video_server on 8081).
# Runs on the dev workstation (WSL2) via the roscar-cv.service systemd unit,
# NOT on the Pi. CV needs the GPU, so it lives off-board.
#
set -eo pipefail

# Wait for a routable network interface before starting ROS2 nodes.
# In WSL2, networking is mirrored from Windows but can take a few seconds
# to settle after the distro boots.
MAX_WAIT=30
waited=0
while ! ip -4 route show default &>/dev/null; do
    if [ "$waited" -ge "$MAX_WAIT" ]; then
        echo "ERROR: No default route after ${MAX_WAIT}s — starting anyway" >&2
        break
    fi
    echo "Waiting for network (${waited}s)..." >&2
    sleep 1
    waited=$((waited + 1))
done

source /opt/ros/jazzy/setup.bash
source "$HOME/roscar_ws/install/setup.bash"

# CycloneDDS unicast-peer discovery to reach the Pi.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# Probe DDS before launching — same race we hit on the Pi.
MAX_DDS_WAIT=30
dds_waited=0
while ! python3 -c "import rclpy; rclpy.init(); n = rclpy.create_node('_dds_probe'); n.destroy_node(); rclpy.shutdown()" 2>/dev/null; do
    if [ "$dds_waited" -ge "$MAX_DDS_WAIT" ]; then
        echo "WARNING: DDS probe still failing after ${MAX_DDS_WAIT}s — launching anyway" >&2
        break
    fi
    echo "Waiting for DDS (${dds_waited}s)..." >&2
    sleep 1
    dds_waited=$((dds_waited + 1))
done

exec ros2 launch roscar_cv cv.launch.py
