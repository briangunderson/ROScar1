#!/usr/bin/env bash
#
# Launcher for the ROScar1 web stack (rosbridge + web_video_server +
# launch_manager + http_server).  Called by the roscar-web.service
# systemd unit on boot.
#
set -eo pipefail

# Wait for a routable network interface before starting ROS2 nodes.
# network-online.target can fire before an IP is actually assigned,
# causing CycloneDDS to fail to bind and all nodes to crash.
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

# Use CycloneDDS for cross-machine discovery (Pi <-> dev workstation)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

exec ros2 launch roscar_web web.launch.py
