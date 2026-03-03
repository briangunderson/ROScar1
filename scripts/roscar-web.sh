#!/usr/bin/env bash
#
# Launcher for the ROScar1 web stack (rosbridge + web_video_server +
# launch_manager + http_server).  Called by the roscar-web.service
# systemd unit on boot.
#
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/roscar_ws/install/setup.bash"

exec ros2 launch roscar_web web.launch.py
