"""Remote slam_toolbox launch — runs on WSL2 GPU PC, not the Pi.

This is the WSL2-side counterpart of T2a (offload SLAM from Pi5).

Pre-flight (2026-05-03): Pi↔WSL2 ROS2 round-trip latency p99 = 24 ms,
max = 29 ms — well within `transform_tolerance: 1.0` on the Pi's
nav2 controller.

Topology when active:
    Pi (sllidar) ──/scan─────────────────────────┐
    Pi (ekf)     ──/odometry/filtered────────────┤
    Pi (rsp)     ──/tf_static, /tf───────────────┤
                                                  v
                                       WSL2: slam_toolbox  (THIS launch)
                                                  │
    Pi  ◄──/map  ──────/tf (map→odom)─────────────┘

Usage:
    # On Pi (in slam mode), opt in:
    ros2 launch roscar_bringup slam.launch.py use_remote_slam:=true

    # On WSL2, in a terminal with ROS2 + CycloneDDS env sourced:
    ros2 launch roscar_cv slam_remote.launch.py

The Pi-side `lifecycle_manager_slam` discovers the slam_toolbox node
on WSL2 via DDS and brings it up — no remote-lifecycle plumbing
needed. When the Pi exits slam mode, the lifecycle_manager_slam tears
the remote slam_toolbox back down to unconfigured.

NOTE: slam_toolbox config is shared with the Pi-local launch — same
slam_toolbox.yaml from roscar_bringup (anti-drift). To pick up
config changes on WSL2 after a master pull, copy the file:
    wsl -d Ubuntu-24.04 -- bash -lc '
      cp /mnt/d/localrepos/ROScar1/roscar_ws/src/roscar_bringup/config/slam_toolbox.yaml \
         ~/roscar_ws/src/roscar_cv/config/'
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('roscar_cv')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config],
            output='screen',
            # No lifecycle manager here — the Pi's lifecycle_manager_slam
            # finds this node via DDS and drives the configure+activate
            # transition. Keeps the Pi as the single point of "is the
            # robot in slam mode" truth.
        ),
    ])
