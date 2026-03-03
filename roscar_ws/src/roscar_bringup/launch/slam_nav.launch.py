"""Combined SLAM + Navigation launch: map and navigate simultaneously.

Usage:
  ros2 launch roscar_bringup slam_nav.launch.py

This launches the full robot, slam_toolbox for mapping, AND the Nav2 stack
so you can send navigation goals while building the map. Useful for
exploration or when you don't have a pre-built map.

In rviz2:
  - Watch the map build in real-time
  - Send goals with "2D Goal Pose" tool (no initial pose needed — SLAM provides localization)

Save the map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # -- Full robot bringup (driver + URDF + lidar + IMU filter + EKF) --
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_lidar': 'true',
        }.items(),
    )

    # -- Nav2 bringup with SLAM mode (slam_toolbox + Nav2 stack) --
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    slam_config = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'slam': 'True',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        robot_launch,
        nav2_launch,
    ])
