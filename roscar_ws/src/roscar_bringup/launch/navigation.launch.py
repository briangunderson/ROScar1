"""Navigation launch: robot bringup + Nav2 on a saved map.

Usage:
  ros2 launch roscar_bringup navigation.launch.py map:=/path/to/map.yaml

In rviz2:
  1. Set initial pose with "2D Pose Estimate" tool
  2. Send goals with "2D Goal Pose" tool
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # -- Launch arguments --
    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to the map yaml file (e.g. /home/user/maps/my_map.yaml)',
    )

    # -- Full robot bringup (driver + URDF + lidar + IMU filter + EKF) --
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_lidar': 'true',    # Navigation requires lidar
        }.items(),
    )

    # -- Nav2 bringup (AMCL + Nav2 stack on saved map) --
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'slam': 'False',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        robot_launch,
        nav2_launch,
    ])
