"""SLAM mapping launch: robot bringup + slam_toolbox (online_async).

Usage:
  ros2 launch roscar_bringup slam.launch.py

Drive around with teleop in a second terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save the map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')

    # -- Full robot bringup (driver + URDF + lidar + IMU filter + EKF) --
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_lidar': 'true',    # SLAM requires lidar
        }.items(),
    )

    # -- slam_toolbox (online async) --
    # NOTE: async_slam_toolbox_node is a lifecycle node in Jazzy.
    # It starts in "unconfigured" state and must be transitioned to
    # "active" before it processes scans. The lifecycle_manager below
    # handles this automatically via the configure -> activate sequence.
    slam_config = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
    )

    # -- Lifecycle manager to auto-activate slam_toolbox --
    # bond_timeout=0.0 disables bond heartbeat monitoring, which
    # slam_toolbox does not implement (would cause a spurious error).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([
        robot_launch,
        slam_node,
        lifecycle_manager,
    ])
