"""Teleop launch: starts the full robot + keyboard teleop.

Usage:
  ros2 launch roscar_bringup teleop.launch.py

Then use the keyboard controls printed by teleop_twist_keyboard to drive.
For mecanum robots, use 'u/i/o/j/k/l/m/,/.' for omni-directional movement.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')

    # -- Full robot bringup --
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
    )

    # -- Keyboard teleop --
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',  # opens in new terminal
        parameters=[{
            'speed': 0.3,          # linear speed (m/s)
            'turn': 1.0,           # angular speed (rad/s)
            'stamped': False,
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_launch,
        teleop_node,
    ])
