"""Launch the ROScar1 driver node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('roscar_driver')
    default_params = os.path.join(pkg_dir, 'config', 'driver_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to driver parameters YAML file',
        ),

        Node(
            package='roscar_driver',
            executable='driver_node',
            name='roscar_driver',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True,
        ),
    ])
