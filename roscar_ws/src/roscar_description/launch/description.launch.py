"""Launch robot_state_publisher with the ROScar1 URDF."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('roscar_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'roscar.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # 2026-05-01: joint_state_publisher REMOVED. It was burning ~10 %
        # Pi5 CPU publishing wheel-rotation joint angles that no current
        # consumer uses (Nav2 has hardcoded footprint; lidar/camera TFs
        # are static URDF transforms not joint-driven; the dashboard map
        # tile draws no wheel rotation). The wheels' visual orientation
        # in RViz is now static — pure cosmetic loss. Revisit if we ever
        # add visualization that genuinely needs animated wheels.
        # See docs/superpowers/specs/2026-05-01-cpu-optimization-proposal.md.
    ])
