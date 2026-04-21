"""Launch Intel RealSense D435i depth camera.

Publishes under the /camera/camera/* namespace. Frames are owned by the URDF
(realsense2_description xacro), not by this driver (publish_tf=false).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'realsense_params.yaml')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        realsense_node,
    ])
