"""Launch Intel RealSense D435i depth camera.

Publishes under the /camera/camera/* namespace. Frames are owned by the URDF
(realsense2_description xacro), not by this driver (publish_tf=false).

Topic remaps: the D435i's color stream is also remapped to the legacy /image_raw
and /camera_info topics, so existing consumers (dashboard MJPEG stream, ArUco
detector, YOLO) keep working with zero code changes. The D435i replaces the
Logitech webcam — there's one camera now, not two.
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
        remappings=[
            # Expose D435i color as the canonical /image_raw + /camera_info
            # so the dashboard (web_video_server), ArUco detector, and YOLO
            # keep working with no config changes.
            ('/camera/camera/color/image_raw', '/image_raw'),
            ('/camera/camera/color/image_raw/compressed', '/image_raw/compressed'),
            ('/camera/camera/color/camera_info', '/camera_info'),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        realsense_node,
    ])
