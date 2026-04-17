"""Launch the camera node.

Supports two backends:
  - 'csi' (default): RPi Camera Module v2 (IMX219) via camera_ros (libcamera)
  - 'usb': USB webcam via v4l2_camera (V4L2)

Set camera_type:=usb to fall back to the Logitech webcam.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_camera(context):
    camera_type = LaunchConfiguration('camera_type').perform(context)
    width = int(LaunchConfiguration('image_width').perform(context))
    height = int(LaunchConfiguration('image_height').perform(context))
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context)

    if camera_type == 'csi':
        # RPi Camera Module via camera_ros (libcamera + RPi PiSP)
        params = {
            'camera': 0,
            'width': width,
            'height': height,
            'format': 'RGB888',
            'frame_id': 'camera_optical_frame',
        }
        if camera_info_url:
            params['camera_info_url'] = camera_info_url

        return [
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera',
                namespace='',
                parameters=[params],
                remappings=[
                    ('~/image_raw', '/image_raw'),
                    ('~/camera_info', '/camera_info'),
                    ('~/image_raw/compressed', '/image_raw/compressed'),
                ],
                output='screen',
            ),
        ]
    else:
        # USB webcam via v4l2_camera
        video_device = LaunchConfiguration('video_device').perform(context)
        params = {
            'video_device': video_device,
            'image_size': [width, height],
            'io_method': 'read',
            'camera_frame_id': 'camera_optical_frame',
        }
        if camera_info_url:
            params['camera_info_url'] = camera_info_url

        return [
            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name='camera',
                parameters=[params],
                output='screen',
            ),
        ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    default_camera_info = 'file://' + os.path.join(
        bringup_dir, 'config', 'camera_calibration.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_type', default_value='csi',
            description='Camera backend: "csi" (RPi Camera Module) or "usb" (USB webcam)',
        ),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/webcam',
            description='Video device path (only used when camera_type:=usb)',
        ),
        DeclareLaunchArgument(
            'image_width', default_value='640',
            description='Image width in pixels',
        ),
        DeclareLaunchArgument(
            'image_height', default_value='480',
            description='Image height in pixels',
        ),
        DeclareLaunchArgument(
            'camera_info_url', default_value=default_camera_info,
            description='URL to camera calibration file (file:// path)',
        ),

        OpaqueFunction(function=_launch_camera),
    ])
