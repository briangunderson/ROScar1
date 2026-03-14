"""Launch the Logitech webcam via v4l2_camera with calibration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_camera(context):
    """Build the camera node with properly-typed parameters.

    v4l2_camera's 'image_size' parameter requires an integer array, but
    LaunchConfiguration substitutions resolve to strings.  OpaqueFunction
    lets us call .perform(context) to get the string values and cast them
    to int before passing to the node.
    """
    video_device = LaunchConfiguration('video_device').perform(context)
    width = int(LaunchConfiguration('image_width').perform(context))
    height = int(LaunchConfiguration('image_height').perform(context))
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context)

    params = {
        'video_device': video_device,
        'image_size': [width, height],
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
    # Default calibration file path
    bringup_dir = get_package_share_directory('roscar_bringup')
    default_camera_info = 'file://' + os.path.join(
        bringup_dir, 'config', 'camera_calibration.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/webcam',
            description='Video device path for the webcam (udev symlink)',
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
