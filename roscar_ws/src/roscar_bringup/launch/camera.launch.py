"""Launch the Logitech webcam via v4l2_camera."""

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

    return [
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': video_device,
                'image_size': [width, height],
                'camera_frame_id': 'camera_optical_frame',
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
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

        OpaqueFunction(function=_launch_camera),
    ])
