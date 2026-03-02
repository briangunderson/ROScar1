"""Launch the Logitech webcam via v4l2_camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='Video device path for the webcam',
        ),
        DeclareLaunchArgument(
            'image_width', default_value='640',
            description='Image width in pixels',
        ),
        DeclareLaunchArgument(
            'image_height', default_value='480',
            description='Image height in pixels',
        ),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_size': [
                    LaunchConfiguration('image_width'),
                    LaunchConfiguration('image_height'),
                ],
                'camera_frame_id': 'camera_optical_frame',
            }],
            output='screen',
        ),
    ])
