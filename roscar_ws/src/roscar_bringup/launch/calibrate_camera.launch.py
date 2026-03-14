"""Launch camera calibration using a checkerboard pattern.

Starts the camera node and the cameracalibrator GUI. Hold a printed
checkerboard in front of the camera at various angles and distances
until the GUI shows enough samples, then click Calibrate -> Save.

The calibration file is saved to ~/.ros/camera_info/camera.yaml by
default (matching the camera node name). v4l2_camera picks it up
automatically on next launch.

Usage:
    ros2 launch roscar_bringup calibrate_camera.launch.py

    # Custom checkerboard (columns x rows of INNER corners):
    ros2 launch roscar_bringup calibrate_camera.launch.py \
        checkerboard_size:=8x6 square_size:=0.025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_calibration(context):
    video_device = LaunchConfiguration('video_device').perform(context)
    width = int(LaunchConfiguration('image_width').perform(context))
    height = int(LaunchConfiguration('image_height').perform(context))
    board = LaunchConfiguration('checkerboard_size').perform(context)
    square = LaunchConfiguration('square_size').perform(context)

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': video_device,
            'image_size': [width, height],
            'camera_frame_id': 'camera_optical_frame',
        }],
        output='screen',
    )

    calibrator_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='cameracalibrator',
        arguments=[
            '--size', board,
            '--square', square,
            '--ros-args',
            '--remap', 'image:=/image_raw',
            '--remap', 'camera:=/camera',
        ],
        output='screen',
    )

    return [camera_node, calibrator_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/webcam',
            description='Video device path',
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
            'checkerboard_size', default_value='9x6',
            description='Inner corners of checkerboard (columns x rows)',
        ),
        DeclareLaunchArgument(
            'square_size', default_value='0.025',
            description='Checkerboard square size in meters',
        ),

        OpaqueFunction(function=_launch_calibration),
    ])
