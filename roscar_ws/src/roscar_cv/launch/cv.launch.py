"""Launch ArUco and YOLO CV nodes for ROScar1."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('roscar_cv')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('use_aruco', default_value='true',
                              description='Launch ArUco detector'),
        DeclareLaunchArgument('use_yolo', default_value='true',
                              description='Launch YOLO detector'),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt',
                              description='YOLO model file (n/s/m/l/x)'),

        # ── ArUco Detector ──
        Node(
            package='roscar_cv',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[
                os.path.join(pkg_share, 'config', 'aruco_params.yaml'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_aruco')),
        ),

        # ── YOLO Detector ──
        Node(
            package='roscar_cv',
            executable='yolo_detector',
            name='yolo_detector',
            parameters=[
                os.path.join(pkg_share, 'config', 'yolo_params.yaml'),
                {'model': LaunchConfiguration('yolo_model')},
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_yolo')),
        ),
    ])
