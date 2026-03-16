"""Web dashboard launch: rosbridge + web_video_server + launch_manager + http_server.

This launch file starts ONLY the web infrastructure. Robot nodes are
started/stopped separately via the launch_manager service from the dashboard.

Ports:
  9090 - rosbridge_server  (WebSocket, ROS2 bridge)
  8080 - web_video_server  (HTTP, MJPEG camera streams)
  8888 - http_server       (HTTP, dashboard static files)

Usage:
  ros2 launch roscar_web web.launch.py

Then open: http://<robot-ip>:8888/
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    web_config = os.path.join(
        get_package_share_directory('roscar_web'), 'config', 'web_params.yaml')

    # rosbridge_server: WebSocket bridge to all ROS2 topics/services/actions
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[web_config],
        output='screen',
    )

    # rosapi: provides /rosapi/* services (list nodes, topics, params, etc.)
    # Required by roslibjs ros.getNodes() used in the STATUS tab
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
    )

    # web_video_server: serves MJPEG streams from ROS2 Image topics over HTTP
    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[web_config],
        output='screen',
    )

    # launch_manager: ROS2 node that starts/stops robot launch modes
    launch_manager = Node(
        package='roscar_web',
        executable='launch_manager',
        name='launch_manager',
        output='screen',
    )

    # http_server: serves the web dashboard static files
    http_server = Node(
        package='roscar_web',
        executable='http_server',
        name='http_server',
        parameters=[web_config],
        output='screen',
    )

    # tf2_web_republisher: required by roslibjs ROSLIB.TFClient to look up
    # TF transforms (e.g., map->base_footprint for robot pose on map).
    # Without this, the dashboard falls back to /odometry/filtered (odom frame)
    # which diverges from the map frame during SLAM.
    tf2_web = Node(
        package='tf2_web_republisher',
        executable='tf2_web_republisher_node',
        name='tf2_web_republisher',
        output='screen',
    )

    return LaunchDescription([
        rosbridge,
        rosapi,
        web_video,
        launch_manager,
        http_server,
        tf2_web,
    ])
