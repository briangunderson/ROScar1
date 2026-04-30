"""Launch Intel RealSense D435i depth camera + depth-to-laserscan helper.

Publishes under the /camera/camera/* namespace. Frames are owned by the URDF
(realsense2_description xacro), not by this driver (publish_tf=false).

Topic remaps: the D435i's color stream is also remapped to the legacy /image_raw
and /camera_info topics, so existing consumers (dashboard MJPEG stream, ArUco
detector, YOLO) keep working with zero code changes. The D435i replaces the
Logitech webcam — there's one camera now, not two.

Depth → virtual laser scan: a depthimage_to_laserscan node back-projects
the raw depth image to a 2D scan (`/scan_depth`) that Nav2's local costmap
obstacle_layer can consume alongside the real lidar. This catches obstacles
the lidar misses — table edges, chair seats, low boxes — at near-zero CPU
cost compared to a full voxel layer. See
docs/superpowers/specs/2026-04-30-d435i-obstacle-avoidance.md.

Why depthimage_to_laserscan and not pointcloud_to_laserscan: verified on
hardware 2026-04-30 that pointcloud_to_laserscan_node, when launched via
launch_ros.actions.Node alongside the realsense camera, silently fails to
publish — its message_filters::TfFilter loses every frame. The same binary
works when run via `ros2 run` against the same topics. depthimage_to_laserscan
takes the depth image directly + camera_info (no message_filters TF chain)
and works reliably under launch_ros.
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

    # depthimage_to_laserscan back-projects a horizontal slice of the depth
    # image into a 2D LaserScan in base_link.
    #
    # scan_height = 120 — number of horizontal pixel rows centered on the
    # depth image's vertical center to collapse into the scan. The 640×480
    # depth image has a 58° vertical FOV, so 120 rows ≈ 14.5° vertical
    # slice. With the camera at base_link z = 0.163 m looking horizontal,
    # that 14.5° slice spans z ≈ [0.04 m, 0.29 m] above the floor at 1 m
    # forward, expanding to [-0.10 m, 0.43 m] at 2 m. That fills the gap
    # below the lidar plane (which is at z ≈ 0.295 m above floor).
    #
    # range_min/max match the D435i's useful depth window: < 0.2 m is
    # below sensor minimum, > 3 m is too noisy.
    #
    # output_frame: the published scan's header.frame_id. base_link puts
    # it in the same frame the costmap obstacle_layer expects.
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        parameters=[{
            'output_frame': 'base_link',
            'scan_height': 120,
            'range_min': 0.2,
            'range_max': 3.0,
            'scan_time': 0.0667,
        }],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan_depth'),
        ],
        output='screen',
    )

    # Cliff / dropoff detector — uses the same depth image stream to
    # emit virtual obstacle points wherever the floor SHOULD be visible
    # but isn't. Feeds a 3rd source on the local_costmap obstacle_layer
    # (PointCloud2 instead of LaserScan, since cliffs project as a fan
    # of points across the floor in front of the robot).
    # See roscar_driver/roscar_driver/cliff_detector_node.py for the
    # algorithm.
    cliff_detector_node = Node(
        package='roscar_driver',
        executable='cliff_detector',
        name='cliff_detector',
        parameters=[{
            'output_frame': 'base_link',
            'camera_link_frame': 'camera_link',
            'floor_tolerance_m': 0.05,
            'min_cliff_drop_m': 0.15,    # require ≥15cm drop before firing
            'max_detect_distance_m': 1.2, # 1.2m forward — closer = more reliable
            'min_valid_depth_m': 0.40,
            'col_stride': 8,             # wider stride → fewer points → less spam
            'row_stride': 8,
            # Invalid depth (NaN/0) is a noisy cliff signal — far walls
            # beyond 3 m, dark surfaces, glass, and matte black all read
            # as invalid. Default off; flip on only for environments
            # with real cliffs that need to be conservatively avoided
            # (e.g., a real stairway).
            'treat_invalid_as_cliff': False,
            'min_consecutive_cliff': 5,  # require 5 stacked cliff pixels
        }],
        output='screen',
    )

    return LaunchDescription([
        realsense_node,
        depth_to_scan_node,
        cliff_detector_node,
    ])
