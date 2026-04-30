"""Launch Intel RealSense D435i depth camera + depth-to-laserscan helper.

Publishes under the /camera/camera/* namespace. Frames are owned by the URDF
(realsense2_description xacro), not by this driver (publish_tf=false).

Topic remaps: the D435i's color stream is also remapped to the legacy /image_raw
and /camera_info topics, so existing consumers (dashboard MJPEG stream, ArUco
detector, YOLO) keep working with zero code changes. The D435i replaces the
Logitech webcam — there's one camera now, not two.

Depth → virtual laser scan: a pointcloud_to_laserscan node collapses the
depth point cloud into a 2D scan (`/scan_depth`) that Nav2's local costmap
obstacle_layer can consume alongside the real lidar. This catches obstacles
the lidar misses — table edges, chair seats, low boxes — at near-zero CPU
cost compared to a full voxel layer. See
docs/superpowers/specs/2026-04-30-d435i-obstacle-avoidance.md.
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

    # Depth point cloud → virtual 2D laser scan in base_link frame.
    # Height filter skips the floor (< 5 cm) and stuff above the robot body
    # (> 50 cm — the lidar already sees that range, and 50 cm is below the
    # mast/lidar mount). The result is a horizontal slice of the depth volume
    # that fills the gap between the floor and the lidar's scan plane —
    # exactly where tables, chair seats, low boxes, and short obstacles
    # live. Range is capped at 3 m (D435i's useful depth limit on this unit).
    depth_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='depth_to_scan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': 0.05,         # 5 cm above floor — skip floor noise
            'max_height': 0.50,         # up to top of robot body (below mast)
            'angle_min': -0.76,         # -43.5°, matches D435i depth FOV
            'angle_max':  0.76,         # +43.5°
            'angle_increment': 0.0087,  # ~0.5° per ray, ~175 rays
            'scan_time': 0.0667,        # 1/15 Hz = depth frame period
            'range_min': 0.20,          # D435i min usable depth
            'range_max': 3.0,           # D435i useful depth range
            'use_inf': False,           # invalid pixels become out-of-range
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', '/camera/camera/depth/color/points'),
            ('scan', '/scan_depth'),
        ],
        output='screen',
    )

    return LaunchDescription([
        realsense_node,
        depth_to_scan_node,
    ])
