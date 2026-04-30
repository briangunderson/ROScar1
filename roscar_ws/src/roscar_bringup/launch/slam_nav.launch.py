"""Combined SLAM + Navigation launch: map and navigate simultaneously.

Usage:
  ros2 launch roscar_bringup slam_nav.launch.py

This launches the full robot, slam_toolbox for mapping, AND the Nav2 stack
so you can send navigation goals while building the map. Useful for
exploration or when you don't have a pre-built map.

In rviz2:
  - Watch the map build in real-time
  - Send goals with "2D Goal Pose" tool (no initial pose needed — SLAM provides localization)

Save the map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # -- Launch arguments --
    # use_depth defaults to true so the D435i's depth → /scan_depth pipeline
    # contributes to the local costmap obstacle_layer alongside the lidar.
    # Set to false to navigate on lidar alone.
    use_depth_arg = DeclareLaunchArgument(
        'use_depth', default_value='true',
        description='Launch the Intel RealSense D435i depth camera + '
                    'depth-to-laserscan helper (publishes /scan_depth as a '
                    'second source for the local costmap obstacle_layer).',
    )

    # -- Full robot bringup (driver + URDF + lidar + IMU filter + EKF) --
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_lidar': 'true',
            'use_depth': LaunchConfiguration('use_depth'),
        }.items(),
    )

    # -- Nav2 bringup with SLAM mode (slam_toolbox + Nav2 stack) --
    # NOTE: slam_toolbox params are in nav2_params.yaml (under slam_toolbox:
    # namespace), not in the separate slam_toolbox.yaml. nav2_bringup's
    # slam_launch.py reads them from the main params_file.
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'slam': 'True',
            'autostart': 'true',
        }.items(),
    )

    # NOTE: Landmark localizer (ArUco pose correction) is NOT included here.
    # See slam.launch.py for the full rationale: slam_toolbox's map
    # optimization (loop closures, scan pose adjustments) shifts the map
    # frame, invalidating learned marker positions and creating a feedback
    # loop with the localizer's /set_pose corrections. Landmark correction
    # only works on a FIXED map (navigation.launch.py with load_learned: true).

    return LaunchDescription([
        use_depth_arg,
        robot_launch,
        nav2_launch,
    ])
