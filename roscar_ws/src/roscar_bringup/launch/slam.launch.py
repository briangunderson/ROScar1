"""SLAM mapping launch: robot bringup + slam_toolbox (online_async).

Usage:
  ros2 launch roscar_bringup slam.launch.py

Drive around with teleop in a second terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save the map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    driver_dir = get_package_share_directory('roscar_driver')

    # -- Forwarded launch args --
    use_depth_arg = DeclareLaunchArgument(
        'use_depth', default_value='true',
        description='Launch the D435i depth camera alongside SLAM',
    )
    # T2a — opt-in: skip Pi-local slam_toolbox and rely on a remote
    # instance running on the WSL2 GPU PC. Pre-flight measurement
    # (2026-05-03) showed Pi↔WSL2 ROS2 round-trip latency p99 = 24 ms,
    # well within transform_tolerance: 1.0. The lifecycle manager on
    # the Pi finds the remote /slam_toolbox node via DDS and manages
    # it just like a local instance — no other launch changes needed.
    # See docs/superpowers/specs/2026-05-01-cpu-optimization-proposal.md
    # and tasks/t2a-poc-2026-05-03.md.
    use_remote_slam_arg = DeclareLaunchArgument(
        'use_remote_slam', default_value='false',
        description='If true, do NOT spawn slam_toolbox locally on the Pi. '
                    'Expects an instance to be running on the WSL2 GPU PC '
                    '(via roscar-slam.service or roscar_slam_remote.launch.py). '
                    'Pi load avg drops by ~5-10 % when offloaded.',
    )

    # -- Full robot bringup (driver + URDF + lidar + IMU filter + EKF) --
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_lidar': 'true',    # SLAM requires lidar
            'use_depth': LaunchConfiguration('use_depth'),
        }.items(),
    )

    # -- slam_toolbox (online async) --
    # NOTE: async_slam_toolbox_node is a lifecycle node in Jazzy.
    # It starts in "unconfigured" state and must be transitioned to
    # "active" before it processes scans. The lifecycle_manager below
    # handles this automatically via the configure -> activate sequence.
    #
    # T2a: when use_remote_slam:=true, we skip spawning the Pi-local
    # node. The lifecycle_manager (still local) discovers the remote
    # /slam_toolbox node via DDS and brings it up over the network —
    # the lifecycle services don't care which machine the node lives on.
    slam_config = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_remote_slam')),
    )

    # -- Lifecycle manager to auto-activate slam_toolbox --
    # bond_timeout=0.0 disables bond heartbeat monitoring, which
    # slam_toolbox does not implement (would cause a spurious error).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
    )

    # NOTE: Landmark localizer (ArUco pose correction) is NOT included here.
    # slam_toolbox's map optimization (loop closures, scan pose adjustments)
    # shifts the map frame, invalidating learned marker positions and causing
    # a feedback loop. Landmark correction only works on a FIXED map
    # (navigation.launch.py with load_learned: true).

    return LaunchDescription([
        use_depth_arg,
        use_remote_slam_arg,
        robot_launch,
        slam_node,
        lifecycle_manager,
    ])
