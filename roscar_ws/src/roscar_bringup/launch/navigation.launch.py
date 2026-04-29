"""Navigation launch: robot bringup + Nav2 on a saved map.

Usage:
  ros2 launch roscar_bringup navigation.launch.py map:=/path/to/map.yaml

In rviz2:
  1. Set initial pose with "2D Pose Estimate" tool
  2. Send goals with "2D Goal Pose" tool
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # -- Launch arguments --
    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to the map yaml file (e.g. /home/user/maps/my_map.yaml)',
    )
    use_depth_arg = DeclareLaunchArgument(
        'use_depth', default_value='true',
        description='Launch the D435i depth camera (feeds voxel_layer in local costmap)',
    )

    # -- Full robot bringup (driver + URDF + lidar + IMU filter + EKF) --
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_lidar': 'true',    # Navigation requires lidar
            'use_depth': LaunchConfiguration('use_depth'),
        }.items(),
    )

    # -- Nav2 bringup (AMCL + Nav2 stack on saved map) --
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'slam': 'False',
            'autostart': 'true',
        }.items(),
    )

    # -- Landmark localizer (ArUco marker drift correction) --
    # Marker positions are stored in a per-map sidecar file
    # `<map_path>.markers.yaml` — bound to the specific map's coordinate
    # frame. Learning new markers in nav mode appends to that sidecar.
    # The sidecar is created/updated by the SaveMap service when SLAM
    # finishes. Missing sidecar = empty marker set (no error).
    driver_dir = get_package_share_directory('roscar_driver')

    def make_landmark_node(context, *args, **kwargs):
        map_path = LaunchConfiguration('map').perform(context)
        base, _ = os.path.splitext(map_path)
        sidecar = base + '.markers.yaml'
        return [Node(
            package='roscar_driver',
            executable='landmark_localizer',
            name='landmark_localizer',
            parameters=[
                os.path.join(driver_dir, 'config', 'landmark_params.yaml'),
                {
                    'load_learned': True,           # reuse markers learned with this map
                    'learned_markers_file': sidecar,  # per-map sidecar
                },
            ],
            output='screen',
        )]

    return LaunchDescription([
        map_arg,
        use_depth_arg,
        robot_launch,
        nav2_launch,
        OpaqueFunction(function=make_landmark_node),
    ])
