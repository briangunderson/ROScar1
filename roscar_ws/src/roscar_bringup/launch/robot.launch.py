"""Full robot bringup: URDF + driver + lidar + IMU filter + EKF."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    driver_dir = get_package_share_directory('roscar_driver')
    description_dir = get_package_share_directory('roscar_description')

    # -- Launch arguments --
    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf', default_value='true',
        description='Publish odom->base_footprint TF from driver (disable if EKF publishes it)',
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='Launch the RPLIDAR C1 node',
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Launch the webcam (v4l2_camera) node',
    )
    use_depth_arg = DeclareLaunchArgument(
        'use_depth', default_value='false',
        description='Launch the Intel RealSense D435i depth camera',
    )

    # -- URDF (robot_state_publisher + joint_state_publisher) --
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'description.launch.py')
        ),
    )

    # -- Driver node --
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_dir, 'launch', 'driver.launch.py')
        ),
    )

    # -- RPLIDAR C1 (publishes directly to /scan) --
    # Lidar is mounted on top platform with clear 360-degree view.
    # No laser_filter needed (was only required when lidar was on middle
    # platform and saw chassis/cables at 0.02-0.09m range).
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    # -- Webcam (v4l2_camera) --
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    # -- Intel RealSense D435i depth camera --
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'depth_camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_depth')),
    )

    # -- IMU filter (Madgwick) --
    imu_filter_config = os.path.join(bringup_dir, 'config', 'imu_filter.yaml')
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[imu_filter_config],
        output='screen',
    )

    # -- EKF (robot_localization) --
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
    )

    return LaunchDescription([
        publish_odom_tf_arg,
        use_lidar_arg,
        use_camera_arg,
        use_depth_arg,
        description_launch,
        driver_launch,
        lidar_node,
        camera_launch,
        depth_camera_launch,
        imu_filter_node,
        ekf_node,
    ])
