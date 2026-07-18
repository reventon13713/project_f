"""Run the Gazebo-native mono VIO sensor chain and publish its fixed TF tree."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    vins_share = get_package_share_directory('vins_fusion_ros2')
    default_config = os.path.join(
        vins_share, 'config', 'euroc', 'euroc_mono_imu_config.yaml')
    vins_launch = os.path.join(vins_share, 'launch', 'vins_fusion_ros2.launch.py')

    # Gazebo physical link axes are FLU.  The camera sensor looks +X in that
    # frame; ROS optical axes are X-right, Y-down, Z-forward.
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('start_camera_relay', default_value='true'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='vio_base_to_imu_tf',
            arguments=['0', '0', '0.02', '0', '0', '0',
                       'base_link', 'vio_imu_link']),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='vio_base_to_camera_tf',
            arguments=['-0.158979', '-0.044050', '0.045074', '0', '0', '0',
                       'base_link', 'vio_camera_link']),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='vio_base_to_lidar_tf',
            arguments=['0', '0', '-0.08', '0', '1.57079632679', '0',
                       'base_link', 'vio_lidar_link']),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='vio_camera_to_optical_tf',
            arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679',
                       'vio_camera_link', 'vio_camera_optical_frame']),
        Node(
            package='VIO_pkg', executable='vio_camera_relay',
            name='vio_camera_relay',
            condition=IfCondition(LaunchConfiguration('start_camera_relay')),
            parameters=[{'use_sim_time': True,
                         'input_topic': '/vio/camera/image_raw',
                         'output_topic': '/vio/camera/vins_image_raw',
                         # Remove only the ~4 ms duplicate render copies;
                         # preserve the native 20 Hz Gazebo camera stream.
                         'target_rate_hz': 25.0}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vins_launch),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'use_sim_time': 'true',
            }.items()),
    ])
