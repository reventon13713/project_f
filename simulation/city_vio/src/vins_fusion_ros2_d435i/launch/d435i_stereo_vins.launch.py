"""Launch only the checkpointed D435i stereo+IMU estimator.

The mono VIO_pkg launch and vins_fusion_ros2 package are intentionally not
included, so this test cannot mutate or accidentally start the mono pipeline.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vins_fusion_ros2_d435i')
    estimator_launch = os.path.join(
        share, 'launch', 'vins_fusion_ros2.launch.py')
    config = os.path.join(
        share, 'config', 'project_f_d435i',
        'd435i_stereo_imu_config.yaml')
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435i_base_to_imu_tf',
            arguments=[
                '-0.158979', '-0.044050', '0.045074',
                '0', '0', '0', 'base_link', 'd435i_imu_link']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435i_imu_to_infra1_optical_tf',
            arguments=[
                '0', '0.025', '0',
                '-1.57079632679', '0', '-1.57079632679',
                'd435i_imu_link', 'd435i_infra1_optical_frame']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435i_imu_to_infra2_optical_tf',
            arguments=[
                '0', '-0.025', '0',
                '-1.57079632679', '0', '-1.57079632679',
                'd435i_imu_link', 'd435i_infra2_optical_frame']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(estimator_launch),
            launch_arguments={
                'config_file': config,
                'use_sim_time': 'true',
            }.items()),
    ])
