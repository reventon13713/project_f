"""One-command interactive PX4/Gazebo VIO demonstration.

This launch deliberately keeps PX4 visual-odometry fusion disabled: the
preflight controller flies using PX4's native SITL estimator while VINS is
evaluated independently and displayed in RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vio_share = get_package_share_directory('VIO_pkg')
    vio_launch = os.path.join(vio_share, 'launch', 'vio_vins.launch.py')
    rviz_config = os.path.join(vio_share, 'config', 'vins_rviz_config.rviz')

    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='microxrce_agent', output='screen', emulate_tty=True)

    # gazebo_ros_init is what publishes /clock.  PX4's make target overwrites
    # Gazebo paths unless this ROS plugin directory is explicitly prepended.
    px4_gazebo = ExecuteProcess(
        cmd=['bash', '-lc',
             'cd /home/reventon/PX4-Autopilot && '
             'GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}" '
             'PX4_SITL_WORLD=vio_feature '
             'make px4_sitl gazebo-classic_iris_fpv_cam'],
        name='px4_gazebo', output='screen', emulate_tty=True)

    vins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vio_launch))
    bridge = Node(
        package='VIO_pkg', executable='vio_bridge', name='vio_bridge',
        output='screen', parameters=[{
            'use_sim_time': True,
            'publish_visual_odometry': False,
        }])
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config])
    preflight = Node(
        package='VIO_pkg', executable='vio_preflight_test',
        name='vio_preflight_test', output='screen',
        parameters=[{
            'use_sim_time': True,
            'warmup_seconds': 5.0,
            # Three passes draw the same XYZ cross repeatedly, making drift
            # visible as a failure of the RViz path to retrace itself.
            'excitation_repeats': 3,
            'excitation_speed': 0.40,
            'cross_xy_extent': 1.20,
            'cross_z_extent': 1.00,
        }])
    feature_overlay = Node(
        package='VIO_pkg', executable='vio_feature_overlay',
        name='vio_feature_overlay', output='screen',
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        agent,
        # Give the agent time to bind before PX4 starts its uXRCE client.
        TimerAction(period=2.0, actions=[px4_gazebo]),
        # PX4/Gazebo needs time to bring up the model and ROS sensor plugins.
        TimerAction(period=15.0, actions=[vins, bridge, feature_overlay]),
        TimerAction(period=17.0, actions=[rviz]),
        # VINS starts before any command to arm or move the aircraft.
        TimerAction(period=25.0, actions=[preflight]),
    ])
