from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory('vins_fusion_ros2'),
        'config',
        'euroc',
        'euroc_mono_imu_config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config_file,
            description='Absolute path to a VINS mono/stereo configuration file.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo /clock.'),
        DeclareLaunchArgument(
            'replay_reset_events', default_value='false',
            description='Apply recorded preflight reset events in-process.'),
        DeclareLaunchArgument(
            'input_downsample_phase', default_value='0',
            description='Estimator 2:1 input phase (0 live default, 1 replay alignment).'),
        DeclareLaunchArgument(
            'replay_skip_initial_images', default_value='0',
            description='Replay diagnostic only: discard this many leading camera images.'),
        Node(
            package='vins_fusion_ros2',
            executable='vins_fusion_ros2_node',
            name='vins_fusion_ros2_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'config_file': LaunchConfiguration('config_file')},
                        {'replay_reset_events':
                         LaunchConfiguration('replay_reset_events')},
                        {'input_downsample_phase':
                         LaunchConfiguration('input_downsample_phase')},
                        {'replay_skip_initial_images':
                         LaunchConfiguration('replay_skip_initial_images')}],
        )
    ])
