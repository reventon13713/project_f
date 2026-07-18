"""Start VINS first, then arm only after a sensor warm-up interval."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('VIO_pkg')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'vio_vins.launch.py'))),
        # The controller itself also enforces warm-up; delaying process start
        # makes it visually obvious in logs that VINS precedes any arm command.
        TimerAction(period=2.0, actions=[Node(
            package='VIO_pkg', executable='vio_preflight_test',
            name='vio_preflight_test', output='screen',
            parameters=[{'use_sim_time': True, 'warmup_seconds': 5.0}])]),
    ])
