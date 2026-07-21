from glob import glob
from os.path import isfile
from setuptools import find_packages, setup

package_name = 'VIO_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            [path for path in glob('config/*') if isfile(path)]),
        ('share/' + package_name + '/launch',
            [path for path in glob('launch/*') if isfile(path)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reventon',
    maintainer_email='reventon@todo.todo',
    description='GPS-denied VIO-based Offboard autonomous flight system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_controller = VIO_pkg.offboard_controller:main',
            'mission_manager = VIO_pkg.mission_manager:main',
            'vio_bridge = VIO_pkg.vio_bridge:main',
            'auto_mission_node = VIO_pkg.auto_mission_node:main',
            'euroc_player = VIO_pkg.euroc_player:main',
            'vio_camera_relay = VIO_pkg.vio_camera_relay:main',
            'vio_feature_overlay = VIO_pkg.vio_feature_overlay:main',
            'vio_preflight_test = VIO_pkg.vio_preflight_test:main',
            'vio_px4_fault_test = VIO_pkg.vio_px4_fault_test:main',
            'vio_bag_metrics = VIO_pkg.vio_bag_metrics:main',
            'vio_ground_truth_metrics = VIO_pkg.vio_ground_truth_metrics:main',
            'vio_global_position = VIO_pkg.vio_global_position:main',
            'vio_global_position_metrics = VIO_pkg.vio_global_position_metrics:main',
            'vio_global_fusion_metrics = VIO_pkg.vio_global_fusion_metrics:main',
            'vio_gps_ev_fusion_test = VIO_pkg.vio_gps_ev_fusion_test:main',
            'vio_gps_dropout_test = VIO_pkg.vio_gps_dropout_test:main',
            'vio_qgc_mission_test = VIO_pkg.vio_qgc_mission_test:main',
            'vio_mission_ground_truth_metrics = VIO_pkg.vio_mission_ground_truth_metrics:main',
            'vio_init_window_metrics = VIO_pkg.vio_init_window_metrics:main',
            'vio_frame_trace_compare = VIO_pkg.vio_frame_trace_compare:main',
            'vio_reset_replayer = VIO_pkg.vio_reset_replayer:main',
        ],
    },
)
