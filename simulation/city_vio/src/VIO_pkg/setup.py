"""Package the ROS observation nodes used by the Jetson image."""
from setuptools import setup

nodes = ("vio_camera_relay", "vio_feature_overlay", "vio_global_position")
setup(
    name="VIO_pkg",
    version="0.1.0",
    packages=["VIO_pkg"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/VIO_pkg"]),
        ("share/VIO_pkg", ["package.xml"]),
    ],
    entry_points={"console_scripts": [f"{name} = VIO_pkg.{name}:main" for name in nodes]},
)
