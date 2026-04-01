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
        ],
    },
)
