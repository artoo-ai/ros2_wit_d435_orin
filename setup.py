import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_wit_d435'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) +
            glob(os.path.join('config', '*.rviz'))),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf*'))),
        # udev rules
        (os.path.join('share', package_name, 'udev'),
            glob(os.path.join('udev', '*.rules'))),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='rico',
    maintainer_email='rico@todo.com',
    description='Platform-agnostic ROS2 navigation with RealSense D435i and WitMotion WT901 IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'witmotion_imu_node = ros2_wit_d435.witmotion_imu_node:main',
        ],
    },
)
