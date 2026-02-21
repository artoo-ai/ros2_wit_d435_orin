#!/usr/bin/env python3
"""
bringup.launch.py — Full Stack Bringup (Sensors + SLAM + Navigation)

Convenience launch file that brings up everything in one command:
  1. Sensors (RealSense D435i + WitMotion WT901 + EKF fusion)
  2. SLAM (RTAB-Map RGB-D mapping)
  3. Navigation (Nav2 stack) — optional

Usage:
  # Mapping mode (build a map):
  ros2 launch ros2_wit_d435 bringup.launch.py mode:=mapping

  # Navigation mode (use existing map):
  ros2 launch ros2_wit_d435 bringup.launch.py mode:=navigation map:=/path/to/map.yaml

  # Sensors only (for debugging):
  ros2 launch ros2_wit_d435 bringup.launch.py mode:=sensors_only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_wit_d435')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # ── Launch arguments ──────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='Operation mode: mapping, navigation, sensors_only'),
        DeclareLaunchArgument('map', default_value='',
                              description='Map YAML file (required for navigation mode)'),
        DeclareLaunchArgument('db_name', default_value='rtabmap_slam',
                              description='RTAB-Map database name'),
        DeclareLaunchArgument('imu_port', default_value='/dev/witmotion_imu',
                              description='WitMotion WT901 serial port'),
        DeclareLaunchArgument('baud_rate', default_value='9600',
                              description='WT901 baud rate'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz/RTAB-Map viz'),
        # Camera & IMU mount overrides
        DeclareLaunchArgument('cam_x', default_value='0.15',
                              description='Camera X offset from base_link (m)'),
        DeclareLaunchArgument('cam_y', default_value='0.0',
                              description='Camera Y offset'),
        DeclareLaunchArgument('cam_z', default_value='0.20',
                              description='Camera Z offset'),
        DeclareLaunchArgument('imu_x', default_value='0.0',
                              description='IMU X offset from base_link (m)'),
        DeclareLaunchArgument('imu_y', default_value='0.0',
                              description='IMU Y offset'),
        DeclareLaunchArgument('imu_z', default_value='0.05',
                              description='IMU Z offset'),
    ]

    # ── Sensors (always launched) ─────────────────────────────────────
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'sensors.launch.py')
        ),
        launch_arguments={
            'imu_port': LaunchConfiguration('imu_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'cam_x': LaunchConfiguration('cam_x'),
            'cam_y': LaunchConfiguration('cam_y'),
            'cam_z': LaunchConfiguration('cam_z'),
            'imu_x': LaunchConfiguration('imu_x'),
            'imu_y': LaunchConfiguration('imu_y'),
            'imu_z': LaunchConfiguration('imu_z'),
        }.items(),
    )

    # ── SLAM (mapping mode) ───────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam.launch.py')
        ),
        launch_arguments={
            'db_name': LaunchConfiguration('db_name'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'mapping'),
    )

    # ── Navigation (navigation mode) ─────────────────────────────────
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'navigation'),
    )

    return LaunchDescription(
        declared_args + [
            sensors_launch,
            slam_launch,
            nav_launch,
        ]
    )
