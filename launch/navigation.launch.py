#!/usr/bin/env python3
"""
navigation.launch.py — Nav2 Navigation Stack Launch File

Launches the full Nav2 navigation stack for autonomous navigation
using a pre-built map.

Prerequisites: Run sensors.launch.py first (or use bringup.launch.py).
               A map must have been previously built using slam.launch.py.

Usage:
  ros2 launch ros2_wit_d435 navigation.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_wit_d435')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Config
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('map', default_value='',
                              description='Full path to map YAML file'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Auto-start Nav2 lifecycle nodes'),
        DeclareLaunchArgument('params_file', default_value=nav2_config,
                              description='Nav2 parameters file'),
    ]

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    # ── Nav2 Bringup ──────────────────────────────────────────────────
    # Uses the official nav2_bringup launch which handles lifecycle
    # management of all Nav2 nodes.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription(
        declared_args + [
            nav2_launch,
        ]
    )
