#!/usr/bin/env python3
"""
slam.launch.py — RTAB-Map RGB-D SLAM Launch File

Launches RTAB-Map SLAM using:
  - RealSense D435i RGB + Depth streams
  - Fused odometry from robot_localization EKF (/odometry/filtered)
  - LaserScan from depthimage_to_laserscan (/scan)

Prerequisites: Run sensors.launch.py first (or use bringup.launch.py).

Usage:
  ros2 launch ros2_wit_d435 slam.launch.py
  ros2 launch ros2_wit_d435 slam.launch.py db_name:=my_map use_visual_odom:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_wit_d435')

    # Config
    rtabmap_config = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')

    # Database directory
    rtabmap_db_dir = os.path.expanduser('~/.ros/rtabmap')
    os.makedirs(rtabmap_db_dir, exist_ok=True)

    # ── Launch arguments ──────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('db_name', default_value='rtabmap_slam',
                              description='RTAB-Map database name'),
        DeclareLaunchArgument('use_visual_odom', default_value='false',
                              description='Use visual odometry in addition to EKF'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localization-only mode (use existing map)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RTAB-Map viz'),
    ]

    db_name = LaunchConfiguration('db_name')
    use_visual_odom = LaunchConfiguration('use_visual_odom')
    localization = LaunchConfiguration('localization')
    with_rviz = LaunchConfiguration('rviz')

    # ── Topic remappings (RealSense v4.56 uses /camera/camera/) ──────
    camera_remappings = [
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('scan', '/scan'),
        ('odom', '/odometry/filtered'),
    ]

    vo_camera_remappings = [
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('odom', '/vo_odom'),
    ]

    # ── Visual Odometry (optional) ────────────────────────────────────
    visual_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        condition=IfCondition(use_visual_odom),
        parameters=[rtabmap_config],
        remappings=vo_camera_remappings,
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ── RTAB-Map SLAM (with EKF odometry) ─────────────────────────────
    rtabmap_mapping = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=UnlessCondition(use_visual_odom),
        parameters=[
            rtabmap_config,
            {
                'database_path': [rtabmap_db_dir, '/', db_name, '.db'],
                'subscribe_rgb': True,
                'subscribe_depth': True,
                'subscribe_scan': True,
                'subscribe_odom_info': False,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'Mem/IncrementalMemory': 'true',
            }
        ],
        remappings=camera_remappings,
        arguments=['--delete_db_on_start', '--ros-args', '--log-level', 'warn'],
    )

    # ── RTAB-Map SLAM (with visual odometry) ──────────────────────────
    rtabmap_with_vo = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(use_visual_odom),
        parameters=[
            rtabmap_config,
            {
                'database_path': [rtabmap_db_dir, '/', db_name, '.db'],
                'subscribe_rgb': True,
                'subscribe_depth': True,
                'subscribe_scan': True,
                'subscribe_odom_info': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'vo_odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'Mem/IncrementalMemory': 'true',
            }
        ],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('scan', '/scan'),
            ('odom', '/vo_odom'),
        ],
        arguments=['--delete_db_on_start', '--ros-args', '--log-level', 'warn'],
    )

    # ── RTAB-Map Visualization ────────────────────────────────────────
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(with_rviz),
        parameters=[rtabmap_config],
        remappings=camera_remappings,
    )

    return LaunchDescription(
        declared_args + [
            visual_odom_node,
            rtabmap_mapping,
            rtabmap_with_vo,
            rtabmap_viz,
        ]
    )
