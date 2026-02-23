#!/usr/bin/env python3
"""
sensors.launch.py — Sensor Bringup Launch File

Launches:
  1. RealSense D435i camera (depth + color + optional internal IMU)
  2. WitMotion WT901 IMU driver (primary IMU)
  3. IMU filter (Madgwick) for D435i internal IMU (if enabled)
  4. robot_localization EKF (fuses IMU(s) + optional wheel odom)
  5. depthimage_to_laserscan (generates /scan from depth)
  6. Robot state publisher (sensor TF frames)

Note on D435i IMU:
  On Jetson Orin, the kernel hid_sensor_trigger module conflicts with the
  D435i internal IMU, causing the entire camera to fail. The D435i IMU is
  DISABLED by default. To enable it, first apply the kernel fix:
    sudo sh -c 'echo "blacklist hid_sensor_trigger" > /etc/modprobe.d/blacklist-hid-sensor.conf'
    sudo reboot
  Then launch with: enable_camera_imu:=true

Usage:
  ros2 launch ros2_wit_d435 sensors.launch.py baud_rate:=115200
  ros2 launch ros2_wit_d435 sensors.launch.py baud_rate:=115200 enable_camera_imu:=true
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_wit_d435')
    rs_pkg_dir = get_package_share_directory('realsense2_camera')

    # ── Launch arguments ──────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('imu_port', default_value='/dev/witmotion_imu',
                              description='WitMotion WT901 serial port'),
        DeclareLaunchArgument('baud_rate', default_value='9600',
                              description='WT901 baud rate'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link',
                              description='TF frame for WitMotion IMU'),
        DeclareLaunchArgument('camera_ns', default_value='camera',
                              description='RealSense camera namespace'),
        DeclareLaunchArgument('enable_camera_imu', default_value='false',
                              description='Enable D435i internal IMU (requires kernel fix on Jetson)'),
        # Sensor mount overrides (passed to URDF xacro)
        DeclareLaunchArgument('cam_x', default_value='0.15',
                              description='Camera X offset from base_link (m)'),
        DeclareLaunchArgument('cam_y', default_value='0.0',
                              description='Camera Y offset'),
        DeclareLaunchArgument('cam_z', default_value='0.20',
                              description='Camera Z offset'),
        DeclareLaunchArgument('cam_roll', default_value='0.0',
                              description='Camera roll'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0',
                              description='Camera pitch'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0',
                              description='Camera yaw'),
        DeclareLaunchArgument('imu_x', default_value='0.0',
                              description='IMU X offset from base_link (m)'),
        DeclareLaunchArgument('imu_y', default_value='0.0',
                              description='IMU Y offset'),
        DeclareLaunchArgument('imu_z', default_value='0.05',
                              description='IMU Z offset'),
    ]

    enable_camera_imu = LaunchConfiguration('enable_camera_imu')

    # ── Process URDF ──────────────────────────────────────────────────
    urdf_path = os.path.join(pkg_dir, 'urdf', 'sensor_frames.urdf.xacro')
    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'cam_x': '0.15', 'cam_y': '0.0', 'cam_z': '0.20',
            'cam_roll': '0.0', 'cam_pitch': '0.0', 'cam_yaw': '0.0',
            'imu_x': '0.0', 'imu_y': '0.0', 'imu_z': '0.05',
            'imu_roll': '0.0', 'imu_pitch': '0.0', 'imu_yaw': '0.0',
        }
    ).toxml()

    # ── Config paths ──────────────────────────────────────────────────
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')

    # ── Nodes ─────────────────────────────────────────────────────────

    # Info message about D435i IMU status
    imu_disabled_msg = LogInfo(
        msg='⚠️  D435i internal IMU DISABLED (default on Jetson). '
            'Using WitMotion WT901 as sole IMU. '
            'To enable D435i IMU: blacklist hid_sensor_trigger, reboot, '
            'then launch with enable_camera_imu:=true',
    )

    # 1) Robot State Publisher (sensor frames TF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='sensor_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                      'use_sim_time': False}],
    )

    # 2) RealSense D435i Camera (depth + color, IMU controlled by param)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_pkg_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_accel': enable_camera_imu,
            'enable_gyro': enable_camera_imu,
            'unite_imu_method': '2',
            'enable_sync': 'false',
            'align_depth.enable': 'false',
            'pointcloud.enable': 'false',
            'spatial_filter.enable': 'false',
            'temporal_filter.enable': 'false',
            'decimation_filter.enable': 'false',
            'depth_module.depth_profile': '480x270x30',
            'rgb_camera.color_profile': '640x480x30',
            'initial_reset': 'false',
        }.items(),
    )

    # 3) WitMotion WT901 IMU driver (primary IMU — always on)
    witmotion_node = Node(
        package='ros2_wit_d435',
        executable='witmotion_imu_node',
        name='witmotion_imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'frame_id': LaunchConfiguration('imu_frame_id'),
            'update_rate': 200.0,
        }],
    )

    # 4) IMU Filter Madgwick — only when D435i IMU is enabled
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_d435i',
        output='screen',
        condition=IfCondition(enable_camera_imu),
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': 'base_link',
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/camera/imu/filtered'),
        ],
    )

    # 5) robot_localization EKF (fuses WT901 + optional D435i IMU)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    # 6) Depth Image to LaserScan (generates /scan from D435i depth)
    #    Uses custom node that subscribes directly (no image_transport)
    #    to avoid 0-height image bug in depthimage_to_laserscan package.
    depth_to_scan = Node(
        package='ros2_wit_d435',
        executable='depth_to_scan_node',
        name='depth_to_scan',
        output='screen',
        parameters=[{
            'scan_height': 1,
            'scan_time': 0.033,
            'range_min': 0.3,
            'range_max': 8.0,
            'output_frame_id': 'camera_depth_optical_frame',
        }],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription(
        declared_args + [
            imu_disabled_msg,
            robot_state_pub,
            realsense_launch,
            witmotion_node,
            imu_filter,
            ekf_node,
            depth_to_scan,
        ]
    )
