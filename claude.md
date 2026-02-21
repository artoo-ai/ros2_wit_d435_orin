# ros2_wit_d435 — Platform-Agnostic Navigation Node

## Overview
ROS2 Python package integrating Intel RealSense D435i depth camera and WitMotion WT901 USB-C IMU for SLAM and autonomous navigation. Runs on Nvidia Jetson Orin Nano Super with ROS2 Humble.

## Architecture
- **WitMotion IMU driver** (`ros2_wit_d435/witmotion_imu_node.py`): Custom serial driver supporting both standard WT901 (11-byte 0x51-0x54 packets) and WT901C USB-C (20-byte 0x61 combined packets). Auto-detects protocol variant.
- **Sensor fusion**: `robot_localization` EKF fuses WT901 (primary orientation) + optional D435i internal IMU (differential mode) + optional wheel odometry from platform nodes.
- **SLAM**: RTAB-Map RGB-D using fused EKF odometry and `depthimage_to_laserscan` for virtual `/scan`.
- **Navigation**: Nav2 stack with platform-agnostic config (generic footprint, conservative speeds).

## Key Files
- `config/ekf_params.yaml` — EKF sensor fusion (dual-IMU + optional wheel odom)
- `config/rtabmap_params.yaml` — RTAB-Map SLAM parameters
- `config/nav2_params.yaml` — Nav2 navigation (override footprint per platform)
- `config/realsense_d435i.yaml` — D435i camera settings for Jetson
- `launch/sensors.launch.py` — Sensor bringup (D435i + WT901 + EKF + depth-to-scan)
- `launch/slam.launch.py` — RTAB-Map SLAM
- `launch/navigation.launch.py` — Nav2 navigation
- `launch/bringup.launch.py` — Full stack (modes: mapping, navigation, sensors_only)
- `urdf/sensor_frames.urdf.xacro` — Parameterized camera/IMU TF frames

## Jetson-Specific Notes
- D435i internal IMU is disabled by default due to `hid_sensor_trigger` kernel conflict. Fix: `sudo sh -c 'echo "blacklist hid_sensor_trigger" > /etc/modprobe.d/blacklist-hid-sensor.conf'` then reboot.
- `initial_reset` for RealSense is disabled to avoid USB device race conditions.
- Camera runs at 640x480@30fps to balance Jetson GPU/CPU load.

## Platform Integration
Robot platform nodes (GO2, tank, RC car) are separate packages that:
1. Subscribe to `/cmd_vel` → translate to motor commands
2. Optionally publish wheel odometry on `/odom/wheel` (uncomment `odom0` in `ekf_params.yaml`)
3. Override sensor mount positions via launch args: `cam_x`, `cam_z`, `imu_x`, `imu_z`

## Build & Run
```bash
colcon build --packages-select ros2_wit_d435 --symlink-install
source install/setup.bash
ros2 launch ros2_wit_d435 bringup.launch.py mode:=mapping baud_rate:=115200
```
