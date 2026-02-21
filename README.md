# ros2_wit_d435 — Platform-Agnostic Navigation with D435i + WT901

ROS2 Python package that integrates an **Intel RealSense D435i** depth camera and **WitMotion WT901 USB-C** 9-axis IMU for SLAM and autonomous navigation. Designed to run on an **Nvidia Jetson Orin Nano Super** with Ubuntu and ROS2 Humble.

**Platform-agnostic** — works with Unitree GO2, tank-drive robots, RC car platforms, or any robot that publishes `/cmd_vel → motor commands`.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ros2_wit_d435 (this package)                     │
│                                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐    │
│  │ RealSense    │  │ WitMotion    │  │ depthimage_to_laserscan│    │
│  │ D435i        │  │ WT901 IMU    │  │ (/scan)                │    │
│  │ (depth+RGB   │  │ (/imu/wit*)  │  └────────────────────────┘    │
│  │  +IMU)       │  └──────┬───────┘                                │
│  └──────┬───────┘         │                                        │
│         │                 │                                        │
│  ┌──────▼─────────────────▼──────┐                                 │
│  │    robot_localization (EKF)   │                                  │
│  │    /odometry/filtered         │──── odom→base_link TF           │
│  └──────────────┬────────────────┘                                 │
│                 │                                                   │
│  ┌──────────────▼────────────────┐   ┌─────────────────────────┐   │
│  │     RTAB-Map SLAM             │   │      Nav2 Stack         │   │
│  │     (mapping / localization)  │   │      (navigation)       │   │
│  └───────────────────────────────┘   └──────────┬──────────────┘   │
│                                                  │                  │
└──────────────────────────────────────────────────┼──────────────────┘
                                                   │
                                          /cmd_vel │
                                                   ▼
                                    ┌──────────────────────────┐
                                    │  Platform Node (separate)│
                                    │  GO2 / Tank / RC Car     │
                                    │  cmd_vel → motor commands│
                                    └──────────────────────────┘
```

Each robot platform only needs a separate node to:
1. Subscribe to `/cmd_vel` and translate to motor commands
2. *(Optional)* Publish wheel odometry on `/odom/wheel` for EKF fusion

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 (Jetson JetPack 6.x) |
| ROS2 | Humble Hawksbill |
| Python | 3.10+ |
| Jetson | Orin Nano Super (JetPack 6.0+) |

---

## 1. System Dependencies

### 1.1 ROS2 Humble (if not already installed)

```bash
# Follow official instructions:
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt update && sudo apt install -y ros-humble-desktop
```

### 1.2 Intel RealSense SDK (librealsense2)

```bash
# Register Intel's server key
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
  sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# Add the repository
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
  https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list

# Install
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# Verify — plug in the D435i and run:
realsense-viewer
```

> **Jetson Note:** If the apt packages don't work with your JetPack version, build librealsense from source with the RSUSB backend:
> ```bash
> git clone https://github.com/IntelRealSense/librealsense.git
> cd librealsense
> mkdir build && cd build
> cmake .. -DFORCE_RSUSB_BACKEND=ON -DCMAKE_BUILD_TYPE=Release
> make -j$(nproc) && sudo make install
> ```

### 1.3 Python Serial Library

```bash
pip3 install pyserial
```

### 1.4 Serial Port Permissions

Add your user to the `dialout` group to access serial devices without `sudo`:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for this to take effect
```

---

## 2. ROS2 Package Dependencies

Install all required ROS2 packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-imu-filter-madgwick \
  ros-humble-robot-localization \
  ros-humble-rtabmap-ros \
  ros-humble-nav2-bringup \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-map-server \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-waypoint-follower \
  ros-humble-nav2-smac-planner \
  ros-humble-dwb-core \
  ros-humble-depthimage-to-laserscan \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-foxglove-bridge
```

Or use `rosdep`:

```bash
cd /path/to/your/workspace
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. Build the Package

```bash
# Navigate to your ROS2 workspace (the parent of ros2_wit_d435)
cd /home/rico/Documents/unitree

# Build
colcon build --packages-select ros2_wit_d435 --symlink-install

# Source
source install/setup.bash
```

---

## 4. WitMotion WT901 Setup

### 4.1 Install udev Rules

Creates a consistent device symlink `/dev/witmotion_imu`:

```bash
sudo cp ros2_wit_d435/udev/99-witmotion.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify after plugging in the WT901:

```bash
ls -la /dev/witmotion_imu
# Should show a symlink to /dev/ttyUSBx
```

### 4.2 Find Your Device (if udev rules don't match)

```bash
# List USB serial devices
ls /dev/ttyUSB*

# Find vendor/product ID for your device
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
```

Update the vendor/product ID in `udev/99-witmotion.rules` if needed.

### 4.3 Baud Rate Configuration

The WT901 defaults to **9600 baud** (~20Hz output). For better SLAM performance, configure it to **115200 baud** (~200Hz) using:

- **WitMotion Windows App** (recommended): Connect via USB, open the app, change baud rate to 115200, and save.
- **Or** pass the `baud_rate` parameter when launching:

```bash
ros2 launch ros2_wit_d435 sensors.launch.py baud_rate:=115200
```

---

## 5. Launch Commands

### 5.1 Sensors Only (testing/debugging)

```bash
ros2 launch ros2_wit_d435 sensors.launch.py
```

Verify topics:

```bash
ros2 topic list
ros2 topic hz /imu/wit_data          # Should be ~100-200Hz
ros2 topic hz /odometry/filtered     # Should be ~50Hz
ros2 topic hz /camera/depth/image_rect_raw  # Should be ~30Hz
ros2 topic hz /scan                  # Should be ~30Hz
```

### 5.2 SLAM Mapping Mode

Build a map of your environment:

```bash
# Terminal 1: Sensors + SLAM
ros2 launch ros2_wit_d435 bringup.launch.py mode:=mapping db_name:=my_map

# Terminal 2: Drive the robot (via your platform node or teleop)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The map is saved to `~/.ros/rtabmap/my_map.db`.

To export a 2D occupancy grid for Nav2:

```bash
# While RTAB-Map is running:
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 5.3 Navigation Mode

Navigate using a pre-built map:

```bash
ros2 launch ros2_wit_d435 bringup.launch.py \
  mode:=navigation \
  map:=/home/rico/maps/my_map.yaml
```

### 5.4 Full Bringup with Custom Sensor Mounts

Override camera/IMU positions for your specific robot:

```bash
ros2 launch ros2_wit_d435 bringup.launch.py \
  mode:=mapping \
  cam_x:=0.10 cam_z:=0.30 \
  imu_x:=0.0 imu_z:=0.08 \
  imu_port:=/dev/ttyUSB0 \
  baud_rate:=115200
```

---

## 6. Integrating with Your Robot Platform

Each robot platform should be a **separate ROS2 package/node** that:

### Minimum Required

```python
# Subscribe to /cmd_vel (from Nav2)
# → Translate to your robot's motor command protocol
self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
```

### Optional (Recommended for Better Accuracy)

```python
# Publish wheel odometry
self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
# → Then uncomment odom0 in config/ekf_params.yaml
```

### Platform Examples

| Platform | cmd_vel → Motors | Wheel Odom |
|---|---|---|
| **Unitree GO2** | WebRTC/CycloneDDS API | Built-in odom available |
| **Tank Drive** | PWM to L/R motors | Encoder-based |
| **RC Car (Ackermann)** | Servo (steering) + ESC (throttle) | Encoder or GPS |

---

## 7. TF Frame Tree

```
map
 └── odom                    (published by EKF / RTAB-Map)
      └── base_link          (robot center)
           ├── camera_link   (D435i mount position)
           │    ├── camera_depth_optical_frame
           │    ├── camera_color_optical_frame
           │    └── camera_imu_frame
           └── imu_link      (WT901 mount position)
```

---

## 8. Published Topics

| Topic | Type | Source | Description |
|---|---|---|---|
| `/imu/wit_data` | `sensor_msgs/Imu` | WT901 driver | Accel + gyro + orientation |
| `/imu/wit_mag` | `sensor_msgs/MagneticField` | WT901 driver | Magnetometer |
| `/imu/wit_temperature` | `sensor_msgs/Temperature` | WT901 driver | Chip temperature |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RealSense | RGB image |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | RealSense | Depth image |
| `/camera/imu` | `sensor_msgs/Imu` | RealSense | D435i internal IMU |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF | Fused odometry |
| `/scan` | `sensor_msgs/LaserScan` | depth_to_scan | Virtual laser scan |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map | 2D occupancy grid |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 | Velocity commands |

---

## 9. Troubleshooting

### D435i not detected

```bash
# Check USB connection
lsusb | grep Intel
# Should show: Intel Corp. RealSense ...

# Check RealSense viewer
realsense-viewer

# If "No devices found", try:
# 1. Use a USB 3.0 port and cable
# 2. Run: sudo apt install librealsense2-dkms
# 3. Reboot
```

### WT901 no data / permission denied

```bash
# Check serial device exists
ls /dev/ttyUSB*

# Check permissions
groups $USER  # Should include 'dialout'

# Manual test (should show binary data):
python3 -c "
import serial
s = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
print(s.read(44).hex())
s.close()
"
```

### EKF not publishing `/odometry/filtered`

```bash
# Check if IMU topics are publishing
ros2 topic hz /imu/wit_data
ros2 topic hz /camera/imu

# Check EKF diagnostics
ros2 topic echo /diagnostics
```

### Poor SLAM quality

1. Ensure good lighting (D435i needs texture for visual features)
2. Move slowly — fast rotation causes motion blur
3. Increase WT901 baud rate to 115200 for better IMU rate
4. Check the D435i depth quality: `ros2 run rqt_image_view rqt_image_view`

---

## 10. File Structure

```
ros2_wit_d435/
├── package.xml                      # ROS2 package manifest
├── setup.py                         # Python package setup
├── setup.cfg                        # Install paths
├── resource/ros2_wit_d435           # ament index marker
├── config/
│   ├── ekf_params.yaml              # EKF sensor fusion config
│   ├── rtabmap_params.yaml          # RTAB-Map SLAM config
│   ├── nav2_params.yaml             # Nav2 navigation config
│   └── realsense_d435i.yaml         # D435i camera config
├── launch/
│   ├── sensors.launch.py            # Sensors + EKF
│   ├── slam.launch.py               # RTAB-Map SLAM
│   ├── navigation.launch.py         # Nav2 navigation
│   └── bringup.launch.py            # Full stack
├── urdf/
│   └── sensor_frames.urdf.xacro     # Sensor TF frames
├── ros2_wit_d435/
│   ├── __init__.py
│   └── witmotion_imu_node.py        # WT901 serial driver
├── udev/
│   └── 99-witmotion.rules           # Device rules
└── README.md                        # This file
```

---

## License

MIT
