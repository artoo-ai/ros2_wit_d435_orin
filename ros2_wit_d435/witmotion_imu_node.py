#!/usr/bin/env python3
"""
WitMotion WT901 USB-C IMU ROS2 Driver Node

Reads the proprietary binary protocol from the WitMotion WT901 9-axis IMU
over a serial (USB-C) connection and publishes standard ROS2 IMU messages.

Supports TWO protocol variants:
  Standard (WT901):  11-byte packets: [0x55] [0x51-0x54] [D0..D7] [CHECKSUM]
  USB-C   (WT901C):  20-byte packets: [0x55] [0x61]      [D0..D17]

Published Topics:
  /imu/wit_data        (sensor_msgs/Imu)           - accel + gyro + orientation
  /imu/wit_mag         (sensor_msgs/MagneticField)  - magnetometer (standard only)
  /imu/wit_temperature (sensor_msgs/Temperature)    - chip temperature

Parameters:
  port       (str)   : Serial port path, default '/dev/witmotion_imu'
  baud_rate  (int)   : Baud rate, default 9600
  frame_id   (str)   : TF frame, default 'imu_link'
  update_rate (float) : Publishing rate cap in Hz, default 200.0
"""

import struct
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header

try:
    import serial
except ImportError:
    serial = None


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
HEADER_BYTE = 0x55

# Standard 11-byte packet types
PKT_ACCEL = 0x51
PKT_GYRO = 0x52
PKT_ANGLE = 0x53
PKT_MAG = 0x54
PKT_QUAT = 0x59
STD_PACKET_LEN = 11

# WT901C USB-C combined packet type
PKT_COMBINED = 0x61
COMBINED_PACKET_LEN = 20

# Conversion factors
ACCEL_SCALE = 16.0 * 9.80665 / 32768.0   # raw → m/s² (±16g range)
GYRO_SCALE = 2000.0 / 32768.0             # raw → °/s  (±2000°/s range)
ANGLE_SCALE = 180.0 / 32768.0             # raw → degrees
MAG_SCALE = 1.0                            # raw → µT
QUAT_SCALE = 1.0 / 32768.0                # raw → normalized quaternion
TEMP_SCALE = 100.0                         # raw / 100 → °C

DEG_TO_RAD = math.pi / 180.0


class WitMotionIMUNode(Node):
    """ROS2 node that reads WitMotion WT901 binary serial data."""

    def __init__(self):
        super().__init__('witmotion_imu_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/witmotion_imu')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('update_rate', 200.0)

        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.update_rate = self.get_parameter('update_rate').value

        # ── Validate serial library ──────────────────────────────────
        if serial is None:
            self.get_logger().fatal(
                'pyserial is not installed! Run: pip3 install pyserial')
            raise RuntimeError('pyserial is required')

        # ── Publishers ───────────────────────────────────────────────
        self.imu_pub = self.create_publisher(Imu, '/imu/wit_data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/wit_mag', 10)
        self.temp_pub = self.create_publisher(Temperature, '/imu/wit_temperature', 10)

        # ── Internal state ───────────────────────────────────────────
        self._accel = [0.0, 0.0, 0.0]
        self._gyro = [0.0, 0.0, 0.0]
        self._euler = [0.0, 0.0, 0.0]
        self._quat = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
        self._mag = [0.0, 0.0, 0.0]
        self._temperature = 0.0
        self._has_quat = False
        self._protocol = 'unknown'  # 'standard' or 'combined'
        self._publish_count = 0

        # ── Covariance ───────────────────────────────────────────────
        self._orientation_cov = self._diag_cov(0.01)
        self._angular_vel_cov = self._diag_cov(0.005)
        self._linear_accel_cov = self._diag_cov(0.1)

        # ── Serial port ──────────────────────────────────────────────
        self._serial = None
        self._buffer = bytearray()
        self._open_serial()

        # ── Timer ────────────────────────────────────────────────────
        period = 1.0 / self.update_rate
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'WitMotion WT901 IMU node started '
            f'[port={self.port}, baud={self.baud_rate}, '
            f'frame={self.frame_id}, rate={self.update_rate}Hz]')

    # ──────────────────────────────────────────────────────────────────
    # Serial
    # ──────────────────────────────────────────────────────────────────
    def _open_serial(self):
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
            )
            self._serial.reset_input_buffer()  # Flush stale data
            self._buffer.clear()               # Clear parse buffer
            self.get_logger().info(f'Opened serial port {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(
                f'Failed to open {self.port}: {e}. '
                f'Check cable, permissions, and udev rules.')
            self._serial = None

    def _timer_callback(self):
        if self._serial is None or not self._serial.is_open:
            self._open_serial()
            return

        try:
            waiting = self._serial.in_waiting or 1
            data = self._serial.read(waiting)
            if data:
                self._buffer.extend(data)
                self._parse_buffer()
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            self._serial = None

    # ──────────────────────────────────────────────────────────────────
    # Packet parsing — supports both protocol variants
    # ──────────────────────────────────────────────────────────────────
    def _parse_buffer(self):
        while len(self._buffer) >= STD_PACKET_LEN:
            # Find header byte
            idx = self._buffer.find(HEADER_BYTE)
            if idx < 0:
                self._buffer.clear()
                return
            if idx > 0:
                del self._buffer[:idx]

            if len(self._buffer) < 2:
                return

            pkt_type = self._buffer[1]

            # ── WT901C USB-C combined packet (0x55 0x61, 20 bytes) ──
            if pkt_type == PKT_COMBINED:
                if len(self._buffer) < COMBINED_PACKET_LEN:
                    return
                packet = self._buffer[:COMBINED_PACKET_LEN]
                self._process_combined_packet(packet)
                del self._buffer[:COMBINED_PACKET_LEN]

            # ── Standard 11-byte packets (0x55 0x51-0x54/0x59) ──────
            elif pkt_type in (PKT_ACCEL, PKT_GYRO, PKT_ANGLE, PKT_MAG, PKT_QUAT):
                if len(self._buffer) < STD_PACKET_LEN:
                    return
                packet = self._buffer[:STD_PACKET_LEN]
                # Validate checksum
                checksum = sum(packet[:STD_PACKET_LEN - 1]) & 0xFF
                if checksum != packet[STD_PACKET_LEN - 1]:
                    del self._buffer[0]
                    continue
                self._process_standard_packet(packet)
                del self._buffer[:STD_PACKET_LEN]

            else:
                # Unknown packet type — skip this 0x55 and try next
                del self._buffer[0]

    def _process_combined_packet(self, pkt: bytes):
        """
        Parse WT901C USB-C combined packet (0x55 0x61 + 18 data bytes).

        Data layout (bytes 2-19):
          [0:1]  AccelX (int16 LE)
          [2:3]  AccelY (int16 LE)
          [4:5]  AccelZ (int16 LE)
          [6:7]  GyroX  (int16 LE)
          [8:9]  GyroY  (int16 LE)
          [10:11] GyroZ (int16 LE)
          [12:13] Roll  (int16 LE)
          [14:15] Pitch (int16 LE)
          [16:17] Yaw   (int16 LE)
        """
        if self._protocol == 'unknown':
            self._protocol = 'combined'
            self.get_logger().info(
                '📡 Detected WT901C USB-C protocol (0x61 combined packets)')

        data = pkt[2:20]
        vals = struct.unpack('<9h', data)

        # Acceleration (m/s²)
        self._accel[0] = vals[0] * ACCEL_SCALE
        self._accel[1] = vals[1] * ACCEL_SCALE
        self._accel[2] = vals[2] * ACCEL_SCALE

        # Angular velocity (rad/s)
        self._gyro[0] = vals[3] * GYRO_SCALE * DEG_TO_RAD
        self._gyro[1] = vals[4] * GYRO_SCALE * DEG_TO_RAD
        self._gyro[2] = vals[5] * GYRO_SCALE * DEG_TO_RAD

        # Euler angles → quaternion (rad)
        roll = vals[6] * ANGLE_SCALE * DEG_TO_RAD
        pitch = vals[7] * ANGLE_SCALE * DEG_TO_RAD
        yaw = vals[8] * ANGLE_SCALE * DEG_TO_RAD
        self._euler = [roll, pitch, yaw]
        self._quat = self._euler_to_quaternion(roll, pitch, yaw)

        # Publish
        now = self.get_clock().now().to_msg()
        self._publish_imu(now)

        # Log periodically
        self._publish_count += 1
        if self._publish_count == 1:
            self.get_logger().info(
                f'✅ First IMU reading: accel=[{self._accel[0]:.2f}, '
                f'{self._accel[1]:.2f}, {self._accel[2]:.2f}] m/s²')
        elif self._publish_count % 1000 == 0:
            self.get_logger().debug(
                f'IMU published {self._publish_count} msgs '
                f'accel_z={self._accel[2]:.2f}')

    def _process_standard_packet(self, pkt: bytes):
        """Parse standard WT901 11-byte packet."""
        if self._protocol == 'unknown':
            self._protocol = 'standard'
            self.get_logger().info(
                '📡 Detected standard WT901 protocol (0x51-0x54 packets)')

        pkt_type = pkt[1]
        data = pkt[2:10]
        vals = struct.unpack('<hhhh', data)
        now = self.get_clock().now().to_msg()

        if pkt_type == PKT_ACCEL:
            self._accel[0] = vals[0] * ACCEL_SCALE
            self._accel[1] = vals[1] * ACCEL_SCALE
            self._accel[2] = vals[2] * ACCEL_SCALE
            self._temperature = vals[3] / TEMP_SCALE
            self._publish_temperature(now)

        elif pkt_type == PKT_GYRO:
            self._gyro[0] = vals[0] * GYRO_SCALE * DEG_TO_RAD
            self._gyro[1] = vals[1] * GYRO_SCALE * DEG_TO_RAD
            self._gyro[2] = vals[2] * GYRO_SCALE * DEG_TO_RAD

        elif pkt_type == PKT_ANGLE:
            self._euler[0] = vals[0] * ANGLE_SCALE * DEG_TO_RAD
            self._euler[1] = vals[1] * ANGLE_SCALE * DEG_TO_RAD
            self._euler[2] = vals[2] * ANGLE_SCALE * DEG_TO_RAD
            if not self._has_quat:
                self._quat = self._euler_to_quaternion(
                    self._euler[0], self._euler[1], self._euler[2])
            self._publish_imu(now)
            self._publish_count += 1
            if self._publish_count == 1:
                self.get_logger().info(
                    f'✅ First IMU reading: accel=[{self._accel[0]:.2f}, '
                    f'{self._accel[1]:.2f}, {self._accel[2]:.2f}] m/s²')

        elif pkt_type == PKT_MAG:
            self._mag[0] = float(vals[0]) * MAG_SCALE
            self._mag[1] = float(vals[1]) * MAG_SCALE
            self._mag[2] = float(vals[2]) * MAG_SCALE
            self._publish_mag(now)

        elif pkt_type == PKT_QUAT:
            self._has_quat = True
            self._quat = [
                vals[0] * QUAT_SCALE,
                vals[1] * QUAT_SCALE,
                vals[2] * QUAT_SCALE,
                vals[3] * QUAT_SCALE,
            ]
            self._publish_imu(now)

    # ──────────────────────────────────────────────────────────────────
    # Publishing
    # ──────────────────────────────────────────────────────────────────
    def _publish_imu(self, stamp):
        msg = Imu()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)

        msg.orientation.w = self._quat[0]
        msg.orientation.x = self._quat[1]
        msg.orientation.y = self._quat[2]
        msg.orientation.z = self._quat[3]
        msg.orientation_covariance = list(self._orientation_cov)

        msg.angular_velocity.x = self._gyro[0]
        msg.angular_velocity.y = self._gyro[1]
        msg.angular_velocity.z = self._gyro[2]
        msg.angular_velocity_covariance = list(self._angular_vel_cov)

        msg.linear_acceleration.x = self._accel[0]
        msg.linear_acceleration.y = self._accel[1]
        msg.linear_acceleration.z = self._accel[2]
        msg.linear_acceleration_covariance = list(self._linear_accel_cov)

        self.imu_pub.publish(msg)

    def _publish_mag(self, stamp):
        msg = MagneticField()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.magnetic_field.x = self._mag[0] * 1e-6
        msg.magnetic_field.y = self._mag[1] * 1e-6
        msg.magnetic_field.z = self._mag[2] * 1e-6
        msg.magnetic_field_covariance = list(self._diag_cov(1e-6))
        self.mag_pub.publish(msg)

    def _publish_temperature(self, stamp):
        msg = Temperature()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.temperature = self._temperature
        msg.variance = 0.5
        self.temp_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    # Utilities
    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _diag_cov(var: float) -> list:
        return [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var,
        ]

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [w, x, y, z]

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WitMotionIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
