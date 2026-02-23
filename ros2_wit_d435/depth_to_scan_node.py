#!/usr/bin/env python3
"""
depth_to_scan_node.py — Depth Image to LaserScan Converter

Subscribes directly to a depth image topic (no image_transport) and converts
the center row(s) into a sensor_msgs/LaserScan.  This replaces the standard
depthimage_to_laserscan package to avoid an image_transport compatibility
issue where the subscriber receives 0-height images despite valid publications.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan


class DepthToScanNode(Node):
    def __init__(self):
        super().__init__('depth_to_scan')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('scan_height', 1)
        self.declare_parameter('scan_time', 0.033)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('output_frame_id', 'camera_color_optical_frame')

        self.scan_height = self.get_parameter('scan_height').value
        self.scan_time = self.get_parameter('scan_time').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        # ── QoS profiles matching the RealSense publishers ────────────
        # Depth image uses RELIABLE / TRANSIENT_LOCAL
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Camera info uses RELIABLE / VOLATILE
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscriptions (direct — no image_transport) ──────────────────
        self.camera_info = None
        self.info_sub = self.create_subscription(
            CameraInfo,
            'depth_camera_info',
            self._info_cb,
            info_qos,
        )
        self.depth_sub = self.create_subscription(
            Image,
            'depth',
            self._depth_cb,
            img_qos,
        )

        # ── Publisher ────────────────────────────────────────────────────
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        self.get_logger().info(
            f'depth_to_scan started  scan_height={self.scan_height}  '
            f'range=[{self.range_min}, {self.range_max}]  '
            f'frame={self.output_frame_id}'
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _depth_cb(self, msg: Image):
        if self.camera_info is None:
            return

        h, w = msg.height, msg.width
        if h == 0 or w == 0:
            self.get_logger().warn(
                'Received empty depth image', throttle_duration_sec=5.0
            )
            return

        # Camera intrinsics
        fx = self.camera_info.k[0]
        cx = self.camera_info.k[2]
        if fx == 0.0:
            return

        # Decode depth pixels
        if msg.encoding == '16UC1':
            depth_arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            scale = 0.001  # mm → m
        elif msg.encoding == '32FC1':
            depth_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            scale = 1.0
        else:
            self.get_logger().warn(
                f'Unsupported encoding: {msg.encoding}',
                throttle_duration_sec=5.0,
            )
            return

        # Extract the centre row band
        mid = h // 2
        half = self.scan_height // 2
        r0 = max(0, mid - half)
        r1 = min(h, mid + half + (1 if self.scan_height % 2 else 0))
        band = depth_arr[r0:r1, :]

        # Collapse to one row (minimum across band)
        if band.shape[0] == 1:
            depths = band[0].astype(np.float64) * scale
        else:
            depths = np.min(band, axis=0).astype(np.float64) * scale

        # Build LaserScan
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.output_frame_id

        angle_max = math.atan2(w / 2.0, fx)
        angle_min = -angle_max
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = (angle_max - angle_min) / w
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Depth pixels are left→right but LaserScan is right→left
        col_idx = np.arange(w - 1, -1, -1)
        angles = np.arctan2(col_idx - cx, fx)
        cos_angles = np.cos(angles)
        d_rev = depths[::-1]

        ranges = np.full(w, float('inf'), dtype=np.float32)
        valid = (d_rev > 0) & (d_rev >= self.range_min) & (d_rev <= self.range_max)
        ranges[valid] = (d_rev[valid] / cos_angles[valid]).astype(np.float32)

        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
