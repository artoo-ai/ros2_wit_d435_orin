#!/usr/bin/env python3
"""Quick test: subscribe to depth with SENSOR_DATA QoS (same as depthimage_to_laserscan uses).
If this receives images with height=480, the issue is in image_transport, not QoS.
Usage: python3 scripts/test_depth_qos.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class DepthTest(Node):
    def __init__(self):
        super().__init__('depth_qos_test')
        self.count = 0
        self.sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.callback,
            qos_profile_sensor_data)
        self.get_logger().info('Subscribed with SENSOR_DATA QoS (BEST_EFFORT/VOLATILE)')

    def callback(self, msg):
        self.count += 1
        self.get_logger().info(
            f'[{self.count}] Image: {msg.width}x{msg.height}, '
            f'encoding={msg.encoding}, data_len={len(msg.data)}')
        if self.count >= 5:
            self.get_logger().info('SUCCESS - Received 5 valid images. Issue is in image_transport.')
            raise SystemExit(0)

def main():
    rclpy.init()
    node = DepthTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
