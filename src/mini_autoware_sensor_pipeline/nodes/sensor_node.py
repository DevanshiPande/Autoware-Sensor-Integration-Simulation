#!/usr/bin/env python3
"""
Simulated LiDAR sensor node.
Publishes fake (x, y) points at ~10 Hz to /raw_lidar (Autoware-style pipeline input).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from mini_autoware_sensor_pipeline.msg import RawLidar
import random
import math


class SensorNode(Node):
    """Emits simulated 2D LiDAR points in a configurable range."""

    def __init__(self):
        super().__init__("sensor_node")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("num_points", 100)
        self.declare_parameter("range_min", 0.5)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("fov_deg", 270.0)  # field of view in degrees

        self.pub = self.create_publisher(RawLidar, "/raw_lidar", 10)
        rate_hz = self.get_parameter("rate_hz").value
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(
            f"Simulated LiDAR publishing at {rate_hz} Hz to /raw_lidar"
        )

    def timer_callback(self):
        rate_hz = self.get_parameter("rate_hz").value
        num_points = self.get_parameter("num_points").value
        range_min = self.get_parameter("range_min").value
        range_max = self.get_parameter("range_max").value
        fov_deg = self.get_parameter("fov_deg").value

        fov_rad = math.radians(fov_deg)
        x_list = []
        y_list = []

        for _ in range(num_points):
            # Random distance and angle within FOV (centered forward = +x)
            r = random.uniform(range_min, range_max)
            theta = random.uniform(-fov_rad / 2, fov_rad / 2)
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            x_list.append(float(x))
            y_list.append(float(y))

        msg = RawLidar()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_frame"
        msg.x = x_list
        msg.y = y_list
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
