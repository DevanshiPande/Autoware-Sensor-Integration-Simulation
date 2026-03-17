#!/usr/bin/env python3
"""
Driver / Adapter node: converts raw LiDAR to Autoware-style PointCloud2.
Subscribes to /raw_lidar, publishes to /points_raw.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from mini_autoware_sensor_pipeline.msg import RawLidar
import struct


class DriverNode(Node):
    """Adapter: RawLidar -> PointCloud2 (sensor_msgs) for /points_raw."""

    def __init__(self):
        super().__init__("driver_node")
        self.sub = self.create_subscription(
            RawLidar, "/raw_lidar", self.raw_callback, 10
        )
        self.pub = self.create_publisher(PointCloud2, "/points_raw", 10)
        self.get_logger().info(
            "Driver node: /raw_lidar -> /points_raw (PointCloud2)"
        )

    def raw_callback(self, msg: RawLidar):
        # Build PointCloud2 with fields x, y, z (z=0 for 2D)
        # PointCloud2 layout: FLOAT32 x, y, z per point -> 12 bytes per point
        point_step = 12
        num_points = len(msg.x)
        if num_points != len(msg.y):
            self.get_logger().warn("Raw LiDAR x/y length mismatch, skipping.")
            return

        buf = []
        for i in range(num_points):
            buf.append(struct.pack("fff", msg.x[i], msg.y[i], 0.0))

        data = b"".join(buf)
        row_step = point_step * num_points

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = num_points
        out.is_dense = True
        out.point_step = point_step
        out.row_step = row_step
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.data = data
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
