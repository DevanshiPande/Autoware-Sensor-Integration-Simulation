#!/usr/bin/env python3
"""
Control node: reads /detected_obstacles, generates simple velocity/steering
commands, publishes to /vehicle_cmd (optional; prints to console for demo).
"""

import rclpy
from rclpy.node import Node
from mini_autoware_sensor_pipeline.msg import DetectedObstacles, VehicleCmd
import math


class ControlNode(Node):
    """Simple control: slow down or steer based on nearest obstacle."""

    def __init__(self):
        super().__init__("control_node")
        self.sub = self.create_subscription(
            DetectedObstacles, "/detected_obstacles", self.obstacles_callback, 10
        )
        self.pub = self.create_publisher(VehicleCmd, "/vehicle_cmd", 10)
        self.get_logger().info(
            "Control node: /detected_obstacles -> /vehicle_cmd (and console)"
        )

        self.declare_parameter("max_velocity", 5.0)
        self.declare_parameter("safe_dist", 3.0)
        self.declare_parameter("min_velocity", 0.5)

    def obstacles_callback(self, msg: DetectedObstacles):
        obstacles = [(o.x, o.y) for o in msg.obstacles]
        max_vel = self.get_parameter("max_velocity").value
        safe_dist = self.get_parameter("safe_dist").value
        min_vel = self.get_parameter("min_velocity").value

        # Vehicle at origin, facing +x. Compute velocity and steering from nearest obstacle.
        velocity = max_vel
        steering_angle = 0.0

        if obstacles:
            # Nearest obstacle (in front: x > 0)
            nearest = min(
                obstacles,
                key=lambda p: math.hypot(p[0], p[1]),
            )
            dist = math.hypot(nearest[0], nearest[1])
            angle = math.atan2(nearest[1], nearest[0])

            if dist < safe_dist:
                # Slow down
                velocity = max(min_vel, (dist / safe_dist) * max_vel)
                # Steer away (simple: steer opposite to obstacle y)
                steering_angle = -0.3 * math.copysign(1, nearest[1])
            else:
                velocity = max_vel
                steering_angle = 0.0

            self.get_logger().info(
                f"Nearest obstacle: ({nearest[0]:.2f}, {nearest[1]:.2f}), "
                f"dist={dist:.2f} -> vel={velocity:.2f}, steer={steering_angle:.2f}"
            )
        else:
            self.get_logger().info("No obstacles -> full speed")

        cmd = VehicleCmd()
        cmd.header = msg.header
        cmd.velocity = velocity
        cmd.steering_angle = steering_angle
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
