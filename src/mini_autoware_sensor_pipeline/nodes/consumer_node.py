#!/usr/bin/env python3
"""
Consumer (perception) node: reads /points_raw, clusters points as obstacles,
publishes /detected_obstacles. Optional 2D Matplotlib visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from mini_autoware_sensor_pipeline.msg import DetectedObstacles, Obstacle
import struct
import math

try:
    import matplotlib
    matplotlib.use("Agg")  # non-interactive backend for ROS thread safety
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def cluster_points(xy_list, dist_threshold=0.5):
    """Simple distance-based clustering: points within dist_threshold form one obstacle."""
    if not xy_list:
        return []
    n = len(xy_list)
    parent = list(range(n))

    def find(i):
        if parent[i] != i:
            parent[i] = find(parent[i])
        return parent[i]

    def union(i, j):
        pi, pj = find(i), find(j)
        if pi != pj:
            parent[pi] = pj

    for i in range(n):
        for j in range(i + 1, n):
            if _dist(xy_list[i], xy_list[j]) <= dist_threshold:
                union(i, j)

    clusters = {}
    for i in range(n):
        root = find(i)
        if root not in clusters:
            clusters[root] = []
        clusters[root].append(xy_list[i])

    obstacles = []
    for pts in clusters.values():
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        obstacles.append((float(cx), float(cy)))
    return obstacles


class ConsumerNode(Node):
    """Perception simulation: point cloud -> obstacle detection -> /detected_obstacles."""

    def __init__(self):
        super().__init__("consumer_node")
        self.declare_parameter("cluster_dist", 0.5)
        self.declare_parameter("visualize", HAS_MATPLOTLIB)
        self.declare_parameter("viz_interval", 1.0)  # plot every N seconds

        self.sub = self.create_subscription(
            PointCloud2, "/points_raw", self.points_callback, 10
        )
        self.pub = self.create_publisher(
            DetectedObstacles, "/detected_obstacles", 10
        )
        self.get_logger().info(
            "Consumer node: /points_raw -> clustering -> /detected_obstacles"
        )

        self._last_xy = []
        self._last_obstacles = []
        self._viz_interval = self.get_parameter("viz_interval").value
        self._last_viz_time = 0.0
        if self.get_parameter("visualize").value and HAS_MATPLOTLIB:
            self._viz_timer = self.create_timer(
                self._viz_interval, self.visualization_callback
            )
        else:
            if not HAS_MATPLOTLIB:
                self.get_logger().info("Matplotlib not found; visualization disabled.")

    def points_callback(self, msg: PointCloud2):
        # Decode PointCloud2 (assume x,y,z float32 at 0,4,8)
        point_step = msg.point_step
        n = msg.width * msg.height
        xy = []
        for i in range(n):
            off = i * point_step
            x, y, z = struct.unpack_from("fff", msg.data, off)
            xy.append((x, y))
        self._last_xy = xy

        cluster_dist = self.get_parameter("cluster_dist").value
        obstacles = cluster_points(xy, cluster_dist)
        self._last_obstacles = obstacles

        out = DetectedObstacles()
        out.header = msg.header
        out.obstacles = [Obstacle(x=ox, y=oy) for (ox, oy) in obstacles]
        self.pub.publish(out)

    def visualization_callback(self):
        if not HAS_MATPLOTLIB or (not self._last_xy and not self._last_obstacles):
            return
        fig, ax = plt.subplots(figsize=(8, 8))
        if self._last_xy:
            xs = [p[0] for p in self._last_xy]
            ys = [p[1] for p in self._last_xy]
            ax.scatter(xs, ys, c="blue", s=2, alpha=0.6, label="LiDAR points")
        if self._last_obstacles:
            ox = [p[0] for p in self._last_obstacles]
            oy = [p[1] for p in self._last_obstacles]
            ax.scatter(ox, oy, c="red", s=80, marker="s", label="Detected obstacles")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("Perception: points_raw + detected_obstacles")
        ax.legend()
        ax.axis("equal")
        ax.grid(True)
        plt.savefig("/tmp/mini_autoware_viz.png", dpi=100)
        plt.close()
        self.get_logger().info("Saved visualization to /tmp/mini_autoware_viz.png")


def main(args=None):
    rclpy.init(args=args)
    node = ConsumerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
