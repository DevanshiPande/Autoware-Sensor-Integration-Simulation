#!/usr/bin/env python3
"""Launch the full Mini Autoware sensor pipeline: sensor -> driver -> consumer -> control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("visualize", default_value="true"),
        DeclareLaunchArgument("rate_hz", default_value="10.0"),
        Node(
            package="mini_autoware_sensor_pipeline",
            executable="sensor_node.py",
            name="sensor_node",
            output="screen",
            parameters=[{"rate_hz": LaunchConfiguration("rate_hz")}],
        ),
        Node(
            package="mini_autoware_sensor_pipeline",
            executable="driver_node.py",
            name="driver_node",
            output="screen",
        ),
        Node(
            package="mini_autoware_sensor_pipeline",
            executable="consumer_node.py",
            name="consumer_node",
            output="screen",
            parameters=[{"visualize": LaunchConfiguration("visualize")}],
        ),
        Node(
            package="mini_autoware_sensor_pipeline",
            executable="control_node.py",
            name="control_node",
            output="screen",
        ),
    ])
