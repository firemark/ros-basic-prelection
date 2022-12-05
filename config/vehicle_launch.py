#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    y = LaunchConfiguration("y")
    vehicle_name = LaunchConfiguration("vehicle_name")
    ns = ["vehicle/", vehicle_name]

    with open("/config/bb1.urdf") as file:
        robot_desc = file.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_name", default_value="bb1"),
            DeclareLaunchArgument("y", default_value="0.0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=ns,
                parameters=[
                    {"robot_description": robot_desc},
                    {"frame_prefix": [vehicle_name, "/"]},
                ],
                output="both",
            ),
            Node(
                package="code_prelection",
                executable="vehicle",
                name="vehicle",
                namespace=ns,
                parameters=[
                    {"vehicle_name": vehicle_name},
                    {"y": y},
                ],
                output="both",
            ),
        ]
    )
