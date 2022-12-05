#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                name="rviz",
                cmd=["ros2", "run", "rviz2", "rviz2", "-d", "/config/rviz.rviz"],
                output="log",
            ),
            ExecuteProcess(
                name="plotjugger",
                cmd=["ros2", "run", "plotjuggler", "plotjuggler"],
                output="log",
            ),
            ExecuteProcess(
                name="obstacles",
                cmd=["ros2", "run", "code_prelection", "obstacles"],
                output="log",
            ),
        ]
    )
