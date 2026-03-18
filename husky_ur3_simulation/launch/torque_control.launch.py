#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "python3",
                    "/home/burak/ur3_ws/src/husky_ur3_simulation/scripts/torque_control.py",
                ],
                output="screen",
            )
        ]
    )
