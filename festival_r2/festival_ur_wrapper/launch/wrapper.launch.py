# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="festival_ur_wrapper",
                executable="festival_ur_wrapper",
                name="festival_ur_wrapper",
                output="screen",
                emulate_tty=True,
                parameters=[{"my_parameter": "earth"}],
            )
        ]
    )
