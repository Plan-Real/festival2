# -*- coding: utf-8 -*-
# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detect_3d_node_cmd = Node(
        package="festival_ur_yolo",
        executable="detect_3d_node",
        name="detect_3d_node",
        parameters=[
            {"target_frame": "base_link"},
        ],
        remappings=[
            ("points", "/camera/depth/color/points"),
            ("detections", "/yolo/tracking"),
            ("detections_3d", "/yolo/detections_3d"),
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(detect_3d_node_cmd)
    return ld
