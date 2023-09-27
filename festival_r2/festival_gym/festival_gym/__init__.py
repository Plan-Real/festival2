# -*- coding: utf-8 -*-
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    target_frame = DeclareLaunchArgument(
        "target_frame",
        default_value="base_link",
        description="Target frame for the robot",
    )

    featival_sim = Node(
        package="festival_gym",
        executable="festival_gym_node",
        name="festival_gym_node",
        output="screen",
        parameters=[{"target_frame": LaunchConfiguration("target_frame")}],
    )

    festival_brod_test = Node(
        package="fesetival_gym",
        executable="festival_gym_node",
        name="static_tf2_broadcaster",
        output="screen",
        parameters=["0.0", "0.3", "0.3", "0", "0", "0", "1", "world", "mystaticturtle"],
    )

    ld = LaunchDescription()
    ld.add_action(featival_sim)
    ld.add_action(festival_brod_test)

    return ld
