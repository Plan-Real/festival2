# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_file_name = "warehouse.world"
    world = os.path.join(
        get_package_share_directory("gole_gazebo"), "worlds", world_file_name
    )

    urdf_path = os.path.join(
        get_package_share_directory("gole_gazebo"),
        "models",
        "gole",
        "model.sdf",
    )

    festival_gym_params = LaunchConfiguration(
        "festival_gym_params",
        default=os.path.join(
            get_package_share_directory("festival_gym"), "configs", "festival_gym.yaml"
        ),
    )

    festival_gym = Node(
        package="festival_gym",
        executable="festival_gym_node",
        name="festival_gym_node",
        output="screen",
        parameters=[festival_gym_params],
        namespace="",
    )

    ld = LaunchDescription()

    ld.add_action(festival_gym)

    return ld
