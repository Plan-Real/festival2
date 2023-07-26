import launch
import os
import sys
import yaml
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch

    # kinematics_parameters = get_kinematics_parameters()
    demo_node = Node(
        package="festival_ur3_core_node",
        executable="festival_ur_core",
        name="festival_ur3_core",
        output="screen",
    )

    return launch.LaunchDescription([demo_node])
