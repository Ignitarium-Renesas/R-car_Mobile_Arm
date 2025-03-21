import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    res = []
    pick_place_node = Node(
        package="rcar_communication",
        executable="rcar_communication_node",
        name="rcar_communication_node",
        output="screen"
    )
    res.append(pick_place_node)

    return LaunchDescription(res)
