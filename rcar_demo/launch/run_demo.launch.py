import os

# from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.conditions import IfCondition, UnlessCondition
# from launch.substitutions import Command, LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    res = []
    
    package_name = "rcar_communication"
    pick_launch_path = os.path.join(FindPackageShare(package_name).find(package_name), "launch", "pick_service.launch.py")
    pick_service_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pick_launch_path),
        )   
    res.append(pick_service_launch)

    pose_est_node = Node(
        package="pose_estimation_pkg",
        executable="camera_pose_srv",
        name="camera_pose_srv",
        output="screen"
    )

    res.append(pose_est_node)

    demo_node = Node(
        package="rcar_demo",
        executable="task_node",
        name="rcar_demo_node",
        output="screen"
    )
    res.append(demo_node)

    return LaunchDescription(res)
