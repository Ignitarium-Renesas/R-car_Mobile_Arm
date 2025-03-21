from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    res = []

    
    camera_pose_node = Node(
        package="pose_estimation_pkg",
        executable="camera_pose",
        name="camera_pose",
        output="screen"
    )
    res.append(camera_pose_node)

    return LaunchDescription(res)
