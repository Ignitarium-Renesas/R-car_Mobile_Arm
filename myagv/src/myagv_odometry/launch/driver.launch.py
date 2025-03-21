
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    
    driver_node = Node(
        package='myagv_odometry',
        executable='myagv_odometry_node',
        name="myagv_odometry_node",
        output="screen"
    )
    accessories_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ydlidar_ros2_driver'), '/launch/ydlidar_launch.py']),
    )

    return LaunchDescription([
        driver_node,
        accessories_launch,
    ])