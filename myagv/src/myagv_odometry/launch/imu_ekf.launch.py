from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    myagv_odometry_path = get_package_share_path('myagv_odometry')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('imu_filter_madgwick'), '/launch/imu_filter.launch.py']),
    )
    # Start robot localization using an Extended Kalman filter
    robot_localization_file_path = myagv_odometry_path / 'cfg/localization_ekf.yaml'
    localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
        )
    
    return LaunchDescription([
        declare_use_sim_time_argument,
        imu_launch,
        localization_node,
    ])