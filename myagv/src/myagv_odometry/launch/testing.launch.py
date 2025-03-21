from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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

    declare_bag_file_argument = DeclareLaunchArgument(
        'bag_file_path',
        default_value='/home/mbasith/myagv_ws/out_bag',
        description='Path to the ROS2 bag file to play')

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('myagv_urdf'), '/launch/display_myagv.launch.py']),
    )
    bag_file_path = LaunchConfiguration('bag_file_path')
    ros2_bag_play_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_path],
        output='screen'
    )
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

    # Add ros2 bag play node
    
    

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_bag_file_argument,
        ros2_bag_play_process,
        imu_launch,
        localization_node,
        tf_launch,
    ])
