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
    myagv_urdf_path = get_package_share_path('myagv_urdf')
    default_model_path = myagv_urdf_path / 'urdf/myAGV.urdf'
    default_rviz_config_path = myagv_urdf_path / 'rviz/urdf.rviz'
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')


    # Start robot localization using an Extended Kalman filter
    robot_localization_file_path = myagv_odometry_path / 'cfg/localization_ekf.yaml'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    
    driver_node = Node(
        package='myagv_odometry',
        executable='myagv_odometry_node',
        name="myagv_odometry_node",
        output="screen"
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('imu_filter_madgwick'), '/launch/imu_filter.launch.py']),
    )
    localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
        )
    accessories_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ydlidar_ros2_driver'), '/launch/ydlidar_launch.py']),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        declare_use_sim_time_argument,
        joint_state_publisher_node,
        robot_state_publisher_node,
        driver_node,
        imu_launch,
        localization_node,
        accessories_launch,
        #rviz_node
    ])