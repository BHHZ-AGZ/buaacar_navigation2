from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='Path to the URDF model file'
    )

    urdf_path = PathJoinSubstitution(
        [FindPackageShare('buaahz_car_urdf_n10p'), 'urdf', 'buaahz_car_urdf_n10p.urdf']
    )
    # livox_lidar_launch_path = "/home/test/robot_ws/src/livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py"
    # cloud_to_scan_launch_path = "/home/test/robot_ws/src/cloud_to_scan/launch/launch/cloud_to_scan_launch.py"
    lslidar_n10p_launch_path = "/home/test/robot_ws/src/LSLIDAR_X_ROS2/src/lslidar_driver/launch/lslidar_double_launch.py"
    rf2o_laser_launch_path = "/home/test/robot_ws/src/rf2o_laser_odometry/launch/rf2o_laser_odometry.launch.py"
    robot_localization_launch_path = "/home/test/robot_ws/src/robot_localization/launch/ekf.launch.py"
    yesense_imu_launch_path = "/home/test/robot_ws/src/yesense_ros2/yesense_std_ros2/launch/yesense_node.launch.py"
    buaacar_navigation2_launch_path = "/home/test/robot_ws/src/buaacar_navigation2/launch/buaacar_nav2.launch.py"
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = ParameterValue(robot_description_content, value_type=str)


    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition('true')
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('buaahz_car_urdf'), 'urdf.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file]
    )
    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='ros2_laser_scan_matcher_node',
        output='screen',
    )
    uart_serial_comm_node = Node(
        package= 'uart_serial_comm',
        executable= 'uart_serial_comm',
        name= 'uart_serial_comm_node',
        output= 'screen',
    )
    ros2_beast_bridge_node = Node(
        package= 'ros2_beast_bridge',
        executable= 'http_bridge_server',
        name= 'http_bridge_server',
        output= 'screen',
    )
    laser_double_node = Node(
        package='laser_double',
        executable='laser_double',
        name='laser_double_node',
        output='screen',
    )
    # livox_lidar_launch_path = IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_lidar_launch_path))
    # cloud_to_scan_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(cloud_to_scan_launch_path))
    lslidar_n10p_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(lslidar_n10p_launch_path))
    rf2o_laser_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(rf2o_laser_launch_path))
    robot_localization_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_localization_launch_path))
    yesense_imu_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(yesense_imu_launch_path))
    buaacar_navigation2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(buaacar_navigation2_launch_path))
    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        ros2_beast_bridge_node,
        uart_serial_comm_node,
        yesense_imu_launch,
        # livox_lidar_launch_path,
        # cloud_to_scan_launch,
        lslidar_n10p_launch,
        laser_double_node,
        rf2o_laser_launch,
        robot_localization_launch,
        # buaacar_navigation2_launch
        # rviz_node
    ])