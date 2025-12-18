#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    driver1_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params','lidar_uart_ros2', 'laser1.yaml')
    driver2_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params','lidar_uart_ros2', 'laser2.yaml')
    
    driver1_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver1_dir],
                                )
    driver2_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver2_dir],
                                )
    return LaunchDescription([
        driver1_node,
        driver2_node,
    ])