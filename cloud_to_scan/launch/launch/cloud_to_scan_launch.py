# launch/cloud_to_scan_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloud_to_scan',
            executable='cloud_to_scan_node',
            name='cloud_to_scan',
            output='screen',
            parameters=[
                {
                    'cloud_topic': '/unilidar/cloud',
                    'scan_topic': '/scan',
                    'target_frame': 'base_link',  # 修改：使用base_link作为目标坐标系
                    'min_height': 0.0,
                    'max_height': 2.0,
                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'angle_increment': 0.0087,
                    'scan_time': 0.1,
                    'range_min': 0.3,
                    'range_max': 100.0,
                    'z_projection_min': 0.5,  # 新增：Z轴投影最小高度0.05
                    'z_projection_max': 29.0,   # 新增：Z轴投影最大高度
                    'outlier_mean_k': 0,      # 新增：离群点过滤参数
                    'outlier_stddev': 10.0      # 新增：离群点过滤参数
                }
            ]
        )
    ])