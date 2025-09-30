from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtk_odom_fusion_cpp',
            executable='complementary_filter_node',
            name='complementary_filter',
            output='screen',
            parameters=[{
                'alpha': 0.02,              # 互补滤波系数
                'gps_min_accuracy': 0.1,    # GPS最小精度(m)
                'gps_max_accuracy': 5.0,    # GPS最大精度(m)
                'publish_tf': True,         # 发布TF变换
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'use_2d': True,             # 使用2D数据
            }]
        )
    ])