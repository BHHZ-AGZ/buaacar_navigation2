from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtk_odom_fusion_cpp',
            executable='complementary_filter_node',
            name='complementary_filter_fusion',
            parameters=[{'alpha': 0.05}],
            output='screen'
        )
    ])