from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_to_utm',
            executable='utm_odometry_node',
            name='utm_transformer',
            output='screen',
            parameters=[{
                'antenna_offset_x': 0.0,      # RTK天线在机器人坐标系中的X偏移(m)
                'antenna_offset_y': 0.0,      # RTK天线在机器人坐标系中的Y偏移(m)
                'antenna_yaw': 0.0,           # RTK天线安装偏航角(rad)
                'auto_set_origin': True,      # 是否自动设置原点
                'manual_origin_lat': 0.0,     # 手动设置原点纬度
                'manual_origin_lon': 0.0,     # 手动设置原点经度
                'publish_tf': False,          # 是否发布TF变换
                'utm_zone': 0,                # UTM区域(0=自动检测)
                'northp': True,               # 是否北半球
                'covariance_scale': 1.0,      # 协方差缩放因子
                'min_accuracy': 0.05,         # 最小精度阈值(m)
                'max_accuracy': 5.0,          # 最大精度阈值(m)
            }]
        ),
    ])