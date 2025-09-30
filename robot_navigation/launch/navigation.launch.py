from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    # 声明地图文件路径参数
    map_yaml_path_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value='/home/test/robot_ws/src/robot_navigation/maps/buaahz_2_3.yaml',
        description=''
    )
    
    # 是否使用rviz可视化
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    # 地图加载节点
    map_loader_node = Node(
        package='robot_navigation',
        executable='map_loader_node',
        name='map_loader_node',
        output='screen',
        parameters=[{
            'map_yaml_path': LaunchConfiguration('map_yaml_path')
        }]
    )
    
    # A*路径规划节点
    path_planner_node = Node(
        package='robot_navigation',
        executable='path_planner_node',
        name='a_star_path_planner_node',
        output='screen',
        parameters=[
            {'base_length': 0.55},  # 实际底盘长度（米）
            {'base_width': 0.55}    # 实际底盘宽度（米）
        ]
    )
    
    # PID控制节点
    pid_controller_node = Node(
        package='robot_navigation',
        executable='position_controller_node',
        name='diff_drive_position_controller',
        output='screen',
        parameters=[
            # 位置环PID参数（根据实际底盘调整）
            {'position.kp_rho': 0.8},
            {'position.kp_alpha': 2.0},
            {'position.kp_beta': -0.5},
            
            # 速度环PID参数
            {'velocity.kp_linear': 0.5},
            {'velocity.ki_linear': 0.1},
            {'velocity.kd_linear': 0.05},
            {'velocity.kp_angular': 0.8},
            {'velocity.ki_angular': 0.2},
            {'velocity.kd_angular': 0.1},
            
            # 底盘参数（根据你的实际底盘尺寸修改）
            {'base.wheel_base': 0.4},  # 轮距（左右轮距离，单位：米）
            {'base.max_linear_vel': 0.5},  # 最大线速度（米/秒）
            {'base.max_angular_vel': 1.5}, # 最大角速度（弧度/秒）
            
            # 控制精度
            {'control.pos_tolerance': 0.05},  # 位置误差容忍度（米）
            {'control.angle_tolerance': 0.05} # 角度误差容忍度（弧度）
        ]
    )
    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        map_yaml_path_arg,
        use_rviz_arg,
        map_loader_node,
        path_planner_node,
        pid_controller_node,
        rviz_node
    ])
