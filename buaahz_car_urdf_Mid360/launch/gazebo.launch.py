from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # 包含Gazebo启动文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'empty_world.launch.py'])
        ]),
        launch_arguments={'world': '', 'gui': 'true'}.items()
    )

    # TF静态变换发布器 (base_link -> base_footprint)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # 获取URDF文件路径
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('buaahz_car_urdf'), 'urdf', 'buaahz_car_urdf.urdf']
    )

    # 在Gazebo中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        output='screen',
        arguments=[
            '-entity', 'buaahz_car_urdf',
            '-file', urdf_path,
            '-robot_namespace', 'buaahz_car_urdf'
        ]
    )

    # 发布/joint_states话题的节点 (ROS 2中替代rostopic pub)
    fake_joint_calibration = Node(
        package='topic_tools',
        executable='relay',
        name='fake_joint_calibration',
        parameters=[{'input_topic': '/calibrated', 'output_topic': '/calibrated'}],
        remappings=[('/calibrated', '/calibrated')]
    )

    # 延迟发布/calibrated消息，等待Gazebo启动完成
    publish_calibrated = ExecuteProcess(
        cmd=[
            [FindExecutable(name='ros2'), 'topic pub -1 /calibrated std_msgs/msg/Bool "data: true"']
        ],
        shell=True
    )

    # 在spawn_entity进程启动后发布/calibrated消息
    delayed_publish = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[publish_calibrated]
        )
    )

    return LaunchDescription([
        gazebo,
        static_tf_publisher,
        spawn_entity,
        delayed_publish
    ])