from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """启动目标物体位置发布节点 - 发布圆周运动轨迹"""
    
    # 声明 launch 参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('track_test'),
            'config',
            'target_publisher_params.yaml'
        ]),
        description='目标发布节点的配置文件路径'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='是否使用仿真时间 (true 表示 SITL 模式, false 表示实机模式)'
    )
    
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 目标物体位置发布节点
    target_publisher_node = Node(
        package='track_test',
        executable='target_publisher_node',
        name='target_publisher_node',
        namespace='',
        parameters=[
            config_file,  # 从 YAML 配置文件加载参数
            {
                'use_sim_time': use_sim_time,  # 根据模式设置仿真时间
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        target_publisher_node,
    ])

