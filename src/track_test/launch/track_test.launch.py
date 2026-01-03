from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch track test node for a single drone.
    
    This node supports two target generation modes:
    1. Static mode (use_target_topic=false): Generate fixed target in front of drone
    2. Topic mode (use_target_topic=true): Subscribe to ROS2 topic for real-time target
    
    Configure the mode in config/tflite_model.yaml
    """
    
    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='Drone ID'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('track_test'),
            'config',
            'tflite_model.yaml'  # Updated to use tflite_model.yaml with target generation params
        ]),
        description='Path to the configuration YAML file'
    )
    
    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id')
    config_file = LaunchConfiguration('config_file')
    
    # Trajectory test node
    track_test_node = Node(
        package='track_test',
        executable='track_test_node',
        name=['track_test_node_', drone_id],
        namespace='',
        parameters=[config_file, {
            'drone_id': drone_id,
            'use_sim_time': True
        }],
        output='screen',
        emulate_tty=True,
        arguments=[drone_id]
    )
    
    return LaunchDescription([
        drone_id_arg,
        config_file_arg,
        track_test_node,
    ])