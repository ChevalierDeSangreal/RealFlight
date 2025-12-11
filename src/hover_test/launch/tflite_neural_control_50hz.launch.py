from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch hover test node (50Hz) with neural network control for a single drone."""
    
    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='Drone ID'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('hover_test'),
            'config',
            'tflite_model_50hz.yaml'
        ]),
        description='Path to the TFLite model configuration YAML file for 50Hz control'
    )
    
    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id')
    config_file = LaunchConfiguration('config_file')
    
    # Hover test node with neural network control (50Hz version)
    hover_test_node_50hz = Node(
        package='hover_test',
        executable='hover_test_node_50hz',
        name=['hover_test_node_50hz_', drone_id],
        namespace='',
        parameters=[
            config_file,
            {
                'drone_id': drone_id,
                'use_sim_time': True,
            }
        ],
        output='screen',
        emulate_tty=True,
        arguments=[drone_id]
    )
    
    return LaunchDescription([
        drone_id_arg,
        config_file_arg,
        hover_test_node_50hz,
    ])

