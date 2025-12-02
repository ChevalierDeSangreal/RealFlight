from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch hover test node with neural network control for a single drone."""
    
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
            'tflite_model.yaml'
        ]),
        description='Path to the TFLite model configuration YAML file'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('hover_test'),
            'config',
            'hoverVer0_policy.tflite'
        ]),
        description='Path to the TFLite model file'
    )
    
    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id')
    config_file = LaunchConfiguration('config_file')
    model_path = LaunchConfiguration('model_path')
    
    # Hover test node with neural network control
    hover_test_node = Node(
        package='hover_test',
        executable='hover_test_node',
        name=['hover_test_node_', drone_id],
        namespace='',
        parameters=[
            config_file,
            {
                'drone_id': drone_id,
                'model_path': model_path,
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
        model_path_arg,
        hover_test_node,
    ])

