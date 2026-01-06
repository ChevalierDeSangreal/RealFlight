from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


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
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='onboard',
        choices=['onboard', 'sitl'],
        description='Operation mode: onboard (default) or sitl'
    )
    
    use_target_topic_arg = DeclareLaunchArgument(
        'use_target_topic',
        default_value='false',
        description='Use ROS2 topic for target (true) or static target (false)'
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
    
    return LaunchDescription([
        drone_id_arg,
        mode_arg,
        use_target_topic_arg,
        config_file_arg,
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    """Setup function to dynamically configure based on mode."""
    drone_id = LaunchConfiguration('drone_id')
    mode = LaunchConfiguration('mode')
    use_target_topic = LaunchConfiguration('use_target_topic')
    config_file = LaunchConfiguration('config_file')
    
    # Get the actual values from context
    mode_value = context.perform_substitution(mode)
    drone_id_value = context.perform_substitution(drone_id)
    use_target_topic_value = context.perform_substitution(use_target_topic)
    config_file_path = context.perform_substitution(config_file)
    
    # Convert use_target_topic string to boolean
    use_target_topic_bool = use_target_topic_value.lower() in ('true', '1', 'yes', 'on')
    
    # Set use_sim_time based on mode
    use_sim_time = (mode_value == 'sitl')
    
    # Load config file and prepare parameters
    with open(config_file_path, 'r') as f:
        config_data = yaml.safe_load(f)
    
    # The YAML key is '/**' not '**'
    yaml_key = '/**' if '/**' in config_data else '**'
    
    if yaml_key not in config_data or 'ros__parameters' not in config_data[yaml_key]:
        raise RuntimeError(f"Invalid YAML structure in config file: {config_file_path}")
    
    ros_params = config_data[yaml_key]['ros__parameters'].copy()
    
    # In SITL mode, replace model_path with model_path_sitl
    if mode_value == 'sitl':
        if 'model_path_sitl' not in ros_params:
            raise RuntimeError(f"model_path_sitl not found in config file: {config_file_path}")
        ros_params['model_path'] = ros_params.pop('model_path_sitl')  # Replace model_path with model_path_sitl
    
    # Override use_target_topic if provided via launch argument
    ros_params['use_target_topic'] = use_target_topic_bool
    
    # Add other parameters
    ros_params['drone_id'] = drone_id_value
    ros_params['use_sim_time'] = use_sim_time
    
    # Prepare node parameters - use dictionary directly instead of YAML file
    node_parameters = [ros_params]
    
    # Trajectory test node
    track_test_node = Node(
        package='track_test',
        executable='track_test_node',
        name=['track_test_node_', drone_id],
        namespace='',
        parameters=node_parameters,
        output='screen',
        emulate_tty=True,
        arguments=[drone_id]
    )
    
    return [track_test_node]