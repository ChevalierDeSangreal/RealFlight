from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def generate_launch_description():
    """Launch hover test node (50Hz) with neural network control for a single drone."""
    
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
    
    return LaunchDescription([
        drone_id_arg,
        mode_arg,
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    """Setup function to dynamically configure based on mode."""
    drone_id = LaunchConfiguration('drone_id')
    mode = LaunchConfiguration('mode')
    
    # Get the actual mode value from context
    mode_value = context.perform_substitution(mode)
    
    # Get package share directory
    package_share_dir = context.perform_substitution(FindPackageShare('hover_test'))
    config_file_path = os.path.join(package_share_dir, 'config', 'tflite_model_50hz.yaml')
    
    # Use the same config file for both modes
    config_file = PathJoinSubstitution([
        FindPackageShare('hover_test'),
        'config',
        'tflite_model_50hz.yaml'
    ])
    
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
    
    # Add other parameters
    ros_params['drone_id'] = drone_id
    ros_params['use_sim_time'] = use_sim_time
    
    # Prepare node parameters - use dictionary directly instead of YAML file
    node_parameters = [ros_params]
    
    # Hover test node with neural network control (50Hz version)
    hover_test_node_50hz = Node(
        package='hover_test',
        executable='hover_test_node_50hz',
        name=['hover_test_node_50hz_', drone_id],
        namespace='',
        parameters=node_parameters,
        output='screen',
        emulate_tty=True,
        arguments=[drone_id]
    )
    
    return [hover_test_node_50hz]

