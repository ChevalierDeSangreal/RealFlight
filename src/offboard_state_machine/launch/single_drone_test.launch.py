#!/usr/bin/env python3
"""
Single drone launch file for offboard FSM
Launches one drone that takes off to 1.5m and hovers at the takeoff position
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
import yaml

def generate_launch_description() -> LaunchDescription:
    """
    Launch a single drone with FSM node.
    The drone will:
    1. Take off to 1.5m above current position (0, 0, -1.5 in NED)
    2. Hover at the takeoff position (goto target same as takeoff target)
    """
    
    # Single drone configuration
    drone_id_env = EnvironmentVariable('DRONE_ID', default_value='0')
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value=drone_id_env,
        description='Drone ID'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='onboard',
        choices=['onboard', 'sitl'],
        description='Operation mode: onboard (default) or sitl'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('offboard_state_machine'),
            'config',
            'fsm_hover.yaml'
        ]),
        description='Path to the configuration YAML file'
    )
    
    return LaunchDescription([
        drone_id_arg,
        mode_arg,
        config_file_arg,
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    """Setup function to dynamically configure based on mode."""
    drone_id_config = LaunchConfiguration('drone_id')
    mode = LaunchConfiguration('mode')
    config_file = LaunchConfiguration('config_file')
    
    # Get the actual values from context
    mode_value = context.perform_substitution(mode)
    config_file_path = context.perform_substitution(config_file)
    drone_id_value = context.perform_substitution(drone_id_config)
    
    # Expand ~ to home directory
    config_file_path = os.path.expanduser(config_file_path)
    
    # Set use_sim_time based on mode
    use_sim_time = (mode_value == 'sitl')
    
    # Load config file and prepare parameters
    if not os.path.exists(config_file_path):
        raise RuntimeError(f"Config file not found: {config_file_path}")
    
    with open(config_file_path, 'r') as f:
        config_data = yaml.safe_load(f)
    
    # The YAML key is '/**' not '**'
    yaml_key = '/**' if '/**' in config_data else '**'
    
    if yaml_key not in config_data or 'ros__parameters' not in config_data[yaml_key]:
        raise RuntimeError(f"Invalid YAML structure in config file: {config_file_path}")
    
    ros_params = config_data[yaml_key]['ros__parameters'].copy()
    
    # Override drone_id and use_sim_time from launch arguments
    ros_params['drone_id'] = int(drone_id_value)
    ros_params['use_sim_time'] = use_sim_time
    
    fsm_node = Node(
        package="offboard_state_machine",
        executable="offboard_fsm_node",
        name=["offboard_fsm_node_", drone_id_config],
        output="screen",
        parameters=[ros_params],
    )

    return [fsm_node]