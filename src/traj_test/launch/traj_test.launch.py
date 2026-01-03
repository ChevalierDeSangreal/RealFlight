from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch trajectory test node for a single drone."""
    
    drone_id_env = EnvironmentVariable('DRONE_ID', default_value='0')
    
    # Declare launch arguments
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
            FindPackageShare('traj_test'),
            'config',
            'traj_params.yaml'
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
    drone_id = LaunchConfiguration('drone_id')
    mode = LaunchConfiguration('mode')
    config_file = LaunchConfiguration('config_file')
    
    # Get the actual mode value from context
    mode_value = context.perform_substitution(mode)
    
    # Set use_sim_time based on mode
    use_sim_time = (mode_value == 'sitl')
    
    # Trajectory test node
    traj_test_node = Node(
        package='traj_test',
        executable='traj_test_node',
        name=['traj_test_node_', drone_id],
        namespace='',
        parameters=[
            config_file,  # Load parameters from YAML config file
            {
                'drone_id': drone_id,
                'use_sim_time': use_sim_time,  # Override use_sim_time based on mode
            }
        ],
        output='screen',
        emulate_tty=True,
        arguments=[drone_id]
    )
    
    return [traj_test_node]