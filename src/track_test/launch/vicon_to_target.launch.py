from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch vicon_to_target_node to convert Vicon topics to target topics.
    
    This node subscribes to Vicon position data (PoseStamped or TransformStamped)
    and publishes target position and velocity topics for track_test_node.
    """
    
    # Declare launch arguments
    vicon_topic_name_arg = DeclareLaunchArgument(
        'vicon_topic_name',
        default_value='/vicon/pose',
        description='Vicon topic name to subscribe (e.g., /vicon/drone1/pose)'
    )
    
    vicon_topic_type_arg = DeclareLaunchArgument(
        'vicon_topic_type',
        default_value='PoseStamped',
        choices=['PoseStamped', 'TransformStamped'],
        description='Vicon topic type: PoseStamped or TransformStamped'
    )
    
    velocity_calc_window_arg = DeclareLaunchArgument(
        'velocity_calc_window',
        default_value='0.1',
        description='Time window for velocity calculation [s]'
    )
    
    max_history_size_arg = DeclareLaunchArgument(
        'max_history_size',
        default_value='50',
        description='Maximum size of position history for velocity calculation'
    )
    
    # Create node
    vicon_to_target_node = Node(
        package='track_test',
        executable='vicon_to_target_node',
        name='vicon_to_target_node',
        parameters=[{
            'vicon_topic_name': LaunchConfiguration('vicon_topic_name'),
            'vicon_topic_type': LaunchConfiguration('vicon_topic_type'),
            'velocity_calc_window': LaunchConfiguration('velocity_calc_window'),
            'max_history_size': LaunchConfiguration('max_history_size'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        vicon_topic_name_arg,
        vicon_topic_type_arg,
        velocity_calc_window_arg,
        max_history_size_arg,
        vicon_to_target_node,
    ])

