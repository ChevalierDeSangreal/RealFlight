from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch vicon_to_target_node to convert PX4 VehicleOdometry topics to target topics.
    
    This node subscribes to PX4 VehicleOdometry messages
    and publishes target position and velocity topics for track_test_node.
    """
    
    # Declare launch arguments
    px4_topic_name_arg = DeclareLaunchArgument(
        'px4_topic_name',
        default_value='/px4_3/fmu/out/vehicle_odometry',
        description='PX4 VehicleOdometry topic name (e.g., /px4_3/fmu/out/vehicle_odometry)'
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
            'px4_topic_name': LaunchConfiguration('px4_topic_name'),
            'velocity_calc_window': LaunchConfiguration('velocity_calc_window'),
            'max_history_size': LaunchConfiguration('max_history_size'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        px4_topic_name_arg,
        velocity_calc_window_arg,
        max_history_size_arg,
        vicon_to_target_node,
    ])

