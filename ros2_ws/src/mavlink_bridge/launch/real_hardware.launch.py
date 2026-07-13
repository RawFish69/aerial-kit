from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'mavlink_bridge_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('mavlink_bridge'), 'config', 'mavlink_bridge_default.yaml']
        ),
    )
    # Real-hardware bringup deliberately omits telemetry_adapter_node and every
    # sim_bridge/sim_gazebo node -- mavlink_bridge_node plays both the
    # backend-adapter and telemetry-adapter roles when talking to a real FC.
    # To change the connection string/baud/thresholds, copy config/mavlink_bridge_default.yaml
    # and pass mavlink_bridge_params_file:=/path/to/your_config.yaml
    return LaunchDescription([
        params_file_arg,

        Node(
            package='mavlink_bridge',
            executable='mavlink_bridge_node',
            name='mavlink_bridge_node',
            output='screen',
            parameters=[
                LaunchConfiguration('mavlink_bridge_params_file'),
            ],
        ),
        Node(
            package='air_unit',
            executable='command_manager_node',
            name='command_manager_node',
            output='screen',
        ),
        Node(
            package='air_unit',
            executable='mission_executor_node',
            name='mission_executor_node',
            output='screen',
        ),
    ])
