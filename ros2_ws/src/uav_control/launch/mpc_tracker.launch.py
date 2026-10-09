"""Launch the predictive mission tracker.

    ros2 launch uav_control mpc_tracker.launch.py                   # config/mpc_tracker.yaml
    ros2 launch uav_control mpc_tracker.launch.py controller:=mppi  # config/mppi_tracker.yaml
    ros2 launch uav_control mpc_tracker.launch.py params_file:=/path/to/yours.yaml

It publishes to the same command topic as air_unit's mission_executor_node, so
start one or the other (e.g. bringup with start_air_unit:=false and launch the
command manager and telemetry adapter yourself), not both.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='mpc',
        description='mpc | mppi - picks config/<controller>_tracker.yaml as the default params file',
    )
    default_params = PathJoinSubstitution([
        FindPackageShare('uav_control'), 'config', [LaunchConfiguration('controller'), '_tracker.yaml'],
    ])
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Parameter file for mpc_tracker_node',
    )
    node = Node(
        package='uav_control',
        executable='mpc_tracker_node',
        name='mpc_tracker_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )
    return LaunchDescription([controller_arg, params_arg, node])
