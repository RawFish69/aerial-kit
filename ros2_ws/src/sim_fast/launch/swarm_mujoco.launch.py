"""N multirotors in one MuJoCo scene, flown in formation.

    ros2 launch sim_fast swarm_mujoco.launch.py n:=12 [viewer:=true] [demo:=true]

Drone i is served on /<prefix><i>/backend/{cmd_twist,enable,odom}, the
single-vehicle contract. The formation node takes /swarm/formation commands
(takeoff [z] | grid [z] | circle [z] | line [z] | land) and reports on /swarm/status;
demo:=true runs takeoff -> circle -> line -> grid -> land by itself.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    n = ParameterValue(LaunchConfiguration('n'), value_type=int)
    prefix = LaunchConfiguration('namespace_prefix')
    spacing = ParameterValue(LaunchConfiguration('spacing_m'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='4', description='number of drones (1..64)'),
        DeclareLaunchArgument('airframe', default_value='swarm_quad',
                              description='any multirotor spec: swarm_quad | fpv_quad_5in | micro_lr_quad | ...'),
        DeclareLaunchArgument('spacing_m', default_value='1.5'),
        DeclareLaunchArgument('namespace_prefix', default_value='uav'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('demo', default_value='false', description='fly the formation demo by itself'),
        Node(
            package='sim_bridge', executable='swarm_backend_adapter_node', name='swarm_backend_adapter_node',
            output='screen',
            parameters=[{'n': n, 'airframe': LaunchConfiguration('airframe'), 'spacing_m': spacing,
                         'namespace_prefix': prefix,
                         'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool)}],
        ),
        Node(
            package='sim_bridge', executable='swarm_formation_node', name='swarm_formation_node',
            output='screen',
            parameters=[{'n': n, 'namespace_prefix': prefix, 'spacing_m': spacing,
                         'demo': ParameterValue(LaunchConfiguration('demo'), value_type=bool)}],
        ),
    ])
