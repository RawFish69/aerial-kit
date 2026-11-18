from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_monitor = DeclareLaunchArgument('start_monitor', default_value='false')
    start_delay = DeclareLaunchArgument('start_delay_sec', default_value='3.0')

    return LaunchDescription([
        start_monitor,
        start_delay,
        Node(
            package='sim_bridge',
            executable='fixedwing_backend_adapter_node',
            name='fixedwing_backend_adapter_node',
            output='screen',
            parameters=[{
                'start_z_m': 55.0,
                'start_speed_mps': 15.0,
                'cruise_airspeed_mps': 15.0,
            }],
        ),
        Node(
            package='uav_control',
            executable='fixed_wing_guidance_node',
            name='fixed_wing_guidance_node',
            output='screen',
        ),
        Node(
            package='sim_fast',
            executable='fixed_wing_demo_node',
            name='fixed_wing_demo_node',
            output='screen',
            parameters=[{
                'start_delay_sec': LaunchConfiguration('start_delay_sec'),
            }],
        ),
        Node(
            package='ground_station',
            executable='ground_station_telemetry_monitor',
            name='ground_station_telemetry_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_monitor')),
        ),
    ])
