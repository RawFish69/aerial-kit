"""Keyboard-fly a MuJoCo multirotor (quad / hex / octo) through ROS.

    ros2 launch sim_fast multirotor_mujoco_teleop.launch.py airframe:=fpv_quad_5in

Starts the MuJoCo multirotor plant (on the ground by default, viewer on) and
the Tk key window. No mission or planner. Until Enter is pressed, and when
manual control is released or goes stale, the plant holds position in the
air or idles on the ground. Closing the teleop window ends the session.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'airframe', default_value='fpv_quad_5in',
            description='fpv_quad_5in | micro_lr_quad | research_hex | research_octo, or a spec YAML path'),
        DeclareLaunchArgument(
            'mode', default_value='auto',
            description='auto (FPV airframes: angle, research: velocity) | velocity | angle | acro'),
        DeclareLaunchArgument('viewer', default_value='true',
                              description='Open the MuJoCo viewer (falls back to headless).'),
        DeclareLaunchArgument('start_z_m', default_value='0.0',
                              description='0 starts resting on the ground; > 0.3 starts hovering.'),
        DeclareLaunchArgument('wind_x_mps', default_value='0.0'),
        DeclareLaunchArgument('wind_y_mps', default_value='0.0'),
        DeclareLaunchArgument('manual_timeout_sec', default_value='0.30'),
    ]
    plant = Node(
        package='sim_bridge',
        executable='multirotor_backend_adapter_node',
        name='multirotor_backend_adapter_node',
        output='screen',
        parameters=[{
            'airframe': LaunchConfiguration('airframe'),
            'viewer': LaunchConfiguration('viewer'),
            'start_z_m': LaunchConfiguration('start_z_m'),
            'wind_x_mps': LaunchConfiguration('wind_x_mps'),
            'wind_y_mps': LaunchConfiguration('wind_y_mps'),
            'manual_timeout_sec': LaunchConfiguration('manual_timeout_sec'),
            'manual_topic': '/uav/backend/manual',
        }],
    )
    teleop = Node(
        package='uav_control',
        executable='multirotor_teleop_node',
        name='multirotor_teleop_node',
        output='screen',
        parameters=[{
            'airframe': LaunchConfiguration('airframe'),
            'mode': LaunchConfiguration('mode'),
            'manual_topic': '/uav/backend/manual',
            'enable_topic': '/uav/backend/enable',
        }],
    )
    shutdown_on_teleop_exit = RegisterEventHandler(OnProcessExit(
        target_action=teleop,
        on_exit=[
            LogInfo(msg='teleop window closed; shutting down the multirotor session'),
            EmitEvent(event=Shutdown(reason='teleop exited')),
        ],
    ))
    return LaunchDescription(args + [plant, teleop, shutdown_on_teleop_exit])
