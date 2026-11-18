"""Keyboard-fly the MuJoCo twin wing through ROS (actuator level).

    ros2 launch sim_fast fixed_wing_mujoco_teleop.launch.py

Starts the MuJoCo plant (airborne at cruise, viewer on by default) and the
Tk key window. There is no mission or guidance node: until Enter is pressed
in the teleop window, and whenever manual control is released or goes stale,
the plant flies its built-in safe cruise circle. Closing the teleop window
(or Esc) shuts the whole launch down.
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
            'viewer', default_value='true',
            description='Open the MuJoCo viewer (chase camera). Without a '
                        'display the plant keeps running headless.'),
        DeclareLaunchArgument(
            'airframe', default_value='twin_wing',
            description='twin_wing | single_wing (single-motor pusher flying wing).'),
        DeclareLaunchArgument(
            'airframe_spec', default_value='',
            description='Twin-wing airframe YAML; empty uses the bundled spec.'),
        DeclareLaunchArgument(
            'start_z_m', default_value='60.0',
            description='Start altitude. The wing starts airborne: it cannot '
                        'hover or take off vertically.'),
        DeclareLaunchArgument(
            'start_speed_mps', default_value='15.0',
            description='Start ground speed along the start heading (cruise).'),
        DeclareLaunchArgument(
            'wind_x_mps', default_value='0.0',
            description='World-frame east wind component.'),
        DeclareLaunchArgument(
            'wind_y_mps', default_value='0.0',
            description='World-frame north wind component.'),
        DeclareLaunchArgument(
            'flight_controller', default_value='sim',
            description="sim | firmware: the AK flight core flies it (angle mode). Enter arms "
                        "it in the hand, throttle up throws it from start_z_m."),
        DeclareLaunchArgument(
            'firmware_accel_model', default_value='physical',
            description="firmware only: physical | gravity (test instrument; see "
                        "docs/evidence/fc-sil-wing-estimator-finding-2026-11-18.md)"),
        DeclareLaunchArgument(
            'manual_timeout_sec', default_value='0.30',
            description='Manual commands older than this fall back to the '
                        'safe cruise circle.'),
    ]

    plant = Node(
        package='sim_bridge',
        executable='fixedwing_backend_adapter_node',
        name='fixedwing_backend_adapter_node',
        output='screen',
        parameters=[{
            'backend': 'mujoco',
            'start_z_m': LaunchConfiguration('start_z_m'),
            'start_speed_mps': LaunchConfiguration('start_speed_mps'),
            'cruise_airspeed_mps': 15.0,
            'wind_x_mps': LaunchConfiguration('wind_x_mps'),
            'wind_y_mps': LaunchConfiguration('wind_y_mps'),
            'mujoco_timestep': 0.002,
            'mujoco_thrust_tau_s': 0.03,
            'mujoco_viewer': LaunchConfiguration('viewer'),
            'airframe_spec': LaunchConfiguration('airframe_spec'),
            'airframe': LaunchConfiguration('airframe'),
            'flight_controller': LaunchConfiguration('flight_controller'),
            'firmware_accel_model': LaunchConfiguration('firmware_accel_model'),
            'manual_command_topic': '/uav/backend/cmd_actuators',
            'manual_timeout_sec': LaunchConfiguration('manual_timeout_sec'),
            'estimator_mode': 'truth',
        }],
    )

    teleop = Node(
        package='uav_control',
        executable='fixed_wing_teleop_node',
        name='fixed_wing_teleop_node',
        output='screen',
        parameters=[{
            'actuator_topic': '/uav/backend/cmd_actuators',
            'enable_topic': '/uav/backend/enable',
            'airframe_spec': LaunchConfiguration('airframe_spec'),
            'airframe': LaunchConfiguration('airframe'),
            'cruise_airspeed_mps': 15.0,
        }],
    )

    # Teleop exit (Esc, window closed, or no display) ends the session, so
    # no plant is left flying without a pilot window.
    shutdown_on_teleop_exit = RegisterEventHandler(OnProcessExit(
        target_action=teleop,
        on_exit=[
            LogInfo(msg='teleop window closed; shutting down the twin-wing session'),
            EmitEvent(event=Shutdown(reason='teleop exited')),
        ],
    ))

    return LaunchDescription(args + [plant, teleop, shutdown_on_teleop_exit])
