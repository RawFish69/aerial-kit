from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# The fastsim backend integrates /uav/backend/cmd_twist as a *world-frame*
# velocity and has no yaw. Both mission followers default to the Gazebo
# convention instead - body frame, forward-only, turn to face the goal - which
# here never turns and so never moves: the stock executor sat at the origin for
# the whole mission. World frame, no heading control.
_FAST_SIM_FRAME = {'command_frame': 'world', 'heading_control_enabled': False}
# The backend's velocity loop is a = 1.5 * (v_sp - v): a 1/1.5 s time constant.
_FAST_SIM_VELOCITY_TAU_S = 1.0 / 1.5


def _import_repo():
    import sys
    from pathlib import Path

    for parent in Path(__file__).resolve().parents:
        if (parent / 'aerial_kit').is_dir() and (parent / 'pyproject.toml').is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            break


def firmware_radio_settings(airframe):
    """Radio backend settings for flying the real AK firmware over the radio.

    The firmware's velocity hold flies it: the host sends world velocity
    (rotated to the FC's heading from telemetry), climb rate and yaw rate as
    sticks on AUX7 at the firmware's own full-scale values (ak_nav: 6 m/s,
    3 m/s climb, 1.5 rad/s yaw), and the FC closes the loops on its own
    sensors instead of the host closing them over delayed telemetry.
    """
    del airframe
    return {
        'stick_mapping': 'velocity',
        'velocity_channel': FIRMWARE_VELOCITY_CHANNEL,
        'manual_max_speed_mps': 6.0,
        'manual_max_climb_mps': 3.0,
        'max_yaw_rate_rps': 1.5,
        'vxy_feedback': False,
        'vz_feedback': False,
    }


#: AUX channel for the AK firmware's velocity hold in the SIL (1-based).
FIRMWARE_VELOCITY_CHANNEL = 7


def _radio_backend(context):
    firmware = LaunchConfiguration('flight_controller').perform(context).strip().lower() == 'firmware'
    radio = LaunchConfiguration('radio_sim').perform(context).strip().lower() == 'true'
    backend = LaunchConfiguration('sim_backend').perform(context)
    airframe = LaunchConfiguration('airframe').perform(context)
    if firmware and (not radio or backend != 'mujoco'):
        raise RuntimeError('flight_controller:=firmware needs sim_backend:=mujoco radio_sim:=true: '
                           'the AK firmware is flown through the simulated ELRS radio')
    if not radio:
        return []
    overrides = {
        'transport': 'ros',
        'send_rate_hz': float(LaunchConfiguration('radio_rate_hz').perform(context)),
        'airframe_name': airframe,
        'vz_feedback': True,
        'vxy_feedback': False,
        'command_frame': 'world',
        'stick_mapping': 'velocity',
    }
    if firmware:
        overrides.update(firmware_radio_settings(airframe))
    radio_config = PathJoinSubstitution([FindPackageShare('hw_bridge'), 'config', 'crsf_radio.yaml'])
    return [Node(
        package='hw_bridge',
        executable='crsf_radio_backend_node',
        name='crsf_radio_backend_node',
        output='screen',
        parameters=[radio_config, overrides],
    )]


def velocity_loop_tau_s(sim_backend, airframe):
    """The plant's velocity-loop time constant, for the predictive trackers.

    pointmass: a = 1.5 (v_sp - v). mujoco: the multirotor adapter's velocity
    law a = velocity_kp (v_sp - v), with velocity_kp from the airframe spec.
    """
    if str(sim_backend) != 'mujoco':
        return _FAST_SIM_VELOCITY_TAU_S
    _import_repo()
    from aerial_kit.airframes.multirotor_specs import load_multirotor_spec

    return 1.0 / float(load_multirotor_spec(str(airframe)).control['velocity_kp'])


def _predictive_trackers(context):
    backend = LaunchConfiguration('sim_backend').perform(context)
    airframe = LaunchConfiguration('airframe').perform(context)
    tau = velocity_loop_tau_s(backend, airframe)
    return [
        Node(
            package='uav_control',
            executable='mpc_tracker_node',
            name='mpc_tracker_node',
            output='screen',
            parameters=[{
                **_FAST_SIM_FRAME,
                'controller': controller,
                'velocity_loop_tau_s': tau,
            }],
            condition=_tracker_is(controller),
        )
        for controller in ('mpc', 'mppi')
    ]


def _backend_is(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('sim_backend'), "' == '", name, "'"]))


def _tracker_is(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('mission_tracker'), "' == '", name, "'"]))


def _topic_when_radio(radio_topic, direct_topic):
    return ParameterValue(
        PythonExpression([
            "'", radio_topic, "' if '", LaunchConfiguration('radio_sim'),
            "' == 'true' else '", direct_topic, "'",
        ]),
        value_type=str,
    )


def _launch_float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    radio_config = PathJoinSubstitution([
        FindPackageShare('hw_bridge'), 'config', 'crsf_radio.yaml',
    ])
    start_ground_arg = DeclareLaunchArgument('start_ground', default_value='true')
    start_demo_arg = DeclareLaunchArgument('start_demo', default_value='false')
    start_monitor_arg = DeclareLaunchArgument('start_monitor', default_value='true')
    start_offboard_planner_arg = DeclareLaunchArgument('start_offboard_planner', default_value='true')
    start_onboard_planner_arg = DeclareLaunchArgument('start_onboard_planner', default_value='false')
    demo_planning_mode_arg = DeclareLaunchArgument('demo_planning_mode', default_value='offboard')
    mission_tracker_arg = DeclareLaunchArgument(
        'mission_tracker', default_value='executor',
        description='executor (air_unit P-controller) | mpc | mppi (uav_control predictive tracker)',
    )
    radio_sim_arg = DeclareLaunchArgument(
        'radio_sim', default_value='false',
        description='route commands and telemetry through the simulated ELRS/CRSF radio',
    )
    flight_controller_arg = DeclareLaunchArgument(
        'flight_controller', default_value='sim',
        description="sim | firmware: the unmodified AK flight core flies the MuJoCo quad, "
                    "commanded over the simulated ELRS radio (needs sim_backend:=mujoco radio_sim:=true, "
                    "quad-X airframes)",
    )
    radio_latency_arg = DeclareLaunchArgument('radio_latency_ms', default_value='8.0')
    radio_loss_arg = DeclareLaunchArgument('radio_packet_loss_percent', default_value='0.0')
    radio_rate_arg = DeclareLaunchArgument('radio_rate_hz', default_value='150.0')

    sim_backend_arg = DeclareLaunchArgument(
        'sim_backend', default_value='pointmass',
        description='pointmass (fast headless, the original) | mujoco (MuJoCo multirotor '
                    'with rotors, motor lag and contact, same /uav/backend contract)',
    )
    airframe_arg = DeclareLaunchArgument(
        'airframe', default_value='fpv_quad_5in',
        description='sim_backend:=mujoco only: fpv_quad_5in | micro_lr_quad | research_hex | '
                    'research_octo, or a multirotor spec YAML path',
    )
    viewer_arg = DeclareLaunchArgument(
        'viewer', default_value='false',
        description='sim_backend:=mujoco only: open the MuJoCo viewer (falls back to headless).')
    wind_x_arg = DeclareLaunchArgument('wind_x_mps', default_value='0.0')
    wind_y_arg = DeclareLaunchArgument('wind_y_mps', default_value='0.0')

    return LaunchDescription([
        sim_backend_arg,
        airframe_arg,
        viewer_arg,
        wind_x_arg,
        wind_y_arg,
        start_ground_arg,
        start_demo_arg,
        start_monitor_arg,
        start_offboard_planner_arg,
        start_onboard_planner_arg,
        demo_planning_mode_arg,
        mission_tracker_arg,
        radio_sim_arg,
        flight_controller_arg,
        radio_latency_arg,
        radio_loss_arg,
        radio_rate_arg,

        Node(
            package='sim_bridge',
            executable='fastsim_backend_adapter_node',
            name='fastsim_backend_adapter_node',
            output='screen',
            condition=_backend_is('pointmass'),
            parameters=[{
                'backend_cmd_topic': _topic_when_radio(
                    '/uav/radio/decoded_cmd_twist', '/uav/backend/cmd_twist'),
                'backend_enable_topic': _topic_when_radio(
                    '/uav/radio/decoded_enable', '/uav/backend/enable'),
                'backend_odom_topic': _topic_when_radio(
                    '/uav/radio/plant_odom', '/uav/backend/odom'),
            }],
        ),
        Node(
            package='sim_bridge',
            executable='multirotor_backend_adapter_node',
            name='multirotor_backend_adapter_node',
            output='screen',
            condition=_backend_is('mujoco'),
            parameters=[{
                'airframe': LaunchConfiguration('airframe'),
                'viewer': LaunchConfiguration('viewer'),
                'wind_x_mps': LaunchConfiguration('wind_x_mps'),
                'wind_y_mps': LaunchConfiguration('wind_y_mps'),
                'backend_cmd_topic': _topic_when_radio(
                    '/uav/radio/decoded_cmd_twist', '/uav/backend/cmd_twist'),
                'backend_enable_topic': _topic_when_radio(
                    '/uav/radio/decoded_enable', '/uav/backend/enable'),
                'backend_odom_topic': _topic_when_radio(
                    '/uav/radio/plant_odom', '/uav/backend/odom'),
                'manual_topic': _topic_when_radio(
                    '/uav/radio/decoded_manual', '/uav/backend/manual'),
                'flight_controller': LaunchConfiguration('flight_controller'),
                # With the firmware, RC comes from the receiver bytes as on a real FC.
                'firmware_rc_input': 'crsf',
                'firmware_velocity_channel': FIRMWARE_VELOCITY_CHANNEL,
            }],
        ),
        OpaqueFunction(function=_radio_backend),
        Node(
            package='hw_bridge',
            executable='sim_elrs_link_node',
            name='sim_elrs_link_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('radio_sim')),
            parameters=[{
                'rate_hz': _launch_float('radio_rate_hz'),
                'latency_ms': _launch_float('radio_latency_ms'),
                'packet_loss_percent': _launch_float('radio_packet_loss_percent'),
            }],
        ),
        Node(
            package='hw_bridge',
            executable='crsf_sim_receiver_node',
            name='crsf_sim_receiver_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('radio_sim')),
            parameters=[
                radio_config,
                {
                    'airframe_name': LaunchConfiguration('airframe'),
                    'output_rate_hz': _launch_float('radio_rate_hz'),
                    'decoded_manual_mode': 'velocity',
                },
            ],
        ),
        Node(
            package='hw_bridge',
            executable='hw_state_estimator_node',
            name='hw_state_estimator_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('radio_sim')),
        ),
        Node(
            package='air_unit',
            executable='telemetry_adapter_node',
            name='telemetry_adapter_node',
            output='screen',
        ),
        Node(
            package='air_unit',
            executable='mission_executor_node',
            name='mission_executor_node',
            output='screen',
            parameters=[_FAST_SIM_FRAME],
            condition=_tracker_is('executor'),
        ),
        # Resolved at launch time: the velocity-loop time constant depends on
        # the backend and, for MuJoCo, on the airframe spec.
        OpaqueFunction(function=_predictive_trackers),
        Node(
            package='air_unit',
            executable='command_manager_node',
            name='command_manager_node',
            output='screen',
        ),
        Node(
            package='planner',
            executable='planner_server_node',
            namespace='uav/planner',
            name='planner_server_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_onboard_planner')),
        ),
        Node(
            package='planner',
            executable='planner_server_node',
            namespace='gs/planner',
            name='planner_server_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_offboard_planner')),
        ),
        Node(
            package='ground_station',
            executable='ground_station_telemetry_monitor',
            name='ground_station_telemetry_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_monitor')),
        ),
        Node(
            package='ground_station',
            executable='ground_station_demo_mission',
            name='ground_station_demo_mission',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_demo')),
            parameters=[{
                'planning_mode': LaunchConfiguration('demo_planning_mode'),
            }],
        ),
    ])
