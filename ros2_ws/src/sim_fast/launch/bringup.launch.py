from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# The fastsim backend integrates /uav/backend/cmd_twist as a *world-frame*
# velocity and has no yaw. Both mission followers default to the Gazebo
# convention instead - body frame, forward-only, turn to face the goal - which
# here never turns and so never moves: the stock executor sat at the origin for
# the whole mission. World frame, no heading control.
_FAST_SIM_FRAME = {'command_frame': 'world', 'heading_control_enabled': False}
# The backend's velocity loop is a = 1.5 * (v_sp - v): a 1/1.5 s time constant.
_FAST_SIM_VELOCITY_TAU_S = 1.0 / 1.5


def _tracker_is(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('mission_tracker'), "' == '", name, "'"]))


def generate_launch_description():
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

    return LaunchDescription([
        start_ground_arg,
        start_demo_arg,
        start_monitor_arg,
        start_offboard_planner_arg,
        start_onboard_planner_arg,
        demo_planning_mode_arg,
        mission_tracker_arg,

        Node(
            package='sim_bridge',
            executable='fastsim_backend_adapter_node',
            name='fastsim_backend_adapter_node',
            output='screen',
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
        *[
            Node(
                package='uav_control',
                executable='mpc_tracker_node',
                name='mpc_tracker_node',
                output='screen',
                parameters=[{
                    **_FAST_SIM_FRAME,
                    'controller': controller,
                    'velocity_loop_tau_s': _FAST_SIM_VELOCITY_TAU_S,
                }],
                condition=_tracker_is(controller),
            )
            for controller in ('mpc', 'mppi')
        ],
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
