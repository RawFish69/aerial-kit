import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('sim_gazebo')

    world = PathJoinSubstitution([pkg_share, 'worlds', 'flat_world.sdf'])
    bridge_config = PathJoinSubstitution([pkg_share, 'config', 'bridge_topics.yaml'])
    models_dir = PathJoinSubstitution([pkg_share, 'models'])

    # ── launch arguments ──────────────────────────────────────────────
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without GUI',
    )
    run_smoke_arg = DeclareLaunchArgument(
        'run_smoke_cmd', default_value='false',
        description='Start a simple smoke command publisher node',
    )
    start_air_arg = DeclareLaunchArgument(
        'start_air_unit', default_value='true',
        description='Start air unit nodes',
    )
    start_air_planner_arg = DeclareLaunchArgument(
        'start_air_planner', default_value='true',
        description='Start air-side planner service (/uav/planner/plan_path)',
    )
    start_terrain_visuals_arg = DeclareLaunchArgument(
        'start_terrain_visuals', default_value='false',
        description='Spawn /terrain/obstacles markers as Gazebo entities',
    )
    terrain_world_name_arg = DeclareLaunchArgument(
        'terrain_world_name', default_value='uav_flat_world',
        description='Gazebo world name for terrain spawning service path',
    )
    start_terrain_surface_arg = DeclareLaunchArgument(
        'start_terrain_surface', default_value='false',
        description='Spawn a generated terrain surface mesh into Gazebo',
    )
    start_path_visuals_arg = DeclareLaunchArgument(
        'start_path_visuals', default_value='false',
        description='Spawn planner waypoint markers in Gazebo',
    )
    path_marker_topic_arg = DeclareLaunchArgument(
        'path_marker_topic', default_value='/gs/planner/planned_path_markers',
        description='Planner MarkerArray topic to mirror into Gazebo',
    )
    terrain_surface_type_arg = DeclareLaunchArgument(
        'terrain_surface_type', default_value='mountains',
        description='Terrain profile for surface mesh generator',
    )
    terrain_config_file_arg = DeclareLaunchArgument(
        'terrain_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('terrain_generator'), 'config', 'terrain_params.yaml',
        ]),
        description='Shared terrain config YAML',
    )
    # ── params-file pass-through for air unit nodes ───────────────────
    # Same pattern as ground.launch.py: two Node entries per node,
    # one with params file (when use_X_params_file=true) and one without.
    mission_tracker_arg = DeclareLaunchArgument(
        'mission_tracker', default_value='executor',
        description='executor (air_unit P-controller) | mpc | mppi (uav_control predictive tracker)',
    )
    tracker_params_arg = DeclareLaunchArgument(
        'tracker_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('uav_control'), 'config', 'mpc_tracker.yaml']),
        description='Parameter file for mpc_tracker_node (mission_tracker:=mpc|mppi)',
    )
    use_mission_executor_params_arg = DeclareLaunchArgument(
        'use_mission_executor_params_file', default_value='false',
        description='Load mission_executor_params_file into mission_executor_node',
    )
    mission_executor_params_arg = DeclareLaunchArgument(
        'mission_executor_params_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'mission_executor_safe_tracking.yaml',
        ]),
        description='Parameter file for mission_executor_node',
    )
    use_command_manager_params_arg = DeclareLaunchArgument(
        'use_command_manager_params_file', default_value='false',
        description='Load command_manager_params_file into command_manager_node',
    )
    command_manager_params_arg = DeclareLaunchArgument(
        'command_manager_params_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'command_manager_safe_custom.yaml',
        ]),
        description='Parameter file for command_manager_node',
    )

    # ── Gazebo model path ─────────────────────────────────────────────
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            models_dir,
            os.pathsep,
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ],
    )

    # ── Gazebo process ────────────────────────────────────────────────
    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    # ── ROS ↔ Gazebo bridge ──────────────────────────────────────────
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    # ── Backend adapter (lr_drone topics) ─────────────────────────────
    gazebo_adapter = Node(
        package='sim_bridge',
        executable='gazebo_backend_adapter_node',
        name='gazebo_backend_adapter',
        output='screen',
        parameters=[{
            'uav_namespace': '/uav',
            'backend_cmd_topic': '/uav/backend/cmd_twist',
            'backend_enable_topic': '/uav/backend/enable',
            'backend_odom_topic': '/uav/backend/odom',
            'gz_cmd_topic': '/lr_drone/gazebo/command/twist',
            'gz_enable_topic': '/lr_drone/enable',
            'gz_odom_topic': '/model/lr_drone/odometry',
        }],
    )

    # ── Optional nodes ────────────────────────────────────────────────
    smoke_node = Node(
        package='sim_bridge',
        executable='gz_smoke_cmd_node',
        name='gz_smoke_cmd_node',
        output='screen',
        parameters=[{
            'cmd_topic': '/uav/backend/cmd_twist',
            'enable_topic': '/uav/backend/enable',
        }],
        condition=IfCondition(LaunchConfiguration('run_smoke_cmd')),
    )

    terrain_visual_spawner = Node(
        package='sim_gazebo',
        executable='terrain_marker_spawner_node',
        name='terrain_marker_spawner',
        output='screen',
        parameters=[{
            'marker_topic': '/terrain/obstacles',
            'world_name': LaunchConfiguration('terrain_world_name'),
            'spawn_once': True,
        }],
        condition=IfCondition(LaunchConfiguration('start_terrain_visuals')),
    )

    terrain_surface_spawner = Node(
        package='sim_gazebo',
        executable='terrain_surface_spawner_node',
        name='terrain_surface_spawner',
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('terrain_world_name'),
            'terrain_type': LaunchConfiguration('terrain_surface_type'),
            'terrain_config_file': LaunchConfiguration('terrain_config_file'),
        }],
        condition=IfCondition(LaunchConfiguration('start_terrain_surface')),
    )

    path_visual_spawner = Node(
        package='sim_gazebo',
        executable='path_marker_spawner_node',
        name='path_marker_spawner',
        output='screen',
        parameters=[{
            'marker_topic': LaunchConfiguration('path_marker_topic'),
            'world_name': LaunchConfiguration('terrain_world_name'),
            'model_name_prefix': 'planned_path',
            # Keep this dynamic so color/progress updates from planner markers show up.
            'spawn_once': False,
            'create_collision': False,
        }],
        condition=IfCondition(LaunchConfiguration('start_path_visuals')),
    )

    # ── Air unit nodes ────────────────────────────────────────────────
    # telemetry_adapter has no params file variation
    telemetry_adapter = Node(
        package='air_unit',
        executable='telemetry_adapter_node',
        name='telemetry_adapter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_air_unit')),
    )

    # mission_executor: two variants (with / without params file)
    _executor_selected = [" and '", LaunchConfiguration('mission_tracker'), "' == 'executor'"]
    _me_with = PythonExpression([
        "'", LaunchConfiguration('start_air_unit'), "' == 'true' and '",
        LaunchConfiguration('use_mission_executor_params_file'), "' == 'true'",
        *_executor_selected,
    ])
    _me_without = PythonExpression([
        "'", LaunchConfiguration('start_air_unit'), "' == 'true' and '",
        LaunchConfiguration('use_mission_executor_params_file'), "' != 'true'",
        *_executor_selected,
    ])
    mission_executor_with_params = Node(
        package='air_unit',
        executable='mission_executor_node',
        name='mission_executor_node',
        output='screen',
        parameters=[LaunchConfiguration('mission_executor_params_file')],
        condition=IfCondition(_me_with),
    )
    mission_executor_no_params = Node(
        package='air_unit',
        executable='mission_executor_node',
        name='mission_executor_node',
        output='screen',
        condition=IfCondition(_me_without),
    )

    # uav_control's predictive tracker in place of the executor. Its defaults
    # are the Gazebo convention (body frame, nose +Y, forward-only), and
    # mission_tracker is also the controller name.
    mission_tracker_node = Node(
        package='uav_control',
        executable='mpc_tracker_node',
        name='mpc_tracker_node',
        output='screen',
        parameters=[
            LaunchConfiguration('tracker_params_file'),
            {'controller': LaunchConfiguration('mission_tracker')},
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('start_air_unit'), "' == 'true' and '",
            LaunchConfiguration('mission_tracker'), "' in ('mpc', 'mppi')",
        ])),
    )

    # command_manager: two variants (with / without params file)
    _cm_with = PythonExpression([
        "'", LaunchConfiguration('start_air_unit'), "' == 'true' and '",
        LaunchConfiguration('use_command_manager_params_file'), "' == 'true'",
    ])
    _cm_without = PythonExpression([
        "'", LaunchConfiguration('start_air_unit'), "' == 'true' and '",
        LaunchConfiguration('use_command_manager_params_file'), "' != 'true'",
    ])
    command_manager_with_params = Node(
        package='air_unit',
        executable='command_manager_node',
        name='command_manager_node',
        output='screen',
        parameters=[LaunchConfiguration('command_manager_params_file')],
        condition=IfCondition(_cm_with),
    )
    command_manager_no_params = Node(
        package='air_unit',
        executable='command_manager_node',
        name='command_manager_node',
        output='screen',
        condition=IfCondition(_cm_without),
    )

    air_planner = Node(
        package='planner',
        executable='planner_server_node',
        name='planner_server_node',
        namespace='uav/planner',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_air_planner')),
    )

    return LaunchDescription([
        # Arguments
        headless_arg,
        run_smoke_arg,
        start_air_arg,
        start_air_planner_arg,
        start_terrain_visuals_arg,
        terrain_world_name_arg,
        start_terrain_surface_arg,
        start_path_visuals_arg,
        path_marker_topic_arg,
        terrain_surface_type_arg,
        terrain_config_file_arg,
        mission_tracker_arg,
        tracker_params_arg,
        use_mission_executor_params_arg,
        mission_executor_params_arg,
        use_command_manager_params_arg,
        command_manager_params_arg,
        # Environment
        set_gz_resource_path,
        # Core
        gz_sim_gui,
        gz_sim_headless,
        bridge_node,
        gazebo_adapter,
        # Optional
        smoke_node,
        terrain_visual_spawner,
        terrain_surface_spawner,
        path_visual_spawner,
        # Air unit
        telemetry_adapter,
        mission_executor_with_params,
        mission_executor_no_params,
        mission_tracker_node,
        command_manager_with_params,
        command_manager_no_params,
        air_planner,
    ])
