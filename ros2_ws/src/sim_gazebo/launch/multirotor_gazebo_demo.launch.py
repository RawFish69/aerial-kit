import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('sim_gazebo')
    models = PathJoinSubstitution([pkg_share, 'models'])
    world = PathJoinSubstitution([
        pkg_share, 'worlds', 'multirotor_demo.sdf'])

    airframe = DeclareLaunchArgument(
        'airframe',
        default_value='fpv_quad_5in',
        description='Multirotor spec name: fpv_quad_5in, micro_lr_quad, '
                    'research_hex or research_octo.',
    )
    headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without its GUI.',
    )
    start_demo = DeclareLaunchArgument(
        'start_demo',
        default_value='false',
        description='Start the standard autonomous demo and arm the plant.',
    )

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models, os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')],
    )

    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    spawn_visual = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_multirotor_visual',
        output='screen',
        arguments=[
            '-world', 'uav_multirotor',
            '-name', LaunchConfiguration('airframe'),
            '-string', [
                '<sdf version="1.8"><include><uri>model://',
                LaunchConfiguration('airframe'),
                '_visual</uri></include></sdf>',
            ],
            '-x', '0', '-y', '0', '-z', '0',
        ],
    )

    pose_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='multirotor_pose_service_bridge',
        arguments=[
            '/world/uav_multirotor/set_pose'
            '@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen',
    )
    pose_node = Node(
        package='sim_bridge',
        executable='gazebo_multirotor_pose_node',
        name='gazebo_multirotor_pose_node',
        output='screen',
        parameters=[{
            'odom_topic': '/uav/backend/odom',
            'set_pose_service': '/world/uav_multirotor/set_pose',
            'entity_name': LaunchConfiguration('airframe'),
            'publish_rate_hz': 20.0,
        }],
    )

    plant = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sim_fast'),
                'launch',
                'bringup.launch.py',
            ])
        ),
        launch_arguments={
            'sim_backend': 'mujoco',
            'airframe': LaunchConfiguration('airframe'),
            'start_demo': LaunchConfiguration('start_demo'),
            'start_ground': 'true',
            'viewer': 'false',
        }.items(),
    )

    return LaunchDescription([
        airframe,
        headless,
        start_demo,
        set_resource_path,
        gazebo_gui,
        gazebo_headless,
        TimerAction(period=2.0, actions=[spawn_visual]),
        pose_service_bridge,
        TimerAction(period=4.0, actions=[pose_node]),
        plant,
    ])
