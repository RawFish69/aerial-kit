import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('sim_gazebo')
    world = PathJoinSubstitution([
        pkg_share, 'worlds', 'fixed_wing_demo.sdf'])
    models = PathJoinSubstitution([pkg_share, 'models'])

    headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without GUI; the headless plant still flies.')
    start_delay = DeclareLaunchArgument(
        'start_delay_sec', default_value='3.0',
        description='Delay before the demonstration arms and publishes its mission.')

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models, os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')])

    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')))
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')))

    pose_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='twin_wing_pose_service_bridge',
        arguments=[
            '/world/uav_fixed_wing/set_pose'
            '@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen')

    pose_node = Node(
        package='sim_bridge',
        executable='gazebo_fixedwing_pose_node',
        name='gazebo_fixedwing_pose_node',
        output='screen')

    flight_loop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sim_fast'),
                'launch',
                'fixed_wing_demo.launch.py',
            ])),
        launch_arguments={
            'start_delay_sec': LaunchConfiguration('start_delay_sec'),
        }.items())

    return LaunchDescription([
        headless,
        start_delay,
        set_resource_path,
        gazebo_gui,
        gazebo_headless,
        pose_service_bridge,
        pose_node,
        flight_loop,
    ])
