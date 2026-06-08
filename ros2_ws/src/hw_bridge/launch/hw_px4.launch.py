from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hw_share = FindPackageShare("hw_bridge")
    px4_config = PathJoinSubstitution([hw_share, "config", "px4_adapter.yaml"])

    mavlink_url_arg = DeclareLaunchArgument(
        "mavlink_url",
        default_value="udpin:0.0.0.0:14540",
        description="PX4 MAVLink endpoint",
    )

    px4_adapter_node = Node(
        package="hw_bridge",
        executable="px4_backend_adapter_node",
        name="px4_backend_adapter_node",
        output="screen",
        parameters=[px4_config, {"mavlink_url": LaunchConfiguration("mavlink_url")}],
    )
    telemetry_adapter_node = Node(
        package="air_unit",
        executable="telemetry_adapter_node",
        name="telemetry_adapter_node",
        output="screen",
    )
    command_manager_node = Node(
        package="air_unit",
        executable="command_manager_node",
        name="command_manager_node",
        output="screen",
    )
    mission_executor_node = Node(
        package="air_unit",
        executable="mission_executor_node",
        name="mission_executor_node",
        output="screen",
    )

    return LaunchDescription(
        [
            mavlink_url_arg,
            px4_adapter_node,
            telemetry_adapter_node,
            command_manager_node,
            mission_executor_node,
        ]
    )
