from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hw_share = FindPackageShare("hw_bridge")
    crsf_config = PathJoinSubstitution([hw_share, "config", "crsf_adapter.yaml"])
    estimator_config = PathJoinSubstitution([hw_share, "config", "hw_estimator.yaml"])

    udp_host_arg = DeclareLaunchArgument(
        "udp_host",
        default_value="192.168.4.1",
        description="ESP32 TX UDP host",
    )
    udp_port_arg = DeclareLaunchArgument(
        "udp_port",
        default_value="9000",
        description="ESP32 TX UDP port",
    )

    estimator_node = Node(
        package="hw_bridge",
        executable="hw_state_estimator_node",
        name="hw_state_estimator_node",
        output="screen",
        parameters=[estimator_config],
    )
    crsf_adapter_node = Node(
        package="hw_bridge",
        executable="crsf_backend_adapter_node",
        name="crsf_backend_adapter_node",
        output="screen",
        parameters=[
            crsf_config,
            {
                "udp_host": LaunchConfiguration("udp_host"),
                "udp_port": LaunchConfiguration("udp_port"),
            },
        ],
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
            udp_host_arg,
            udp_port_arg,
            estimator_node,
            crsf_adapter_node,
            telemetry_adapter_node,
            command_manager_node,
            mission_executor_node,
        ]
    )
