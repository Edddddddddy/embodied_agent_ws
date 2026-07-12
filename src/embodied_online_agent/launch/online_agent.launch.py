import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

from embodied_agent_core.agent_launch_contract import (
    agent_control_configurations,
    agent_control_parameter_overrides,
    declare_agent_control_arguments,
)
from embodied_agent_core.voice_frontend_launch_contract import (
    declare_voice_frontend_arguments,
    voice_frontend_nodes,
)


def generate_launch_description():
    config = LaunchConfiguration("config")
    agent_config = agent_control_configurations()
    microphone_enabled = agent_config["microphone_enabled"]
    hardware_backend = LaunchConfiguration("hardware_backend")
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    lifecycle_autostart = LaunchConfiguration("lifecycle_autostart")
    uart_device = LaunchConfiguration("uart_device")
    uart_baud_rate = LaunchConfiguration("uart_baud_rate")
    spi_device = LaunchConfiguration("spi_device")
    spi_speed_hz = LaunchConfiguration("spi_speed_hz")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=os.path.join(
                    get_package_share_directory("embodied_online_agent"),
                    "config",
                    "online_agent.yaml",
                ),
                description="Online provider YAML profile.",
            ),
            # 节点 schema 是默认值的单一权威来源；launch 只负责部署期显式覆盖。
            *declare_agent_control_arguments("online"),
            *declare_voice_frontend_arguments(microphone_enabled),
            DeclareLaunchArgument("hardware_backend", default_value="mock"),
            DeclareLaunchArgument("hardware_enabled", default_value="true"),
            DeclareLaunchArgument("lifecycle_autostart", default_value="true"),
            DeclareLaunchArgument("uart_device", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("uart_baud_rate", default_value="115200"),
            DeclareLaunchArgument("spi_device", default_value="/dev/spidev0.0"),
            DeclareLaunchArgument("spi_speed_hz", default_value="1000000"),
            LifecycleNode(
                package="embodied_online_agent",
                executable="online_agent",
                name="online_agent",
                namespace="",
                output="screen",
                parameters=[
                    config,
                    agent_control_parameter_overrides(),
                    {"agent_lifecycle_autostart": False},
                ],
            ),
            *voice_frontend_nodes(config),
            LifecycleNode(
                package="embodied_agent_cpp",
                executable="action_guard",
                name="action_guard",
                namespace="",
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="agent_control_lifecycle_manager",
                output="screen",
                parameters=[{
                    "autostart": ParameterValue(lifecycle_autostart, value_type=bool),
                    "node_names": ["action_guard", "online_agent"],
                    "bond_timeout": 0.0,
                }],
            ),
            Node(
                package="embodied_agent_cpp",
                executable="hardware_controller",
                name="hardware_controller",
                output="screen",
                condition=IfCondition(hardware_enabled),
                parameters=[{
                    "backend": hardware_backend,
                    "uart_device": uart_device,
                    "uart_baud_rate": ParameterValue(uart_baud_rate, value_type=int),
                    "spi_device": spi_device,
                    "spi_speed_hz": ParameterValue(spi_speed_hz, value_type=int),
                }],
            ),
        ]
    )
