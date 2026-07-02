from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = LaunchConfiguration("config")
    mode = LaunchConfiguration("mode")
    microphone_enabled = LaunchConfiguration("microphone_enabled")
    capture_enabled = LaunchConfiguration("capture_enabled")
    wake_word_enabled = LaunchConfiguration("wake_word_enabled")
    speaker_enabled = LaunchConfiguration("speaker_enabled")
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
                default_value="/home/ubuntu/embodied_agent_ws/src/embodied_online_agent/config/online_agent.yaml",
            ),
            DeclareLaunchArgument("mode", default_value="mock"),
            DeclareLaunchArgument("microphone_enabled", default_value="false"),
            DeclareLaunchArgument("capture_enabled", default_value=microphone_enabled),
            DeclareLaunchArgument("wake_word_enabled", default_value="true"),
            DeclareLaunchArgument("speaker_enabled", default_value="false"),
            DeclareLaunchArgument("hardware_backend", default_value="mock"),
            DeclareLaunchArgument("hardware_enabled", default_value="true"),
            DeclareLaunchArgument("lifecycle_autostart", default_value="true"),
            DeclareLaunchArgument("uart_device", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("uart_baud_rate", default_value="115200"),
            DeclareLaunchArgument("spi_device", default_value="/dev/spidev0.0"),
            DeclareLaunchArgument("spi_speed_hz", default_value="1000000"),
            Node(
                package="embodied_online_agent",
                executable="online_agent",
                name="online_agent",
                output="screen",
                parameters=[
                    config,
                    {
                        "mode": mode,
                        "microphone_enabled": ParameterValue(
                            microphone_enabled, value_type=bool
                        ),
                        "wake_word_enabled": ParameterValue(
                            wake_word_enabled, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="embodied_agent_cpp",
                executable="audio_frontend",
                name="audio_frontend",
                output="screen",
                parameters=[
                    config,
                    {
                        "capture_enabled": ParameterValue(
                            capture_enabled, value_type=bool
                        ),
                        "speaker_enabled": ParameterValue(
                            speaker_enabled, value_type=bool
                        ),
                    },
                ],
            ),
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
                name="action_guard_lifecycle_manager",
                output="screen",
                parameters=[{
                    "autostart": ParameterValue(lifecycle_autostart, value_type=bool),
                    "node_names": ["action_guard"],
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
