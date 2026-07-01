from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("embodied_offline_agent"), "config", "offline_agent.yaml"
    )
    mode = LaunchConfiguration("mode")
    microphone = LaunchConfiguration("microphone_enabled")
    capture = LaunchConfiguration("capture_enabled")
    speaker = LaunchConfiguration("speaker_enabled")
    hardware_backend = LaunchConfiguration("hardware_backend")
    wake_word_enabled = LaunchConfiguration("wake_word_enabled")
    uart_device = LaunchConfiguration("uart_device")
    uart_baud_rate = LaunchConfiguration("uart_baud_rate")
    spi_device = LaunchConfiguration("spi_device")
    spi_speed_hz = LaunchConfiguration("spi_speed_hz")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mock"),
        DeclareLaunchArgument("microphone_enabled", default_value="false"),
        DeclareLaunchArgument("capture_enabled", default_value=microphone),
        DeclareLaunchArgument("speaker_enabled", default_value="false"),
        DeclareLaunchArgument("hardware_backend", default_value="mock"),
        DeclareLaunchArgument("wake_word_enabled", default_value="true"),
        DeclareLaunchArgument("uart_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("uart_baud_rate", default_value="115200"),
        DeclareLaunchArgument("spi_device", default_value="/dev/spidev0.0"),
        DeclareLaunchArgument("spi_speed_hz", default_value="1000000"),
        Node(
            package="embodied_offline_agent", executable="offline_agent",
            name="offline_agent", output="screen",
            parameters=[config, {
                "mode": mode,
                "microphone_enabled": microphone,
                "wake_word_enabled": ParameterValue(wake_word_enabled, value_type=bool),
            }],
        ),
        Node(
            package="embodied_agent_cpp", executable="audio_frontend",
            name="audio_frontend", output="screen",
            parameters=[config, {"capture_enabled": capture, "speaker_enabled": speaker}],
        ),
        Node(package="embodied_agent_cpp", executable="action_guard", name="action_guard", output="screen"),
        Node(
            package="embodied_agent_cpp", executable="hardware_controller",
            name="hardware_controller", output="screen",
            parameters=[{
                "backend": hardware_backend,
                "uart_device": uart_device,
                "uart_baud_rate": ParameterValue(uart_baud_rate, value_type=int),
                "spi_device": spi_device,
                "spi_speed_hz": ParameterValue(spi_speed_hz, value_type=int),
            }],
        ),
    ])
