from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    uart_device = LaunchConfiguration("uart_device")
    uart_baud_rate = LaunchConfiguration("uart_baud_rate")
    spi_device = LaunchConfiguration("spi_device")
    spi_speed_hz = LaunchConfiguration("spi_speed_hz")
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mock"),
        DeclareLaunchArgument("uart_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("uart_baud_rate", default_value="115200"),
        DeclareLaunchArgument("spi_device", default_value="/dev/spidev0.0"),
        DeclareLaunchArgument("spi_speed_hz", default_value="1000000"),
        Node(
            package="embodied_agent_cpp",
            executable="action_guard",
            name="action_guard",
            output="screen",
        ),
        Node(
            package="embodied_agent_cpp",
            executable="hardware_controller",
            name="hardware_controller",
            output="screen",
            parameters=[{
                "backend": backend,
                "uart_device": uart_device,
                "uart_baud_rate": ParameterValue(uart_baud_rate, value_type=int),
                "spi_device": spi_device,
                "spi_speed_hz": ParameterValue(spi_speed_hz, value_type=int),
            }],
        ),
    ])
