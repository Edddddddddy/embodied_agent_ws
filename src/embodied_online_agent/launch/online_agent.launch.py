from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config")
    mode = LaunchConfiguration("mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="/home/ubuntu/embodied_agent_ws/src/embodied_online_agent/config/online_agent.yaml",
            ),
            DeclareLaunchArgument("mode", default_value="mock"),
            Node(
                package="embodied_online_agent",
                executable="online_agent",
                name="online_agent",
                output="screen",
                parameters=[config, {"mode": mode}],
            ),
        ]
    )

