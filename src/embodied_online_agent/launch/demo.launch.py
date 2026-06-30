from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = "/home/ubuntu/embodied_agent_ws/src/embodied_online_agent/config/online_agent.yaml"
    return LaunchDescription(
        [
            Node(
                package="embodied_online_agent",
                executable="online_agent",
                name="online_agent",
                output="screen",
                parameters=[config, {"mode": "mock"}],
            ),
            Node(
                package="embodied_agent_cpp",
                executable="audio_frontend",
                name="audio_frontend",
                output="screen",
                parameters=[config],
            ),
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
                parameters=[{"backend": "mock"}],
            ),
        ]
    )
