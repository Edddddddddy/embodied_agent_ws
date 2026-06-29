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
                package="embodied_online_agent",
                executable="robot_action_stub",
                name="robot_action_stub",
                output="screen",
            ),
        ]
    )

