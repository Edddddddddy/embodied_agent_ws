from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    config = "/home/ubuntu/embodied_agent_ws/src/embodied_online_agent/config/online_agent.yaml"
    return LaunchDescription(
        [
            LifecycleNode(
                package="embodied_online_agent",
                executable="online_agent",
                name="online_agent",
                namespace="",
                output="screen",
                parameters=[
                    config,
                    {"mode": "mock", "agent_lifecycle_autostart": False},
                ],
            ),
            Node(
                package="embodied_agent_cpp",
                executable="audio_frontend",
                name="audio_frontend",
                output="screen",
                parameters=[config],
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
                name="agent_control_lifecycle_manager",
                output="screen",
                parameters=[{
                    "autostart": True,
                    "node_names": ["action_guard", "online_agent"],
                    "bond_timeout": 0.0,
                }],
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
