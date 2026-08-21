from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


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
                    "autostart": True,
                    "node_names": ["action_guard"],
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
