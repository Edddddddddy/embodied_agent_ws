from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("embodied_offline_agent"), "config", "offline_agent.yaml"
    )
    mode = LaunchConfiguration("mode")
    microphone = LaunchConfiguration("microphone_enabled")
    speaker = LaunchConfiguration("speaker_enabled")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mock"),
        DeclareLaunchArgument("microphone_enabled", default_value="false"),
        DeclareLaunchArgument("speaker_enabled", default_value="false"),
        Node(
            package="embodied_offline_agent", executable="offline_agent",
            name="offline_agent", output="screen",
            parameters=[config, {"mode": mode, "microphone_enabled": microphone}],
        ),
        Node(
            package="embodied_agent_cpp", executable="audio_frontend",
            name="audio_frontend", output="screen",
            parameters=[config, {"capture_enabled": microphone, "speaker_enabled": speaker}],
        ),
        Node(package="embodied_agent_cpp", executable="action_guard", name="action_guard", output="screen"),
        Node(package="embodied_agent_cpp", executable="robot_action_stub", name="robot_action_stub", output="screen"),
    ])
