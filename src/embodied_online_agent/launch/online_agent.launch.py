from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = LaunchConfiguration("config")
    mode = LaunchConfiguration("mode")
    microphone_enabled = LaunchConfiguration("microphone_enabled")
    speaker_enabled = LaunchConfiguration("speaker_enabled")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="/home/ubuntu/embodied_agent_ws/src/embodied_online_agent/config/online_agent.yaml",
            ),
            DeclareLaunchArgument("mode", default_value="mock"),
            DeclareLaunchArgument("microphone_enabled", default_value="false"),
            DeclareLaunchArgument("speaker_enabled", default_value="false"),
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
                            microphone_enabled, value_type=bool
                        ),
                        "speaker_enabled": ParameterValue(
                            speaker_enabled, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="embodied_agent_cpp",
                executable="action_guard",
                name="action_guard",
                output="screen",
            ),
        ]
    )
