from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

from embodied_agent_core.agent_launch_contract import (
    agent_control_configurations,
    agent_control_parameter_overrides,
    declare_agent_control_arguments,
)
from embodied_agent_core.voice_frontend_launch_contract import (
    declare_voice_frontend_arguments,
    voice_frontend_nodes,
)
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("embodied_offline_agent"), "config", "offline_agent.yaml"
    )
    agent_config = agent_control_configurations()
    microphone = agent_config["microphone_enabled"]
    asr_hotwords_score = LaunchConfiguration("asr_hotwords_score")
    tts_provider = LaunchConfiguration("tts_provider")
    summer_tts_binary = LaunchConfiguration("summer_tts_binary")
    summer_tts_model = LaunchConfiguration("summer_tts_model")
    summer_tts_timeout_s = LaunchConfiguration("summer_tts_timeout_s")
    summer_tts_service_name = LaunchConfiguration("summer_tts_service_name")
    summer_tts_service_timeout_s = LaunchConfiguration("summer_tts_service_timeout_s")
    summer_tts_service_speaker_id = LaunchConfiguration("summer_tts_service_speaker_id")
    summer_tts_service_length_scale = LaunchConfiguration("summer_tts_service_length_scale")
    summer_tts_cache_enabled = LaunchConfiguration("summer_tts_cache_enabled")
    summer_tts_cache_max_entries = LaunchConfiguration("summer_tts_cache_max_entries")
    summer_tts_cache_max_text_chars = LaunchConfiguration("summer_tts_cache_max_text_chars")
    hardware_backend = LaunchConfiguration("hardware_backend")
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    lifecycle_autostart = LaunchConfiguration("lifecycle_autostart")
    uart_device = LaunchConfiguration("uart_device")
    uart_baud_rate = LaunchConfiguration("uart_baud_rate")
    spi_device = LaunchConfiguration("spi_device")
    spi_speed_hz = LaunchConfiguration("spi_speed_hz")
    return LaunchDescription([
        # 与在线 launch 共用参数接口和默认值来源，避免两条链路配置漂移。
        *declare_agent_control_arguments("offline"),
        *declare_voice_frontend_arguments(microphone),
        DeclareLaunchArgument("asr_hotwords_score", default_value="3.0"),
        DeclareLaunchArgument("tts_provider", default_value="sherpa"),
        DeclareLaunchArgument(
            "summer_tts_binary",
            default_value="/home/ubuntu/embodied_agent_ws/third_party/SummerTTS/build/tts_test",
        ),
        DeclareLaunchArgument(
            "summer_tts_model",
            default_value="/home/ubuntu/embodied_agent_ws/third_party/SummerTTS/models/single_speaker_fast.bin",
        ),
        DeclareLaunchArgument("summer_tts_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("summer_tts_service_name", default_value="/tts/synthesize"),
        DeclareLaunchArgument("summer_tts_service_timeout_s", default_value="10.0"),
        DeclareLaunchArgument("summer_tts_service_speaker_id", default_value="-1"),
        DeclareLaunchArgument("summer_tts_service_length_scale", default_value="0.0"),
        DeclareLaunchArgument("summer_tts_cache_enabled", default_value="true"),
        DeclareLaunchArgument("summer_tts_cache_max_entries", default_value="64"),
        DeclareLaunchArgument("summer_tts_cache_max_text_chars", default_value="24"),
        Node(
            package="embodied_agent_cpp",
            executable="summer_tts_service",
            name="summer_tts_service",
            output="screen",
            condition=IfCondition(
                PythonExpression(["'", tts_provider, "' == 'summer_ros'"])
            ),
            parameters=[{
                "model_path": summer_tts_model,
                "service_name": summer_tts_service_name,
                "speaker_id": ParameterValue(
                    summer_tts_service_speaker_id, value_type=int
                ),
                "length_scale": ParameterValue(
                    summer_tts_service_length_scale, value_type=float
                ),
                "cache_enabled": ParameterValue(
                    summer_tts_cache_enabled, value_type=bool
                ),
                "cache_max_entries": ParameterValue(
                    summer_tts_cache_max_entries, value_type=int
                ),
                "cache_max_text_chars": ParameterValue(
                    summer_tts_cache_max_text_chars, value_type=int
                ),
            }],
        ),
        DeclareLaunchArgument("hardware_backend", default_value="mock"),
        DeclareLaunchArgument("hardware_enabled", default_value="true"),
        DeclareLaunchArgument("lifecycle_autostart", default_value="true"),
        DeclareLaunchArgument("uart_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("uart_baud_rate", default_value="115200"),
        DeclareLaunchArgument("spi_device", default_value="/dev/spidev0.0"),
        DeclareLaunchArgument("spi_speed_hz", default_value="1000000"),
        LifecycleNode(
            package="embodied_offline_agent", executable="offline_agent",
            name="offline_agent", namespace="", output="screen",
            parameters=[config, {
                **agent_control_parameter_overrides(),
                "agent_lifecycle_autostart": False,
                "asr_hotwords_score": ParameterValue(
                    asr_hotwords_score, value_type=float
                ),
                "tts_provider": tts_provider,
                "summer_tts_binary": summer_tts_binary,
                "summer_tts_model": summer_tts_model,
                "summer_tts_timeout_s": ParameterValue(
                    summer_tts_timeout_s, value_type=float
                ),
                "summer_tts_service_name": summer_tts_service_name,
                "summer_tts_service_timeout_s": ParameterValue(
                    summer_tts_service_timeout_s, value_type=float
                ),
                "summer_tts_service_speaker_id": ParameterValue(
                    summer_tts_service_speaker_id, value_type=int
                ),
                "summer_tts_service_length_scale": ParameterValue(
                    summer_tts_service_length_scale, value_type=float
                ),
            }],
        ),
        *voice_frontend_nodes(config),
        LifecycleNode(
            package="embodied_agent_cpp", executable="action_guard",
            name="action_guard", namespace="", output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="agent_control_lifecycle_manager", output="screen",
            parameters=[{
                "autostart": ParameterValue(lifecycle_autostart, value_type=bool),
                "node_names": ["action_guard", "offline_agent"],
                "bond_timeout": 0.0,
            }],
        ),
        Node(
            package="embodied_agent_cpp", executable="hardware_controller",
            name="hardware_controller", output="screen",
            condition=IfCondition(hardware_enabled),
            parameters=[{
                "backend": hardware_backend,
                "uart_device": uart_device,
                "uart_baud_rate": ParameterValue(uart_baud_rate, value_type=int),
                "spi_device": spi_device,
                "spi_speed_hz": ParameterValue(spi_speed_hz, value_type=int),
            }],
        ),
    ])
