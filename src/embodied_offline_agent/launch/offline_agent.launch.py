from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

from embodied_agent_bringup.agent_deployment_launch_contract import (
    agent_deployment_nodes,
    declare_agent_deployment_arguments,
)
from embodied_agent_bringup.agent_launch_contract import (
    agent_control_configurations,
    agent_control_parameter_overrides,
    declare_agent_control_arguments,
)
from embodied_agent_bringup.voice_frontend_launch_contract import (
    declare_voice_frontend_arguments,
    voice_frontend_nodes,
)
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    runtime_root = os.path.expanduser(
        os.environ.get(
            "EMBODIED_RUNTIME_ROOT",
            os.path.join(os.path.expanduser("~"), "embodied_agent_ws"),
        )
    )
    asr_model_default = os.path.join(
        runtime_root,
        "models",
        "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16",
    )
    tts_model_default = os.path.join(
        runtime_root, "models", "vits-melo-tts-zh_en"
    )
    summer_binary_default = os.path.join(
        runtime_root, "third_party", "SummerTTS", "build", "tts_test"
    )
    summer_model_default = os.path.join(
        runtime_root,
        "third_party",
        "SummerTTS",
        "models",
        "single_speaker_fast.bin",
    )
    config = os.path.join(
        get_package_share_directory("embodied_offline_agent"), "config", "offline_agent.yaml"
    )
    agent_config = agent_control_configurations()
    microphone = agent_config["microphone_enabled"]
    asr_model_dir = LaunchConfiguration("asr_model_dir")
    asr_hotwords_score = LaunchConfiguration("asr_hotwords_score")
    tts_model_dir = LaunchConfiguration("tts_model_dir")
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
    return LaunchDescription([
        # 与在线 launch 共用参数接口和默认值来源，避免两条链路配置漂移。
        *declare_agent_control_arguments("offline"),
        *declare_voice_frontend_arguments(microphone),
        # 代码/install 跟随 worktree，数 GB 模型默认复用共享 runtime root；
        # launch 参数仍允许部署者显式覆盖单个 provider。
        DeclareLaunchArgument("asr_model_dir", default_value=asr_model_default),
        DeclareLaunchArgument("asr_hotwords_score", default_value="3.0"),
        DeclareLaunchArgument("tts_model_dir", default_value=tts_model_default),
        DeclareLaunchArgument("tts_provider", default_value="sherpa"),
        DeclareLaunchArgument(
            "summer_tts_binary",
            default_value=summer_binary_default,
        ),
        DeclareLaunchArgument(
            "summer_tts_model",
            default_value=summer_model_default,
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
        *declare_agent_deployment_arguments(),
        LifecycleNode(
            package="embodied_offline_agent", executable="offline_agent",
            name="offline_agent", namespace="", output="screen",
            parameters=[config, {
                **agent_control_parameter_overrides(),
                "agent_lifecycle_autostart": False,
                "asr_model_dir": asr_model_dir,
                "asr_hotwords_score": ParameterValue(
                    asr_hotwords_score, value_type=float
                ),
                "tts_model_dir": tts_model_dir,
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
        *agent_deployment_nodes("offline_agent"),
    ])
