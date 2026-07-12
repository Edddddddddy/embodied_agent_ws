import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def include_launch(package, filename, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def as_python_bool(value):
    return PythonExpression(["'True' if '", value, "'.lower() == 'true' else 'False'"])


def generate_launch_description():
    simulation_share = get_package_share_directory("embodied_simulation")
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(nav2_share, "params", "nav2_params.yaml")
    nav2_map = os.path.join(simulation_share, "maps", "voice_demo.yaml")
    default_world = os.path.join(simulation_share, "worlds", "voice_demo.sdf.xacro")
    default_rviz = os.path.join(simulation_share, "rviz", "voice_nav2_demo.rviz")
    control_config = os.path.join(
        simulation_share, "config", "simulation_control.yaml"
    )

    launch_agent = LaunchConfiguration("launch_agent")
    agent_type = LaunchConfiguration("agent_type")
    provider_mode = LaunchConfiguration("provider_mode")
    microphone = LaunchConfiguration("microphone_enabled")
    capture_enabled = LaunchConfiguration("capture_enabled")
    speaker_enabled = LaunchConfiguration("speaker_enabled")
    vad_provider = LaunchConfiguration("vad_provider")
    speech_start_threshold = LaunchConfiguration("speech_start_threshold")
    vad_speech_start_ms = LaunchConfiguration("vad_speech_start_ms")
    speech_end_silence_s = LaunchConfiguration("speech_end_silence_s")
    min_utterance_ms = LaunchConfiguration("min_utterance_ms")
    max_utterance_s = LaunchConfiguration("max_utterance_s")
    silero_model_path = LaunchConfiguration("silero_model_path")
    silero_use_onnx = LaunchConfiguration("silero_use_onnx")
    silero_threshold = LaunchConfiguration("silero_threshold")
    silero_end_threshold = LaunchConfiguration("silero_end_threshold")
    kws_provider = LaunchConfiguration("kws_provider")
    sherpa_tokens = LaunchConfiguration("sherpa_tokens")
    sherpa_encoder = LaunchConfiguration("sherpa_encoder")
    sherpa_decoder = LaunchConfiguration("sherpa_decoder")
    sherpa_joiner = LaunchConfiguration("sherpa_joiner")
    sherpa_keywords_file = LaunchConfiguration("sherpa_keywords_file")
    openwakeword_models = LaunchConfiguration("openwakeword_models")
    openwakeword_threshold = LaunchConfiguration("openwakeword_threshold")
    livekit_wakeword_models = LaunchConfiguration("livekit_wakeword_models")
    livekit_wakeword_threshold = LaunchConfiguration("livekit_wakeword_threshold")
    audio_enhancer = LaunchConfiguration("audio_enhancer")
    aec_enabled = LaunchConfiguration("aec_enabled")
    noise_suppression_enabled = LaunchConfiguration("noise_suppression_enabled")
    auto_gain_enabled = LaunchConfiguration("auto_gain_enabled")
    continuous_control = LaunchConfiguration("continuous_control_enabled")
    voice_session_timeout_s = LaunchConfiguration("voice_session_timeout_s")
    continuous_command_queue_size = LaunchConfiguration("continuous_command_queue_size")
    continuous_command_max_age_s = LaunchConfiguration("continuous_command_max_age_s")
    continuous_duplicate_window_s = LaunchConfiguration("continuous_duplicate_window_s")
    command_normalization_enabled = LaunchConfiguration("command_normalization_enabled")
    command_normalization_feedback_enabled = LaunchConfiguration(
        "command_normalization_feedback_enabled"
    )
    command_normalization_fuzzy_threshold = LaunchConfiguration(
        "command_normalization_fuzzy_threshold"
    )
    command_normalization_path = LaunchConfiguration("command_normalization_path")
    command_completion_enabled = LaunchConfiguration("command_completion_enabled")
    asr_commit_delay_ms = LaunchConfiguration("asr_commit_delay_ms")
    wake_word = LaunchConfiguration("wake_word_enabled")
    lifecycle_autostart = LaunchConfiguration("lifecycle_autostart")
    nav_action_timeout_s = LaunchConfiguration("nav_action_timeout_s")
    slam = LaunchConfiguration("slam")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    use_composition = LaunchConfiguration("use_composition")

    online_condition = IfCondition(
        PythonExpression([
            "'", launch_agent, "' == 'true' and '", agent_type, "' == 'online'"
        ])
    )
    offline_condition = IfCondition(
        PythonExpression([
            "'", launch_agent, "' == 'true' and '", agent_type, "' == 'offline'"
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument("launch_agent", default_value="true"),
        DeclareLaunchArgument("agent_type", default_value="online"),
        DeclareLaunchArgument("provider_mode", default_value="mock"),
        DeclareLaunchArgument("microphone_enabled", default_value="false"),
        DeclareLaunchArgument("capture_enabled", default_value=microphone),
        DeclareLaunchArgument("speaker_enabled", default_value="false"),
        DeclareLaunchArgument("vad_provider", default_value="energy"),
        DeclareLaunchArgument("speech_start_threshold", default_value="0.018"),
        DeclareLaunchArgument("vad_speech_start_ms", default_value="96.0"),
        DeclareLaunchArgument("speech_end_silence_s", default_value="0.7"),
        DeclareLaunchArgument("min_utterance_ms", default_value="100.0"),
        DeclareLaunchArgument("max_utterance_s", default_value="12.0"),
        DeclareLaunchArgument("silero_model_path", default_value=""),
        DeclareLaunchArgument("silero_use_onnx", default_value="true"),
        DeclareLaunchArgument("silero_threshold", default_value="0.5"),
        DeclareLaunchArgument("silero_end_threshold", default_value="0.35"),
        DeclareLaunchArgument("kws_provider", default_value="none"),
        DeclareLaunchArgument("sherpa_tokens", default_value=""),
        DeclareLaunchArgument("sherpa_encoder", default_value=""),
        DeclareLaunchArgument("sherpa_decoder", default_value=""),
        DeclareLaunchArgument("sherpa_joiner", default_value=""),
        DeclareLaunchArgument("sherpa_keywords_file", default_value=""),
        DeclareLaunchArgument("openwakeword_models", default_value=""),
        DeclareLaunchArgument("openwakeword_threshold", default_value="0.5"),
        DeclareLaunchArgument("livekit_wakeword_models", default_value=""),
        DeclareLaunchArgument("livekit_wakeword_threshold", default_value="0.5"),
        DeclareLaunchArgument("audio_enhancer", default_value="nlms"),
        DeclareLaunchArgument("aec_enabled", default_value="true"),
        DeclareLaunchArgument("noise_suppression_enabled", default_value="false"),
        DeclareLaunchArgument("auto_gain_enabled", default_value="false"),
        DeclareLaunchArgument("continuous_control_enabled", default_value="false"),
        DeclareLaunchArgument("voice_session_timeout_s", default_value="120.0"),
        DeclareLaunchArgument("continuous_command_queue_size", default_value="8"),
        DeclareLaunchArgument("continuous_command_max_age_s", default_value="120.0"),
        DeclareLaunchArgument("continuous_duplicate_window_s", default_value="1.2"),
        DeclareLaunchArgument("command_normalization_enabled", default_value="true"),
        DeclareLaunchArgument(
            "command_normalization_feedback_enabled", default_value="true"
        ),
        DeclareLaunchArgument("command_normalization_fuzzy_threshold", default_value="0.82"),
        DeclareLaunchArgument("command_normalization_path", default_value=""),
        DeclareLaunchArgument("command_completion_enabled", default_value="true"),
        DeclareLaunchArgument("asr_commit_delay_ms", default_value="300"),
        DeclareLaunchArgument("wake_word_enabled", default_value="true"),
        DeclareLaunchArgument("lifecycle_autostart", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("map", default_value=nav2_map),
        DeclareLaunchArgument("params_file", default_value=nav2_params),
        DeclareLaunchArgument("rviz_config_file", default_value=default_rviz),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("use_composition", default_value="true"),
        DeclareLaunchArgument("nav_action_timeout_s", default_value="180.0"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        # 复用 Nav2 官方 TurtleBot3 bringup 的成熟导航栈；默认 map/world/RViz
        # 指向本项目资产，让演示脚本、审计报告和简历讲解都有稳定的项目内入口。
        include_launch(
            "nav2_bringup",
            "tb3_simulation_launch.py",
            {
                # Nav2 官方 launch 内部使用 PythonExpression(['not ', use_composition])
                # 这类表达式，必须收到 Python 可识别的 True/False；本项目对外仍保留
                # ROS 常见的小写 true/false 参数，避免用户命令行习惯被打破。
                "slam": as_python_bool(slam),
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "rviz_config_file": rviz_config_file,
                "use_rviz": as_python_bool(use_rviz),
                "headless": as_python_bool(headless),
                "world": world,
                "autostart": "true",
                "use_sim_time": "true",
                "use_composition": as_python_bool(use_composition),
                "x_pose": LaunchConfiguration("x_pose"),
                "y_pose": LaunchConfiguration("y_pose"),
                "yaw": LaunchConfiguration("yaw"),
            },
        ),
        include_launch(
            "embodied_simulation",
            "simulation_control.launch.py",
            {
                "config": control_config,
                "use_sim_time": "true",
                "use_typed_actions": "true",
                "use_behavior_tree": "true",
                "executor_plugin": "embodied_simulation/Nav2RobotExecutor",
                "autostart": lifecycle_autostart,
                "action_timeout_s": nav_action_timeout_s,
                "readiness_profile": "voice_nav2",
                "readiness_required_components": (
                    "audio_frontend,agent,action_guard,"
                    "typed_action_bridge,simulation_control"
                ),
            },
        ),
        include_launch(
            "embodied_online_agent",
            "online_agent.launch.py",
            {
                "mode": provider_mode,
                "microphone_enabled": microphone,
                "capture_enabled": capture_enabled,
                "speaker_enabled": speaker_enabled,
                "vad_provider": vad_provider,
                "speech_start_threshold": speech_start_threshold,
                "vad_speech_start_ms": vad_speech_start_ms,
                "speech_end_silence_s": speech_end_silence_s,
                "min_utterance_ms": min_utterance_ms,
                "max_utterance_s": max_utterance_s,
                "silero_model_path": silero_model_path,
                "silero_use_onnx": silero_use_onnx,
                "silero_threshold": silero_threshold,
                "silero_end_threshold": silero_end_threshold,
                "kws_provider": kws_provider,
                "sherpa_tokens": sherpa_tokens,
                "sherpa_encoder": sherpa_encoder,
                "sherpa_decoder": sherpa_decoder,
                "sherpa_joiner": sherpa_joiner,
                "sherpa_keywords_file": sherpa_keywords_file,
                "openwakeword_models": openwakeword_models,
                "openwakeword_threshold": openwakeword_threshold,
                "livekit_wakeword_models": livekit_wakeword_models,
                "livekit_wakeword_threshold": livekit_wakeword_threshold,
                "audio_enhancer": audio_enhancer,
                "aec_enabled": aec_enabled,
                "noise_suppression_enabled": noise_suppression_enabled,
                "auto_gain_enabled": auto_gain_enabled,
                "wake_word_enabled": wake_word,
                "continuous_control_enabled": continuous_control,
                "voice_session_timeout_s": voice_session_timeout_s,
                "continuous_command_queue_size": continuous_command_queue_size,
                "continuous_command_max_age_s": continuous_command_max_age_s,
                "continuous_duplicate_window_s": continuous_duplicate_window_s,
                "command_normalization_enabled": command_normalization_enabled,
                "command_normalization_feedback_enabled": command_normalization_feedback_enabled,
                "command_normalization_fuzzy_threshold": command_normalization_fuzzy_threshold,
                "command_normalization_path": command_normalization_path,
                "command_completion_enabled": command_completion_enabled,
                "asr_commit_delay_ms": asr_commit_delay_ms,
                "hardware_enabled": "false",
                "lifecycle_autostart": lifecycle_autostart,
            },
            online_condition,
        ),
        include_launch(
            "embodied_offline_agent",
            "offline_agent.launch.py",
            {
                "mode": provider_mode,
                "microphone_enabled": microphone,
                "capture_enabled": capture_enabled,
                "speaker_enabled": speaker_enabled,
                "vad_provider": vad_provider,
                "speech_start_threshold": speech_start_threshold,
                "vad_speech_start_ms": vad_speech_start_ms,
                "speech_end_silence_s": speech_end_silence_s,
                "min_utterance_ms": min_utterance_ms,
                "max_utterance_s": max_utterance_s,
                "silero_model_path": silero_model_path,
                "silero_use_onnx": silero_use_onnx,
                "silero_threshold": silero_threshold,
                "silero_end_threshold": silero_end_threshold,
                "kws_provider": kws_provider,
                "sherpa_tokens": sherpa_tokens,
                "sherpa_encoder": sherpa_encoder,
                "sherpa_decoder": sherpa_decoder,
                "sherpa_joiner": sherpa_joiner,
                "sherpa_keywords_file": sherpa_keywords_file,
                "openwakeword_models": openwakeword_models,
                "openwakeword_threshold": openwakeword_threshold,
                "livekit_wakeword_models": livekit_wakeword_models,
                "livekit_wakeword_threshold": livekit_wakeword_threshold,
                "audio_enhancer": audio_enhancer,
                "aec_enabled": aec_enabled,
                "noise_suppression_enabled": noise_suppression_enabled,
                "auto_gain_enabled": auto_gain_enabled,
                "wake_word_enabled": wake_word,
                "continuous_control_enabled": continuous_control,
                "voice_session_timeout_s": voice_session_timeout_s,
                "continuous_command_queue_size": continuous_command_queue_size,
                "continuous_command_max_age_s": continuous_command_max_age_s,
                "continuous_duplicate_window_s": continuous_duplicate_window_s,
                "command_normalization_enabled": command_normalization_enabled,
                "command_normalization_feedback_enabled": command_normalization_feedback_enabled,
                "command_normalization_fuzzy_threshold": command_normalization_fuzzy_threshold,
                "command_normalization_path": command_normalization_path,
                "command_completion_enabled": command_completion_enabled,
                "asr_commit_delay_ms": asr_commit_delay_ms,
                "hardware_enabled": "false",
                "lifecycle_autostart": lifecycle_autostart,
            },
            offline_condition,
        ),
        LifecycleNode(
            package="embodied_agent_cpp",
            executable="action_guard",
            name="action_guard",
            namespace="",
            output="screen",
            condition=UnlessCondition(launch_agent),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="action_guard_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": ParameterValue(lifecycle_autostart, value_type=bool),
                "node_names": ["action_guard"],
                "bond_timeout": 0.0,
            }],
            condition=UnlessCondition(launch_agent),
        ),
    ])
