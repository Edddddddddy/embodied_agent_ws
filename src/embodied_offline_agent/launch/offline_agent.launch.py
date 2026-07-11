from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("embodied_offline_agent"), "config", "offline_agent.yaml"
    )
    mode = LaunchConfiguration("mode")
    microphone = LaunchConfiguration("microphone_enabled")
    capture = LaunchConfiguration("capture_enabled")
    speaker = LaunchConfiguration("speaker_enabled")
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
    speaker_identity_enabled = LaunchConfiguration("speaker_identity_enabled")
    speaker_identity_mode = LaunchConfiguration("speaker_identity_mode")
    speaker_identity_sherpa_model = LaunchConfiguration("speaker_identity_sherpa_model")
    speaker_identity_sherpa_file = LaunchConfiguration("speaker_identity_sherpa_file")
    speaker_identity_min_margin = LaunchConfiguration("speaker_identity_min_margin")
    audio_enhancer = LaunchConfiguration("audio_enhancer")
    aec_enabled = LaunchConfiguration("aec_enabled")
    noise_suppression_enabled = LaunchConfiguration("noise_suppression_enabled")
    auto_gain_enabled = LaunchConfiguration("auto_gain_enabled")
    continuous_control_enabled = LaunchConfiguration("continuous_control_enabled")
    voice_session_timeout_s = LaunchConfiguration("voice_session_timeout_s")
    continuous_command_queue_size = LaunchConfiguration("continuous_command_queue_size")
    continuous_command_max_age_s = LaunchConfiguration("continuous_command_max_age_s")
    continuous_duplicate_window_s = LaunchConfiguration("continuous_duplicate_window_s")
    user_memory_retention_days = LaunchConfiguration("user_memory_retention_days")
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
    asr_hotwords_score = LaunchConfiguration("asr_hotwords_score")
    asr_partial_merge_enabled = LaunchConfiguration("asr_partial_merge_enabled")
    asr_partial_max_age_s = LaunchConfiguration("asr_partial_max_age_s")
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
    wake_word_enabled = LaunchConfiguration("wake_word_enabled")
    lifecycle_autostart = LaunchConfiguration("lifecycle_autostart")
    uart_device = LaunchConfiguration("uart_device")
    uart_baud_rate = LaunchConfiguration("uart_baud_rate")
    spi_device = LaunchConfiguration("spi_device")
    spi_speed_hz = LaunchConfiguration("spi_speed_hz")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mock"),
        DeclareLaunchArgument("microphone_enabled", default_value="false"),
        DeclareLaunchArgument("capture_enabled", default_value=microphone),
        DeclareLaunchArgument("speaker_enabled", default_value="false"),
        DeclareLaunchArgument("vad_provider", default_value="energy"),
        DeclareLaunchArgument("speech_start_threshold", default_value="0.018"),
        DeclareLaunchArgument("vad_speech_start_ms", default_value="96.0"),
        DeclareLaunchArgument("speech_end_silence_s", default_value="0.4"),
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
        DeclareLaunchArgument("speaker_identity_enabled", default_value="false"),
        DeclareLaunchArgument("speaker_identity_mode", default_value="mock"),
        DeclareLaunchArgument("speaker_identity_sherpa_model", default_value=""),
        DeclareLaunchArgument("speaker_identity_sherpa_file", default_value=""),
        DeclareLaunchArgument("speaker_identity_min_margin", default_value="0.05"),
        DeclareLaunchArgument("audio_enhancer", default_value="nlms"),
        DeclareLaunchArgument("aec_enabled", default_value="true"),
        DeclareLaunchArgument("noise_suppression_enabled", default_value="false"),
        DeclareLaunchArgument("auto_gain_enabled", default_value="false"),
        DeclareLaunchArgument("continuous_control_enabled", default_value="false"),
        DeclareLaunchArgument("voice_session_timeout_s", default_value="60.0"),
        DeclareLaunchArgument("continuous_command_queue_size", default_value="8"),
        DeclareLaunchArgument("continuous_command_max_age_s", default_value="30.0"),
        DeclareLaunchArgument("continuous_duplicate_window_s", default_value="1.2"),
        DeclareLaunchArgument("user_memory_retention_days", default_value="90.0"),
        DeclareLaunchArgument("command_normalization_enabled", default_value="true"),
        DeclareLaunchArgument(
            "command_normalization_feedback_enabled", default_value="true"
        ),
        DeclareLaunchArgument(
            "command_normalization_fuzzy_threshold", default_value="0.82"
        ),
        DeclareLaunchArgument("command_normalization_path", default_value=""),
        DeclareLaunchArgument("command_completion_enabled", default_value="true"),
        DeclareLaunchArgument("asr_commit_delay_ms", default_value="0"),
        DeclareLaunchArgument("asr_hotwords_score", default_value="3.0"),
        DeclareLaunchArgument("asr_partial_merge_enabled", default_value="true"),
        DeclareLaunchArgument("asr_partial_max_age_s", default_value="2.0"),
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
        DeclareLaunchArgument("wake_word_enabled", default_value="true"),
        DeclareLaunchArgument("lifecycle_autostart", default_value="true"),
        DeclareLaunchArgument("uart_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("uart_baud_rate", default_value="115200"),
        DeclareLaunchArgument("spi_device", default_value="/dev/spidev0.0"),
        DeclareLaunchArgument("spi_speed_hz", default_value="1000000"),
        Node(
            package="embodied_offline_agent", executable="offline_agent",
            name="offline_agent", output="screen",
            parameters=[config, {
                "mode": mode,
                "microphone_enabled": microphone,
                "wake_word_enabled": ParameterValue(wake_word_enabled, value_type=bool),
                "continuous_control_enabled": ParameterValue(
                    continuous_control_enabled, value_type=bool
                ),
                "voice_session_timeout_s": ParameterValue(
                    voice_session_timeout_s, value_type=float
                ),
                "continuous_command_queue_size": ParameterValue(
                    continuous_command_queue_size, value_type=int
                ),
                "continuous_command_max_age_s": ParameterValue(
                    continuous_command_max_age_s, value_type=float
                ),
                "continuous_duplicate_window_s": ParameterValue(
                    continuous_duplicate_window_s, value_type=float
                ),
                "user_memory_retention_days": ParameterValue(
                    user_memory_retention_days, value_type=float
                ),
                "command_normalization_enabled": ParameterValue(
                    command_normalization_enabled, value_type=bool
                ),
                "command_normalization_feedback_enabled": ParameterValue(
                    command_normalization_feedback_enabled, value_type=bool
                ),
                "command_normalization_fuzzy_threshold": ParameterValue(
                    command_normalization_fuzzy_threshold, value_type=float
                ),
                "command_normalization_path": command_normalization_path,
                "command_completion_enabled": ParameterValue(
                    command_completion_enabled, value_type=bool
                ),
                "asr_commit_delay_ms": ParameterValue(
                    asr_commit_delay_ms, value_type=int
                ),
                "asr_hotwords_score": ParameterValue(
                    asr_hotwords_score, value_type=float
                ),
                "asr_partial_merge_enabled": ParameterValue(
                    asr_partial_merge_enabled, value_type=bool
                ),
                "asr_partial_max_age_s": ParameterValue(
                    asr_partial_max_age_s, value_type=float
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
        Node(
            package="embodied_online_agent",
            executable="speaker_identity",
            name="speaker_identity",
            output="screen",
            condition=IfCondition(speaker_identity_enabled),
            parameters=[{
                "mode": speaker_identity_mode,
                "sherpa_model": speaker_identity_sherpa_model,
                "sherpa_speaker_file": speaker_identity_sherpa_file,
                "sherpa_min_margin": ParameterValue(
                    speaker_identity_min_margin, value_type=float
                ),
            }],
        ),
        Node(
            package="embodied_agent_cpp", executable="audio_frontend",
            name="audio_frontend", output="screen",
            parameters=[config, {
                "capture_enabled": capture,
                "speaker_enabled": speaker,
                "vad_provider": vad_provider,
                "vad_rms_threshold": ParameterValue(
                    speech_start_threshold, value_type=float
                ),
                "speech_end_silence_s": ParameterValue(
                    speech_end_silence_s, value_type=float
                ),
                "min_utterance_ms": ParameterValue(min_utterance_ms, value_type=float),
                "max_utterance_s": ParameterValue(max_utterance_s, value_type=float),
                "audio_enhancer": audio_enhancer,
                "aec_enabled": ParameterValue(aec_enabled, value_type=bool),
                "noise_suppression_enabled": ParameterValue(
                    noise_suppression_enabled, value_type=bool
                ),
                "auto_gain_enabled": ParameterValue(auto_gain_enabled, value_type=bool),
                "endpoint_events_enabled": ParameterValue(
                    PythonExpression([
                        "'", vad_provider,
                        "' != 'silero' and '", vad_provider, "' != 'webrtc'",
                    ]),
                    value_type=bool,
                ),
            }],
        ),
        Node(
            package="embodied_online_agent", executable="silero_vad",
            name="silero_vad", output="screen",
            condition=IfCondition(
                PythonExpression(["'", vad_provider, "' == 'silero'"])
            ),
            parameters=[
                config,
                {
                    "model_path": silero_model_path,
                    "use_onnx": ParameterValue(silero_use_onnx, value_type=bool),
                    "threshold": ParameterValue(silero_threshold, value_type=float),
                    "speech_start_ms": ParameterValue(
                        vad_speech_start_ms, value_type=float
                    ),
                    "speech_end_threshold": ParameterValue(
                        silero_end_threshold, value_type=float
                    ),
                    "speech_end_silence_s": ParameterValue(
                        speech_end_silence_s, value_type=float
                    ),
                    "min_utterance_ms": ParameterValue(
                        min_utterance_ms, value_type=float
                    ),
                    "max_utterance_s": ParameterValue(
                        max_utterance_s, value_type=float
                    ),
                },
            ],
        ),
        Node(
            package="embodied_online_agent", executable="webrtc_vad",
            name="webrtc_vad", output="screen",
            condition=IfCondition(
                PythonExpression(["'", vad_provider, "' == 'webrtc'"])
            ),
            parameters=[
                config,
                {
                    "speech_start_ms": ParameterValue(
                        vad_speech_start_ms, value_type=float
                    ),
                    "speech_end_silence_s": ParameterValue(
                        speech_end_silence_s, value_type=float
                    ),
                    "min_utterance_ms": ParameterValue(
                        min_utterance_ms, value_type=float
                    ),
                    "max_utterance_s": ParameterValue(
                        max_utterance_s, value_type=float
                    ),
                },
            ],
        ),
        Node(
            package="embodied_online_agent", executable="keyword_wake",
            name="keyword_wake", output="screen",
            condition=IfCondition(
                PythonExpression(["'", kws_provider, "' != 'none'"])
            ),
            parameters=[
                config,
                {
                    "mode": kws_provider,
                    "sherpa_tokens": sherpa_tokens,
                    "sherpa_encoder": sherpa_encoder,
                    "sherpa_decoder": sherpa_decoder,
                    "sherpa_joiner": sherpa_joiner,
                    "sherpa_keywords_file": sherpa_keywords_file,
                    "openwakeword_models": openwakeword_models,
                    "openwakeword_threshold": ParameterValue(
                        openwakeword_threshold, value_type=float
                    ),
                    "livekit_wakeword_models": livekit_wakeword_models,
                    "livekit_wakeword_threshold": ParameterValue(
                        livekit_wakeword_threshold, value_type=float
                    ),
                },
            ],
        ),
        LifecycleNode(
            package="embodied_agent_cpp", executable="action_guard",
            name="action_guard", namespace="", output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="action_guard_lifecycle_manager", output="screen",
            parameters=[{
                "autostart": ParameterValue(lifecycle_autostart, value_type=bool),
                "node_names": ["action_guard"],
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
