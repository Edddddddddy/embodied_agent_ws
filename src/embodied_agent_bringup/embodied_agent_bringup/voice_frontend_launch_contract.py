"""在线/离线 Agent 共用的语音前端 launch 契约。"""

from __future__ import annotations

from typing import Any

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from .agent_launch_contract import LaunchArgumentSpec, launch_default


VOICE_FRONTEND_ARGUMENTS = (
    LaunchArgumentSpec("capture_enabled", bool, "Capture microphone audio."),
    LaunchArgumentSpec("speaker_enabled", bool, "Enable playback reference input."),
    LaunchArgumentSpec("vad_provider", str, "Endpoint provider: energy/silero/webrtc."),
    LaunchArgumentSpec("speech_start_threshold", float, "Energy VAD RMS threshold."),
    LaunchArgumentSpec("vad_speech_start_ms", float, "Speech start debounce."),
    LaunchArgumentSpec("speech_end_silence_s", float, "Endpoint trailing silence."),
    LaunchArgumentSpec("min_utterance_ms", float, "Minimum accepted utterance."),
    LaunchArgumentSpec("max_utterance_s", float, "Maximum utterance duration."),
    LaunchArgumentSpec("silero_model_path", str, "Silero VAD ONNX model."),
    LaunchArgumentSpec("silero_use_onnx", bool, "Use ONNX Silero provider."),
    LaunchArgumentSpec("silero_threshold", float, "Silero speech threshold."),
    LaunchArgumentSpec("silero_end_threshold", float, "Silero end threshold."),
    LaunchArgumentSpec("kws_provider", str, "Keyword spotting provider."),
    LaunchArgumentSpec("sherpa_tokens", str, "Sherpa KWS tokens."),
    LaunchArgumentSpec("sherpa_encoder", str, "Sherpa KWS encoder."),
    LaunchArgumentSpec("sherpa_decoder", str, "Sherpa KWS decoder."),
    LaunchArgumentSpec("sherpa_joiner", str, "Sherpa KWS joiner."),
    LaunchArgumentSpec("sherpa_keywords_file", str, "Sherpa keyword file."),
    LaunchArgumentSpec("openwakeword_models", str, "openWakeWord models."),
    LaunchArgumentSpec("openwakeword_threshold", float, "openWakeWord threshold."),
    LaunchArgumentSpec("livekit_wakeword_models", str, "LiveKit wake models."),
    LaunchArgumentSpec("livekit_wakeword_threshold", float, "LiveKit wake threshold."),
    LaunchArgumentSpec("speaker_identity_enabled", bool, "Enable speaker identity."),
    LaunchArgumentSpec("speaker_identity_mode", str, "Speaker identity provider."),
    LaunchArgumentSpec("speaker_identity_sherpa_model", str, "Speaker model."),
    LaunchArgumentSpec("speaker_identity_sherpa_file", str, "Speaker enrollment file."),
    LaunchArgumentSpec("speaker_identity_min_margin", float, "Identity score margin."),
    LaunchArgumentSpec("audio_enhancer", str, "Audio enhancement backend."),
    LaunchArgumentSpec("aec_enabled", bool, "Enable echo cancellation."),
    LaunchArgumentSpec("noise_suppression_enabled", bool, "Enable noise suppression."),
    LaunchArgumentSpec("auto_gain_enabled", bool, "Enable automatic gain."),
)

VOICE_FRONTEND_DEFAULTS: dict[str, Any] = {
    "capture_enabled": False,
    "speaker_enabled": False,
    "vad_provider": "energy",
    "speech_start_threshold": 0.018,
    "vad_speech_start_ms": 96.0,
    "speech_end_silence_s": 0.4,
    "min_utterance_ms": 100.0,
    "max_utterance_s": 12.0,
    "silero_model_path": "",
    "silero_use_onnx": True,
    "silero_threshold": 0.5,
    "silero_end_threshold": 0.35,
    "kws_provider": "none",
    "sherpa_tokens": "",
    "sherpa_encoder": "",
    "sherpa_decoder": "",
    "sherpa_joiner": "",
    "sherpa_keywords_file": "",
    "openwakeword_models": "",
    "openwakeword_threshold": 0.5,
    "livekit_wakeword_models": "",
    "livekit_wakeword_threshold": 0.5,
    "speaker_identity_enabled": False,
    "speaker_identity_mode": "mock",
    "speaker_identity_sherpa_model": "",
    "speaker_identity_sherpa_file": "",
    "speaker_identity_min_margin": 0.05,
    "audio_enhancer": "nlms",
    "aec_enabled": True,
    "noise_suppression_enabled": False,
    "auto_gain_enabled": False,
}


def declare_voice_frontend_arguments(capture_default: Any):
    defaults = {**VOICE_FRONTEND_DEFAULTS, "capture_enabled": capture_default}
    return [
        DeclareLaunchArgument(
            spec.name,
            default_value=(
                defaults[spec.name]
                if spec.name == "capture_enabled"
                else launch_default(defaults[spec.name])
            ),
            description=spec.description,
        )
        for spec in VOICE_FRONTEND_ARGUMENTS
    ]


def voice_frontend_configurations() -> dict[str, LaunchConfiguration]:
    return {spec.name: LaunchConfiguration(spec.name) for spec in VOICE_FRONTEND_ARGUMENTS}


def _typed(name: str, value_type: type, configs: dict[str, LaunchConfiguration]):
    # launch 参数天然是字符串替换；这里集中附加 ROS 类型，避免两个入口各自漏标 bool/float。
    value = configs[name]
    return value if value_type is str else ParameterValue(value, value_type=value_type)


def voice_frontend_nodes(config: Any):
    """构造共享前端节点；调用方只决定 provider YAML，不拥有节点内部映射。"""
    c = voice_frontend_configurations()
    typed = {spec.name: _typed(spec.name, spec.value_type, c) for spec in VOICE_FRONTEND_ARGUMENTS}
    # 外部 VAD sidecar 接管 endpoint 时，音频节点必须停止重复发布 start/end 事件。
    external_endpoint = ParameterValue(
        PythonExpression(["'", c["vad_provider"], "' != 'silero' and '", c["vad_provider"], "' != 'webrtc'"]),
        value_type=bool,
    )
    endpoint_params = {
        "speech_start_ms": typed["vad_speech_start_ms"],
        "speech_end_silence_s": typed["speech_end_silence_s"],
        "min_utterance_ms": typed["min_utterance_ms"],
        "max_utterance_s": typed["max_utterance_s"],
    }
    return [
        Node(
            package="embodied_voice_frontend", executable="speaker_identity",
            name="speaker_identity", output="screen",
            condition=IfCondition(c["speaker_identity_enabled"]),
            parameters=[{
                "mode": c["speaker_identity_mode"],
                "sherpa_model": c["speaker_identity_sherpa_model"],
                "sherpa_speaker_file": c["speaker_identity_sherpa_file"],
                "sherpa_min_margin": typed["speaker_identity_min_margin"],
            }],
        ),
        Node(
            package="embodied_agent_cpp", executable="audio_frontend",
            name="audio_frontend", output="screen", parameters=[config, {
                "capture_enabled": typed["capture_enabled"],
                "speaker_enabled": typed["speaker_enabled"],
                "vad_provider": c["vad_provider"],
                "vad_rms_threshold": typed["speech_start_threshold"],
                "speech_end_silence_s": typed["speech_end_silence_s"],
                "min_utterance_ms": typed["min_utterance_ms"],
                "max_utterance_s": typed["max_utterance_s"],
                "audio_enhancer": c["audio_enhancer"],
                "aec_enabled": typed["aec_enabled"],
                "noise_suppression_enabled": typed["noise_suppression_enabled"],
                "auto_gain_enabled": typed["auto_gain_enabled"],
                "endpoint_events_enabled": external_endpoint,
            }],
        ),
        Node(
            package="embodied_voice_frontend", executable="webrtc_vad",
            name="webrtc_vad", output="screen",
            condition=IfCondition(PythonExpression(["'", c["vad_provider"], "' == 'webrtc'"])),
            parameters=[config, endpoint_params],
        ),
        Node(
            package="embodied_voice_frontend", executable="silero_vad",
            name="silero_vad", output="screen",
            condition=IfCondition(PythonExpression(["'", c["vad_provider"], "' == 'silero'"])),
            parameters=[config, {
                "model_path": c["silero_model_path"],
                "use_onnx": typed["silero_use_onnx"],
                "threshold": typed["silero_threshold"],
                "speech_end_threshold": typed["silero_end_threshold"],
                **endpoint_params,
            }],
        ),
        Node(
            package="embodied_voice_frontend", executable="keyword_wake",
            name="keyword_wake", output="screen",
            condition=IfCondition(PythonExpression(["'", c["kws_provider"], "' != 'none'"])),
            parameters=[config, {
                "mode": c["kws_provider"],
                "sherpa_tokens": c["sherpa_tokens"],
                "sherpa_encoder": c["sherpa_encoder"],
                "sherpa_decoder": c["sherpa_decoder"],
                "sherpa_joiner": c["sherpa_joiner"],
                "sherpa_keywords_file": c["sherpa_keywords_file"],
                "openwakeword_models": c["openwakeword_models"],
                "openwakeword_threshold": typed["openwakeword_threshold"],
                "livekit_wakeword_models": c["livekit_wakeword_models"],
                "livekit_wakeword_threshold": typed["livekit_wakeword_threshold"],
            }],
        ),
    ]
