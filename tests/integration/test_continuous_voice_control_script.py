#!/usr/bin/env python3
"""Checks for the human-facing continuous voice control launcher script."""

import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_continuous_voice_control_prints_resolved_config_without_microphone():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_SESSION_TIMEOUT": "44",
            "WAKE_WORD_ENABLED": "false",
            "SPEAKER_ENABLED": "true",
            "PULSE_CAPTURE_BRIDGE": "false",
            "VAD_PROVIDER": "silero",
            "SPEECH_START_THRESHOLD": "0.021",
            "VAD_SPEECH_START_MS": "128",
            "SPEECH_END_SILENCE_S": "0.38",
            "MIN_UTTERANCE_MS": "240",
            "MAX_UTTERANCE_S": "7.5",
            "SILERO_VAD_MODEL_PATH": "/models/vad/silero_vad.onnx",
            "SILERO_VAD_USE_ONNX": "true",
            "SILERO_VAD_THRESHOLD": "0.61",
            "SILERO_VAD_END_THRESHOLD": "0.33",
            "KWS_PROVIDER": "openwakeword",
            "SHERPA_KWS_TOKENS": "/models/kws/tokens.txt",
            "SHERPA_KWS_ENCODER": "/models/kws/encoder.onnx",
            "SHERPA_KWS_DECODER": "/models/kws/decoder.onnx",
            "SHERPA_KWS_JOINER": "/models/kws/joiner.onnx",
            "SHERPA_KWS_KEYWORDS_FILE": "/models/kws/keywords.txt",
            "OPENWAKEWORD_MODELS": "/models/kws/xiaozhi.onnx,/models/kws/nihaoxiaozhi.onnx",
            "OPENWAKEWORD_THRESHOLD": "0.42",
            "LIVEKIT_WAKEWORD_MODELS": "/models/kws/livekit-xiaozhi.onnx",
            "LIVEKIT_WAKEWORD_THRESHOLD": "0.63",
            "AUDIO_ENHANCER": "webrtc",
            "AEC_ENABLED": "false",
            "NOISE_SUPPRESSION_ENABLED": "true",
            "AUTO_GAIN_ENABLED": "true",
            "CONTINUOUS_MONITOR_ENABLED": "false",
            "CONTINUOUS_MONITOR_AUDIO_SAMPLE_LIMIT": "42",
            "CONTINUOUS_SAMPLE_LOG": "/tmp/asr_nlu_samples.jsonl",
            "CONTINUOUS_PREFLIGHT_ENABLED": "true",
            "CONTINUOUS_READINESS_ENABLED": "true",
            "CONTINUOUS_READINESS_DURATION": "3.5",
            "CONTINUOUS_COMMAND_QUEUE_SIZE": "12",
            "CONTINUOUS_COMMAND_MAX_AGE": "18",
            "CONTINUOUS_DUPLICATE_WINDOW_S": "2.4",
            "COMMAND_NORMALIZATION_ENABLED": "true",
            "COMMAND_NORMALIZATION_FEEDBACK_ENABLED": "false",
            "COMMAND_NORMALIZATION_FUZZY_THRESHOLD": "0.77",
            "COMMAND_NORMALIZATION_PATH": "/tmp/custom_normalization.yaml",
            "COMMAND_COMPLETION_ENABLED": "false",
            "ASR_COMMIT_DELAY_MS": "450",
            "ASR_HOTWORDS_SCORE": "3.4",
            "ASR_PARTIAL_MERGE_ENABLED": "false",
            "ASR_PARTIAL_MAX_AGE_S": "1.7",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "online"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "连续语音控制模式=online" in result.stdout
    assert "通过标准：至少识别 6 条 ASR final" in result.stdout
    assert "continuous-voice-benchmark online" in result.stdout
    assert "VOICE_SESSION_TIMEOUT=44" in result.stdout
    assert "WAKE_WORD_ENABLED=false" in result.stdout
    assert "SPEAKER_ENABLED=true" in result.stdout
    assert "PULSE_CAPTURE_BRIDGE=false" in result.stdout
    assert "VAD_PROVIDER=silero" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.021" in result.stdout
    assert "VAD_SPEECH_START_MS=128" in result.stdout
    assert "SPEECH_END_SILENCE_S=0.38" in result.stdout
    assert "MIN_UTTERANCE_MS=240" in result.stdout
    assert "MAX_UTTERANCE_S=7.5" in result.stdout
    assert "SILERO_VAD_MODEL_PATH=/models/vad/silero_vad.onnx" in result.stdout
    assert "SILERO_VAD_USE_ONNX=true" in result.stdout
    assert "SILERO_VAD_THRESHOLD=0.61" in result.stdout
    assert "SILERO_VAD_END_THRESHOLD=0.33" in result.stdout
    assert "KWS_PROVIDER=openwakeword" in result.stdout
    assert "OPENWAKEWORD_MODELS=/models/kws/xiaozhi.onnx,/models/kws/nihaoxiaozhi.onnx" in result.stdout
    assert "OPENWAKEWORD_THRESHOLD=0.42" in result.stdout
    assert "LIVEKIT_WAKEWORD_MODELS=/models/kws/livekit-xiaozhi.onnx" in result.stdout
    assert "LIVEKIT_WAKEWORD_THRESHOLD=0.63" in result.stdout
    assert "AUDIO_ENHANCER=webrtc" in result.stdout
    assert "AEC_ENABLED=false" in result.stdout
    assert "NOISE_SUPPRESSION_ENABLED=true" in result.stdout
    assert "AUTO_GAIN_ENABLED=true" in result.stdout
    assert "CONTINUOUS_MONITOR_AUDIO_SAMPLE_LIMIT=42" in result.stdout
    assert "CONTINUOUS_SAMPLE_LOG=/tmp/asr_nlu_samples.jsonl" in result.stdout
    assert "CONTINUOUS_PREFLIGHT_ENABLED=true" in result.stdout
    assert "CONTINUOUS_READINESS_ENABLED=true" in result.stdout
    assert "CONTINUOUS_READINESS_DURATION=3.5" in result.stdout
    assert "CONTINUOUS_COMMAND_QUEUE_SIZE=12" in result.stdout
    assert "CONTINUOUS_COMMAND_MAX_AGE=18" in result.stdout
    assert "CONTINUOUS_DUPLICATE_WINDOW_S=2.4" in result.stdout
    assert "COMMAND_NORMALIZATION_ENABLED=true" in result.stdout
    assert "COMMAND_NORMALIZATION_FEEDBACK_ENABLED=false" in result.stdout
    assert "COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.77" in result.stdout
    assert "COMMAND_NORMALIZATION_PATH=/tmp/custom_normalization.yaml" in result.stdout
    assert "COMMAND_COMPLETION_ENABLED=false" in result.stdout
    assert "ASR_COMMIT_DELAY_MS=450" in result.stdout
    assert "ASR_HOTWORDS_SCORE=3.4" in result.stdout
    assert "ASR_PARTIAL_MERGE_ENABLED=false" in result.stdout
    assert "ASR_PARTIAL_MAX_AGE_S=1.7" in result.stdout
    assert "wake_word_enabled:=false" in result.stdout
    assert "speech_start_threshold:=0.021" in result.stdout
    assert "vad_speech_start_ms:=128" in result.stdout
    assert "speech_end_silence_s:=0.38" in result.stdout
    assert "min_utterance_ms:=240" in result.stdout
    assert "max_utterance_s:=7.5" in result.stdout
    assert "silero_model_path:=/models/vad/silero_vad.onnx" in result.stdout
    assert "silero_use_onnx:=true" in result.stdout
    assert "silero_threshold:=0.61" in result.stdout
    assert "silero_end_threshold:=0.33" in result.stdout
    assert "continuous_command_queue_size:=12" in result.stdout
    assert "voice_session_timeout_s:=44" in result.stdout
    assert "continuous_command_max_age_s:=18" in result.stdout
    assert "continuous_duplicate_window_s:=2.4" in result.stdout
    assert "command_normalization_enabled:=true" in result.stdout
    assert "command_normalization_feedback_enabled:=false" in result.stdout
    assert "command_normalization_fuzzy_threshold:=0.77" in result.stdout
    assert "command_normalization_path:=/tmp/custom_normalization.yaml" in result.stdout
    assert "command_completion_enabled:=false" in result.stdout
    assert "asr_commit_delay_ms:=450" in result.stdout
    assert "asr_hotwords_score:=3.4" in result.stdout
    assert "asr_partial_merge_enabled:=false" in result.stdout
    assert "asr_partial_max_age_s:=1.7" in result.stdout
    assert "sherpa_tokens:=/models/kws/tokens.txt" in result.stdout
    assert "openwakeword_models:=/models/kws/xiaozhi.onnx,/models/kws/nihaoxiaozhi.onnx" in result.stdout
    assert "openwakeword_threshold:=0.42" in result.stdout
    assert "livekit_wakeword_models:=/models/kws/livekit-xiaozhi.onnx" in result.stdout
    assert "livekit_wakeword_threshold:=0.63" in result.stdout
    assert "audio_enhancer:=webrtc" in result.stdout
    assert "noise_suppression_enabled:=true" in result.stdout


def test_continuous_voice_control_can_use_wsl_pulse_capture_bridge():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "PULSE_CAPTURE_BRIDGE": "true",
            "PULSE_CAPTURE_SOURCE": "RDPSource",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "PULSE_CAPTURE_BRIDGE=true（active=true" in result.stdout
    assert "PULSE_CAPTURE_SOURCE=RDPSource" in result.stdout
    assert "PULSE_ENDPOINT_EVENTS_ENABLED=true" in result.stdout
    assert "capture_enabled:=false" in result.stdout


def test_continuous_voice_control_disables_pulse_endpoint_events_when_silero_owns_vad():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "PULSE_CAPTURE_BRIDGE": "true",
            "PULSE_CAPTURE_SOURCE": "RDPSource",
            "VAD_PROVIDER": "silero",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "VAD_PROVIDER=silero" in result.stdout
    assert "PULSE_ENDPOINT_EVENTS_ENABLED=false" in result.stdout
    assert "vad_provider:=silero" in result.stdout


def test_continuous_voice_control_profile_tunes_microphone_defaults():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CALIBRATION_ENV": "/tmp/embodied_agent_missing_voice_calibration.env",
            "VOICE_CONTROL_PROFILE": "noisy_room",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "VOICE_CONTROL_PROFILE=noisy_room" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.026" in result.stdout
    assert "SPEECH_END_SILENCE_S=0.85" in result.stdout
    assert "ASR_COMMIT_DELAY_MS=300" in result.stdout
    assert "MIN_UTTERANCE_MS=180" in result.stdout
    assert "MAX_UTTERANCE_S=10.0" in result.stdout
    assert "COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.78" in result.stdout
    assert "CONTINUOUS_COMMAND_QUEUE_SIZE=5" in result.stdout
    assert "speech_start_threshold:=0.026" in result.stdout
    assert "speech_end_silence_s:=0.85" in result.stdout
    assert "asr_commit_delay_ms:=300" in result.stdout
    assert "continuous_command_queue_size:=5" in result.stdout


def test_continuous_voice_control_low_gain_profile_lowers_vad_and_disables_aec():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CALIBRATION_ENV": "/tmp/embodied_agent_missing_voice_calibration.env",
            "VOICE_CONTROL_PROFILE": "low_gain",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "VOICE_CONTROL_PROFILE=low_gain" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.0012" in result.stdout
    assert "SPEECH_END_SILENCE_S=0.8" in result.stdout
    assert "ASR_COMMIT_DELAY_MS=450" in result.stdout
    assert "AEC_ENABLED=false" in result.stdout
    assert "speech_start_threshold:=0.0012" in result.stdout
    assert "speech_end_silence_s:=0.8" in result.stdout
    assert "asr_commit_delay_ms:=450" in result.stdout
    assert "aec_enabled:=false" in result.stdout


def test_continuous_voice_control_can_apply_calibration_env(tmp_path):
    calibration_env = tmp_path / "voice_calibration.env"
    calibration_env.write_text(
        "\n".join(
            [
                "export VOICE_CONTROL_PROFILE=low_gain",
                "export SPEECH_START_THRESHOLD=0.0016",
                "export VAD_PROVIDER=energy",
                "",
            ]
        ),
        encoding="utf-8",
    )
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "APPLY_VOICE_CALIBRATION": "true",
            "VOICE_CALIBRATION_ENV": str(calibration_env),
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "APPLY_VOICE_CALIBRATION=true" in result.stdout
    assert f"VOICE_CALIBRATION_ENV={calibration_env}（applied=true）" in result.stdout
    assert "VOICE_CONTROL_PROFILE=low_gain" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.0016" in result.stdout
    assert "VAD_PROVIDER=energy" in result.stdout
    assert "speech_start_threshold:=0.0016" in result.stdout


def test_continuous_voice_control_auto_applies_existing_calibration_env(tmp_path):
    calibration_env = tmp_path / "voice_calibration.env"
    calibration_env.write_text(
        "\n".join(
            [
                "export VOICE_CONTROL_PROFILE=low_gain",
                "export SPEECH_START_THRESHOLD=0.0017",
                "export VAD_PROVIDER=energy",
                "",
            ]
        ),
        encoding="utf-8",
    )
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CALIBRATION_ENV": str(calibration_env),
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "APPLY_VOICE_CALIBRATION=auto" in result.stdout
    assert f"VOICE_CALIBRATION_ENV={calibration_env}（applied=true）" in result.stdout
    assert "VOICE_CONTROL_PROFILE=low_gain" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.0017" in result.stdout
    assert "VAD_PROVIDER=energy" in result.stdout


def test_continuous_voice_control_auto_calibration_keeps_explicit_user_env(tmp_path):
    calibration_env = tmp_path / "voice_calibration.env"
    calibration_env.write_text(
        "\n".join(
            [
                "export VOICE_CONTROL_PROFILE=low_gain",
                "export SPEECH_START_THRESHOLD=0.0017",
                "export VAD_PROVIDER=energy",
                "",
            ]
        ),
        encoding="utf-8",
    )
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CALIBRATION_ENV": str(calibration_env),
            "VOICE_CONTROL_PROFILE": "noisy_room",
            "SPEECH_START_THRESHOLD": "0.031",
            "VAD_PROVIDER": "webrtc",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "APPLY_VOICE_CALIBRATION=auto" in result.stdout
    assert f"VOICE_CALIBRATION_ENV={calibration_env}（applied=true）" in result.stdout
    assert "VOICE_CONTROL_PROFILE=noisy_room" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.031" in result.stdout
    assert "VAD_PROVIDER=webrtc" in result.stdout


def test_continuous_voice_control_omits_empty_optional_launch_arguments():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CALIBRATION_ENV": "/tmp/embodied_agent_missing_voice_calibration.env",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "空的可选模型/配置路径参数会省略" in result.stdout
    # Silero 模型现在随仓库提供默认路径，因此不是空参数；其余未配置的 KWS 路径仍应省略。
    assert "silero_model_path:=" in result.stdout
    assert "sherpa_tokens:=" not in result.stdout
    assert "sherpa_encoder:=" not in result.stdout
    assert "sherpa_decoder:=" not in result.stdout
    assert "sherpa_joiner:=" not in result.stdout
    assert "sherpa_keywords_file:=" not in result.stdout
    assert "openwakeword_models:=" not in result.stdout
    assert "livekit_wakeword_models:=" not in result.stdout
    assert "command_normalization_path:=" not in result.stdout
    assert "command_completion_enabled:=true" in result.stdout
    assert "asr_commit_delay_ms:=300" in result.stdout


def test_continuous_voice_control_manual_env_overrides_profile():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CONTROL_PROFILE": "noisy_room",
            "SPEECH_START_THRESHOLD": "0.031",
            "SPEECH_END_SILENCE_S": "0.62",
            "ASR_COMMIT_DELAY_MS": "150",
            "CONTINUOUS_COMMAND_QUEUE_SIZE": "9",
            "CONTINUOUS_DUPLICATE_WINDOW_S": "0.6",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "VOICE_CONTROL_PROFILE=noisy_room" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.031" in result.stdout
    assert "SPEECH_END_SILENCE_S=0.62" in result.stdout
    assert "ASR_COMMIT_DELAY_MS=150" in result.stdout
    assert "CONTINUOUS_COMMAND_QUEUE_SIZE=9" in result.stdout
    assert "CONTINUOUS_DUPLICATE_WINDOW_S=0.6" in result.stdout
    assert "speech_start_threshold:=0.031" in result.stdout
    assert "speech_end_silence_s:=0.62" in result.stdout
    assert "asr_commit_delay_ms:=150" in result.stdout
    assert "continuous_command_queue_size:=9" in result.stdout
    assert "continuous_duplicate_window_s:=0.6" in result.stdout


def test_continuous_voice_control_rejects_unknown_profile():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CONTROL_PROFILE": "storm",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 2
    assert "unknown VOICE_CONTROL_PROFILE" in result.stderr


def test_voice_launches_expose_audio_enhancer_arguments():
    files = [
        ROOT / "src" / "embodied_simulation" / "launch" / "voice_turtlebot3.launch.py",
        ROOT / "src" / "embodied_online_agent" / "launch" / "online_agent.launch.py",
        ROOT / "src" / "embodied_offline_agent" / "launch" / "offline_agent.launch.py",
    ]

    for path in files:
        content = path.read_text(encoding="utf-8")
        assert "audio_enhancer" in content, path
        assert "aec_enabled" in content, path
        assert "noise_suppression_enabled" in content, path
        assert "auto_gain_enabled" in content, path
        assert "speech_start_threshold" in content, path
        assert "speech_end_silence_s" in content, path
        assert "min_utterance_ms" in content, path
        assert "max_utterance_s" in content, path
        assert "silero_model_path" in content, path
        assert "silero_use_onnx" in content, path
        assert "silero_threshold" in content, path
        assert "continuous_command_queue_size" in content, path
        assert "continuous_duplicate_window_s" in content, path
        assert "command_normalization_enabled" in content, path
        assert "command_normalization_feedback_enabled" in content, path
        assert "command_normalization_fuzzy_threshold" in content, path
        assert "command_normalization_path" in content, path
        assert "command_completion_enabled" in content, path
        assert "asr_commit_delay_ms" in content, path
        assert "openwakeword_models" in content, path
        assert "livekit_wakeword_models" in content, path
        assert "sherpa_tokens" in content, path


def test_continuous_voice_control_stops_monitor_gracefully_for_summary():
    content = (ROOT / "scripts" / "continuous_voice_control.sh").read_text(
        encoding="utf-8"
    )

    assert 'kill -INT "$MONITOR_PID"' in content
    assert '--audio-sample-limit "$MONITOR_AUDIO_SAMPLE_LIMIT"' in content


def test_continuous_voice_control_waits_for_readiness_after_launch():
    content = (ROOT / "scripts" / "continuous_voice_control.sh").read_text(
        encoding="utf-8"
    )

    assert "CONTINUOUS_READINESS_ENABLED" in content
    assert "voice_control_readiness_check.py" in content
    assert "system_readiness_check.py" in content
    assert "--profile voice_simulation" in content
    assert '--duration "$READINESS_DURATION"' in content
    assert "--require-kws" in content
    assert "系统已就绪，可以开始说：小智" in content
    assert "麦克风 source" in content
    assert "VOICE_CONTROL_PROFILE=low_gain/quiet/noisy_room" in content


def test_online_and_offline_publish_queue_rejected_feedback():
    files = [
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "online_agent_node.py",
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py",
    ]

    for path in files:
        content = path.read_text(encoding="utf-8")
        assert "_publish_queue_rejected_recognition" in content, path
        assert "self._events.publish_queue_rejected" in content, path

    shared_events = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "ros_agent_events.py"
    ).read_text(encoding="utf-8")
    assert '"status": "queue_rejected"' in shared_events
