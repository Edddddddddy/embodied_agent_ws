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
            "WAKE_WORD_ENABLED": "false",
            "SPEAKER_ENABLED": "true",
            "VAD_PROVIDER": "silero",
            "SPEECH_START_THRESHOLD": "0.021",
            "SPEECH_END_SILENCE_S": "0.38",
            "MIN_UTTERANCE_MS": "240",
            "MAX_UTTERANCE_S": "7.5",
            "SILERO_VAD_MODEL_PATH": "/models/vad/silero_vad.onnx",
            "SILERO_VAD_USE_ONNX": "true",
            "SILERO_VAD_THRESHOLD": "0.61",
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
            "CONTINUOUS_PREFLIGHT_ENABLED": "true",
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
    assert "WAKE_WORD_ENABLED=false" in result.stdout
    assert "SPEAKER_ENABLED=true" in result.stdout
    assert "VAD_PROVIDER=silero" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.021" in result.stdout
    assert "SPEECH_END_SILENCE_S=0.38" in result.stdout
    assert "MIN_UTTERANCE_MS=240" in result.stdout
    assert "MAX_UTTERANCE_S=7.5" in result.stdout
    assert "SILERO_VAD_MODEL_PATH=/models/vad/silero_vad.onnx" in result.stdout
    assert "SILERO_VAD_USE_ONNX=true" in result.stdout
    assert "SILERO_VAD_THRESHOLD=0.61" in result.stdout
    assert "KWS_PROVIDER=openwakeword" in result.stdout
    assert "OPENWAKEWORD_MODELS=/models/kws/xiaozhi.onnx,/models/kws/nihaoxiaozhi.onnx" in result.stdout
    assert "OPENWAKEWORD_THRESHOLD=0.42" in result.stdout
    assert "LIVEKIT_WAKEWORD_MODELS=/models/kws/livekit-xiaozhi.onnx" in result.stdout
    assert "LIVEKIT_WAKEWORD_THRESHOLD=0.63" in result.stdout
    assert "AUDIO_ENHANCER=webrtc" in result.stdout
    assert "AEC_ENABLED=false" in result.stdout
    assert "NOISE_SUPPRESSION_ENABLED=true" in result.stdout
    assert "AUTO_GAIN_ENABLED=true" in result.stdout
    assert "CONTINUOUS_PREFLIGHT_ENABLED=true" in result.stdout
    assert "wake_word_enabled:=false" in result.stdout
    assert "speech_start_threshold:=0.021" in result.stdout
    assert "speech_end_silence_s:=0.38" in result.stdout
    assert "min_utterance_ms:=240" in result.stdout
    assert "max_utterance_s:=7.5" in result.stdout
    assert "silero_model_path:=\"/models/vad/silero_vad.onnx\"" in result.stdout
    assert "silero_use_onnx:=true" in result.stdout
    assert "silero_threshold:=0.61" in result.stdout
    assert "sherpa_tokens:=\"/models/kws/tokens.txt\"" in result.stdout
    assert "openwakeword_models:=\"/models/kws/xiaozhi.onnx,/models/kws/nihaoxiaozhi.onnx\"" in result.stdout
    assert "openwakeword_threshold:=0.42" in result.stdout
    assert "livekit_wakeword_models:=\"/models/kws/livekit-xiaozhi.onnx\"" in result.stdout
    assert "livekit_wakeword_threshold:=0.63" in result.stdout
    assert "audio_enhancer:=webrtc" in result.stdout
    assert "noise_suppression_enabled:=true" in result.stdout


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
        assert "openwakeword_models" in content, path
        assert "livekit_wakeword_models" in content, path
        assert "sherpa_tokens" in content, path


def test_continuous_voice_control_stops_monitor_gracefully_for_summary():
    content = (ROOT / "scripts" / "continuous_voice_control.sh").read_text(
        encoding="utf-8"
    )

    assert 'kill -INT "$MONITOR_PID"' in content
