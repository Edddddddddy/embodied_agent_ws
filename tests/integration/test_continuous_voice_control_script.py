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
            "KWS_PROVIDER": "openwakeword",
            "AUDIO_ENHANCER": "webrtc",
            "AEC_ENABLED": "false",
            "NOISE_SUPPRESSION_ENABLED": "true",
            "AUTO_GAIN_ENABLED": "true",
            "CONTINUOUS_MONITOR_ENABLED": "false",
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
    assert "KWS_PROVIDER=openwakeword" in result.stdout
    assert "AUDIO_ENHANCER=webrtc" in result.stdout
    assert "AEC_ENABLED=false" in result.stdout
    assert "NOISE_SUPPRESSION_ENABLED=true" in result.stdout
    assert "AUTO_GAIN_ENABLED=true" in result.stdout
    assert "wake_word_enabled:=false" in result.stdout
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
