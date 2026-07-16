#!/usr/bin/env python3
"""Checks for the human-facing continuous Nav2 voice launcher."""

import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_continuous_nav2_voice_control_prints_nav2_launch_config():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CONTROL_PROFILE": "quiet",
            "VOICE_SESSION_TIMEOUT": "166",
            "CONTINUOUS_COMMAND_QUEUE_SIZE": "7",
            "NAV_ACTION_TIMEOUT_S": "321.0",
            "NAV2_INITIAL_X": "-1.8",
            "NAV2_INITIAL_Y": "-0.4",
            "NAV2_INITIAL_YAW": "0.2",
            "SPEECH_END_SILENCE_S": "0.66",
            "ASR_COMMIT_DELAY_MS": "250",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "online"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "Nav2 连续语音导航模式=online" in result.stdout
    assert "去门口" in result.stdout
    assert "依次去门口、书桌、起点" in result.stdout
    assert "VOICE_SESSION_TIMEOUT=166" in result.stdout
    assert "CONTINUOUS_COMMAND_QUEUE_SIZE=7" in result.stdout
    assert "NAV_ACTION_TIMEOUT_S=321.0" in result.stdout
    assert "NAV2_INITIAL_X=-1.8" in result.stdout
    assert "NAV2_INITIAL_Y=-0.4" in result.stdout
    assert "NAV2_INITIAL_YAW=0.2" in result.stdout
    assert "VAD_PROVIDER=energy" in result.stdout
    assert "voice_nav2_turtlebot3.launch.py" in result.stdout
    assert "microphone_enabled:=true" in result.stdout
    assert "continuous_control_enabled:=true" in result.stdout
    assert "nav_action_timeout_s:=321.0" in result.stdout
    assert "x_pose:=-1.8" in result.stdout
    assert "y_pose:=-0.4" in result.stdout
    assert "yaw:=0.2" in result.stdout
    assert "speech_end_silence_s:=0.66" in result.stdout
    assert "asr_commit_delay_ms:=250" in result.stdout


def test_continuous_nav2_voice_control_low_gain_profile_lowers_vad_and_disables_aec():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CONTROL_PROFILE": "low_gain",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "VOICE_CONTROL_PROFILE=low_gain" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.0012" in result.stdout
    assert "SPEECH_END_SILENCE_S=0.85" in result.stdout
    assert "ASR_COMMIT_DELAY_MS=450" in result.stdout
    assert "AEC_ENABLED=false" in result.stdout
    assert "speech_start_threshold:=0.0012" in result.stdout
    assert "speech_end_silence_s:=0.85" in result.stdout
    assert "asr_commit_delay_ms:=450" in result.stdout
    assert "aec_enabled:=false" in result.stdout


def test_nav2_voice_control_reuses_calibration_and_wsl_pulse_bridge(tmp_path):
    calibration = tmp_path / "voice_calibration.env"
    calibration.write_text(
        "\n".join(
            [
                "export VOICE_CONTROL_PROFILE=low_gain",
                "export SPEECH_START_THRESHOLD=0.0016",
                "export ASR_COMMIT_DELAY_MS=525",
                "export VAD_PROVIDER=energy",
                "export KWS_PROVIDER=openwakeword",
                "export AEC_ENABLED=false",
            ]
        ),
        encoding="utf-8",
    )
    env = os.environ.copy()
    for key in (
        "VOICE_CONTROL_PROFILE",
        "SPEECH_START_THRESHOLD",
        "ASR_COMMIT_DELAY_MS",
        "VAD_PROVIDER",
        "AEC_ENABLED",
    ):
        env.pop(key, None)
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "APPLY_VOICE_CALIBRATION": "true",
            "VOICE_CALIBRATION_ENV": str(calibration),
            "PULSE_CAPTURE_BRIDGE": "auto",
            "PULSE_SERVER": "unix:/mnt/wslg/PulseServer",
            "KWS_PROVIDER": "sherpa",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "VOICE_CONTROL_PROFILE=low_gain" in result.stdout
    assert "VOICE_CALIBRATION_ENV=" in result.stdout
    assert "applied=true" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.0016" in result.stdout
    assert "ASR_COMMIT_DELAY_MS=525" in result.stdout
    assert "KWS_PROVIDER=sherpa" in result.stdout
    assert "PULSE_CAPTURE_BRIDGE=auto（active=true" in result.stdout
    assert "capture_enabled:=false" in result.stdout


def test_nav2_voice_control_disables_pulse_bridge_without_microphone():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "NAV2_MICROPHONE_ENABLED": "false",
            "NAV2_CAPTURE_ENABLED": "false",
            "PULSE_CAPTURE_BRIDGE": "auto",
            "PULSE_SERVER": "unix:/mnt/wslg/PulseServer",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "MICROPHONE_ENABLED=false" in result.stdout
    assert "PULSE_CAPTURE_BRIDGE=auto（active=false" in result.stdout


def test_nav2_voice_control_health_checks_pulse_before_launch():
    script = (ROOT / "scripts" / "continuous_nav2_voice_control.sh").read_text(
        encoding="utf-8"
    )

    bridge_call = script.index("start_pulse_capture_bridge\nbuild_launch_args")
    launch_call = script.index('setsid ros2 launch "${LAUNCH_ARGS[@]}"')
    assert bridge_call < launch_call
    assert 'kill -0 "$PULSE_BRIDGE_PID"' in script
    assert "PulseAudio capture bridge startup failed" in script


def test_continuous_nav2_voice_control_rejects_unknown_mode():
    env = os.environ.copy()
    env.update({"WORKSPACE": str(ROOT), "CONTINUOUS_PRINT_CONFIG": "true"})

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "cloud"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 2
    assert "Usage:" in result.stderr
