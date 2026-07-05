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
    assert "voice_nav2_turtlebot3.launch.py" in result.stdout
    assert "microphone_enabled:=true" in result.stdout
    assert "continuous_control_enabled:=true" in result.stdout
    assert "nav_action_timeout_s:=321.0" in result.stdout
    assert "x_pose:=-1.8" in result.stdout
    assert "y_pose:=-0.4" in result.stdout
    assert "yaw:=0.2" in result.stdout
    assert "speech_end_silence_s:=0.66" in result.stdout
    assert "asr_commit_delay_ms:=250" in result.stdout


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
