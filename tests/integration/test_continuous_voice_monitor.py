#!/usr/bin/env python3
"""Unit-style checks for the human-facing continuous voice monitor."""

import importlib.util
import json
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MONITOR = ROOT / "scripts" / "continuous_voice_monitor.py"

sys.modules.setdefault("rclpy", types.SimpleNamespace())
sys.modules.setdefault(
    "rclpy.node",
    types.SimpleNamespace(Node=object),
)
sys.modules.setdefault(
    "std_msgs.msg",
    types.SimpleNamespace(String=object),
)

spec = importlib.util.spec_from_file_location("continuous_voice_monitor", MONITOR)
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def test_monitor_formats_session_and_wake_events():
    wake = json.dumps({"kind": "wake", "provider": "text"}, ensure_ascii=False)
    kws = json.dumps(
        {"provider": "mock_kws", "transcript": "小智", "score": 1.0},
        ensure_ascii=False,
    )
    audio = json.dumps(
        {"rms": 0.0305, "peak": 1000, "speech": True, "dropped_input_frames": 0},
        ensure_ascii=False,
    )

    assert monitor.format_session_state("awake") == "[session] awake"
    assert monitor.format_wake_event(wake) == "[wake] text:wake"
    assert monitor.format_kws_event(kws) == "[kws] mock_kws detected 小智 score=1.0"
    assert monitor.format_audio_metrics(audio) == "[audio] rms=0.0305 peak=1000 speech=True dropped=0"


def test_monitor_formats_kws_score_events():
    score = json.dumps(
        {
            "provider": "openwakeword_test",
            "top_keyword": "fake_wake",
            "top_score": 0.91,
            "threshold": 0.5,
            "above_threshold": True,
        },
        ensure_ascii=False,
    )

    assert (
        monitor.format_kws_score(score)
        == "[kws-score] openwakeword_test fake_wake=0.910 threshold=0.500 above=True"
    )


def test_monitor_formats_asr_queue_action_and_result_events():
    queue_event = json.dumps(
        {"event": "enqueue", "text": "向前走一秒", "size": 2},
        ensure_ascii=False,
    )
    expired_event = json.dumps(
        {
            "event": "expired",
            "text": "向前走一秒",
            "size": 1,
            "reason": "stale_command",
        },
        ensure_ascii=False,
    )
    execution_event = json.dumps(
        {"event": "started", "text": "向前走一秒"},
        ensure_ascii=False,
    )
    candidate = json.dumps(
        {"name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}},
        ensure_ascii=False,
    )
    result = json.dumps({"success": True, "message": "succeeded"}, ensure_ascii=False)
    feedback = json.dumps(
        {"phase": 2, "progress": 0.42, "detail": "mock_execution"},
        ensure_ascii=False,
    )

    assert monitor.format_asr_final("向前走一秒") == "[asr] 向前走一秒"
    assert monitor.format_queue_event(queue_event) == "[queue] enqueue 向前走一秒 size=2"
    assert (
        monitor.format_queue_event(expired_event)
        == "[queue] expired 向前走一秒 reason=stale_command size=1"
    )
    assert monitor.format_execution_event(execution_event) == "[exec] started 向前走一秒"
    assert monitor.format_action_candidate(candidate) == "[action] executing move"
    assert monitor.format_action_feedback(feedback) == "[feedback] executing 42% mock_execution"
    assert monitor.format_action_result(result) == "[result] succeeded"


def test_monitor_formats_normalization_feedback():
    feedback = json.dumps(
        {
            "status": "normalized",
            "reason": "command_normalized",
            "original": "钱进一秒",
            "normalized": "前进一秒",
        },
        ensure_ascii=False,
    )

    assert monitor.format_recognition_feedback(feedback) == "[normalize] 钱进一秒 -> 前进一秒"
