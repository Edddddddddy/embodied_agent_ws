#!/usr/bin/env python3
"""Unit-style checks for the human-facing continuous voice monitor."""

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MONITOR = ROOT / "scripts" / "continuous_voice_monitor.py"

spec = importlib.util.spec_from_file_location("continuous_voice_monitor", MONITOR)
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def test_monitor_formats_session_and_wake_events():
    wake = json.dumps({"kind": "wake", "provider": "text"}, ensure_ascii=False)

    assert monitor.format_session_state("awake") == "[session] awake"
    assert monitor.format_wake_event(wake) == "[wake] text:wake"


def test_monitor_formats_asr_queue_action_and_result_events():
    queue_event = json.dumps(
        {"event": "enqueue", "text": "向前走一秒", "size": 2},
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

    assert monitor.format_asr_final("向前走一秒") == "[asr] 向前走一秒"
    assert monitor.format_queue_event(queue_event) == "[queue] enqueue 向前走一秒 size=2"
    assert monitor.format_execution_event(execution_event) == "[exec] started 向前走一秒"
    assert monitor.format_action_candidate(candidate) == "[action] executing move"
    assert monitor.format_action_result(result) == "[result] succeeded"
