#!/usr/bin/env python3
"""Unit-style checks for continuous_live_check report logic."""

import importlib.util
import json
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "continuous_live_check.py"


class _FakeNode:
    def __init__(self, *args, **kwargs):
        del args, kwargs

    def create_subscription(self, *args, **kwargs):
        del args, kwargs
        return None


sys.modules.setdefault("rclpy", types.SimpleNamespace())
sys.modules.setdefault("rclpy.node", types.SimpleNamespace(Node=_FakeNode))
sys.modules.setdefault("geometry_msgs.msg", types.SimpleNamespace(Twist=object))
sys.modules.setdefault("std_msgs.msg", types.SimpleNamespace(String=object))

spec = importlib.util.spec_from_file_location("continuous_live_check", SCRIPT)
live_check = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = live_check
spec.loader.exec_module(live_check)


def test_live_check_report_passes_when_required_evidence_is_present():
    node = live_check.LiveCheckNode()
    node.asr.extend(["小智", "向前走一秒", "左转九十度", "后退一秒", "绕圈", "退出控制"])
    node.session_states.extend(["awake", "sleeping"])
    node.candidates.extend([{} for _ in range(4)])
    node.results.extend([{"success": True} for _ in range(4)])
    node.queue_events.extend([{"event": "enqueue"} for _ in range(4)])
    node.execution_events.extend([{"event": "started"}, {"event": "finished"}])
    node.velocities.append((0.0, 0.0))

    report = node.build_report(live_check.LiveCheckThresholds())

    assert report.ok is True
    assert report.missing == []
    assert report.asr_count == 6
    assert report.action_success_count == 4


def test_live_check_report_lists_missing_evidence():
    node = live_check.LiveCheckNode()
    node.asr.append("小智")
    node.session_states.append("awake")
    node.velocities.append((0.1, 0.0))

    report = node.build_report(live_check.LiveCheckThresholds())

    assert report.ok is False
    assert "ASR final >= 6" in report.missing
    assert "session sleeping observed" in report.missing
    assert "final cmd_vel is zero" in report.missing


def test_json_dict_ignores_non_json_payloads():
    assert live_check._json_dict("not-json") == {}
    assert live_check._json_dict(json.dumps({"event": "enqueue"})) == {"event": "enqueue"}
