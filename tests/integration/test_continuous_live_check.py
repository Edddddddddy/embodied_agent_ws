#!/usr/bin/env python3
"""Unit-style checks for continuous_live_check report logic."""

import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "continuous_live_check.py"


class _FakeNode:
    def __init__(self, *args, **kwargs):
        del args, kwargs

    def create_subscription(self, *args, **kwargs):
        del args, kwargs
        return None


sys.modules["rclpy"] = types.SimpleNamespace()
sys.modules["rclpy.node"] = types.SimpleNamespace(Node=_FakeNode)
sys.modules["geometry_msgs.msg"] = types.SimpleNamespace(Twist=object)
sys.modules["std_msgs.msg"] = types.SimpleNamespace(String=object)

spec = importlib.util.spec_from_file_location("continuous_live_check", SCRIPT)
live_check = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = live_check
spec.loader.exec_module(live_check)


def test_live_check_report_passes_when_required_evidence_is_present():
    node = live_check.LiveCheckNode()
    node.asr.extend(["小智", "向前走一秒", "左转九十度", "后退一秒", "绕圈", "退出控制"])
    node.session_states.extend(["awake", "sleeping"])
    node.candidates.extend(
        [{"name": "move"}, {"name": "turn"}, {"name": "arc"}, {"name": "stop"}]
    )
    node.results.extend([{"success": True} for _ in range(4)])
    node.queue_events.extend([{"event": "enqueue"} for _ in range(4)])
    node.execution_events.extend([{"event": "started"}, {"event": "finished"}])
    node.velocities.append((0.0, 0.0))

    report = node.build_report(live_check.LiveCheckThresholds())

    assert report.ok is True
    assert report.missing == []
    assert report.asr_count == 6
    assert report.action_success_count == 4
    assert report.action_candidate_names["move"] == 1


def test_live_check_report_requires_navigation_candidates():
    node = live_check.LiveCheckNode()
    node.asr.extend(["小智", "去门口", "前往书桌", "依次去门口书桌起点"])
    node.session_states.extend(["awake", "sleeping"])
    node.candidates.extend(
        [
            {"name": "navigate_to", "arguments": {"target": "door"}},
            {"name": "navigate_to", "arguments": {"target": "desk"}},
            {
                "name": "follow_waypoints",
                "arguments": {"waypoints": ["door", "desk", "home"]},
            },
        ]
    )
    node.results.extend([{"success": True} for _ in range(3)])
    node.velocities.append((0.0, 0.0))

    report = node.build_report(
        live_check.LiveCheckThresholds(
            min_asr=3,
            min_candidates=2,
            min_success=2,
            required_candidates=["navigate_to", "follow_waypoints"],
            require_navigation_details=True,
        )
    )

    assert report.ok is True
    assert report.action_candidate_names["navigate_to"] == 2
    assert report.action_candidate_names["follow_waypoints"] == 1


def test_live_check_report_requires_navigation_details_when_requested():
    node = live_check.LiveCheckNode()
    node.asr.extend(["小智", "去门口", "依次去门口书桌起点"])
    node.session_states.extend(["awake", "sleeping"])
    node.candidates.extend(
        [{"name": "navigate_to"}, {"name": "follow_waypoints", "arguments": {}}]
    )
    node.results.extend([{"success": True} for _ in range(2)])
    node.velocities.append((0.0, 0.0))

    report = node.build_report(
        live_check.LiveCheckThresholds(
            min_asr=3,
            min_candidates=2,
            min_success=2,
            required_candidates=["navigate_to", "follow_waypoints"],
            require_navigation_details=True,
        )
    )

    assert report.ok is False
    assert "navigate_to target observed" in report.missing
    assert "follow_waypoints waypoints observed" in report.missing


def test_live_check_report_lists_missing_required_navigation_candidate():
    node = live_check.LiveCheckNode()
    node.asr.extend(["小智", "去门口", "退出控制"])
    node.session_states.extend(["awake", "sleeping"])
    node.candidates.append({"name": "navigate_to"})
    node.results.append({"success": True})
    node.velocities.append((0.0, 0.0))

    report = node.build_report(
        live_check.LiveCheckThresholds(
            min_asr=1,
            min_candidates=1,
            min_success=1,
            required_candidates=["navigate_to", "follow_waypoints"],
        )
    )

    assert report.ok is False
    assert "candidate follow_waypoints observed" in report.missing


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


def test_write_report_creates_parent_directory(tmp_path):
    report = live_check.LiveCheckReport(
        asr_count=1,
        action_candidate_count=1,
        action_success_count=1,
        command_enqueue_count=1,
        execution_started_count=1,
        execution_finished_count=1,
        action_candidate_names={"navigate_to": 1},
        saw_awake=True,
        saw_sleeping=True,
        final_cmd_vel_zero=True,
        ok=True,
        missing=[],
    )
    destination = tmp_path / "nested" / "nav2-live-check.json"

    live_check.write_report(str(destination), report)

    payload = json.loads(destination.read_text(encoding="utf-8"))
    assert payload["ok"] is True
    assert payload["action_candidate_names"] == {"navigate_to": 1}


def test_saved_report_is_re_evaluated_with_navigation_thresholds(tmp_path):
    destination = tmp_path / "nav2-live-check.json"
    destination.write_text(
        json.dumps(
            {
                "asr_count": 4,
                "action_candidate_count": 2,
                "action_success_count": 2,
                "command_enqueue_count": 2,
                "execution_started_count": 2,
                "execution_finished_count": 2,
                "action_candidate_names": {"navigate_to": 1, "follow_waypoints": 1},
                "saw_awake": True,
                "saw_sleeping": True,
                "final_cmd_vel_zero": True,
                "ok": False,
                "missing": ["old stale failure"],
                "asr_samples": ["小智", "去门口", "巡逻门口书桌起点", "退出控制"],
                "action_candidate_samples": [
                    {"name": "navigate_to", "arguments": {"target": "door"}},
                    {
                        "name": "follow_waypoints",
                        "arguments": {"waypoints": ["door", "desk", "home"]},
                    },
                ],
                "successful_action_samples": [
                    {"success": True, "message": "succeeded"},
                    {"success": True, "message": "succeeded"},
                ],
            }
        ),
        encoding="utf-8",
    )

    report = live_check.evaluate_report(
        live_check.load_report(str(destination)),
        live_check.LiveCheckThresholds(
            min_asr=4,
            min_candidates=2,
            min_success=2,
            required_candidates=["navigate_to", "follow_waypoints"],
            require_navigation_details=True,
        ),
    )

    assert report.ok is True
    assert report.missing == []


def test_saved_report_fails_when_patrol_candidate_is_missing(tmp_path):
    destination = tmp_path / "nav2-live-check.json"
    destination.write_text(
        json.dumps(
            {
                "asr_count": 4,
                "action_candidate_count": 2,
                "action_success_count": 2,
                "command_enqueue_count": 2,
                "execution_started_count": 2,
                "execution_finished_count": 2,
                "action_candidate_names": {"navigate_to": 2},
                "saw_awake": True,
                "saw_sleeping": True,
                "final_cmd_vel_zero": True,
                "ok": True,
                "missing": [],
                "asr_samples": ["小智", "去门口", "退出控制", "退出控制"],
                "action_candidate_samples": [
                    {"name": "navigate_to", "arguments": {"target": "door"}},
                    {"name": "navigate_to", "arguments": {"target": "desk"}},
                ],
                "successful_action_samples": [
                    {"success": True, "message": "succeeded"},
                    {"success": True, "message": "succeeded"},
                ],
            }
        ),
        encoding="utf-8",
    )

    report = live_check.evaluate_report(
        live_check.load_report(str(destination)),
        live_check.LiveCheckThresholds(
            min_asr=4,
            min_candidates=2,
            min_success=2,
            required_candidates=["navigate_to", "follow_waypoints"],
        ),
    )

    assert report.ok is False
    assert "candidate follow_waypoints observed" in report.missing


def test_saved_report_rejects_old_schema_without_samples(tmp_path):
    destination = tmp_path / "old-nav2-live-check.json"
    destination.write_text(
        json.dumps(
            {
                "asr_count": 4,
                "action_candidate_count": 2,
                "action_success_count": 2,
                "command_enqueue_count": 2,
                "execution_started_count": 2,
                "execution_finished_count": 2,
                "action_candidate_names": {"navigate_to": 1, "follow_waypoints": 1},
                "saw_awake": True,
                "saw_sleeping": True,
                "final_cmd_vel_zero": True,
                "ok": True,
                "missing": [],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="asr_samples"):
        live_check.load_report(str(destination))
