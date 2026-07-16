#!/usr/bin/env python3
"""Unit-style checks for continuous_live_check report logic."""

import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest
from std_msgs.msg import Header as RosHeader


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "continuous_live_check.py"


class _FakeNode:
    def __init__(self, *args, **kwargs):
        del args, kwargs

    def create_subscription(self, *args, **kwargs):
        del args, kwargs
        return None


class _FakeQosProfile:
    def __init__(self, **kwargs):
        self.settings = kwargs
        for key, value in kwargs.items():
            setattr(self, key, value)


_FAKED_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.qos",
    "geometry_msgs.msg",
    "std_msgs.msg",
)
_saved_modules = {name: sys.modules.get(name) for name in _FAKED_MODULES}

sys.modules["rclpy"] = types.SimpleNamespace()
sys.modules["rclpy.node"] = types.SimpleNamespace(Node=_FakeNode)
sys.modules["rclpy.qos"] = types.SimpleNamespace(
    DurabilityPolicy=types.SimpleNamespace(VOLATILE=0, TRANSIENT_LOCAL=1),
    HistoryPolicy=types.SimpleNamespace(KEEP_LAST=0),
    QoSProfile=_FakeQosProfile,
    ReliabilityPolicy=types.SimpleNamespace(RELIABLE=0),
)
sys.modules["geometry_msgs.msg"] = types.SimpleNamespace(Twist=object)
# RobotCommand 的生成代码会在实例化时延迟导入 Header；测试桩必须保留它，
# 否则同一 pytest 进程中的消息转换测试会被这里的全局模块替换污染。
sys.modules["std_msgs.msg"] = types.SimpleNamespace(String=object, Header=RosHeader)

spec = importlib.util.spec_from_file_location("continuous_live_check", SCRIPT)
live_check = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = live_check
spec.loader.exec_module(live_check)

# 仅在加载被测脚本期间注入 fake ROS；收集其他集成测试前恢复真实消息模块。
for _name, _module in _saved_modules.items():
    if _module is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _module


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


def test_initial_latched_sleeping_does_not_fake_session_exit():
    node = live_check.LiveCheckNode()
    node.session_states.extend(["sleeping", "awake"])
    node.asr.extend(["小智"] * 6)
    node.candidates.extend([{"name": "move"}] * 4)
    node.results.extend([{"success": True}] * 4)
    node.velocities.append((0.0, 0.0))

    report = node.build_report(live_check.LiveCheckThresholds())

    assert report.saw_awake
    assert not report.saw_sleeping
    assert "session sleeping observed" in report.missing


def test_live_check_report_counts_partial_final_recovery_feedback():
    node = live_check.LiveCheckNode()
    message = live_check.RecognitionFeedback()
    message.status = message.STATUS_ASR_FINAL_RECOVERED
    message.original = "把灯"
    message.rewritten = "把灯设成蓝色"
    node._on_recognition_feedback(message)

    report = node.build_report(
        live_check.LiveCheckThresholds(min_asr=0, min_candidates=0, min_success=0)
    )

    assert report.recognition_feedback_count == 1
    assert report.asr_final_recovery_count == 1
    assert report.recognition_feedback_samples[0]["recovered"] == "把灯设成蓝色"


def test_live_check_report_counts_queue_reject_ignore_and_retry_feedback():
    node = live_check.LiveCheckNode(agent_mode="online")
    for status in (
        live_check.RecognitionFeedback.STATUS_QUEUE_REJECTED,
        live_check.RecognitionFeedback.STATUS_IGNORED,
        live_check.RecognitionFeedback.STATUS_RETRY,
    ):
        message = live_check.RecognitionFeedback()
        message.status = status
        node._on_recognition_feedback(message)

    report = node.build_report(
        live_check.LiveCheckThresholds(min_asr=0, min_candidates=0, min_success=0)
    )

    assert report.agent_mode == "online"
    assert report.queue_rejected_count == 1
    assert report.ignored_transcript_count == 1
    assert report.recognition_retry_count == 1


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


def test_live_check_report_extracts_nav2_failure_reasons():
    node = live_check.LiveCheckNode()
    node.asr.extend(["小智", "去门口", "退出控制"])
    node.session_states.extend(["awake", "sleeping"])
    node.candidates.append({"name": "navigate_to", "arguments": {"target": "door"}})
    node.results.extend(
        [
            {
                "success": False,
                "message": "nav2:navigate_to_pose:aborted target=door error_code=304 error_msg=planner_failed",
            },
            {
                "success": False,
                "message": "nav2:follow_waypoints:canceled waypoints=door,desk missed_waypoints=1",
            },
        ]
    )
    node.velocities.append((0.0, 0.0))

    report = node.build_report(
        live_check.LiveCheckThresholds(min_asr=1, min_candidates=1, min_success=0)
    )

    assert report.navigation_failure_reasons == [
        {
            "backend": "nav2",
            "action": "navigate_to_pose",
            "status": "aborted",
            "target": "door",
            "error_code": "304",
            "error_msg": "planner_failed",
            "failure_class": "planner_failed",
            "retry_hint": "检查 map/goal 是否可达、全局代价地图和 planner server 日志。",
        },
        {
            "backend": "nav2",
            "action": "follow_waypoints",
            "status": "canceled",
            "waypoints": "door,desk",
            "missed_waypoints": "1",
            "failure_class": "canceled",
            "retry_hint": "确认是否由语音 stop/cancel_navigation 或人工取消触发。",
        },
    ]
    assert report.action_result_samples[0]["message"].startswith("nav2:navigate_to_pose")
    assert live_check._parse_nav2_result_message(
        "nav2:navigate_to_pose:aborted target=desk error_code=12 error_msg=planner failed near obstacle"
    )["error_msg"] == "planner failed near obstacle"


def test_nav2_failure_parser_classifies_common_failure_layers():
    cases = {
        "planner_failed": "nav2:navigate_to_pose:aborted target=door error_msg=planner_failed",
        "controller_failed": "nav2:navigate_to_pose:aborted target=door error_msg=controller failed near obstacle",
        "localization_lost": "nav2:navigate_to_pose:aborted target=door error_msg=tf transform unavailable",
        "waypoint_missed": "nav2:follow_waypoints:aborted waypoints=door,desk missed_waypoints=1",
        "timeout": "nav2:navigate_to_pose:timed_out target=door error_msg=timeout waiting for result",
        "canceled": "nav2:navigate_to_pose:canceled target=door",
    }

    for expected, message in cases.items():
        parsed = live_check._parse_nav2_result_message(message)
        assert parsed is not None
        assert parsed["failure_class"] == expected
        assert parsed["retry_hint"]

    # FollowWaypoints 的 Action 终态可以是 SUCCEEDED，但 missed_waypoints 非空仍
    # 表示巡检路线没有完整执行，现场诊断不能把它归为成功。
    partial = live_check._parse_nav2_result_message(
        "nav2:follow_waypoints:succeeded waypoints=door,desk missed_waypoints=1"
    )
    assert partial is not None
    assert partial["failure_class"] == "waypoint_missed"


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
