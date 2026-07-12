#!/usr/bin/env python3
"""辅助真实麦克风连续控制验收。

先在终端 1 启动 `bash scripts/acceptance_test.sh continuous-offline` 或 online；
再在终端 2 运行本脚本。人工说完固定话术后，本脚本基于 ROS topic 统计 PASS/FAIL。
"""

from __future__ import annotations

import argparse
import json
import re
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import rclpy
from embodied_agent_interfaces.msg import (
    CommandExecutionEvent,
    CommandQueueEvent,
    NluParseEvent,
    RecognitionFeedback,
    RobotCommand,
    RobotCommandResult,
)
from embodied_online_agent.ros_action_transport import (
    command_message_to_dict,
    result_message_to_dict,
)
from embodied_online_agent.ros_event_transport import (
    execution_event_message_to_dict,
    nlu_parse_message_to_dict,
    queue_event_message_to_dict,
    recognition_feedback_message_to_dict,
)
from embodied_online_agent.ros_qos import command_event_qos, latched_state_qos
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class LiveCheckThresholds:
    min_asr: int = 6
    min_candidates: int = 4
    min_success: int = 4
    required_candidates: list[str] | None = None
    require_navigation_details: bool = False


@dataclass
class LiveCheckReport:
    asr_count: int
    action_candidate_count: int
    action_success_count: int
    command_enqueue_count: int
    execution_started_count: int
    execution_finished_count: int
    action_candidate_names: dict[str, int]
    saw_awake: bool
    saw_sleeping: bool
    final_cmd_vel_zero: bool
    ok: bool
    missing: list[str]
    recognition_feedback_count: int = 0
    asr_final_recovery_count: int = 0
    recognition_feedback_samples: list[dict] = field(default_factory=list)
    asr_samples: list[str] = field(default_factory=list)
    action_candidate_samples: list[dict] = field(default_factory=list)
    action_result_samples: list[dict] = field(default_factory=list)
    successful_action_samples: list[dict] = field(default_factory=list)
    navigation_failure_reasons: list[dict] = field(default_factory=list)
    duration_s: float = 0.0
    metrics_samples: list[dict] = field(default_factory=list)
    action_e2e_latency_ms: list[float] = field(default_factory=list)
    capture_source: str = "unspecified"


class LiveCheckNode(Node):
    def __init__(self, capture_source: str = "unspecified"):
        super().__init__("continuous_live_check")
        self.asr: list[str] = []
        self.session_states: list[str] = []
        self.queue_events: list[dict] = []
        self.execution_events: list[dict] = []
        self.candidates: list[dict] = []
        self.results: list[dict] = []
        self.velocities: list[tuple[float, float]] = []
        self.metrics: list[dict] = []
        self.recognition_feedback: list[dict] = []
        self.started_at = time.monotonic()
        self._last_asr_at = 0.0
        self._candidate_asr_at: dict[str, float] = {}
        self._candidate_name_by_id: dict[str, str] = {}
        self.action_e2e_latency_ms: list[float] = []
        self.capture_source = capture_source
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(
            String, "/agent/session_state", self._on_session, latched_state_qos()
        )
        self.create_subscription(
            CommandQueueEvent,
            "/agent/command_queue",
            self._on_queue,
            command_event_qos(),
        )
        self.create_subscription(
            CommandExecutionEvent,
            "/agent/command_execution",
            self._on_execution,
            command_event_qos(),
        )
        self.create_subscription(
            RobotCommand, "/agent/action_candidate", self._on_candidate, 10
        )
        self.create_subscription(
            RobotCommandResult, "/robot/action_result", self._on_result, 10
        )
        self.create_subscription(
            RecognitionFeedback,
            "/agent/recognition_feedback",
            self._on_recognition_feedback,
            command_event_qos(),
        )
        self.create_subscription(
            NluParseEvent,
            "/agent/nlu_parse",
            self._on_nlu_parse,
            command_event_qos(),
        )
        self.create_subscription(String, "/agent/metrics", self._on_metrics, 10)
        # 在线与离线 Agent 为避免指标语义混淆使用了不同 topic；评测探针同时监听，
        # 让同一套 benchmark 能覆盖两条链路，而不是让离线报告悄悄缺失延迟数据。
        self.create_subscription(String, "/offline_agent/metrics", self._on_metrics, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_asr(self, message: String) -> None:
        self.asr.append(message.data)
        self._last_asr_at = time.monotonic()

    def _on_session(self, message: String) -> None:
        self.session_states.append(message.data)

    def _on_queue(self, message: CommandQueueEvent) -> None:
        self.queue_events.append(queue_event_message_to_dict(message))

    def _on_execution(self, message: CommandExecutionEvent) -> None:
        self.execution_events.append(execution_event_message_to_dict(message))

    def _on_candidate(self, message: RobotCommand) -> None:
        candidate = command_message_to_dict(message)
        self.candidates.append(candidate)
        request_id = str(candidate.get("request_id") or "")
        if request_id and self._last_asr_at > 0.0:
            self._candidate_asr_at[request_id] = self._last_asr_at
        if request_id:
            self._candidate_name_by_id[request_id] = str(candidate.get("name") or "")

    def _on_result(self, message: RobotCommandResult) -> None:
        result = result_message_to_dict(message)
        command_id = str(result.get("command_id") or "")
        action_name = self._candidate_name_by_id.pop(command_id, "")
        if action_name:
            # C++ bridge 的 result 只携带 command_id/status；在采集端用同一个
            # command_id 补回动作名，才能证明“成功的是哪条命令”，而不只是成功总数。
            result["action_name"] = action_name
        self.results.append(result)
        started = self._candidate_asr_at.pop(command_id, None)
        if started is not None:
            # 该延迟包含排队和实际动作时长，表达“ASR final 到机器人终态”，
            # 与只看 LLM/TTS 首包的交互延迟是不同指标。
            self.action_e2e_latency_ms.append(
                round((time.monotonic() - started) * 1000.0, 3)
            )

    def _on_velocity(self, message: Twist) -> None:
        self.velocities.append((message.linear.x, message.angular.z))

    def _on_metrics(self, message: String) -> None:
        self.metrics.append(_json_dict(message.data))

    def _on_recognition_feedback(self, message: RecognitionFeedback) -> None:
        payload = recognition_feedback_message_to_dict(message)
        if payload:
            self.recognition_feedback.append(payload)

    def _on_nlu_parse(self, message: NluParseEvent) -> None:
        self.recognition_feedback.append(nlu_parse_message_to_dict(message))

    def build_report(self, thresholds: LiveCheckThresholds) -> LiveCheckReport:
        success_count = sum(1 for result in self.results if result.get("success") is True)
        enqueue_count = sum(1 for event in self.queue_events if event.get("event") == "enqueue")
        started_count = sum(1 for event in self.execution_events if event.get("event") == "started")
        finished_count = sum(1 for event in self.execution_events if event.get("event") == "finished")
        final_zero = bool(self.velocities) and all(abs(v) < 1e-6 for v in self.velocities[-1])
        candidate_names: dict[str, int] = {}
        for candidate in self.candidates:
            name = str(candidate.get("name") or "")
            if name:
                candidate_names[name] = candidate_names.get(name, 0) + 1
        saw_awake = "awake" in self.session_states
        # transient-local 会在探针刚加入时回放当前 sleeping；验收真正关心的是
        # 一次会话醒来后是否又正常休眠，不能把启动快照当成“退出控制”证据。
        saw_sleeping_after_awake = _contains_ordered_states(
            self.session_states, "awake", "sleeping"
        )
        checks = {
            f"ASR final >= {thresholds.min_asr}": len(self.asr) >= thresholds.min_asr,
            f"action candidate >= {thresholds.min_candidates}": len(self.candidates)
            >= thresholds.min_candidates,
            f"successful action result >= {thresholds.min_success}": success_count
            >= thresholds.min_success,
            "session awake observed": saw_awake,
            "session sleeping observed": saw_sleeping_after_awake,
            "final cmd_vel is zero": final_zero,
        }
        for required in thresholds.required_candidates or []:
            checks[f"candidate {required} observed"] = candidate_names.get(required, 0) > 0
        if thresholds.require_navigation_details:
            checks["navigate_to target observed"] = _has_navigate_target(self.candidates)
            checks["follow_waypoints waypoints observed"] = _has_waypoint_patrol(self.candidates)
        missing = [name for name, passed in checks.items() if not passed]
        recovery_count = sum(
            1
            for item in self.recognition_feedback
            if item.get("status") == "asr_final_recovered"
        )
        return LiveCheckReport(
            asr_count=len(self.asr),
            action_candidate_count=len(self.candidates),
            action_success_count=success_count,
            command_enqueue_count=enqueue_count,
            execution_started_count=started_count,
            execution_finished_count=finished_count,
            action_candidate_names=candidate_names,
            saw_awake=saw_awake,
            saw_sleeping=saw_sleeping_after_awake,
            final_cmd_vel_zero=final_zero,
            ok=not missing,
            missing=missing,
            recognition_feedback_count=len(self.recognition_feedback),
            asr_final_recovery_count=recovery_count,
            recognition_feedback_samples=_tail(self.recognition_feedback),
            asr_samples=_tail(self.asr),
            action_candidate_samples=_tail(self.candidates),
            action_result_samples=_tail(self.results),
            successful_action_samples=_tail(
                [result for result in self.results if result.get("success") is True]
            ),
            navigation_failure_reasons=_navigation_failure_reasons(self.results),
            duration_s=round(time.monotonic() - self.started_at, 3),
            metrics_samples=_tail(self.metrics),
            action_e2e_latency_ms=_tail(self.action_e2e_latency_ms),
            capture_source=self.capture_source,
        )


def evaluate_report(report: LiveCheckReport, thresholds: LiveCheckThresholds) -> LiveCheckReport:
    """按当前验收阈值重新判定历史报告。

    现场报告会被保存成证据文件，后续复盘时不能只相信文件里的 ok 字段；
    这里用同一套规则重新计算 missing，保证“留证文件”可以独立验收。
    """
    checks = {
        f"ASR final >= {thresholds.min_asr}": report.asr_count >= thresholds.min_asr,
        f"action candidate >= {thresholds.min_candidates}": report.action_candidate_count
        >= thresholds.min_candidates,
        f"successful action result >= {thresholds.min_success}": report.action_success_count
        >= thresholds.min_success,
        "session awake observed": report.saw_awake,
        "session sleeping observed": report.saw_sleeping,
        "final cmd_vel is zero": report.final_cmd_vel_zero,
    }
    for required in thresholds.required_candidates or []:
        checks[f"candidate {required} observed"] = (
            report.action_candidate_names.get(required, 0) > 0
        )
    if thresholds.require_navigation_details:
        checks["navigate_to target observed"] = _has_navigate_target(
            report.action_candidate_samples
        )
        checks["follow_waypoints waypoints observed"] = _has_waypoint_patrol(
            report.action_candidate_samples
        )
    missing = [name for name, passed in checks.items() if not passed]
    return LiveCheckReport(
        asr_count=report.asr_count,
        action_candidate_count=report.action_candidate_count,
        action_success_count=report.action_success_count,
        command_enqueue_count=report.command_enqueue_count,
        execution_started_count=report.execution_started_count,
        execution_finished_count=report.execution_finished_count,
        action_candidate_names=report.action_candidate_names,
        saw_awake=report.saw_awake,
        saw_sleeping=report.saw_sleeping,
        final_cmd_vel_zero=report.final_cmd_vel_zero,
        ok=not missing,
        missing=missing,
        recognition_feedback_count=report.recognition_feedback_count,
        asr_final_recovery_count=report.asr_final_recovery_count,
        recognition_feedback_samples=report.recognition_feedback_samples,
        asr_samples=report.asr_samples,
        action_candidate_samples=report.action_candidate_samples,
        action_result_samples=report.action_result_samples,
        successful_action_samples=report.successful_action_samples,
        navigation_failure_reasons=report.navigation_failure_reasons,
        duration_s=report.duration_s,
        metrics_samples=report.metrics_samples,
        action_e2e_latency_ms=report.action_e2e_latency_ms,
        capture_source=report.capture_source,
    )


def _json_dict(serialized: str) -> dict:
    try:
        payload = json.loads(serialized)
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


def _tail(items: list, limit: int = 50) -> list:
    return items[-limit:]


def _contains_ordered_states(states: list[str], first: str, second: str) -> bool:
    """只有 first 之后出现的 second 才能证明完成了一次状态往返。"""
    seen_first = False
    for state in states:
        if state == first:
            seen_first = True
        elif state == second and seen_first:
            return True
    return False


def _has_navigate_target(candidates: list[dict]) -> bool:
    for candidate in candidates:
        if candidate.get("name") != "navigate_to":
            continue
        arguments = candidate.get("arguments") or {}
        if isinstance(arguments, dict) and arguments.get("target"):
            return True
    return False


def _has_waypoint_patrol(candidates: list[dict]) -> bool:
    for candidate in candidates:
        if candidate.get("name") != "follow_waypoints":
            continue
        arguments = candidate.get("arguments") or {}
        waypoints = arguments.get("waypoints") if isinstance(arguments, dict) else None
        if isinstance(waypoints, list) and len(waypoints) >= 2:
            return True
    return False


def _parse_nav2_result_message(message: str) -> dict | None:
    """Parse Nav2 executor detail strings into stable report fields.

    C++ executor messages follow this shape:
    `nav2:<action>:<status> key=value key=value`.  Keeping the parser in the
    evidence script avoids changing ROS messages while still making saved reports
    queryable for planner/controller/localization-style failures.
    """

    if not message.startswith("nav2:"):
        return None
    parts = message.split()
    head = parts[0].split(":")
    if len(head) < 3 or head[0] != "nav2":
        return None
    parsed = {
        "backend": "nav2",
        "action": head[1],
        "status": head[2],
    }
    tail = message[len(parts[0]):].strip()
    for match in re.finditer(r"(\w+)=([^=]*?)(?=\s+\w+=|$)", tail):
        key = match.group(1)
        value = match.group(2).strip()
        if key and value:
            parsed[key] = value
    parsed.update(_classify_nav2_failure(parsed))
    return parsed


def _classify_nav2_failure(parsed: dict) -> dict[str, str]:
    """Classify raw Nav2 result details into stable demo/debug categories.

    Nav2 action result strings vary between planner/controller/localization plugins.
    The live-check report should keep raw fields for evidence, but also provide a
    stable `failure_class` so interview/demo debugging can say exactly which layer
    failed without reverse-engineering free-form messages on the spot.
    """

    status = str(parsed.get("status", "")).lower()
    action = str(parsed.get("action", "")).lower()
    error_msg = str(parsed.get("error_msg", "")).lower()
    error_code = str(parsed.get("error_code", "")).lower()
    missed_waypoints = str(parsed.get("missed_waypoints", "")).strip()
    combined = " ".join([status, action, error_msg, error_code])

    if status == "succeeded":
        return {"failure_class": "none", "retry_hint": "no retry needed"}
    if "cancel" in combined:
        return {
            "failure_class": "canceled",
            "retry_hint": "确认是否由语音 stop/cancel_navigation 或人工取消触发。",
        }
    if "timeout" in combined or "timed" in combined:
        return {
            "failure_class": "timeout",
            "retry_hint": "检查目标点距离、Nav2 超时参数和机器人是否被障碍物卡住。",
        }
    if missed_waypoints and missed_waypoints not in {"0", "[]", "none"}:
        return {
            "failure_class": "waypoint_missed",
            "retry_hint": "检查 waypoint 顺序、地图目标点和局部避障是否导致某些点被跳过。",
        }
    if "planner" in combined or "planning" in combined or "compute_path" in combined:
        return {
            "failure_class": "planner_failed",
            "retry_hint": "检查 map/goal 是否可达、全局代价地图和 planner server 日志。",
        }
    if "controller" in combined or "control" in combined or "follow_path" in combined:
        return {
            "failure_class": "controller_failed",
            "retry_hint": "检查局部代价地图、cmd_vel 输出、障碍物和 controller server 日志。",
        }
    if (
        "localization" in combined
        or "amcl" in combined
        or "tf" in combined
        or "transform" in combined
    ):
        return {
            "failure_class": "localization_lost",
            "retry_hint": "检查 /tf、/map->/odom、initial pose 和 AMCL/RViz 定位状态。",
        }
    if status == "aborted":
        return {
            "failure_class": "aborted_unknown",
            "retry_hint": "查看 Nav2 planner/controller/bt_navigator 日志以定位具体 server。",
        }
    return {
        "failure_class": "nav2_failure",
        "retry_hint": "查看 Nav2 action result message 和相关 server 日志。",
    }


def _navigation_failure_reasons(results: list[dict]) -> list[dict]:
    reasons: list[dict] = []
    for result in results:
        message = result.get("message", "")
        if not isinstance(message, str):
            continue
        parsed = _parse_nav2_result_message(message)
        if parsed is None:
            continue
        if parsed.get("status") == "succeeded" and result.get("success") is True:
            continue
        reasons.append(parsed)
    return reasons


def load_report(path: str) -> LiveCheckReport:
    payload = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("report must be an object")
    candidate_names = _required_dict(payload, "action_candidate_names")
    return LiveCheckReport(
        asr_count=int(_required(payload, "asr_count")),
        action_candidate_count=int(_required(payload, "action_candidate_count")),
        action_success_count=int(_required(payload, "action_success_count")),
        command_enqueue_count=int(_required(payload, "command_enqueue_count")),
        execution_started_count=int(_required(payload, "execution_started_count")),
        execution_finished_count=int(_required(payload, "execution_finished_count")),
        action_candidate_names={str(k): int(v) for k, v in candidate_names.items()},
        saw_awake=bool(_required(payload, "saw_awake")),
        saw_sleeping=bool(_required(payload, "saw_sleeping")),
        final_cmd_vel_zero=bool(_required(payload, "final_cmd_vel_zero")),
        ok=bool(_required(payload, "ok")),
        missing=_required_str_list(payload, "missing"),
        recognition_feedback_count=int(payload.get("recognition_feedback_count", 0)),
        asr_final_recovery_count=int(payload.get("asr_final_recovery_count", 0)),
        recognition_feedback_samples=_optional_dict_list(
            payload, "recognition_feedback_samples"
        ),
        asr_samples=_required_str_list(payload, "asr_samples"),
        action_candidate_samples=_required_dict_list(
            payload, "action_candidate_samples"
        ),
        action_result_samples=_optional_dict_list(payload, "action_result_samples"),
        successful_action_samples=_required_dict_list(
            payload, "successful_action_samples"
        ),
        navigation_failure_reasons=_optional_dict_list(
            payload, "navigation_failure_reasons"
        ),
        duration_s=float(payload.get("duration_s", 0.0)),
        metrics_samples=_optional_dict_list(payload, "metrics_samples"),
        action_e2e_latency_ms=[
            float(value) for value in payload.get("action_e2e_latency_ms", [])
        ],
        capture_source=str(payload.get("capture_source", "unspecified")),
    )


def _required(payload: dict, key: str) -> object:
    if key not in payload:
        raise ValueError(f"report missing required field: {key}")
    return payload[key]


def _required_dict(payload: dict, key: str) -> dict:
    value = _required(payload, key)
    if not isinstance(value, dict):
        raise ValueError(f"report field must be an object: {key}")
    return value


def _required_list(payload: dict, key: str) -> list:
    value = _required(payload, key)
    if not isinstance(value, list):
        raise ValueError(f"report field must be a list: {key}")
    return value


def _required_str_list(payload: dict, key: str) -> list[str]:
    values = _required_list(payload, key)
    if any(not isinstance(item, str) for item in values):
        raise ValueError(f"report field must be a list of strings: {key}")
    return values


def _required_dict_list(payload: dict, key: str) -> list[dict]:
    values = _required_list(payload, key)
    if any(not isinstance(item, dict) for item in values):
        raise ValueError(f"report field must be a list of objects: {key}")
    return values


def _optional_dict_list(payload: dict, key: str) -> list[dict]:
    if key not in payload:
        return []
    return _required_dict_list(payload, key)


def write_report(path: str | None, report: LiveCheckReport) -> None:
    if not path:
        return
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(asdict(report), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def wait_with_progress(duration_s: float, interval_s: float) -> None:
    """等待统计窗口并持续显示剩余时间，避免真人验收看起来像卡死。"""

    duration_s = max(1.0, duration_s)
    interval_s = max(1.0, interval_s)
    started_at = time.monotonic()
    deadline = started_at + duration_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        elapsed = time.monotonic() - started_at
        print(
            f"[benchmark] 已统计 {elapsed:.0f}s，剩余 {remaining:.0f}s...",
            flush=True,
        )
        time.sleep(min(interval_s, remaining))
    print(f"[benchmark] 统计窗口 {duration_s:.0f}s 已结束，正在生成报告...", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=180.0, help="统计窗口秒数")
    parser.add_argument(
        "--progress-interval",
        type=float,
        default=15.0,
        help="倒计时输出间隔秒数",
    )
    parser.add_argument("--min-asr", type=int, default=6)
    parser.add_argument("--min-candidates", type=int, default=4)
    parser.add_argument("--min-success", type=int, default=4)
    parser.add_argument(
        "--require-candidate",
        action="append",
        default=[],
        help="要求现场至少出现一次指定 action candidate，可重复传入",
    )
    parser.add_argument(
        "--scenario",
        choices=("motion", "nav2", "benchmark"),
        default="motion",
        help="打印哪套人工验收话术",
    )
    parser.add_argument(
        "--capture-source",
        choices=("unspecified", "real_microphone", "synthetic"),
        default="unspecified",
        help="显式声明输入证据来源；时长本身不能证明是真人麦克风",
    )
    parser.add_argument(
        "--control-managed",
        action="store_true",
        help="控制链路已由一键留证脚本后台管理，不再提示另开终端",
    )
    parser.add_argument("--output", default="", help="可选：把验收统计写入证据文件")
    parser.add_argument("--input-report", default="", help="读取已有证据文件并重新判定")
    parser.add_argument(
        "--require-navigation-details",
        action="store_true",
        help="要求报告中出现带 target 的 navigate_to 和带 waypoints 的 follow_waypoints",
    )
    args = parser.parse_args()
    thresholds = LiveCheckThresholds(
        args.min_asr,
        args.min_candidates,
        args.min_success,
        args.require_candidate,
        args.require_navigation_details,
    )

    if args.input_report:
        report = evaluate_report(load_report(args.input_report), thresholds)
        print(json.dumps(asdict(report), ensure_ascii=False, indent=2), flush=True)
        if report.ok:
            print("PASS: saved live microphone evidence is sufficient")
            return
        print(
            "FAIL: saved live microphone evidence is incomplete; missing="
            + "、".join(report.missing)
        )
        raise SystemExit(1)

    if args.scenario == "nav2":
        print("请在另一个终端启动 continuous-nav2-offline/online，然后按顺序说：", flush=True)
        print("小智 / 去门口 / 前往书桌 / 依次去门口、书桌、起点 / 停止巡航 / 退出控制", flush=True)
    elif args.scenario == "benchmark":
        if args.control_managed:
            print("后台控制链路已就绪，请按顺序说：", flush=True)
        else:
            print("请在另一个终端启动 continuous-offline/online，然后按顺序说：", flush=True)
        print(
            "小智 / 向前走一秒 / 左转九十度 / 后退一秒 / 右转九十度 / "
            "绕圈 / 挥手两次 / 把灯设成蓝色 / 去门口 / 取消导航 / 停下 / 退出控制",
            flush=True,
        )
    else:
        print("请在另一个终端启动 continuous-offline/online，然后按顺序说：", flush=True)
        print("小智 / 向前走一秒 / 左转九十度 / 后退一秒 / 绕圈 / 走正方形 / 停下 / 退出控制", flush=True)
    print(f"开始统计 {args.duration:.0f}s 内的连续语音链路事件...", flush=True)

    rclpy.init()
    node = LiveCheckNode(capture_source=args.capture_source)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_with_progress(args.duration, args.progress_interval)
        report = node.build_report(thresholds)
        write_report(args.output, report)
        print(json.dumps(asdict(report), ensure_ascii=False, indent=2), flush=True)
        if report.ok:
            print("PASS: live microphone continuous voice control evidence is sufficient")
        else:
            print("FAIL: live microphone evidence is incomplete; missing=" + "、".join(report.missing))
            raise SystemExit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
