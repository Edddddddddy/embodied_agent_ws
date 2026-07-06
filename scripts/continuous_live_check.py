#!/usr/bin/env python3
"""辅助真实麦克风连续控制验收。

先在终端 1 启动 `bash scripts/acceptance_test.sh continuous-offline` 或 online；
再在终端 2 运行本脚本。人工说完固定话术后，本脚本基于 ROS topic 统计 PASS/FAIL。
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import rclpy
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
    asr_samples: list[str] = field(default_factory=list)
    action_candidate_samples: list[dict] = field(default_factory=list)
    successful_action_samples: list[dict] = field(default_factory=list)


class LiveCheckNode(Node):
    def __init__(self):
        super().__init__("continuous_live_check")
        self.asr: list[str] = []
        self.session_states: list[str] = []
        self.queue_events: list[dict] = []
        self.execution_events: list[dict] = []
        self.candidates: list[dict] = []
        self.results: list[dict] = []
        self.velocities: list[tuple[float, float]] = []
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(String, "/agent/session_state", self._on_session, 10)
        self.create_subscription(String, "/agent/command_queue", self._on_queue, 10)
        self.create_subscription(String, "/agent/command_execution", self._on_execution, 10)
        self.create_subscription(String, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_asr(self, message: String) -> None:
        self.asr.append(message.data)

    def _on_session(self, message: String) -> None:
        self.session_states.append(message.data)

    def _on_queue(self, message: String) -> None:
        self.queue_events.append(_json_dict(message.data))

    def _on_execution(self, message: String) -> None:
        self.execution_events.append(_json_dict(message.data))

    def _on_candidate(self, message: String) -> None:
        self.candidates.append(_json_dict(message.data))

    def _on_result(self, message: String) -> None:
        self.results.append(_json_dict(message.data))

    def _on_velocity(self, message: Twist) -> None:
        self.velocities.append((message.linear.x, message.angular.z))

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
        checks = {
            f"ASR final >= {thresholds.min_asr}": len(self.asr) >= thresholds.min_asr,
            f"action candidate >= {thresholds.min_candidates}": len(self.candidates)
            >= thresholds.min_candidates,
            f"successful action result >= {thresholds.min_success}": success_count
            >= thresholds.min_success,
            "session awake observed": "awake" in self.session_states,
            "session sleeping observed": "sleeping" in self.session_states,
            "final cmd_vel is zero": final_zero,
        }
        for required in thresholds.required_candidates or []:
            checks[f"candidate {required} observed"] = candidate_names.get(required, 0) > 0
        if thresholds.require_navigation_details:
            checks["navigate_to target observed"] = _has_navigate_target(self.candidates)
            checks["follow_waypoints waypoints observed"] = _has_waypoint_patrol(self.candidates)
        missing = [name for name, passed in checks.items() if not passed]
        return LiveCheckReport(
            asr_count=len(self.asr),
            action_candidate_count=len(self.candidates),
            action_success_count=success_count,
            command_enqueue_count=enqueue_count,
            execution_started_count=started_count,
            execution_finished_count=finished_count,
            action_candidate_names=candidate_names,
            saw_awake=checks["session awake observed"],
            saw_sleeping=checks["session sleeping observed"],
            final_cmd_vel_zero=final_zero,
            ok=not missing,
            missing=missing,
            asr_samples=_tail(self.asr),
            action_candidate_samples=_tail(self.candidates),
            successful_action_samples=_tail(
                [result for result in self.results if result.get("success") is True]
            ),
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
        asr_samples=report.asr_samples,
        action_candidate_samples=report.action_candidate_samples,
        successful_action_samples=report.successful_action_samples,
    )


def _json_dict(serialized: str) -> dict:
    try:
        payload = json.loads(serialized)
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


def _tail(items: list, limit: int = 12) -> list:
    return items[-limit:]


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
        asr_samples=_required_str_list(payload, "asr_samples"),
        action_candidate_samples=_required_dict_list(
            payload, "action_candidate_samples"
        ),
        successful_action_samples=_required_dict_list(
            payload, "successful_action_samples"
        ),
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


def write_report(path: str | None, report: LiveCheckReport) -> None:
    if not path:
        return
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(asdict(report), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=180.0, help="统计窗口秒数")
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
        choices=("motion", "nav2"),
        default="motion",
        help="打印哪套人工验收话术",
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
    else:
        print("请在另一个终端启动 continuous-offline/online，然后按顺序说：", flush=True)
        print("小智 / 向前走一秒 / 左转九十度 / 后退一秒 / 绕圈 / 走正方形 / 停下 / 退出控制", flush=True)
    print(f"开始统计 {args.duration:.0f}s 内的连续语音链路事件...", flush=True)

    rclpy.init()
    node = LiveCheckNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(max(1.0, args.duration))
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
