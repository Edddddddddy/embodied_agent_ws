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
from dataclasses import asdict, dataclass

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
        )


def _json_dict(serialized: str) -> dict:
    try:
        payload = json.loads(serialized)
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


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
    args = parser.parse_args()
    thresholds = LiveCheckThresholds(
        args.min_asr,
        args.min_candidates,
        args.min_success,
        args.require_candidate,
    )

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
