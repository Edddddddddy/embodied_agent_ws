#!/usr/bin/env python3
"""验证 typed Action 与 ASR 系统意图共同驱动 SLAM 会话状态机。"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

from embodied_agent_interfaces.action import ExecuteRobotCommand, ManageSlamSession
from embodied_agent_interfaces.msg import RobotCommand, SlamSessionState
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class SessionProbe(Node):
    def __init__(self) -> None:
        super().__init__("voice_slam_session_probe")
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.states: list[SlamSessionState] = []
        self.feedback_phases: list[int] = []
        self.asr_pub = self.create_publisher(String, "/agent/asr_final", 10)
        self.create_subscription(
            SlamSessionState,
            "/slam/session_state",
            self.states.append,
            state_qos,
        )
        self.client = ActionClient(
            self, ManageSlamSession, "/slam/manage_session"
        )
        self.robot_client = ActionClient(
            self, ExecuteRobotCommand, "/robot/execute_command"
        )

    def has_phase(self, phase: int) -> bool:
        return any(state.phase == phase for state in self.states)


def wait_until(predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--transition-timeout", type=float, default=10.0)
    parser.add_argument("--evidence-kind", default="dry_run_process_adapter")
    parser.add_argument("--explore-before-save", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = SessionProbe()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.has_phase(SlamSessionState.MAPPING),
            args.transition_timeout,
            "orchestrator did not enter MAPPING",
        )
        wait_until(
            lambda: node.asr_pub.get_subscription_count() > 0,
            5.0,
            "ASR final subscriber unavailable",
        )

        exploration_succeeded = None
        if args.explore_before_save:
            assert node.robot_client.wait_for_server(timeout_sec=10.0)
            explore_goal = ExecuteRobotCommand.Goal()
            explore_goal.command.action_type = RobotCommand.MOVE
            explore_goal.command.linear_x = 0.18
            explore_goal.command.duration_s = 2.0
            explore_goal.command.command_id = "showcase-session-exploration"
            explore_goal.command.source = "slam_session_acceptance"
            explore_future = node.robot_client.send_goal_async(explore_goal)
            wait_until(explore_future.done, 10.0, "exploration goal response missing")
            explore_handle = explore_future.result()
            assert explore_handle.accepted
            explore_result_future = explore_handle.get_result_async()
            wait_until(
                explore_result_future.done,
                30.0,
                "mapping exploration action did not finish",
            )
            exploration_succeeded = bool(
                explore_result_future.result().result.success
            )
            assert exploration_succeeded

        # 语音系统意图负责保存地图；dry-run Adapter 不写伪造文件，但状态契约相同。
        node.asr_pub.publish(String(data="保存地图"))
        wait_until(
            lambda: node.has_phase(SlamSessionState.MAP_SAVED),
            args.transition_timeout,
            "voice save-map command did not reach MAP_SAVED",
        )

        assert node.client.wait_for_server(timeout_sec=5.0)
        goal = ManageSlamSession.Goal()
        goal.command = ManageSlamSession.Goal.START_NAVIGATION

        def on_feedback(message) -> None:
            node.feedback_phases.append(message.feedback.state.phase)

        goal_future = node.client.send_goal_async(goal, feedback_callback=on_feedback)
        wait_until(goal_future.done, 5.0, "session Action goal response missing")
        goal_handle = goal_future.result()
        assert goal_handle.accepted
        result_future = goal_handle.get_result_async()
        wait_until(
            result_future.done,
            args.transition_timeout,
            "session Action result missing",
        )
        result = result_future.result().result
        assert result.success, result.message
        assert result.state.phase == SlamSessionState.NAVIGATING
        assert result.state.map_saved
        assert node.has_phase(SlamSessionState.SWITCHING_TO_NAVIGATION)
        assert node.has_phase(SlamSessionState.STARTING_NAVIGATION)

        # ASR 抖动造成的重复“开始导航”是幂等操作，不能把会话打入 FAILED。
        node.asr_pub.publish(String(data="开始导航"))
        node.asr_pub.publish(String(data="开始导航"))
        time.sleep(0.3)
        assert not node.has_phase(SlamSessionState.FAILED)

        report = {
            "passed": True,
            "state_sequence": [int(state.phase) for state in node.states],
            "feedback_phases": node.feedback_phases,
            "map_saved": bool(result.state.map_saved),
            "map_yaml_path": result.state.map_yaml_path,
            "final_phase": int(result.state.phase),
            "evidence_kind": args.evidence_kind,
            "exploration_succeeded": exploration_succeeded,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
