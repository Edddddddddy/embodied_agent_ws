#!/usr/bin/env python3
"""等待建图阶段就绪，并通过 typed ROS 2 Action 启动完整自动巡检任务。"""

from __future__ import annotations

import argparse
import json
import math
import time

from embodied_agent_interfaces.action import ManageSlamSession
from embodied_agent_interfaces.msg import SlamSessionState
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def positive_finite_seconds(value: str) -> float:
    """Parse a CLI timeout without allowing instant or unbounded waits."""

    try:
        seconds = float(value)
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError(f"invalid timeout: {value!r}") from error
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise argparse.ArgumentTypeError("timeout must be a positive finite number")
    return seconds


def mapping_is_ready(state: SlamSessionState | None) -> bool:
    """Return readiness and fail fast once this mission can no longer start."""

    if state is None:
        return False
    phase = state.phase
    if phase == SlamSessionState.MAPPING:
        return True
    if phase == SlamSessionState.STARTING_MAPPING:
        return False
    if phase == SlamSessionState.STOPPED:
        # 编排器构造后会先发布一次无 detail 的 STOPPED，再由 worker 启动 mapping；
        # 真正停止后的 STOPPED 带有原因，此时继续等满 180 秒没有意义。
        if not state.detail.strip():
            return False
        raise RuntimeError(f"mapping session stopped: {state.detail.strip()}")
    if phase in {
        SlamSessionState.AUTOMATIC_MAPPING,
        SlamSessionState.AUTOMATIC_NAVIGATING,
        SlamSessionState.MISSION_COMPLETED,
    }:
        raise RuntimeError(f"automatic mission already started; phase={phase}")
    if phase == SlamSessionState.FAILED:
        detail = state.detail.strip() or "no failure detail"
        raise RuntimeError(f"mapping session failed: {detail}")
    raise RuntimeError(f"automatic mission cannot start; phase={phase}")


class AutomaticMissionTrigger(Node):
    """ASR 故障时的 typed 备用入口；机器人控制链仍完整经过 Agent/Guard/Action。"""

    def __init__(self) -> None:
        super().__init__("automatic_slam_mission_trigger")
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state: SlamSessionState | None = None
        self.create_subscription(
            SlamSessionState, "/slam/session_state", self._on_state, state_qos
        )
        self.client = ActionClient(
            self, ManageSlamSession, "/slam/manage_session"
        )

    def _on_state(self, message: SlamSessionState) -> None:
        self.state = message

    @staticmethod
    def _feedback(message) -> None:
        feedback = message.feedback
        print(
            f"[mission] progress={feedback.progress * 100:.0f}% "
            f"phase={feedback.state.phase} detail={feedback.state.detail}",
            flush=True,
        )

    def wait_for_mapping(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if mapping_is_ready(self.state):
                return True
        return False

    def _cancel_and_wait(self, goal_handle, result_future) -> None:
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        if not cancel_future.done():
            raise TimeoutError("automatic mission cancel request timed out")
        cancel_response = cancel_future.result()
        if cancel_response is None:
            raise RuntimeError("automatic mission cancel returned no response")

        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        if result_future.done():
            # An empty goals_canceling list can mean the goal completed while the
            # cancel request was in flight. A delivered result resolves that race.
            return
        if not cancel_response.goals_canceling:
            raise RuntimeError("automatic mission cancel request was rejected")
        raise TimeoutError("automatic mission did not stop after cancellation")

    def send(self, timeout_s: float):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise TimeoutError("/slam/manage_session action server is unavailable")
        goal = ManageSlamSession.Goal()
        goal.command = ManageSlamSession.Goal.RUN_AUTOMATIC_MISSION
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            raise TimeoutError("automatic mission goal response timed out")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("automatic mission goal was rejected")

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        try:
            while rclpy.ok() and not result_future.done():
                if time.monotonic() >= deadline:
                    try:
                        self._cancel_and_wait(goal_handle, result_future)
                    except Exception as cancel_error:
                        raise TimeoutError(
                            "automatic mission result timed out and cancellation "
                            f"failed: {cancel_error}"
                        ) from cancel_error
                    raise TimeoutError(
                        "automatic mission result timed out; goal was canceled"
                    )
                rclpy.spin_once(self, timeout_sec=0.2)
        except KeyboardInterrupt:
            # Ctrl-C 必须通过 Action cancel 进入编排器清理，而不是只杀客户端。
            try:
                self._cancel_and_wait(goal_handle, result_future)
            except Exception as cancel_error:
                self.get_logger().error(
                    f"automatic mission cancellation failed: {cancel_error}"
                )
            raise
        if not result_future.done():
            raise RuntimeError("ROS context shut down before the mission completed")
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("automatic mission returned no result")
        return wrapped_result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mapping-timeout", type=positive_finite_seconds, default=180.0
    )
    parser.add_argument(
        "--mission-timeout", type=positive_finite_seconds, default=1200.0
    )
    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        node = AutomaticMissionTrigger()
        print("等待 /slam/session_state 进入 MAPPING...", flush=True)
        if not node.wait_for_mapping(args.mapping_timeout):
            raise TimeoutError("mapping stage did not become ready")
        print("MAPPING ready；发送 typed RUN_AUTOMATIC_MISSION goal。", flush=True)
        wrapped = node.send(args.mission_timeout)
        result = wrapped.result
        report = {
            "success": bool(result.success),
            "message": result.message,
            "phase": int(result.state.phase),
            "map_saved": bool(result.state.map_saved),
            "map_yaml_path": result.state.map_yaml_path,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        mission_completed = result.success and (
            result.state.phase == SlamSessionState.MISSION_COMPLETED
        )
        return 0 if mission_completed else 1
    except KeyboardInterrupt:
        print("automatic mission canceled by user", flush=True)
        return 130
    except Exception as error:
        print(f"FAIL: {error}", flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
