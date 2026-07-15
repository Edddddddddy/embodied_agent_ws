#!/usr/bin/env python3
"""验证 typed Action 与 ASR 系统意图共同驱动 SLAM 会话状态机。"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from pathlib import Path

from embodied_agent_interfaces.action import ExecuteRobotCommand, ManageSlamSession
from embodied_agent_interfaces.msg import (
    RobotCommand,
    RobotCommandResult,
    SlamSessionState,
)
from embodied_agent_core.ros_qos import command_qos, event_qos, sensor_qos, state_qos
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from typed_action_test_utils import candidate_dict, result_dict
import yaml


class SessionProbe(Node):
    def __init__(self) -> None:
        super().__init__("voice_slam_session_probe")
        self.states: list[SlamSessionState] = []
        self.feedback_phases: list[int] = []
        self.candidates: list[dict] = []
        self.results: list[dict] = []
        self.positions: list[tuple[float, float]] = []
        self.map_stats: dict | None = None
        self.scan_count = 0
        self.asr_pub = self.create_publisher(
            String, "/agent/asr_final", command_qos(depth=10)
        )
        self.text_pub = self.create_publisher(
            String, "/agent/text_input", command_qos(depth=10)
        )
        self.create_subscription(
            SlamSessionState,
            "/slam/session_state",
            self.states.append,
            state_qos(),
        )
        self.create_subscription(
            RobotCommand,
            "/agent/action_candidate",
            lambda message: self.candidates.append(candidate_dict(message)),
            command_qos(depth=20),
        )
        self.create_subscription(
            RobotCommandResult,
            "/robot/action_result",
            lambda message: self.results.append(result_dict(message)),
            event_qos(depth=20),
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            sensor_qos(),
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            state_qos(),
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            sensor_qos(),
        )
        self.client = ActionClient(
            self, ManageSlamSession, "/slam/manage_session"
        )
        self.robot_client = ActionClient(
            self, ExecuteRobotCommand, "/robot/execute_command"
        )

    def has_phase(self, phase: int) -> bool:
        return any(state.phase == phase for state in self.states)

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.positions.append((position.x, position.y))

    def _on_map(self, message: OccupancyGrid) -> None:
        known = sum(1 for value in message.data if value >= 0)
        occupied = sum(1 for value in message.data if value >= 65)
        self.map_stats = {
            "frame_id": message.header.frame_id,
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "known_cells": known,
            "occupied_cells": occupied,
        }

    def _on_scan(self, _message: LaserScan) -> None:
        self.scan_count += 1

    def result_for(self, command_id: str) -> dict | None:
        for result in reversed(self.results):
            if result.get("command_id") == command_id:
                return result
        return None


def wait_until(predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def cumulative_distance(positions: list[tuple[float, float]]) -> float:
    """累计 mapping 里程，忽略 Gazebo 重启或里程计重置产生的瞬时跳变。"""

    distance = 0.0
    for previous, current in zip(positions, positions[1:]):
        step = math.hypot(current[0] - previous[0], current[1] - previous[1])
        if step <= 0.5:
            distance += step
    return distance


def run_text_action(
    node: SessionProbe,
    *,
    text: str,
    expected_action: str,
    timeout: float,
) -> dict:
    """通过 Agent 文本入口执行一步；自动门禁只替换声学 ASR，不绕过控制链。"""

    candidate_start = len(node.candidates)
    node.text_pub.publish(String(data=text))
    wait_until(
        lambda: any(
            item.get("name") == expected_action
            for item in node.candidates[candidate_start:]
        ),
        15.0,
        f"no {expected_action} candidate for {text!r}",
    )
    candidate = next(
        item
        for item in node.candidates[candidate_start:]
        if item.get("name") == expected_action
    )
    command_id = str(candidate.get("request_id") or "")
    if not command_id:
        raise RuntimeError(f"candidate has no command id: {candidate}")
    wait_until(
        lambda: node.result_for(command_id) is not None,
        timeout,
        f"action result timeout for {text!r}",
    )
    result = node.result_for(command_id)
    if result is None or result.get("success") is not True:
        raise RuntimeError(f"action failed for {text!r}: {result}")
    return {"text": text, "candidate": candidate, "result": result}


def load_survey_plan(path: Path) -> tuple[list[dict], dict]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    route = payload.get("mapping_route") if isinstance(payload, dict) else None
    acceptance = payload.get("acceptance") if isinstance(payload, dict) else None
    if not isinstance(route, list) or not route:
        raise ValueError("survey plan requires a non-empty mapping_route")
    if not isinstance(acceptance, dict):
        raise ValueError("survey plan requires acceptance thresholds")
    return route, acceptance


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--transition-timeout", type=float, default=10.0)
    parser.add_argument("--evidence-kind", default="dry_run_process_adapter")
    parser.add_argument("--explore-before-save", action="store_true")
    parser.add_argument(
        "--survey-plan",
        type=Path,
        help="通过 Agent 顺序执行工作场景 mapping_route，并验证地图覆盖与里程",
    )
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
        survey_steps: list[dict] = []
        survey_distance_m = 0.0
        survey_map: dict | None = None
        if args.survey_plan:
            route, thresholds = load_survey_plan(args.survey_plan)
            wait_until(
                lambda: (
                    node.text_pub.get_subscription_count() > 0
                    and node.positions
                    and node.map_stats is not None
                    and node.scan_count >= 3
                ),
                30.0,
                "Agent/odom/map/scan inputs unavailable for mapping survey",
            )
            odom_start = len(node.positions)
            for index, step in enumerate(route, start=1):
                print(
                    f"[mapping {index}/{len(route)}] {step['label']}: {step['text']}",
                    flush=True,
                )
                survey_steps.append(
                    run_text_action(
                        node,
                        text=str(step["text"]),
                        expected_action=str(step["action"]),
                        timeout=25.0,
                    )
                )
            wait_until(
                lambda: node.map_stats is not None
                and node.map_stats["known_cells"]
                >= int(thresholds["min_known_map_cells"]),
                15.0,
                "SLAM map did not reach the required known-cell coverage",
            )
            survey_distance_m = cumulative_distance(node.positions[odom_start:])
            survey_map = dict(node.map_stats or {})
            assert survey_distance_m >= float(thresholds["min_mapping_path_m"]), (
                survey_distance_m,
                thresholds,
            )
            assert survey_map["occupied_cells"] >= int(
                thresholds["min_occupied_map_cells"]
            )
            exploration_succeeded = True
        elif args.explore_before_save:
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
        observed_phases = {
            *(state.phase for state in node.states),
            *node.feedback_phases,
        }
        assert SlamSessionState.SWITCHING_TO_NAVIGATION in observed_phases
        assert SlamSessionState.STARTING_NAVIGATION in observed_phases

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
            "survey_step_count": len(survey_steps),
            "survey_distance_m": round(survey_distance_m, 3),
            "survey_map": survey_map,
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
