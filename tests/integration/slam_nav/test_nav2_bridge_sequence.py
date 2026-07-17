#!/usr/bin/env python3
"""Verify voice navigation commands are bridged to Nav2 action goals.

这里用轻量 fake Nav2 action server 代替完整 Nav2 bringup。它不做路径规划，
只检查 `Nav2RobotExecutor` 是否真的把语义地点转换为 NavigateToPose /
FollowWaypoints goal。这样 CI 不需要地图和 Gazebo，也能覆盖最关键的 bridge seam。
"""

import json
import threading
import time
from pathlib import Path

import rclpy
from embodied_agent_interfaces.msg import RobotCommand, RobotCommandResult
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from std_msgs.msg import String
from tools.acceptance.typed_action_probe_utils import candidate_dict, result_dict


class FakeNav2BridgeProbe(Node):
    def __init__(self):
        super().__init__("fake_nav2_bridge_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.candidates = []
        self.results = []
        self.navigate_goals = []
        self.follow_goals = []
        self.navigate_cancel_requests = 0
        self.home_goal_running = threading.Event()
        self.home_goal_canceled = threading.Event()
        self.create_subscription(RobotCommand, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)
        self.navigate_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            self._execute_navigate,
            cancel_callback=self._cancel_navigate,
        )
        self.follow_server = ActionServer(
            self, FollowWaypoints, "follow_waypoints", self._execute_follow
        )

    def _on_candidate(self, message):
        self.candidates.append(candidate_dict(message))

    def _on_result(self, message):
        self.results.append(result_dict(message))

    def _execute_navigate(self, goal_handle):
        self.navigate_goals.append(goal_handle.request.pose)
        pose = goal_handle.request.pose.pose.position
        if abs(pose.x - 4.0) < 1e-6 and abs(pose.y - 4.0) < 1e-6:
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.error_code = 42
            result.error_msg = "planner_no_path"
            return result
        if abs(pose.x) < 1e-6 and abs(pose.y) < 1e-6:
            # home 目标故意保持运行，用于证明上层“取消导航”会到达真正的
            # NavigateToPose cancel 协议，而不只是让 Agent 本地队列停止等待。
            self.home_goal_running.set()
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline and not goal_handle.is_cancel_requested:
                time.sleep(0.02)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.home_goal_canceled.set()
                return NavigateToPose.Result()
            goal_handle.abort()
            return NavigateToPose.Result()
        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.error_code = NavigateToPose.Result.NONE
        return result

    def _cancel_navigate(self, _goal_handle):
        self.navigate_cancel_requests += 1
        return CancelResponse.ACCEPT

    def _execute_follow(self, goal_handle):
        self.follow_goals.append(goal_handle.request)
        goal_handle.succeed()
        result = FollowWaypoints.Result()
        result.error_code = FollowWaypoints.Result.NONE
        return result


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = FakeNav2BridgeProbe()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
            ),
            15.0,
            "Nav2 bridge pipeline was not discovered",
        )
        time.sleep(0.5)

        navigate_started_at = time.monotonic()
        node.text_pub.publish(String(data="去门口"))
        wait_until(
            lambda: node.navigate_goals
            and any(candidate.get("name") == "navigate_to" for candidate in node.candidates),
            10.0,
            "NavigateToPose goal was not received",
        )
        navigate_candidate = next(
            candidate for candidate in node.candidates
            if candidate.get("name") == "navigate_to"
        )
        navigate_id = navigate_candidate["request_id"]
        wait_until(
            lambda: any(
                result.get("command_id") == navigate_id and result.get("success") is True
                for result in node.results
            ),
            12.0,
            "navigate_to command did not finish",
        )
        navigate_elapsed = time.monotonic() - navigate_started_at
        if navigate_elapsed > 2.5:
            raise RuntimeError(
                "navigate_to result waited for local duration instead of Nav2 result: "
                f"{navigate_elapsed:.2f}s"
            )
        pose = node.navigate_goals[-1]
        if pose.header.frame_id != "map":
            raise RuntimeError(f"unexpected nav frame: {pose.header.frame_id}")
        if abs(pose.pose.position.x - 1.2) > 1e-6 or abs(pose.pose.position.y) > 1e-6:
            raise RuntimeError(f"unexpected nav pose: {pose}")

        follow_started_at = time.monotonic()
        node.text_pub.publish(String(data="依次去门口、书桌、起点"))
        wait_until(
            lambda: node.follow_goals
            and any(candidate.get("name") == "follow_waypoints" for candidate in node.candidates),
            10.0,
            "FollowWaypoints goal was not received",
        )
        follow_candidate = next(
            candidate for candidate in node.candidates
            if candidate.get("name") == "follow_waypoints"
        )
        follow_id = follow_candidate["request_id"]
        wait_until(
            lambda: any(
                result.get("command_id") == follow_id and result.get("success") is True
                for result in node.results
            ),
            15.0,
            "follow_waypoints command did not finish",
        )
        follow_elapsed = time.monotonic() - follow_started_at
        if follow_elapsed > 3.0:
            raise RuntimeError(
                "follow_waypoints result waited for local duration instead of Nav2 result: "
                f"{follow_elapsed:.2f}s"
            )
        follow = node.follow_goals[-1]
        # RobotCommand 的 1 表示“总共走一遍”；Nav2 的 0 才表示不做额外重复。
        if follow.number_of_loops != 0 or len(follow.poses) != 3:
            raise RuntimeError(f"unexpected follow goal: {follow}")
        positions = [(pose.pose.position.x, pose.pose.position.y) for pose in follow.poses]
        expected = [(1.2, 0.0), (1.2, 1.0), (0.0, 0.0)]
        for actual, want in zip(positions, expected):
            if abs(actual[0] - want[0]) > 1e-6 or abs(actual[1] - want[1]) > 1e-6:
                raise RuntimeError(f"unexpected waypoint positions: {positions}")

        node.text_pub.publish(String(data="回到起点"))
        wait_until(
            lambda: node.home_goal_running.is_set(),
            10.0,
            "home NavigateToPose goal did not enter running state",
        )
        home_candidate = next(
            candidate
            for candidate in reversed(node.candidates)
            if candidate.get("name") == "navigate_to"
            and candidate.get("arguments", {}).get("target") == "home"
        )
        node.text_pub.publish(String(data="取消导航"))
        wait_until(
            lambda: node.home_goal_canceled.is_set()
            and any(candidate.get("name") == "cancel_navigation" for candidate in node.candidates),
            12.0,
            "voice cancel did not reach the active Nav2 goal",
        )
        cancel_candidate = next(
            candidate
            for candidate in reversed(node.candidates)
            if candidate.get("name") == "cancel_navigation"
        )
        wait_until(
            lambda: any(
                result.get("command_id") == cancel_candidate["request_id"]
                and result.get("success") is True
                for result in node.results
            ),
            10.0,
            "cancel_navigation command did not return success",
        )
        home_results = [
            result
            for result in node.results
            if result.get("command_id") == home_candidate["request_id"]
        ]
        if not home_results or home_results[-1].get("success") is not False:
            raise RuntimeError(f"active navigation was not reported canceled: {home_results}")
        if node.navigate_cancel_requests < 1:
            raise RuntimeError("fake Nav2 server did not observe a cancel request")

        node.text_pub.publish(String(data="去封闭区"))
        wait_until(
            lambda: any(
                candidate.get("arguments", {}).get("target") == "unreachable_zone"
                for candidate in node.candidates
            ),
            10.0,
            "unreachable semantic target was not published",
        )
        unreachable_candidate = next(
            candidate for candidate in reversed(node.candidates)
            if candidate.get("arguments", {}).get("target") == "unreachable_zone"
        )
        wait_until(
            lambda: any(
                result.get("command_id") == unreachable_candidate["request_id"]
                for result in node.results
            ),
            10.0,
            "aborted Nav2 goal did not propagate a result",
        )
        unreachable_result = next(
            result for result in reversed(node.results)
            if result.get("command_id") == unreachable_candidate["request_id"]
        )
        if unreachable_result.get("success") is not False:
            raise RuntimeError(f"unreachable goal unexpectedly succeeded: {unreachable_result}")
        detail = str(unreachable_result.get("message") or "")
        if "aborted" not in detail or "error_code=42" not in detail or "planner_no_path" not in detail:
            raise RuntimeError(f"Nav2 failure detail was lost: {unreachable_result}")

        report = {
                    "navigate_pose": {
                        "frame_id": pose.header.frame_id,
                        "x": pose.pose.position.x,
                        "y": pose.pose.position.y,
                    },
                    "follow_waypoints": positions,
                    "result_latency_s": {
                        "navigate_to": round(navigate_elapsed, 3),
                        "follow_waypoints": round(follow_elapsed, 3),
                    },
                    "result_count": len(node.results),
                    "navigation_cancel": {
                        "home_command_id": home_candidate["request_id"],
                        "cancel_command_id": cancel_candidate["request_id"],
                        "nav2_cancel_requests": node.navigate_cancel_requests,
                        "home_goal_canceled": node.home_goal_canceled.is_set(),
                    },
                    "unreachable_goal": {
                        "command_id": unreachable_candidate["request_id"],
                        "result": unreachable_result,
                    },
                    "status": "PASS",
                }
        output_path = Path(__file__).resolve().parents[3] / "logs" / "nav2_bridge_report.json"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print(f"Evidence: {output_path}")
    finally:
        node.navigate_server.destroy()
        node.follow_server.destroy()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
