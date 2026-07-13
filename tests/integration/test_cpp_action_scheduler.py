#!/usr/bin/env python3
"""验证 C++ Action scheduler 的 FIFO、优先取消、结果关联和 diagnostics。"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from embodied_agent_interfaces.msg import (
    RobotCommand,
    RobotCommandFeedback,
    RobotCommandResult,
)
from rclpy.node import Node


class SchedulerProbe(Node):
    def __init__(self):
        super().__init__("cpp_action_scheduler_probe")
        self.command_pub = self.create_publisher(
            RobotCommand, "/robot/action_command_typed", 10
        )
        self.results: list[RobotCommandResult] = []
        self.feedback: list[RobotCommandFeedback] = []
        self.scheduler_diagnostics: list[dict[str, str | int]] = []
        self.create_subscription(
            RobotCommandResult, "/robot/action_result", self.results.append, 10
        )
        self.create_subscription(
            RobotCommandFeedback, "/robot/action_feedback", self.feedback.append, 10
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )

    def _on_diagnostics(self, message: DiagnosticArray):
        for status in message.status:
            if not status.name.endswith(": action_scheduler"):
                continue
            values = {item.key: item.value for item in status.values}
            # ROS 2 Python 在部分发行版中把 uint8 暴露成单字节 bytes；统一转成整数，
            # 避免验收报告出现 "b'\\x00'" 这种与业务无关的展示差异。
            level = status.level
            values["level"] = (
                int.from_bytes(level, "little")
                if isinstance(level, (bytes, bytearray))
                else int(level)
            )
            values["message"] = status.message
            self.scheduler_diagnostics.append(values)

    def publish_command(
        self,
        command_id: str,
        action_type: int,
        *,
        linear_x: float = 0.0,
        angular_z: float = 0.0,
        duration_s: float = 0.0,
        priority: bool = False,
    ):
        message = RobotCommand()
        message.command_id = command_id
        message.source = "cpp_scheduler_test"
        message.action_type = action_type
        message.linear_x = linear_x
        message.angular_z = angular_z
        message.duration_s = duration_s
        message.priority = priority
        self.command_pub.publish(message)


def wait_until(predicate, timeout: float, description: str):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise TimeoutError(description)


def result_by_id(node: SchedulerProbe, command_id: str):
    return next((item for item in node.results if item.command_id == command_id), None)


def main():
    rclpy.init()
    node = SchedulerProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.command_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
            ),
            10.0,
            "C++ scheduler topics not discovered",
        )

        # 三条命令紧密发布。旧 bridge 会把它们同时发成 goals 并互相抢占；
        # 新 scheduler 必须只保留一个 active goal，其余命令严格 FIFO。
        fifo_ids = ["fifo-move", "fifo-turn", "fifo-move-2"]
        node.publish_command(
            fifo_ids[0], RobotCommand.MOVE, linear_x=0.15, duration_s=0.35
        )
        node.publish_command(
            fifo_ids[1], RobotCommand.TURN, angular_z=0.5, duration_s=0.25
        )
        node.publish_command(
            fifo_ids[2], RobotCommand.MOVE, linear_x=-0.12, duration_s=0.2
        )
        wait_until(
            lambda: all(result_by_id(node, item) for item in fifo_ids),
            8.0,
            "FIFO commands did not finish",
        )
        fifo_results = [result_by_id(node, item) for item in fifo_ids]
        if [item.command_id for item in node.results[:3]] != fifo_ids:
            raise RuntimeError(
                f"scheduler did not preserve FIFO result order: "
                f"{[item.command_id for item in node.results]}"
            )
        if not all(item.success for item in fifo_results):
            raise RuntimeError(
                f"FIFO command failed: {[(item.command_id, item.message) for item in fifo_results]}"
            )

        # STOP 是控制面命令：清掉 pending turn，取消 active move，再独占执行 stop。
        node.publish_command(
            "priority-active", RobotCommand.MOVE, linear_x=0.18, duration_s=3.0
        )
        wait_until(
            lambda: any(item.command_id == "priority-active" for item in node.feedback),
            4.0,
            "active command feedback missing",
        )
        node.publish_command(
            "priority-pending", RobotCommand.TURN, angular_z=0.5, duration_s=0.5
        )
        time.sleep(0.1)
        node.publish_command("priority-stop", RobotCommand.STOP, priority=True)
        priority_ids = ["priority-active", "priority-pending", "priority-stop"]
        wait_until(
            lambda: all(result_by_id(node, item) for item in priority_ids),
            8.0,
            "priority cancellation results missing",
        )

        active_result = result_by_id(node, "priority-active")
        pending_result = result_by_id(node, "priority-pending")
        stop_result = result_by_id(node, "priority-stop")
        if active_result.status != RobotCommandResult.STATUS_CANCELED:
            raise RuntimeError(
                f"active command was not canceled: {active_result.status} {active_result.message}"
            )
        if (
            pending_result.status != RobotCommandResult.STATUS_CANCELED
            or pending_result.message != "queue_cleared_by_priority_command"
        ):
            raise RuntimeError(
                f"pending command was not cleared explicitly: "
                f"{pending_result.status} {pending_result.message}"
            )
        if not stop_result.success:
            raise RuntimeError(f"priority stop failed: {stop_result.message}")

        wait_until(
            lambda: any(
                int(item.get("accepted_count", "0")) >= 6
                and item.get("state") == "idle"
                for item in node.scheduler_diagnostics
            ),
            4.0,
            "scheduler diagnostics did not report final idle state",
        )
        diagnostics = node.scheduler_diagnostics[-1]
        print(
            json.dumps(
                {
                    "fifo_result_order": [item.command_id for item in node.results[:3]],
                    "priority": {
                        "active_status": active_result.status,
                        "pending_message": pending_result.message,
                        "stop_success": stop_result.success,
                    },
                    "diagnostics": diagnostics,
                    "status": "PASS",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
