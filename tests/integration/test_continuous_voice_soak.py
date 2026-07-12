#!/usr/bin/env python3
"""Long-session continuous voice soak test through public ROS topics.

这个探针模拟“真人长时间连续说多条命令”的核心压力：
一次唤醒后，连续输入多条不同动作命令，Agent 忙时必须排队而不是丢弃；
所有动作最终都要按顺序进入 action_candidate，并收到机器人执行结果。
"""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import (
    CommandExecutionEvent,
    CommandQueueEvent,
    RobotCommand,
    RobotCommandResult,
    WakeEvent,
)
from embodied_agent_core.ros_event_transport import (
    execution_event_message_to_dict,
    queue_event_message_to_dict,
    wake_event_message_to_dict,
)
from embodied_agent_core.ros_qos import command_event_qos, latched_state_qos
from rclpy.node import Node
from std_msgs.msg import String
from typed_action_test_utils import candidate_dict, result_dict


COMMANDS = [
    "向前走一秒",
    "左转九十度",
    "后退一秒",
    "右转九十度",
    "绕圈",
    "挥手三次",
    "把灯设为蓝色",
]

EXPECTED_CANDIDATES = ["move", "turn", "move", "turn", "arc", "wave", "set_led"]


class ContinuousSoakProbe(Node):
    def __init__(self):
        super().__init__("continuous_voice_soak_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.session_states = []
        self.wake_events = []
        self.queue_events = []
        self.execution_events = []
        self.candidates = []
        self.results = []
        self.create_subscription(String, "/agent/session_state", self._on_session, latched_state_qos())
        self.create_subscription(WakeEvent, "/agent/wake_event", self._on_wake, command_event_qos())
        self.create_subscription(CommandQueueEvent, "/agent/command_queue", self._on_queue, command_event_qos())
        self.create_subscription(
            CommandExecutionEvent, "/agent/command_execution", self._on_execution, command_event_qos()
        )
        self.create_subscription(RobotCommand, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)

    def _on_session(self, message):
        self.session_states.append(message.data)

    def _on_wake(self, message):
        self.wake_events.append(wake_event_message_to_dict(message))

    def _on_queue(self, message):
        self.queue_events.append(queue_event_message_to_dict(message))

    def _on_execution(self, message):
        self.execution_events.append(execution_event_message_to_dict(message))

    def _on_candidate(self, message):
        self.candidates.append(candidate_dict(message))

    def _on_result(self, message):
        self.results.append(result_dict(message))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = ContinuousSoakProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
                and node.count_subscribers("/robot/action_command_typed") > 0
                and node.count_publishers("/robot/action_result") > 0
            ),
            15.0,
            "continuous voice pipeline was not discovered",
        )
        time.sleep(0.5)

        node.text_pub.publish(String(data="小智"))
        wait_until(
            lambda: "awake" in node.session_states,
            5.0,
            "wake word did not open a continuous session",
        )

        for command in COMMANDS:
            node.text_pub.publish(String(data=command))
            time.sleep(0.18)

        wait_until(
            lambda: len(node.candidates) >= len(EXPECTED_CANDIDATES),
            35.0,
            "not all continuous commands produced action candidates",
        )
        wait_until(
            lambda: len(node.results) >= len(EXPECTED_CANDIDATES),
            45.0,
            "not all continuous commands reached robot action results",
        )
        # robot result 会先解除 ActionSequence 等待，随后 worker 才发布 execution finished。
        # 因此 result 数量不是控制面生命周期完成的同步屏障，需要单独等待最终 finished。
        wait_until(
            lambda: sum(
                event.get("event") == "finished" for event in node.execution_events
            )
            >= len(COMMANDS),
            10.0,
            "not all command execution lifecycle events reached finished",
        )

        names = [
            candidate.get("name")
            for candidate in node.candidates[: len(EXPECTED_CANDIDATES)]
        ]
        if names != EXPECTED_CANDIDATES:
            raise RuntimeError(f"unexpected long-session command order: {names}")
        if any(
            result.get("success") is not True
            for result in node.results[: len(EXPECTED_CANDIDATES)]
        ):
            raise RuntimeError(f"continuous command result failed: {node.results}")

        enqueued = [
            event for event in node.queue_events if event.get("event") == "enqueue"
        ]
        started = [
            event for event in node.execution_events if event.get("event") == "started"
        ]
        finished = [
            event for event in node.execution_events if event.get("event") == "finished"
        ]
        if len(enqueued) < 3:
            raise RuntimeError(
                f"long-session commands were not queued while busy: {node.queue_events}"
            )
        if len(started) < len(COMMANDS) or len(finished) < len(COMMANDS):
            raise RuntimeError(
                f"execution lifecycle did not cover all commands: {node.execution_events}"
            )
        if "wake" not in [event.get("kind") for event in node.wake_events]:
            raise RuntimeError(f"wake event missing: {node.wake_events}")

        node.text_pub.publish(String(data="退出控制"))
        wait_until(
            lambda: "sleeping" in node.session_states,
            5.0,
            "session did not sleep after exit command",
        )

        print(
            json.dumps(
                {
                    "commands": COMMANDS,
                    "candidate_sequence": names,
                    "result_count": len(node.results),
                    "enqueue_count": len(enqueued),
                    "execution_started": len(started),
                    "execution_finished": len(finished),
                    "session_states_tail": node.session_states[-6:],
                    "status": "PASS",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
