#!/usr/bin/env python3
"""Verify stale continuous-control commands expire before execution.

这个探针覆盖真实 ROS 链路，而不是只测 Python 队列：
1. 唤醒连续语音会话；
2. 发送一个长组合动作，让 Agent worker 忙于等待 Action result；
3. 再发送一条普通命令，使它在队列里超过 continuous_command_max_age_s；
4. 验证 /agent/command_queue 发布 expired，且过期命令没有进入执行生命周期。
"""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import CommandExecutionEvent, CommandQueueEvent, RobotCommand, RobotCommandResult
from embodied_agent_core.ros_event_transport import execution_event_message_to_dict, queue_event_message_to_dict
from embodied_agent_core.ros_qos import event_qos
from rclpy.node import Node
from std_msgs.msg import String
from tools.acceptance.typed_action_probe_utils import candidate_dict, result_dict


STALE_COMMAND = "手动模式"


class ContinuousTtlProbe(Node):
    def __init__(self):
        super().__init__("continuous_command_ttl_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.queue_events = []
        self.execution_events = []
        self.candidates = []
        self.results = []
        self.create_subscription(CommandQueueEvent, "/agent/command_queue", self._on_queue, event_qos())
        self.create_subscription(
            CommandExecutionEvent, "/agent/command_execution", self._on_execution, event_qos()
        )
        self.create_subscription(RobotCommand, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)

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
    node = ContinuousTtlProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
                and node.count_subscribers("/robot/action_command_typed") > 0
            ),
            15.0,
            "continuous voice pipeline was not discovered",
        )
        time.sleep(0.5)

        node.text_pub.publish(String(data="小智"))
        time.sleep(0.2)
        node.text_pub.publish(String(data="演示一下"))
        wait_until(
            lambda: any(candidate.get("name") == "set_led" for candidate in node.candidates),
            5.0,
            "demo sequence did not start",
        )

        stale_candidate_start = len(node.candidates)
        node.text_pub.publish(String(data=STALE_COMMAND))
        wait_until(
            lambda: any(
                event.get("event") == "enqueue" and event.get("text") == STALE_COMMAND
                for event in node.queue_events
            ),
            5.0,
            "stale candidate command was not enqueued",
        )
        wait_until(
            lambda: any(
                event.get("event") == "expired" and event.get("text") == STALE_COMMAND
                for event in node.queue_events
            ),
            30.0,
            "queued command did not expire after the busy sequence",
        )

        time.sleep(0.5)
        stale_started = [
            event
            for event in node.execution_events
            if event.get("event") == "started" and event.get("text") == STALE_COMMAND
        ]
        if stale_started:
            raise RuntimeError(f"expired command was executed: {stale_started}")
        stale_candidates = [
            candidate
            for candidate in node.candidates[stale_candidate_start:]
            if candidate.get("name") == "set_mode"
        ]
        if stale_candidates:
            raise RuntimeError(f"expired command published action candidates: {stale_candidates}")

        expired_event = next(
            event
            for event in node.queue_events
            if event.get("event") == "expired" and event.get("text") == STALE_COMMAND
        )
        print(
            json.dumps(
                {
                    "expired_event": expired_event,
                    "candidate_count_after_stale_enqueue": len(node.candidates)
                    - stale_candidate_start,
                    "execution_texts": [
                        event.get("text")
                        for event in node.execution_events
                        if event.get("event") == "started"
                    ],
                    "result_count": len(node.results),
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
