#!/usr/bin/env python3
"""Verify natural multi-target speech produces waypoint patrol commands.

这里覆盖更接近真人说法的目标点巡航表达：

1. “先去门口再去书桌最后回起点”应被理解成一条 follow_waypoints；
2. “巡逻门口、书桌、起点”也应被理解成一条 follow_waypoints；
3. 两条命令在连续会话里按队列顺序执行，并且 result 与 request_id 对齐。
"""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import (
    CommandExecutionEvent,
    CommandQueueEvent,
    NluParseEvent,
    RecognitionFeedback,
    RobotCommand,
    RobotCommandResult,
)
from embodied_agent_core.ros_event_transport import (
    execution_event_message_to_dict,
    nlu_parse_message_to_dict,
    queue_event_message_to_dict,
    recognition_feedback_message_to_dict,
)
from embodied_agent_core.ros_qos import event_qos
from rclpy.node import Node
from std_msgs.msg import String
from tests.integration.typed_action_test_utils import candidate_dict, result_dict


class NaturalNavigationProbe(Node):
    def __init__(self):
        super().__init__("continuous_navigation_natural_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.candidates = []
        self.queue_events = []
        self.execution_events = []
        self.recognition_feedback = []
        self.results = []
        self.create_subscription(RobotCommand, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(CommandQueueEvent, "/agent/command_queue", self._on_queue, event_qos())
        self.create_subscription(CommandExecutionEvent, "/agent/command_execution", self._on_execution, event_qos())
        self.create_subscription(
            RecognitionFeedback,
            "/agent/recognition_feedback",
            self._on_recognition,
            event_qos(),
        )
        self.create_subscription(
            NluParseEvent,
            "/agent/nlu_parse",
            self._on_nlu_parse,
            event_qos(),
        )
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)

    def _on_candidate(self, message):
        self.candidates.append(candidate_dict(message))

    def _on_queue(self, message):
        self.queue_events.append(queue_event_message_to_dict(message))

    def _on_execution(self, message):
        self.execution_events.append(execution_event_message_to_dict(message))

    def _on_recognition(self, message):
        self.recognition_feedback.append(recognition_feedback_message_to_dict(message))

    def _on_nlu_parse(self, message):
        self.recognition_feedback.append(nlu_parse_message_to_dict(message))

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
    node = NaturalNavigationProbe()
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
            "continuous natural navigation pipeline was not discovered",
        )
        time.sleep(0.5)
        node.text_pub.publish(String(data="小智"))
        time.sleep(0.2)

        node.text_pub.publish(String(data="先去门口再去书桌最后回起点"))
        time.sleep(0.2)
        node.text_pub.publish(String(data="巡逻门口、书桌、起点"))

        wait_until(
            lambda: (
                len(node.candidates) >= 2
                and len(node.results) >= 2
                and sum(
                    1
                    for event in node.execution_events
                    if event.get("event") == "started"
                )
                >= 2
                and sum(
                    1
                    for event in node.execution_events
                    if event.get("event") == "finished"
                )
                >= 2
            ),
            45.0,
            "continuous natural navigation queue did not finish",
        )

        selected = node.candidates[:2]
        names = [candidate.get("name") for candidate in selected]
        if names != ["follow_waypoints", "follow_waypoints"]:
            raise RuntimeError(f"unexpected natural navigation candidate order: {names}")
        waypoint_sets = [
            candidate.get("arguments", {}).get("waypoints") for candidate in selected
        ]
        if waypoint_sets != [["door", "desk", "home"], ["door", "desk", "home"]]:
            raise RuntimeError(f"unexpected natural waypoint sets: {selected}")

        enqueue_events = [
            event for event in node.queue_events if event.get("event") == "enqueue"
        ]
        source_texts = [event.get("source_text") for event in enqueue_events[:2]]
        if source_texts != ["先去门口再去书桌最后回起点", "巡逻门口、书桌、起点"]:
            raise RuntimeError(f"natural source text was not preserved: {node.queue_events}")
        if any(event.get("nlu_intent") != "follow_waypoints" for event in enqueue_events[:2]):
            raise RuntimeError(f"natural commands were not queued as waypoint patrol: {node.queue_events}")

        nlu_events = [
            event
            for event in node.recognition_feedback
            if event.get("status") == "nlu_parsed"
        ]
        if len(nlu_events) < 2:
            raise RuntimeError(f"NLU feedback missing for natural patrol: {node.recognition_feedback}")

        command_ids = [candidate.get("request_id") for candidate in selected]
        result_ids = [result.get("command_id") for result in node.results[:2]]
        if command_ids != result_ids:
            raise RuntimeError(
                f"results were not correlated: candidates={command_ids}, results={result_ids}"
            )
        if any(result.get("success") is not True for result in node.results[:2]):
            raise RuntimeError(f"natural navigation queue action failed: {node.results[:2]}")

        print(
            json.dumps(
                {
                    "candidate_sequence": names,
                    "waypoints": waypoint_sets,
                    "command_ids": command_ids,
                    "result_ids": result_ids,
                    "nlu_events": len(nlu_events),
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
