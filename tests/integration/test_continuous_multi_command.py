#!/usr/bin/env python3
"""Verify multi-command ASR finals are queued and executed in order."""

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
from embodied_online_agent.ros_event_transport import (
    execution_event_message_to_dict,
    nlu_parse_message_to_dict,
    queue_event_message_to_dict,
    recognition_feedback_message_to_dict,
)
from embodied_online_agent.ros_qos import command_event_qos
from rclpy.node import Node
from std_msgs.msg import String
from typed_action_test_utils import candidate_dict, result_dict


class MultiCommandProbe(Node):
    def __init__(self):
        super().__init__("continuous_multi_command_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.candidates = []
        self.queue_events = []
        self.execution_events = []
        self.recognition_feedback = []
        self.results = []
        self.create_subscription(RobotCommand, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(CommandQueueEvent, "/agent/command_queue", self._on_queue, command_event_qos())
        self.create_subscription(
            CommandExecutionEvent, "/agent/command_execution", self._on_execution, command_event_qos()
        )
        self.create_subscription(
            RecognitionFeedback,
            "/agent/recognition_feedback",
            self._on_recognition,
            command_event_qos(),
        )
        self.create_subscription(
            NluParseEvent,
            "/agent/nlu_parse",
            self._on_nlu_parse,
            command_event_qos(),
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
    node = MultiCommandProbe()
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
            "continuous multi-command pipeline was not discovered",
        )
        time.sleep(0.5)
        node.text_pub.publish(String(data="小智"))
        time.sleep(0.2)
        node.text_pub.publish(String(data="向右转，向前走一秒"))
        time.sleep(0.2)
        node.text_pub.publish(String(data="左转九十度"))

        wait_until(
            lambda: len(node.candidates) >= 3 and len(node.results) >= 3,
            35.0,
            "multi-command sequence did not finish",
        )
        names = [candidate.get("name") for candidate in node.candidates[:3]]
        if names != ["turn", "move", "turn"]:
            raise RuntimeError(f"unexpected candidate order: {names}")

        enqueue_events = [
            event for event in node.queue_events if event.get("event") == "enqueue"
        ]
        first_batch_id = enqueue_events[0].get("batch_id")
        first_batch = [
            event for event in enqueue_events if event.get("batch_id") == first_batch_id
        ]
        if [event.get("batch_index") for event in first_batch[:2]] != [1, 2]:
            raise RuntimeError(f"multi-command batch metadata missing: {node.queue_events}")
        if first_batch[0].get("batch_id") != first_batch[1].get("batch_id"):
            raise RuntimeError(f"multi-command items did not share batch_id: {first_batch}")

        nlu_events = [
            event for event in node.recognition_feedback
            if event.get("status") == "nlu_parsed"
        ]
        if not nlu_events:
            raise RuntimeError(f"NLU feedback missing: {node.recognition_feedback}")
        parsed_commands = nlu_events[0].get("commands") or []
        if not parsed_commands or any("slots" not in item for item in parsed_commands):
            raise RuntimeError(f"NLU slot observability missing: {nlu_events[0]}")

        command_ids = [candidate.get("request_id") for candidate in node.candidates[:3]]
        result_ids = [result.get("command_id") for result in node.results[:3]]
        if command_ids != result_ids:
            raise RuntimeError(
                f"action results were not correlated: candidates={command_ids}, results={result_ids}"
            )
        if any(result.get("success") is not True for result in node.results[:3]):
            raise RuntimeError(f"multi-command action failed: {node.results}")

        print(
            json.dumps(
                {
                    "candidate_sequence": names,
                    "batch_indexes": [event.get("batch_index") for event in first_batch],
                    "command_ids": command_ids,
                    "result_ids": result_ids,
                    "nlu_events": len(nlu_events),
                    "nlu_slots": [item.get("slots") for item in parsed_commands],
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
