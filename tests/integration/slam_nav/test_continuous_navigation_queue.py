#!/usr/bin/env python3
"""Verify continuous voice sessions queue navigation and patrol commands.

本测试把“连续语音队列”和“语音目标点导航/多目标点巡航”合在一起验收：

1. 一次唤醒后，说一句包含两个目标点的 ASR final；
2. 在动作执行过程中继续说多目标点巡航；
3. 验证三个导航类动作都进入队列，并且 result 与 request_id 顺序对应。

它不启动真实麦克风和完整 Nav2，而是使用 mock executor 做稳定回归；真实
Nav2/TurtleBot3 仍由 `nav2-turtlebot3` 和 `continuous-nav2-*` 做重型/人工验收。
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
from tools.acceptance.typed_action_probe_utils import candidate_dict, result_dict


class ContinuousNavigationProbe(Node):
    def __init__(self):
        super().__init__("continuous_navigation_queue_probe")
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
    node = ContinuousNavigationProbe()
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
            "continuous navigation pipeline was not discovered",
        )
        time.sleep(0.5)
        node.text_pub.publish(String(data="小智"))
        time.sleep(0.2)

        # 一条 ASR final 内包含两个目标点导航，验证 NLU 能拆出多个 navigate_to。
        node.text_pub.publish(String(data="去门口，然后前往书桌"))
        time.sleep(0.2)
        # 在前序动作可能仍 busy 时继续说巡航命令，验证它进入队列等待执行。
        node.text_pub.publish(String(data="依次去门口、书桌、起点"))

        wait_until(
            lambda: (
                len(node.candidates) >= 3
                and len(node.results) >= 3
                and sum(
                    1
                    for event in node.execution_events
                    if event.get("event") == "started"
                )
                >= 3
                and sum(
                    1
                    for event in node.execution_events
                    if event.get("event") == "finished"
                )
                >= 3
            ),
            45.0,
            "continuous navigation queue did not finish",
        )

        selected = node.candidates[:3]
        names = [candidate.get("name") for candidate in selected]
        if names != ["navigate_to", "navigate_to", "follow_waypoints"]:
            raise RuntimeError(f"unexpected navigation candidate order: {names}")

        targets = [candidate.get("arguments", {}).get("target") for candidate in selected[:2]]
        patrol_args = selected[2].get("arguments", {})
        if targets != ["door", "desk"]:
            raise RuntimeError(f"unexpected navigation targets: {selected}")
        if patrol_args.get("waypoints") != ["door", "desk", "home"]:
            raise RuntimeError(f"unexpected patrol waypoints: {selected[2]}")

        enqueue_events = [
            event for event in node.queue_events if event.get("event") == "enqueue"
        ]
        if len(enqueue_events) < 3:
            raise RuntimeError(f"missing queue enqueue events: {node.queue_events}")
        first_batch_id = enqueue_events[0].get("batch_id")
        first_batch = [
            event for event in enqueue_events if event.get("batch_id") == first_batch_id
        ]
        if [event.get("batch_index") for event in first_batch[:2]] != [1, 2]:
            raise RuntimeError(f"multi-target batch metadata missing: {node.queue_events}")
        if (
            enqueue_events[2].get("source_text") != "依次去门口、书桌、起点"
            or enqueue_events[2].get("nlu_intent") != "follow_waypoints"
        ):
            raise RuntimeError(f"patrol command was not queued as a later item: {node.queue_events}")

        nlu_events = [
            event
            for event in node.recognition_feedback
            if event.get("status") == "nlu_parsed"
        ]
        if len(nlu_events) < 2:
            raise RuntimeError(f"NLU feedback missing for nav/patrol: {node.recognition_feedback}")

        command_ids = [candidate.get("request_id") for candidate in selected]
        result_ids = [result.get("command_id") for result in node.results[:3]]
        if command_ids != result_ids:
            raise RuntimeError(
                f"results were not correlated: candidates={command_ids}, results={result_ids}"
            )
        if any(result.get("success") is not True for result in node.results[:3]):
            raise RuntimeError(f"navigation queue action failed: {node.results[:3]}")

        execution_events = [event.get("event") for event in node.execution_events]
        if execution_events.count("started") < 3 or execution_events.count("finished") < 3:
            raise RuntimeError(f"execution events missing: {node.execution_events}")

        # 再启动一个 3 秒导航，并在执行中说“取消导航”。取消必须绕过 FIFO，
        # 当前 navigate_to 应返回失败/取消，cancel_navigation 自身应成功。
        candidate_count = len(node.candidates)
        node.text_pub.publish(String(data="去门口"))
        wait_until(
            lambda: len(node.candidates) > candidate_count
            and node.candidates[-1].get("name") == "navigate_to",
            10.0,
            "active navigation was not started for cancel scenario",
        )
        active_navigation = node.candidates[-1]
        time.sleep(0.2)
        node.text_pub.publish(String(data="取消导航"))
        wait_until(
            lambda: any(
                candidate.get("name") == "cancel_navigation"
                for candidate in node.candidates[candidate_count + 1 :]
            ),
            5.0,
            "cancel_navigation was queued behind the active goal",
        )
        cancel_candidate = next(
            candidate
            for candidate in reversed(node.candidates)
            if candidate.get("name") == "cancel_navigation"
        )
        wait_until(
            lambda: any(
                result.get("command_id") == active_navigation["request_id"]
                and result.get("success") is False
                for result in node.results
            )
            and any(
                result.get("command_id") == cancel_candidate["request_id"]
                and result.get("success") is True
                for result in node.results
            ),
            10.0,
            "navigation cancel results were not correlated",
        )
        cancel_clear_events = [
            event
            for event in node.queue_events
            if event.get("event") == "clear" and event.get("text") == "取消导航"
        ]
        if not cancel_clear_events or (
            cancel_clear_events[-1].get("reason") != "priority_navigation_cancel"
        ):
            raise RuntimeError(
                f"navigation cancel priority was not observable: {cancel_clear_events}"
            )

        print(
            json.dumps(
                {
                    "candidate_sequence": names,
                    "targets": targets,
                    "waypoints": patrol_args.get("waypoints"),
                    "command_ids": command_ids,
                    "result_ids": result_ids,
                    "nlu_events": len(nlu_events),
                    "navigation_cancel": {
                        "active_command_id": active_navigation["request_id"],
                        "cancel_command_id": cancel_candidate["request_id"],
                        "priority_latency_budget_s": 5.0,
                    },
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
