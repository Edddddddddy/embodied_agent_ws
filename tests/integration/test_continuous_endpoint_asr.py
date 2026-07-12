#!/usr/bin/env python3
"""Verify speech endpoint commits drive continuous commands while the Agent is busy.

这个验收不走 /agent/text_input，而是模拟真实麦克风链路里的 endpoint：
/audio/speech_ended -> ASR commit -> /agent/asr_final -> 连续命令队列。
"""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import (
    CommandExecutionEvent,
    CommandQueueEvent,
    RecognitionFeedback,
    RobotCommand,
    RobotCommandResult,
)
from embodied_online_agent.ros_event_transport import (
    execution_event_message_to_dict,
    queue_event_message_to_dict,
    recognition_feedback_message_to_dict,
)
from embodied_online_agent.ros_qos import command_event_qos, latched_state_qos
from rclpy.node import Node
from std_msgs.msg import Empty, String
from typed_action_test_utils import candidate_dict, result_dict


EXPECTED_ASR = [
    "小智", "向前走一秒", "左转", "前进", "后退一秒",
    "把灯设成蓝色", "把灯", "我九十", "退出",
]
EXPECTED_CANDIDATES = ["move", "turn", "move", "move", "set_led"]


class EndpointAsrProbe(Node):
    def __init__(self):
        super().__init__("continuous_endpoint_asr_probe")
        self.speech_ended_pub = self.create_publisher(Empty, "/audio/speech_ended", 10)
        self.asr_finals = []
        self.session_states = []
        self.queue_events = []
        self.execution_events = []
        self.recognition_feedback = []
        self.candidates = []
        self.results = []
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(String, "/agent/session_state", self._on_session, latched_state_qos())
        self.create_subscription(CommandQueueEvent, "/agent/command_queue", self._on_queue, command_event_qos())
        self.create_subscription(
            CommandExecutionEvent, "/agent/command_execution", self._on_execution, command_event_qos()
        )
        self.create_subscription(RobotCommand, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(
            RecognitionFeedback,
            "/agent/recognition_feedback",
            self._on_recognition_feedback,
            command_event_qos(),
        )
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)

    def _on_asr(self, message):
        self.asr_finals.append(message.data)

    def _on_session(self, message):
        self.session_states.append(message.data)

    def _on_queue(self, message):
        self.queue_events.append(queue_event_message_to_dict(message))

    def _on_execution(self, message):
        self.execution_events.append(execution_event_message_to_dict(message))

    def _on_candidate(self, message):
        self.candidates.append(candidate_dict(message))

    def _on_recognition_feedback(self, message):
        self.recognition_feedback.append(recognition_feedback_message_to_dict(message))

    def _on_result(self, message):
        self.results.append(result_dict(message))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def publish_endpoint(node: EndpointAsrProbe, count: int, delay_s: float = 0.12):
    for _ in range(count):
        node.speech_ended_pub.publish(Empty())
        time.sleep(delay_s)


def main():
    rclpy.init()
    node = EndpointAsrProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.speech_ended_pub.get_subscription_count() > 0
                and node.count_subscribers("/robot/action_command_typed") > 0
                and node.count_publishers("/robot/action_result") > 0
            ),
            15.0,
            "endpoint-driven continuous voice pipeline was not discovered",
        )
        time.sleep(0.5)

        publish_endpoint(node, 1)
        wait_until(lambda: "awake" in node.session_states, 5.0, "wake ASR final missing")

        publish_endpoint(node, 5)
        wait_until(
            lambda: len(node.candidates) >= len(EXPECTED_CANDIDATES),
            25.0,
            "endpoint ASR commands did not produce action candidates",
        )
        wait_until(
            lambda: len(node.results) >= len(EXPECTED_CANDIDATES),
            35.0,
            "endpoint ASR commands did not reach robot action results",
        )

        recovered = [
            item for item in node.recognition_feedback
            if item.get("status") == "asr_final_recovered"
        ]
        if not any(
            item.get("original_final") == "把灯"
            and item.get("recovered") == "把灯设成蓝色"
            for item in recovered
        ):
            raise RuntimeError(
                f"safe partial tail was not recovered: {node.recognition_feedback}"
            )

        # 无可信 partial 的缺颜色/方向样本必须提示重说，不能让 LLM 猜动作。
        publish_endpoint(node, 2)
        wait_until(
            lambda: {
                item.get("reason") for item in node.recognition_feedback
            }.issuperset({"missing_led_color", "missing_turn_direction"}),
            5.0,
            "incomplete command retry feedback missing",
        )
        candidate_count_before_exit = len(node.candidates)
        if candidate_count_before_exit != len(EXPECTED_CANDIDATES):
            raise RuntimeError(
                f"incomplete ASR command unexpectedly produced actions: {node.candidates}"
            )

        publish_endpoint(node, 1)
        wait_until(
            lambda: "sleeping" in node.session_states,
            5.0,
            "endpoint ASR exit command did not close the session",
        )
        # 退出会话会主动发布 stop 归零，因此只允许新增这一条安全动作。
        candidates_after_exit = node.candidates[candidate_count_before_exit:]
        if [item.get("name") for item in candidates_after_exit] != ["stop"]:
            raise RuntimeError(
                f"exit did not produce exactly one stop action: {candidates_after_exit}"
            )

        names = [
            candidate.get("name")
            for candidate in node.candidates[: len(EXPECTED_CANDIDATES)]
        ]
        if names != EXPECTED_CANDIDATES:
            raise RuntimeError(f"unexpected endpoint command order: {names}")
        if node.asr_finals[: len(EXPECTED_ASR)] != EXPECTED_ASR:
            raise RuntimeError(f"unexpected ASR finals: {node.asr_finals}")
        if len([event for event in node.queue_events if event.get("event") == "enqueue"]) < 3:
            raise RuntimeError(f"endpoint commands were not queued: {node.queue_events}")
        if any(
            result.get("success") is not True
            for result in node.results[: len(EXPECTED_CANDIDATES)]
        ):
            raise RuntimeError(f"endpoint command failed: {node.results}")
        completed = [
            item for item in node.recognition_feedback
            if item.get("reason") == "completed_missing_slot"
        ]
        completed_pairs = {
            (item.get("original"), item.get("completed")) for item in completed
        }
        expected_completed = {("左转", "左转九十度"), ("前进", "前进一秒")}
        if not expected_completed.issubset(completed_pairs):
            raise RuntimeError(
                f"short ASR finals were not completed: {node.recognition_feedback}"
            )
        retry_reasons = {
            item.get("reason") for item in node.recognition_feedback
            if item.get("status") == "retry"
        }
        if not {"missing_led_color", "missing_turn_direction"}.issubset(retry_reasons):
            raise RuntimeError(
                f"incomplete command retries were not published: {node.recognition_feedback}"
            )
        endpoint_feedback = [
            item for item in node.recognition_feedback
            if item.get("status") == "asr_endpoint"
        ]
        commit_feedback = [
            item for item in node.recognition_feedback
            if item.get("status") == "asr_commit"
        ]
        if len(endpoint_feedback) < len(EXPECTED_ASR) or len(commit_feedback) < len(EXPECTED_ASR):
            raise RuntimeError(
                f"ASR endpoint/commit feedback missing: {node.recognition_feedback}"
            )
        if any(item.get("delay_ms") != 100 for item in endpoint_feedback):
            raise RuntimeError(f"unexpected ASR commit delay feedback: {endpoint_feedback}")

        print(
            json.dumps(
                {
                    "asr_finals": node.asr_finals[: len(EXPECTED_ASR)],
                    "candidate_sequence": names,
                    "result_count": len(node.results),
                    "queue_events": [
                        event.get("event") for event in node.queue_events
                    ],
                    "execution_events": [
                        event.get("event") for event in node.execution_events
                    ],
                    "completed_commands": sorted(completed_pairs),
                    "partial_recovery_count": len(recovered),
                    "incomplete_retry_reasons": sorted(retry_reasons),
                    "asr_endpoint_count": len(endpoint_feedback),
                    "asr_commit_count": len(commit_feedback),
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
