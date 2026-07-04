#!/usr/bin/env python3
"""Verify speech endpoint commits drive continuous commands while the Agent is busy.

这个验收不走 /agent/text_input，而是模拟真实麦克风链路里的 endpoint：
/audio/speech_ended -> ASR commit -> /agent/asr_final -> 连续命令队列。
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


EXPECTED_ASR = ["小智", "向前走一秒", "左转九十度", "后退一秒", "退出控制"]
EXPECTED_CANDIDATES = ["move", "turn", "move"]


class EndpointAsrProbe(Node):
    def __init__(self):
        super().__init__("continuous_endpoint_asr_probe")
        self.speech_ended_pub = self.create_publisher(Empty, "/audio/speech_ended", 10)
        self.asr_finals = []
        self.session_states = []
        self.queue_events = []
        self.execution_events = []
        self.candidates = []
        self.results = []
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(String, "/agent/session_state", self._on_session, 10)
        self.create_subscription(String, "/agent/command_queue", self._on_queue, 10)
        self.create_subscription(
            String, "/agent/command_execution", self._on_execution, 10
        )
        self.create_subscription(String, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)

    def _on_asr(self, message):
        self.asr_finals.append(message.data)

    def _on_session(self, message):
        self.session_states.append(message.data)

    def _on_queue(self, message):
        self.queue_events.append(json.loads(message.data))

    def _on_execution(self, message):
        self.execution_events.append(json.loads(message.data))

    def _on_candidate(self, message):
        self.candidates.append(json.loads(message.data))

    def _on_result(self, message):
        self.results.append(json.loads(message.data))


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

        publish_endpoint(node, 3)
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

        publish_endpoint(node, 1)
        wait_until(
            lambda: "sleeping" in node.session_states,
            5.0,
            "endpoint ASR exit command did not close the session",
        )

        names = [candidate.get("name") for candidate in node.candidates[:3]]
        if names != EXPECTED_CANDIDATES:
            raise RuntimeError(f"unexpected endpoint command order: {names}")
        if node.asr_finals[: len(EXPECTED_ASR)] != EXPECTED_ASR:
            raise RuntimeError(f"unexpected ASR finals: {node.asr_finals}")
        if len([event for event in node.queue_events if event.get("event") == "enqueue"]) < 3:
            raise RuntimeError(f"endpoint commands were not queued: {node.queue_events}")
        if any(result.get("success") is not True for result in node.results[:3]):
            raise RuntimeError(f"endpoint command failed: {node.results}")

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
