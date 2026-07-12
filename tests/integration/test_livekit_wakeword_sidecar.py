#!/usr/bin/env python3
"""Verify keyword_wake livekit mode bridges audio frames to wake events."""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import KwsEvent, KwsScore, WakeEvent
from embodied_agent_core.ros_event_transport import wake_event_message_to_dict
from embodied_agent_core.runtime_status_transport import kws_event_to_dict, kws_score_to_dict
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


class LiveKitWakeWordProbe(Node):
    def __init__(self):
        super().__init__("livekit_wakeword_probe")
        self.audio_pub = self.create_publisher(UInt8MultiArray, "/audio/clean_pcm", 10)
        self.wake_events = []
        self.kws_events = []
        self.score_events = []
        self.create_subscription(WakeEvent, "/agent/wake_event_input", self._on_wake, 10)
        self.create_subscription(KwsEvent, "/agent/kws_event", self._on_kws, 10)
        self.create_subscription(KwsScore, "/agent/kws_score", self._on_score, 10)

    def _on_wake(self, message):
        self.wake_events.append(wake_event_message_to_dict(message))

    def _on_kws(self, message):
        self.kws_events.append(kws_event_to_dict(message))

    def _on_score(self, message):
        self.score_events.append(kws_score_to_dict(message))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def publish_until(node, predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        node.audio_pub.publish(UInt8MultiArray(data=[1, 0, 2, 0]))
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = LiveKitWakeWordProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.audio_pub.get_subscription_count() > 0,
            10.0,
            "keyword_wake did not subscribe to /audio/clean_pcm",
        )
        publish_until(
            node,
            lambda: node.wake_events and node.kws_events and node.score_events,
            5.0,
            "keyword_wake livekit mode did not publish wake events",
        )
        wake = node.wake_events[0]
        detected = node.kws_events[0]
        score = node.score_events[0]
        if wake.get("kind") != "wake" or wake.get("provider") != "livekit_test":
            raise RuntimeError(f"unexpected wake payload: {wake}")
        if wake.get("transcript") != "fake_livekit_wake":
            raise RuntimeError(f"unexpected detected keyword: {wake}")
        if detected.get("score", 0.0) < 0.5:
            raise RuntimeError(f"unexpected low score: {detected}")
        if score.get("top_keyword") != "fake_livekit_wake" or not score.get("above_threshold"):
            raise RuntimeError(f"unexpected score payload: {score}")
        print(
            json.dumps(
                {"wake": wake, "score": score, "status": "PASS"},
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
