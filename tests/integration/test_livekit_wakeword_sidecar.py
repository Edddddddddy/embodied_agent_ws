#!/usr/bin/env python3
"""Verify keyword_wake livekit mode bridges audio frames to wake events."""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


class LiveKitWakeWordProbe(Node):
    def __init__(self):
        super().__init__("livekit_wakeword_probe")
        self.audio_pub = self.create_publisher(UInt8MultiArray, "/audio/clean_pcm", 10)
        self.wake_events = []
        self.kws_events = []
        self.create_subscription(String, "/agent/wake_event_input", self._on_wake, 10)
        self.create_subscription(String, "/agent/kws_event", self._on_kws, 10)

    def _on_wake(self, message):
        self.wake_events.append(json.loads(message.data))

    def _on_kws(self, message):
        self.kws_events.append(json.loads(message.data))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
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
        node.audio_pub.publish(UInt8MultiArray(data=[1, 0, 2, 0]))
        wait_until(
            lambda: node.wake_events and node.kws_events,
            5.0,
            "keyword_wake livekit mode did not publish wake events",
        )
        wake = node.wake_events[0]
        if wake.get("kind") != "wake" or wake.get("provider") != "livekit_test":
            raise RuntimeError(f"unexpected wake payload: {wake}")
        if wake.get("transcript") != "fake_livekit_wake":
            raise RuntimeError(f"unexpected detected keyword: {wake}")
        if wake.get("score", 0.0) < 0.5:
            raise RuntimeError(f"unexpected low score: {wake}")
        print(json.dumps({"wake": wake, "status": "PASS"}, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
