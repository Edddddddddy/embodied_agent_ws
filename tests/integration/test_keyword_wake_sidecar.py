#!/usr/bin/env python3
"""Verify keyword_wake sidecar bridges keyword text to wake_event_input."""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import KwsEvent, WakeEvent
from embodied_online_agent.ros_event_transport import wake_event_message_to_dict
from embodied_online_agent.runtime_status_transport import kws_event_to_dict
from rclpy.node import Node
from std_msgs.msg import String


class KeywordWakeProbe(Node):
    def __init__(self):
        super().__init__("keyword_wake_probe")
        self.text_pub = self.create_publisher(String, "/agent/kws_text_input", 10)
        self.wake_events = []
        self.kws_events = []
        self.create_subscription(WakeEvent, "/agent/wake_event_input", self._on_wake, 10)
        self.create_subscription(KwsEvent, "/agent/kws_event", self._on_kws, 10)

    def _on_wake(self, message):
        self.wake_events.append(wake_event_message_to_dict(message))

    def _on_kws(self, message):
        self.kws_events.append(kws_event_to_dict(message))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = KeywordWakeProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.text_pub.get_subscription_count() > 0,
            10.0,
            "keyword_wake did not subscribe to /agent/kws_text_input",
        )
        node.text_pub.publish(String(data="你好小志"))
        wait_until(
            lambda: node.wake_events and node.kws_events,
            5.0,
            "keyword_wake did not publish wake events",
        )
        wake = node.wake_events[0]
        if wake.get("kind") != "wake" or wake.get("provider") != "mock_kws":
            raise RuntimeError(f"unexpected wake payload: {wake}")
        if wake.get("transcript") != "小志":
            raise RuntimeError(f"unexpected detected keyword: {wake}")
        print(json.dumps({"wake": wake, "status": "PASS"}, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
