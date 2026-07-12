#!/usr/bin/env python3
"""从公开 ROS topic 验证识别失败后 Agent 会反馈并继续监听。"""

import time

import rclpy
from embodied_agent_interfaces.msg import RecognitionFeedback
from embodied_online_agent.ros_event_transport import recognition_feedback_message_to_dict
from embodied_online_agent.ros_qos import command_event_qos, latched_state_qos
from rclpy.node import Node
from std_msgs.msg import String


class RetryProbe(Node):
    def __init__(self):
        super().__init__("recognition_retry_probe")
        self.feedback = None
        self.state = None
        self.input_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.create_subscription(
            RecognitionFeedback,
            "/agent/recognition_feedback",
            self._on_feedback,
            command_event_qos(),
        )
        self.create_subscription(
            String, "/agent/state", self._on_state, latched_state_qos()
        )

    def _on_feedback(self, message):
        payload = recognition_feedback_message_to_dict(message)
        if payload.get("reason") == "wake_word_not_detected":
            self.feedback = payload

    def _on_state(self, message):
        self.state = message.data


def wait_until(node, predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = RetryProbe()
    try:
        # text_input subscription 在 Lifecycle configure 前就存在；只看订阅数量可能把
        # 消息发给 inactive Agent。必须同时观察 latched listening 状态和反馈 publisher，
        # 才能证明业务入口与返回路径都已就绪。
        wait_until(
            node,
            lambda: node.input_pub.get_subscription_count() > 0
            and node.count_publishers("/agent/recognition_feedback") > 0
            and node.state == "listening",
            10.0,
            "online Agent recognition path did not become ready",
        )
        message = String()
        message.data = "完全没听清"
        node.input_pub.publish(message)
        wait_until(
            node,
            lambda: node.feedback is not None and node.state == "retry_listening",
            10.0,
            "online Agent did not publish retry feedback",
        )
        assert node.feedback["status"] == "retry"
        assert node.feedback["attempt"] >= 1
        print("PASS: failed recognition emitted feedback and Agent kept listening")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
