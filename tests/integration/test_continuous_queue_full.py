#!/usr/bin/env python3
"""Verify continuous voice reports queue_full through the real ROS topics.

这个探针覆盖“真人连续说太快”的真实链路：
1. 唤醒连续语音会话；
2. 发布一个长组合动作，让 Agent worker 忙于等待 Action result；
3. 队列容量设为 1，连续发送两条普通命令；
4. 验证第一条进入队列，第二条被 rejected/queue_full，并且 recognition_feedback
   给出 queue_rejected，方便 monitor 现场提示用户放慢语速或调大队列。
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FIRST_QUEUED_COMMAND = "左转九十度"
REJECTED_COMMAND = "绕圈"


class ContinuousQueueFullProbe(Node):
    def __init__(self):
        super().__init__("continuous_queue_full_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.queue_events = []
        self.feedback = []
        self.candidates = []
        self.create_subscription(String, "/agent/command_queue", self._on_queue, 10)
        self.create_subscription(
            String, "/agent/recognition_feedback", self._on_feedback, 10
        )
        self.create_subscription(String, "/agent/action_candidate", self._on_candidate, 10)

    def _on_queue(self, message):
        self.queue_events.append(json.loads(message.data))

    def _on_feedback(self, message):
        self.feedback.append(json.loads(message.data))

    def _on_candidate(self, message):
        self.candidates.append(json.loads(message.data))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = ContinuousQueueFullProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
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

        node.text_pub.publish(String(data=FIRST_QUEUED_COMMAND))
        wait_until(
            lambda: any(
                event.get("event") == "enqueue"
                and event.get("text") == FIRST_QUEUED_COMMAND
                and event.get("size") == 1
                for event in node.queue_events
            ),
            5.0,
            "first command was not queued into the bounded queue",
        )

        node.text_pub.publish(String(data=REJECTED_COMMAND))
        wait_until(
            lambda: any(
                event.get("event") == "rejected"
                and event.get("text") == REJECTED_COMMAND
                and event.get("reason") == "queue_full"
                for event in node.queue_events
            ),
            5.0,
            "second command did not publish queue_full rejection",
        )
        wait_until(
            lambda: any(
                item.get("status") == "queue_rejected"
                and item.get("reason") == "queue_full"
                and item.get("transcript") == REJECTED_COMMAND
                for item in node.feedback
            ),
            5.0,
            "queue_full recognition feedback was not published",
        )

        rejected_event = next(
            event
            for event in node.queue_events
            if event.get("event") == "rejected"
            and event.get("text") == REJECTED_COMMAND
        )
        rejected_feedback = next(
            item
            for item in node.feedback
            if item.get("status") == "queue_rejected"
            and item.get("transcript") == REJECTED_COMMAND
        )
        print(
            json.dumps(
                {
                    "queued_command": FIRST_QUEUED_COMMAND,
                    "rejected_event": rejected_event,
                    "rejected_feedback": rejected_feedback,
                    "candidate_count": len(node.candidates),
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
