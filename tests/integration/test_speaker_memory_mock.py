import json
import os
import time
from pathlib import Path

import rclpy
from std_msgs.msg import String


def _spin_until(node, predicate, timeout_s=8.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main():
    memory_dir = Path(os.environ["USER_MEMORY_DIR"])
    rclpy.init()
    node = rclpy.create_node("speaker_memory_mock_test")
    responses = []
    actions = []
    enroll_requests = []
    node.create_subscription(String, "/agent/response_text", lambda msg: responses.append(msg.data), 10)
    node.create_subscription(String, "/agent/action_candidate", lambda msg: actions.append(msg.data), 10)
    node.create_subscription(
        String,
        "/agent/speaker_enroll_request",
        lambda msg: enroll_requests.append(json.loads(msg.data)),
        10,
    )
    identity_pub = node.create_publisher(String, "/agent/speaker_identity", 10)
    text_pub = node.create_publisher(String, "/agent/text_input", 10)

    assert _spin_until(
        node,
        lambda: identity_pub.get_subscription_count() > 0
        and text_pub.get_subscription_count() > 0,
        timeout_s=10.0,
    ), "agent subscriptions were not ready"

    identity_pub.publish(
        String(
            data=json.dumps(
                {
                    "speaker_id": "lcy",
                    "display_name": "小李",
                    "confidence": 0.93,
                    "enrolled": True,
                    "model": "mock-speaker",
                },
                ensure_ascii=False,
            )
        )
    )
    time.sleep(0.2)
    rclpy.spin_once(node, timeout_sec=0.2)

    text_pub.publish(String(data="记住我，我是小李"))
    assert _spin_until(node, lambda: any("小李" in item for item in responses))
    assert _spin_until(
        node,
        lambda: any(item.get("speaker_id") == "lcy" for item in enroll_requests),
    )

    text_pub.publish(String(data="我喜欢慢一点"))
    assert _spin_until(node, lambda: any("movement_speed=slow" in item for item in responses))

    text_pub.publish(String(data="我是谁"))
    assert _spin_until(node, lambda: any("当前用户" in item and "小李" in item for item in responses))

    text_pub.publish(String(data="向前走"))
    assert _spin_until(node, lambda: any('"name": "move"' in item for item in actions))
    profile_path = memory_dir / "lcy.json"
    assert _spin_until(
        node,
        lambda: profile_path.exists()
        and json.loads(profile_path.read_text(encoding="utf-8"))
        .get("command_counts", {})
        .get("move", 0)
        >= 1,
    )

    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    assert profile["display_name"] == "小李"
    assert profile["preferences"]["movement_speed"] == "slow"
    assert profile["command_counts"]["move"] >= 1

    node.destroy_node()
    rclpy.shutdown()
    print("PASS: speaker identity -> user memory -> prompt/action record")


if __name__ == "__main__":
    main()
