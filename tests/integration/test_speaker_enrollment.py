import os
import struct
import time
from pathlib import Path

import rclpy
from embodied_agent_interfaces.msg import SpeakerEnrollRequest, SpeakerEnrollStatus
from embodied_online_agent.speaker_transport import enroll_status_to_dict
from std_msgs.msg import Empty, UInt8MultiArray


def _spin_until(node, predicate, timeout_s=8.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main():
    enroll_dir = Path(os.environ["SPEAKER_ENROLL_DIR"])
    rclpy.init()
    node = rclpy.create_node("speaker_enrollment_test")
    statuses = []
    node.create_subscription(
        SpeakerEnrollStatus,
        "/agent/speaker_enroll_status",
        lambda msg: statuses.append(enroll_status_to_dict(msg)),
        10,
    )
    request_pub = node.create_publisher(
        SpeakerEnrollRequest, "/agent/speaker_enroll_request", 10
    )
    audio_pub = node.create_publisher(UInt8MultiArray, "/audio/clean_pcm", 10)
    ended_pub = node.create_publisher(Empty, "/audio/speech_ended", 10)

    assert _spin_until(
        node,
        lambda: request_pub.get_subscription_count() > 0
        and audio_pub.get_subscription_count() > 0
        and ended_pub.get_subscription_count() > 0,
        timeout_s=10.0,
    ), "speaker identity node subscriptions were not ready"

    request_pub.publish(
        SpeakerEnrollRequest(
            speaker_id="lcy", display_name="小李", samples_required=3
        )
    )
    assert _spin_until(node, lambda: any(item.get("status") == "started" for item in statuses))

    pcm = struct.pack("<" + "h" * 1600, *([1200] * 1600))
    for _ in range(3):
        audio_pub.publish(UInt8MultiArray(data=list(pcm)))
        ended_pub.publish(Empty())
        time.sleep(0.15)

    assert _spin_until(
        node,
        lambda: any(item.get("status") == "completed" for item in statuses),
        timeout_s=10.0,
    )
    speaker_file = enroll_dir / "speakers.txt"
    assert speaker_file.exists()
    lines = [
        line.strip()
        for line in speaker_file.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert len(lines) >= 3
    assert all(line.startswith("lcy ") for line in lines)
    assert all(Path(line.split(maxsplit=1)[1]).exists() for line in lines)

    node.destroy_node()
    rclpy.shutdown()
    print("PASS: speaker enrollment request -> wav samples -> speakers.txt")


if __name__ == "__main__":
    main()
