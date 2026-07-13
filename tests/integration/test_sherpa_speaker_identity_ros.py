#!/usr/bin/env python3
"""Feed real PCM to the sherpa speaker sidecar and verify its ROS identity event."""

from __future__ import annotations

import json
import os
import threading
import time
import wave
from pathlib import Path

import rclpy
from embodied_agent_interfaces.msg import SpeakerIdentity
from embodied_agent_core.ros_qos import audio_qos, event_qos, state_qos
from embodied_agent_core.speaker_transport import identity_message_to_dict
from rclpy.node import Node
from std_msgs.msg import Empty, UInt8MultiArray


WORKSPACE = Path(__file__).resolve().parents[2]


class SpeakerIdentityProbe(Node):
    def __init__(self):
        super().__init__("sherpa_speaker_identity_probe")
        self.audio_pub = self.create_publisher(
            UInt8MultiArray, "/audio/clean_pcm", audio_qos(depth=20)
        )
        self.end_pub = self.create_publisher(
            Empty, "/audio/speech_ended", event_qos(depth=10)
        )
        self.identities = []
        self.create_subscription(
            SpeakerIdentity, "/agent/speaker_identity", self._on_identity, state_qos()
        )

    def _on_identity(self, message):
        self.identities.append(identity_message_to_dict(message))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def read_pcm(path: Path) -> bytes:
    with wave.open(str(path), "rb") as stream:
        if stream.getsampwidth() != 2 or stream.getnchannels() != 1:
            raise ValueError("speaker ROS smoke requires mono int16 WAV")
        return stream.readframes(stream.getnframes())


def main() -> int:
    wav_path = Path(os.environ["SPEAKER_TEST_WAV"])
    expected_speaker = os.environ.get("SPEAKER_TEST_ID", "runtime_probe_user")
    pcm = read_pcm(wav_path)

    rclpy.init()
    node = SpeakerIdentityProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.audio_pub.get_subscription_count() > 0
            and node.end_pub.get_subscription_count() > 0,
            30.0,
            "speaker identity sidecar did not subscribe to audio endpoint topics",
        )
        # 分帧发布，覆盖和真实 audio_frontend 相同的 best-effort PCM 数据路径。
        for offset in range(0, len(pcm), 3200):
            node.audio_pub.publish(UInt8MultiArray(data=list(pcm[offset : offset + 3200])))
            time.sleep(0.003)
        node.end_pub.publish(Empty())
        wait_until(
            lambda: any(item.get("reason") == "matched" for item in node.identities),
            20.0,
            "real sherpa identity event was not published",
        )
        identity = next(
            item for item in reversed(node.identities) if item.get("reason") == "matched"
        )
        if identity.get("speaker_id") != expected_speaker:
            raise RuntimeError(f"unexpected speaker identity: {identity}")
        if float(identity.get("confidence", 0.0)) < 0.6:
            raise RuntimeError(f"speaker confidence below threshold: {identity}")
        report = {
            "schema_version": 1,
            "status": "PASS",
            "provider": identity.get("model"),
            "speaker_id": identity.get("speaker_id"),
            "confidence": identity.get("confidence"),
            "second_best_score": identity.get("second_best_score"),
            "score_margin": identity.get("score_margin"),
            "reason": identity.get("reason"),
            "audio_bytes": len(pcm),
            "evidence_scope": "real_sherpa_ros_sidecar_self_match",
            "multi_speaker_accuracy_evaluated": False,
        }
        output_path = WORKSPACE / "logs" / "speaker_identity_ros_report.json"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print(f"Evidence: {output_path}")
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
