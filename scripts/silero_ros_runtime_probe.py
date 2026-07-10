#!/usr/bin/env python3
"""Publish a real wav to the Silero sidecar and verify paired ROS endpoint events."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
import wave

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt8MultiArray


class SileroRuntimeProbe(Node):
    def __init__(self, frames: list[bytes], frame_ms: int):
        super().__init__("silero_runtime_probe")
        self._frames = frames
        self._index = 0
        self._events: list[dict] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._publisher = self.create_publisher(UInt8MultiArray, "/audio/clean_pcm", qos)
        self.create_subscription(String, "/audio/vad_event", self._on_event, 10)
        self._timer = self.create_timer(frame_ms / 1000.0, self._publish_next)
        self.finished_at: float | None = None

    def _publish_next(self) -> None:
        if self._index >= len(self._frames):
            if self.finished_at is None:
                self.finished_at = time.monotonic()
            return
        self._publisher.publish(UInt8MultiArray(data=list(self._frames[self._index])))
        self._index += 1

    def _on_event(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"name": "invalid_json", "raw": message.data}
        self._events.append(payload)

    @property
    def events(self) -> list[dict]:
        return list(self._events)


def load_frames(path: Path, frame_samples: int, trailing_silence_s: float) -> list[bytes]:
    with wave.open(str(path), "rb") as source:
        if (
            source.getnchannels() != 1
            or source.getsampwidth() != 2
            or source.getframerate() != 16000
        ):
            raise ValueError("probe wav must be mono PCM16 at 16000Hz")
        pcm = source.readframes(source.getnframes())

    frame_bytes = frame_samples * 2
    frames: list[bytes] = []
    for offset in range(0, len(pcm), frame_bytes):
        frame = pcm[offset:offset + frame_bytes]
        frames.append(frame.ljust(frame_bytes, b"\x00"))
    trailing_frames = max(1, int(trailing_silence_s * 16000 / frame_samples))
    frames.extend([b"\x00" * frame_bytes] * trailing_frames)
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wav", type=Path, required=True)
    parser.add_argument("--frame-ms", type=int, default=32)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--trailing-silence", type=float, default=1.0)
    args = parser.parse_args()

    frame_samples = int(16000 * args.frame_ms / 1000)
    frames = load_frames(args.wav, frame_samples, args.trailing_silence)
    rclpy.init()
    node = SileroRuntimeProbe(frames, args.frame_ms)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            names = [event.get("name") for event in node.events]
            if (
                node.finished_at is not None
                and "speech_started" in names
                and "speech_ended" in names
            ):
                report = {
                    "status": "PASS",
                    "published_frames": len(frames),
                    "event_names": names,
                    "providers": sorted(
                        {str(event.get("provider")) for event in node.events}
                    ),
                }
                print(json.dumps(report, ensure_ascii=False, indent=2))
                return
        raise RuntimeError(
            "Silero ROS sidecar did not publish paired speech_started/speech_ended events: "
            f"{node.events}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
