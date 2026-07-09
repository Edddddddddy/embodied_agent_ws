#!/usr/bin/env python3
"""Capture WSLg/PulseAudio microphone audio and publish the existing ROS audio topics.

PortAudio 在 WSL 里有时会落到不存在的 ALSA card 0，导致 C++ audio_frontend
只能读到近静音；但 WSLg 的 PulseAudio `RDPSource` 仍可通过 parecord 正常录音。
这个 bridge 复用项目已有 topic 协议，不改 ASR/LLM/TTS/ActionGuard 链路。
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray


class EndpointDetector:
    def __init__(self, end_silence_s: float, min_utterance_s: float, max_utterance_s: float):
        self.end_silence_s = end_silence_s
        self.min_utterance_s = min_utterance_s
        self.max_utterance_s = max_utterance_s
        self.in_utterance = False
        self.speech_s = 0.0
        self.silence_s = 0.0

    def update(self, speech: bool, frame_s: float) -> tuple[bool, bool]:
        started = False
        ended = False
        if speech and not self.in_utterance:
            self.in_utterance = True
            self.speech_s = 0.0
            self.silence_s = 0.0
            started = True
        if not self.in_utterance:
            return started, ended

        if speech:
            self.speech_s += frame_s
            self.silence_s = 0.0
        else:
            self.silence_s += frame_s

        if self.speech_s >= self.max_utterance_s:
            ended = True
        elif self.speech_s >= self.min_utterance_s and self.silence_s >= self.end_silence_s:
            ended = True
        if ended:
            self.in_utterance = False
            self.speech_s = 0.0
            self.silence_s = 0.0
        return started, ended


class PulseAudioCaptureBridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("pulse_audio_capture_bridge")
        self.args = args
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.clean_pub = self.create_publisher(UInt8MultiArray, "/audio/clean_pcm", qos)
        self.metrics_pub = self.create_publisher(String, "/audio/frontend_metrics", 10)
        self.started_pub = self.create_publisher(Empty, "/audio/speech_started", 10)
        self.ended_pub = self.create_publisher(Empty, "/audio/speech_ended", 10)
        self.silence_pub = self.create_publisher(Empty, "/audio/silence_timeout", 10)
        self.endpoint_events_enabled = bool(args.endpoint_events_enabled)
        self.endpoint = EndpointDetector(
            args.speech_end_silence_s,
            args.min_utterance_ms / 1000.0,
            args.max_utterance_s,
        )
        self.frame_samples = max(1, int(args.sample_rate * args.frame_ms / 1000))
        self.frame_bytes = self.frame_samples * 2
        self.last_metrics_at = 0.0
        self.dropped_input_frames = 0

    def run(self) -> None:
        if shutil.which("parecord") is None:
            raise RuntimeError("parecord not found; install pulseaudio-utils")
        command = [
            "parecord",
            f"--device={self.args.source}",
            "--format=s16le",
            f"--rate={self.args.sample_rate}",
            "--channels=1",
            "--raw",
        ]
        self.get_logger().info(
            f"PulseAudio capture bridge ready: source={self.args.source}, "
            f"rate={self.args.sample_rate}, frame={self.args.frame_ms}ms"
        )
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE) as process:
            assert process.stdout is not None
            while rclpy.ok():
                chunk = process.stdout.read(self.frame_bytes)
                if not chunk:
                    stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
                    raise RuntimeError(f"parecord stopped unexpectedly: {stderr.strip()}")
                if not rclpy.ok():
                    break
                try:
                    self.publish_frame(chunk)
                except Exception:
                    # Ctrl-C/launch shutdown 时 ROS context 可能先失效；这不是采集失败。
                    if not rclpy.ok():
                        break
                    raise
                rclpy.spin_once(self, timeout_sec=0.0)

    def publish_frame(self, chunk: bytes) -> None:
        message = UInt8MultiArray()
        message.data = list(chunk)
        self.clean_pub.publish(message)

        samples = struct.unpack("<" + "h" * (len(chunk) // 2), chunk[: len(chunk) // 2 * 2])
        if samples:
            peak = max(abs(value) for value in samples)
            rms = math.sqrt(sum(value * value for value in samples) / len(samples)) / 32768.0
        else:
            peak = 0
            rms = 0.0
        speech = rms >= self.args.vad_rms_threshold
        frame_s = len(samples) / float(self.args.sample_rate)
        started, ended = self.endpoint.update(speech, frame_s)
        if self.endpoint_events_enabled:
            if started:
                self.started_pub.publish(Empty())
            if ended:
                self.ended_pub.publish(Empty())
                self.silence_pub.publish(Empty())

        now = time.monotonic()
        if now - self.last_metrics_at >= self.args.metrics_period_s:
            self.last_metrics_at = now
            payload = {
                "rms": rms,
                "peak": peak,
                "speech": speech,
                "vad_provider": "energy",
                "endpoint_events_enabled": self.endpoint_events_enabled,
                "audio_enhancer_requested": "pulse_bridge",
                "audio_enhancer_active": "pulse_bridge",
                "aec_active": False,
                "noise_suppression_requested": False,
                "noise_suppression_active": False,
                "auto_gain_requested": False,
                "auto_gain_active": False,
                "dropped_input_frames": self.dropped_input_frames,
                "dropped_playback_chunks": 0,
            }
            self.metrics_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="@DEFAULT_SOURCE@")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--frame-ms", type=int, default=20)
    parser.add_argument("--vad-rms-threshold", type=float, default=0.018)
    parser.add_argument("--speech-end-silence-s", type=float, default=0.7)
    parser.add_argument("--min-utterance-ms", type=float, default=100.0)
    parser.add_argument("--max-utterance-s", type=float, default=12.0)
    parser.add_argument("--metrics-period-s", type=float, default=0.5)
    parser.add_argument(
        "--endpoint-events-enabled",
        choices=("true", "false"),
        default="true",
        help="publish speech_started/speech_ended events; set false when a VAD sidecar owns endpoints",
    )
    args = parser.parse_args()
    args.endpoint_events_enabled = args.endpoint_events_enabled == "true"

    rclpy.init()
    node = PulseAudioCaptureBridge(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
