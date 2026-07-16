#!/usr/bin/env python3
import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from embodied_agent_interfaces.msg import AgentTurnMetrics, RobotActionAck
from embodied_agent_core.metrics_transport import agent_turn_metrics_message_to_dict
from embodied_agent_core.ros_qos import audio_qos, diagnostics_qos, event_qos
from embodied_agent_core.runtime_status_transport import action_ack_to_dict
from rclpy.node import Node
from std_msgs.msg import Empty, String, UInt8MultiArray

from embodied_offline_agent.providers.sherpa_tts import SherpaVitsTts


class VoiceProbe(Node):
    def __init__(self):
        super().__init__("offline_voice_probe")
        self.audio_pub = self.create_publisher(
            UInt8MultiArray, "/audio/clean_pcm", audio_qos(depth=20)
        )
        self.silence_pub = self.create_publisher(
            Empty, "/audio/silence_timeout", event_qos(depth=10)
        )
        self.final_text = None
        self.ack = None
        self.metrics = None
        self.done = threading.Event()
        self.create_subscription(
            String, "/agent/asr_final", self._on_asr, event_qos(depth=10)
        )
        self.create_subscription(
            RobotActionAck, "/robot/action_ack", self._on_ack, event_qos(depth=10)
        )
        self.create_subscription(
            AgentTurnMetrics,
            "/agent/metrics",
            self._on_metrics,
            diagnostics_qos(depth=10),
        )

    def _on_asr(self, message):
        self.final_text = message.data

    def _on_ack(self, message):
        payload = action_ack_to_dict(message)
        if payload.get("action") == "move":
            self.ack = payload
            if self.metrics is not None:
                self.done.set()

    def _on_metrics(self, message):
        self.metrics = agent_turn_metrics_message_to_dict(message)
        if self.ack is not None:
            self.done.set()


def resample(pcm: bytes, source_rate: int, target_rate: int) -> bytes:
    source = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
    size = round(source.size * target_rate / source_rate)
    values = np.interp(
        np.linspace(0, source.size - 1, size), np.arange(source.size), source
    )
    return np.clip(values, -32768, 32767).astype("<i2").tobytes()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    model_dir = "/home/ubuntu/embodied_agent_ws/models/vits-melo-tts-zh_en"
    tts = SherpaVitsTts(model_dir, 2, 0, 1.0)
    command_pcm = resample(tts.synthesize("小智向前走一秒"), tts.sample_rate, 16000)
    # /audio/clean_pcm 使用 Best Effort QoS；按接近实时速度发布并保留前后静音，
    # 避免验收探针自己挤爆订阅队列，导致“向前走一秒”的尾部音频被丢弃。
    pcm = bytes(8000) + command_pcm + bytes(25600)

    rclpy.init()
    node = VoiceProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        deadline = time.monotonic() + 10.0
        while node.count_subscribers("/audio/clean_pcm") == 0:
            if time.monotonic() >= deadline:
                raise TimeoutError("offline ASR audio subscriber was not discovered")
            time.sleep(0.1)
        for offset in range(0, len(pcm), 3200):
            node.audio_pub.publish(UInt8MultiArray(data=list(pcm[offset : offset + 3200])))
            time.sleep(0.09)
        node.silence_pub.publish(Empty())
        if not node.done.wait(35.0):
            raise TimeoutError(
                f"voice E2E incomplete: asr={node.final_text!r}, ack={node.ack}, metrics={node.metrics}"
            )
        if not node.final_text or not any(
            word in node.final_text for word in ("向前", "前进")
        ):
            raise RuntimeError(f"unexpected ASR transcript: {node.final_text!r}")
        report = {
            "schema_version": 1,
            "scenario": "offline_voice_e2e_benchmark",
            "measurement_scope": "speech_endpoint_to_first_tts_pcm_chunk",
            "audio_delivery": "near_realtime_best_effort_ros_topic",
            "targets": {"asr_finalize_ms": 600.0, "end_to_first_audio_ms": 3500.0},
            "ok": bool(node.metrics and node.metrics.get("e2e_target_met")),
            "asr_text": node.final_text,
            "action_ack": node.ack,
            "metrics": node.metrics,
        }
        rendered = json.dumps(report, ensure_ascii=False, indent=2)
        print(rendered)
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered + "\n", encoding="utf-8")
        if not report["ok"]:
            raise RuntimeError(
                "offline voice pipeline completed but missed the 3.5s first-audio target"
            )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
