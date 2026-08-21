#!/usr/bin/env python3
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray

from embodied_offline_agent.providers.sherpa_tts import SherpaVitsTts


class VoiceProbe(Node):
    def __init__(self):
        super().__init__("offline_voice_probe")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.audio_pub = self.create_publisher(UInt8MultiArray, "/audio/clean_pcm", qos)
        self.silence_pub = self.create_publisher(Empty, "/audio/silence_timeout", 10)
        self.final_text = None
        self.ack = None
        self.metrics = None
        self.done = threading.Event()
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(String, "/robot/action_ack", self._on_ack, 10)
        self.create_subscription(String, "/offline_agent/metrics", self._on_metrics, 10)

    def _on_asr(self, message):
        self.final_text = message.data

    def _on_ack(self, message):
        payload = json.loads(message.data)
        if payload.get("action") == "move":
            self.ack = payload
            if self.metrics is not None:
                self.done.set()

    def _on_metrics(self, message):
        self.metrics = json.loads(message.data)
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
    model_dir = "/home/ubuntu/embodied_agent_ws/models/vits-melo-tts-zh_en"
    tts = SherpaVitsTts(model_dir, 2, 0, 1.0)
    pcm = resample(tts.synthesize("小智向前走一秒"), tts.sample_rate, 16000)
    pcm += bytes(16000)  # 0.5 s silence.

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
            time.sleep(0.02)
        node.silence_pub.publish(Empty())
        if not node.done.wait(35.0):
            raise TimeoutError(
                f"voice E2E incomplete: asr={node.final_text!r}, ack={node.ack}, metrics={node.metrics}"
            )
        if not node.final_text or not any(
            word in node.final_text for word in ("向前", "前进")
        ):
            raise RuntimeError(f"unexpected ASR transcript: {node.final_text!r}")
        print(json.dumps({
            "asr_text": node.final_text,
            "action_ack": node.ack,
            "metrics": node.metrics,
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
