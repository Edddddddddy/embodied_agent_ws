#!/usr/bin/env python3
"""Inject synthesized speech and verify physical TurtleBot3 motion in Gazebo."""

import json
import math
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String, UInt8MultiArray

from embodied_offline_agent.providers.sherpa_tts import SherpaVitsTts


class VoiceGazeboProbe(Node):
    def __init__(self):
        super().__init__("voice_gazebo_probe")
        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        self.audio_pub = self.create_publisher(
            UInt8MultiArray, "/audio/clean_pcm", qos
        )
        self.silence_pub = self.create_publisher(
            Empty, "/audio/silence_timeout", 10
        )
        self.position = None
        self.scan_received = False
        self.asr_text = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)

    def _on_odom(self, message):
        self.position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )

    def _on_scan(self, _message):
        self.scan_received = True

    def _on_asr(self, message):
        self.asr_text = message.data


def resample(pcm, source_rate, target_rate):
    source = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
    size = round(source.size * target_rate / source_rate)
    values = np.interp(
        np.linspace(0, source.size - 1, size), np.arange(source.size), source
    )
    return np.clip(values, -32768, 32767).astype("<i2").tobytes()


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(description)


def main():
    tts = SherpaVitsTts(
        "/home/ubuntu/embodied_agent_ws/models/vits-melo-tts-zh_en", 2, 0, 1.0
    )
    pcm = resample(tts.synthesize("小智向前走一秒"), tts.sample_rate, 16000)
    pcm += bytes(16000)

    rclpy.init()
    node = VoiceGazeboProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.scan_received and node.position is not None
            and node.audio_pub.get_subscription_count() > 0
            and node.silence_pub.get_subscription_count() > 0,
            30.0,
            "voice Agent or Gazebo topics were not ready",
        )
        time.sleep(1.0)
        start = node.position
        for offset in range(0, len(pcm), 3200):
            chunk = pcm[offset:offset + 3200]
            node.audio_pub.publish(UInt8MultiArray(data=list(chunk)))
            time.sleep(0.02)
        node.silence_pub.publish(Empty())
        wait_until(
            lambda: node.asr_text is not None,
            15.0,
            "ZipFormer produced no final transcript",
        )
        wait_until(
            lambda: node.position is not None
            and math.hypot(node.position[0] - start[0], node.position[1] - start[1]) > 0.05,
            35.0,
            f"voice command did not move TurtleBot3; asr={node.asr_text!r}",
        )
        distance = math.hypot(
            node.position[0] - start[0], node.position[1] - start[1]
        )
        if distance > 0.5:
            raise RuntimeError(f"implausible odometry jump: {distance:.3f} m")
        print(json.dumps({
            "asr_text": node.asr_text,
            "distance_m": round(distance, 3),
            "voice_to_gazebo_motion": True,
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
