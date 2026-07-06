#!/usr/bin/env python3
"""Verify real Sherpa-ONNX offline voice reaches the typed simulation pipeline.

本测试覆盖的链路：

Sherpa-TTS 生成命令音频 -> /audio/clean_pcm -> Sherpa ZipFormer ASR ->
Offline Agent -> /agent/action_candidate -> C++ ActionGuard ->
/robot/action_command_typed -> ROS 2 ExecuteRobotCommand Action ->
simulation_control mock executor -> /cmd_vel + /robot/action_result。

它不启动重型 Gazebo GUI，但会经过项目真实的 typed Action 控制链路。
"""

from __future__ import annotations

import json
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray

from embodied_agent_interfaces.msg import RobotCommand
from embodied_offline_agent.providers.sherpa_tts import SherpaVitsTts


def resample(pcm: bytes, source_rate: int, target_rate: int) -> bytes:
    source = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
    size = round(source.size * target_rate / source_rate)
    values = np.interp(
        np.linspace(0, source.size - 1, size), np.arange(source.size), source
    )
    return np.clip(values, -32768, 32767).astype("<i2").tobytes()


def wait_until(predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


class OfflineSherpaTypedProbe(Node):
    def __init__(self):
        super().__init__("offline_sherpa_typed_probe")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.audio_pub = self.create_publisher(UInt8MultiArray, "/audio/clean_pcm", qos)
        self.silence_pub = self.create_publisher(Empty, "/audio/silence_timeout", 10)
        self.asr_text: str | None = None
        self.candidates: list[dict] = []
        self.typed_commands: list[RobotCommand] = []
        self.action_results: list[dict] = []
        self.metrics: dict | None = None
        self.velocities: list[tuple[float, float]] = []
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(String, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(
            RobotCommand, "/robot/action_command_typed", self._on_typed_command, 10
        )
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)
        self.create_subscription(String, "/offline_agent/metrics", self._on_metrics, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_asr(self, message: String) -> None:
        self.asr_text = message.data

    def _on_candidate(self, message: String) -> None:
        self.candidates.append(json.loads(message.data))

    def _on_typed_command(self, message: RobotCommand) -> None:
        self.typed_commands.append(message)

    def _on_result(self, message: String) -> None:
        self.action_results.append(json.loads(message.data))

    def _on_metrics(self, message: String) -> None:
        self.metrics = json.loads(message.data)

    def _on_velocity(self, message: Twist) -> None:
        self.velocities.append((message.linear.x, message.angular.z))

    def latest_candidate(self, name: str) -> dict | None:
        for candidate in reversed(self.candidates):
            if candidate.get("name") == name:
                return candidate
        return None

    def successful_result_for(self, command_id: str) -> dict | None:
        for result in self.action_results:
            if result.get("command_id") == command_id and result.get("success") is True:
                return result
        return None

    def has_forward_velocity(self) -> bool:
        return any(linear > 0.01 for linear, _ in self.velocities)


def main() -> None:
    tts = SherpaVitsTts(
        "/home/ubuntu/embodied_agent_ws/models/vits-melo-tts-zh_en", 2, 0, 1.0
    )
    pcm = resample(tts.synthesize("小智向前走一秒"), tts.sample_rate, 16000)
    pcm += bytes(16000)

    rclpy.init()
    node = OfflineSherpaTypedProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.audio_pub.get_subscription_count() > 0
                and node.silence_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
                and node.count_publishers("/cmd_vel") > 0
            ),
            25.0,
            "offline ASR, typed action, or simulation topics were not ready",
        )
        time.sleep(0.5)
        for offset in range(0, len(pcm), 3200):
            node.audio_pub.publish(UInt8MultiArray(data=list(pcm[offset : offset + 3200])))
            time.sleep(0.02)
        node.silence_pub.publish(Empty())

        wait_until(lambda: node.asr_text is not None, 20.0, "Sherpa ASR produced no final")
        if not any(word in node.asr_text for word in ("向前", "前进")):
            raise RuntimeError(f"unexpected ASR transcript: {node.asr_text!r}")

        wait_until(
            lambda: node.latest_candidate("move") is not None,
            70.0,
            f"Offline Agent produced no move candidate; asr={node.asr_text!r}",
        )
        candidate = node.latest_candidate("move")
        command_id = candidate["request_id"]

        wait_until(
            lambda: any(
                command.command_id == command_id
                and command.action_type == RobotCommand.MOVE
                for command in node.typed_commands
            ),
            10.0,
            "ActionGuard did not publish typed MOVE command",
        )
        wait_until(
            lambda: node.successful_result_for(command_id) is not None
            and node.has_forward_velocity()
            and node.metrics is not None,
            25.0,
            "typed action did not finish, cmd_vel did not move, or metrics missing",
        )
        result = node.successful_result_for(command_id)
        print(
            json.dumps(
                {
                    "asr_text": node.asr_text,
                    "action_candidate": candidate,
                    "typed_command": {
                        "command_id": command_id,
                        "action_type": "MOVE",
                    },
                    "action_result": result,
                    "forward_cmd_vel_observed": True,
                    "metrics": node.metrics,
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
