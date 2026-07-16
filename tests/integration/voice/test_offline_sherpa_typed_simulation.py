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
from std_msgs.msg import Empty, String, UInt8MultiArray

from embodied_agent_interfaces.msg import AgentTurnMetrics, RobotCommand, RobotCommandResult
from embodied_agent_core.metrics_transport import agent_turn_metrics_message_to_dict
from embodied_agent_core.ros_qos import (
    audio_qos,
    command_qos,
    diagnostics_qos,
    event_qos,
)
from tests.integration.typed_action_test_utils import candidate_dict, result_dict
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
        self.audio_pub = self.create_publisher(
            UInt8MultiArray, "/audio/clean_pcm", audio_qos(depth=20)
        )
        self.silence_pub = self.create_publisher(
            Empty, "/audio/silence_timeout", event_qos(depth=10)
        )
        self.asr_text: str | None = None
        self.candidates: list[dict] = []
        self.typed_commands: list[RobotCommand] = []
        self.action_results: list[dict] = []
        self.metrics: dict | None = None
        self.velocities: list[tuple[float, float]] = []
        self.create_subscription(
            String, "/agent/asr_final", self._on_asr, event_qos(depth=10)
        )
        self.create_subscription(
            RobotCommand,
            "/agent/action_candidate",
            self._on_candidate,
            command_qos(depth=10),
        )
        self.create_subscription(
            RobotCommand,
            "/robot/action_command_typed",
            self._on_typed_command,
            command_qos(depth=10),
        )
        self.create_subscription(
            RobotCommandResult,
            "/robot/action_result",
            self._on_result,
            event_qos(depth=10),
        )
        self.create_subscription(
            AgentTurnMetrics,
            "/agent/metrics",
            self._on_metrics,
            diagnostics_qos(depth=10),
        )
        self.create_subscription(
            Twist, "/cmd_vel", self._on_velocity, command_qos(depth=10)
        )

    def _on_asr(self, message: String) -> None:
        self.asr_text = message.data

    def _on_candidate(self, message: RobotCommand) -> None:
        self.candidates.append(candidate_dict(message))

    def _on_typed_command(self, message: RobotCommand) -> None:
        self.typed_commands.append(message)

    def _on_result(self, message: RobotCommandResult) -> None:
        self.action_results.append(result_dict(message))

    def _on_metrics(self, message: AgentTurnMetrics) -> None:
        self.metrics = agent_turn_metrics_message_to_dict(message)

    def _on_velocity(self, message: Twist) -> None:
        self.velocities.append((message.linear.x, message.angular.z))

    def candidates_by_name(self, name: str) -> list[dict]:
        return [candidate for candidate in self.candidates if candidate.get("name") == name]

    def move_command_ids(self) -> set[str]:
        return {
            command.command_id
            for command in self.typed_commands
            if command.action_type == RobotCommand.MOVE
        }

    def successful_move_result(self) -> dict | None:
        move_ids = self.move_command_ids()
        for result in self.action_results:
            if result.get("command_id") in move_ids and result.get("success") is True:
                return result
        return None

    def has_forward_velocity(self) -> bool:
        return any(linear > 0.01 for linear, _ in self.velocities)


def main() -> None:
    tts = SherpaVitsTts(
        "/home/ubuntu/embodied_agent_ws/models/vits-melo-tts-zh_en", 2, 0, 1.0
    )
    # 使用“三秒”给 /cmd_vel 订阅留出更宽的观测窗口；即便 ASR 漏掉时长，
    # CommandCompleter 也会把“向前走”补成安全的默认前进命令。
    pcm = resample(tts.synthesize("小智向前走三秒"), tts.sample_rate, 16000)
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
            lambda: bool(node.candidates_by_name("move")),
            70.0,
            f"Offline Agent produced no move candidate; asr={node.asr_text!r}",
        )

        wait_until(
            lambda: bool(node.move_command_ids()),
            10.0,
            "ActionGuard did not publish typed MOVE command",
        )
        wait_until(
            lambda: node.successful_move_result() is not None
            and node.has_forward_velocity()
            and node.metrics is not None,
            25.0,
            "typed action did not finish, cmd_vel did not move, or metrics missing",
        )
        result = node.successful_move_result()
        command_id = result["command_id"]
        candidate = next(
            (
                item
                for item in node.candidates_by_name("move")
                if item.get("request_id") == command_id
            ),
            node.candidates_by_name("move")[-1],
        )
        print(
            json.dumps(
                {
                    "asr_text": node.asr_text,
                    "move_candidates": node.candidates_by_name("move"),
                    "typed_move_command_ids": sorted(node.move_command_ids()),
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
