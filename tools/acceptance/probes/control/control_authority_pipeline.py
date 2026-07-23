#!/usr/bin/env python3
"""不依赖 Gazebo，验证自治/键盘/HOLD/急停的真实 ROS 速度链路。"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time

from embodied_agent_interfaces.msg import ControlAuthorityState
from embodied_agent_interfaces.srv import (
    AcknowledgeAutonomyQuiescence,
    SetControlAuthority,
)
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ControlAuthorityProbe(Node):
    def __init__(self) -> None:
        super().__init__("control_authority_pipeline_probe")
        self._nav2 = self.create_publisher(
            Twist, "/control/nav2/cmd_vel", 10
        )
        self._voice = self.create_publisher(
            Twist, "/control/voice/cmd_vel", 10
        )
        self._keyboard = self.create_publisher(
            Twist, "/control/keyboard/cmd_vel", 10
        )
        self._selected: list[tuple[float, float, float, float, float, float]] = []
        self._authority: ControlAuthorityState | None = None
        self._replacement_manager: subprocess.Popen[bytes] | None = None
        self.create_subscription(
            Twist,
            "/control/selected/cmd_vel",
            lambda message: self._selected.append(self._sample(message)),
            10,
        )
        self.create_subscription(
            ControlAuthorityState,
            "/control/authority/state",
            self._on_authority,
            STATE_QOS,
        )
        self._client = self.create_client(
            SetControlAuthority, "/control/set_authority"
        )
        self._ack_client = self.create_client(
            AcknowledgeAutonomyQuiescence,
            "/control/acknowledge_autonomy_quiescence",
        )

    def _on_authority(self, message: ControlAuthorityState) -> None:
        self._authority = message

    @staticmethod
    def _sample(
        message: Twist,
    ) -> tuple[float, float, float, float, float, float]:
        return (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)

    def wait_ready(self) -> None:
        if not self._client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError("control authority service unavailable")
        if not self._ack_client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError("quiescence ACK service unavailable")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self._authority is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._authority is None:
            raise TimeoutError("control authority state unavailable")

    def request(self, command: int, requester: str = "stage_operator") -> None:
        request = SetControlAuthority.Request()
        request.command = command
        request.requester = requester
        request.reason = "control_plane_stage"
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.accepted:
            message = "no response" if response is None else response.message
            raise RuntimeError(
                f"authority command={command} rejected: {message}"
            )
        deadline = time.monotonic() + 2.0
        while (
            time.monotonic() < deadline
            and (
                self._authority is None
                or self._authority.transition_sequence
                < response.state.transition_sequence
            )
        ):
            rclpy.spin_once(self, timeout_sec=0.03)

    def acknowledge_quiescence(self) -> None:
        """用当前 manager 身份确认最小 stage 已经没有自治 owner。

        这个探针没有 Explore/Nav2 任务，因此稳定零速就是剩余的运行时证据；
        仍必须走生产 typed ACK，不能为了测试方便绕过恢复协议。
        """

        if self._authority is None:
            raise RuntimeError("authority state unavailable")
        if self._authority.authority != ControlAuthorityState.HOLD:
            raise RuntimeError("quiescence ACK requires HOLD")
        request = AcknowledgeAutonomyQuiescence.Request()
        request.manager_epoch = self._authority.manager_epoch
        request.autonomy_revocation_sequence = (
            self._authority.pending_autonomy_revocation_sequence
        )
        request.requester = "control_authority_stage"
        request.detail = "no active autonomous owner; selected velocity is zero"
        future = self._ack_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.accepted:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"quiescence ACK rejected: {message}")
        self._authority = response.state

    def assert_authority(self, expected: int) -> None:
        if self._authority is None or self._authority.authority != expected:
            actual = None if self._authority is None else self._authority.authority
            raise RuntimeError(
                f"authority mismatch: expected={expected}, actual={actual}"
            )

    def current_manager_epoch(self) -> int:
        if self._authority is None:
            raise RuntimeError("authority state unavailable")
        return self._authority.manager_epoch

    @staticmethod
    def _twist(linear_x: float, angular_z: float = 0.0) -> Twist:
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        return message

    def assert_selected(
        self,
        *,
        expected_linear_x: float,
        expected_angular_z: float = 0.0,
        nav2_linear_x: float = 0.0,
        voice_linear_x: float = 0.0,
        keyboard_linear_x: float = 0.0,
        settle_s: float = 0.30,
        stable_s: float = 0.45,
    ) -> None:
        # 先给 DDS、mux 与 gate 一个明确过渡期；随后稳定窗口逐帧检查，不能再用
        # “曾经出现过一个正确样本”掩盖未授权速度偶发穿透。
        self._selected.clear()
        transition_deadline = time.monotonic() + settle_s
        while time.monotonic() < transition_deadline:
            self._nav2.publish(self._twist(nav2_linear_x))
            self._voice.publish(self._twist(voice_linear_x))
            self._keyboard.publish(self._twist(keyboard_linear_x))
            rclpy.spin_once(self, timeout_sec=0.03)

        self._selected.clear()
        stable_deadline = time.monotonic() + stable_s
        expected = (
            expected_linear_x,
            0.0,
            0.0,
            0.0,
            0.0,
            expected_angular_z,
        )
        while time.monotonic() < stable_deadline:
            self._nav2.publish(self._twist(nav2_linear_x))
            self._voice.publish(self._twist(voice_linear_x))
            self._keyboard.publish(self._twist(keyboard_linear_x))
            rclpy.spin_once(self, timeout_sec=0.03)

        if len(self._selected) < 4:
            raise RuntimeError(
                "selected velocity evidence too sparse: "
                f"samples={self._selected}"
            )
        violations = [
            sample
            for sample in self._selected
            if not all(
                math.isclose(actual, target, abs_tol=1e-6)
                for actual, target in zip(sample, expected, strict=True)
            )
        ]
        if violations:
            unauthorized = math.isclose(
                expected_linear_x, 0.0, abs_tol=1e-9
            ) and math.isclose(expected_angular_z, 0.0, abs_tol=1e-9)
            label = (
                "unauthorized non-zero velocity observed"
                if unauthorized
                else "authorized velocity was not stable"
            )
            raise RuntimeError(
                f"{label}: expected={expected}, "
                f"violations={violations[:12]}, "
                f"sample_count={len(self._selected)}"
            )

    def release_autonomy_quarantine(self) -> None:
        """用新一代零速握手解除自治源隔离。

        Twist 本身不携带 manager epoch。RESUME 后直接出现的非零样本可能是 DDS
        补送的旧速度，因此生产 Gate 会先丢弃它们。探针必须像真实自治控制器一样
        先发布明确零速，再从下一条新命令开始运动，不能为了测试绕过这条安全协议。
        """

        self.assert_selected(
            expected_linear_x=0.0,
            nav2_linear_x=0.0,
            voice_linear_x=0.0,
            keyboard_linear_x=0.0,
            settle_s=0.15,
            stable_s=0.20,
        )

    def wait_for_manager_epoch(
        self,
        *,
        previous_epoch: int,
        expected_authority: int,
        timeout_s: float = 5.0,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self._authority is not None
                and self._authority.manager_epoch != previous_epoch
                and self._authority.authority == expected_authority
            ):
                return
        raise RuntimeError(
            "replacement manager did not establish a fresh HOLD epoch: "
            f"previous_epoch={previous_epoch}, current={self._authority}"
        )

    @staticmethod
    def _original_manager_process_group() -> int:
        configured = os.environ.get(
            "CONTROL_AUTHORITY_PROCESS_GROUP_ID", ""
        ).strip()
        if not configured:
            raise RuntimeError(
                "CONTROL_AUTHORITY_PROCESS_GROUP_ID is required for lease test"
            )
        return int(configured)

    def pause_original_manager(self) -> None:
        os.killpg(self._original_manager_process_group(), signal.SIGSTOP)

    def restart_manager(self) -> None:
        original_group = self._original_manager_process_group()
        # 先把 TERM 置为 pending 再 CONT；若顺序相反，旧进程可能抢在 TERM 前重发
        # 一次 AUTONOMY 心跳，使刚因 lease 失效归零的 gate 短暂复驶。
        os.killpg(original_group, signal.SIGTERM)
        os.killpg(original_group, signal.SIGCONT)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self._client.service_is_ready():
                break
        else:
            raise RuntimeError(
                "original authority service did not disappear before restart"
            )
        self._replacement_manager = subprocess.Popen(
            [
                "ros2",
                "run",
                "embodied_agent_cpp",
                "control_authority",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def cleanup_replacement_manager(self) -> None:
        try:
            # 断言中途失败时也不能把 session 管理的原进程永久留在 SIGSTOP。
            os.killpg(
                self._original_manager_process_group(), signal.SIGCONT
            )
        except (ProcessLookupError, RuntimeError):
            pass
        process = self._replacement_manager
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)


def main() -> int:
    rclpy.init()
    probe = ControlAuthorityProbe()
    try:
        probe.wait_ready()
        probe.assert_authority(ControlAuthorityState.HOLD)
        probe.assert_selected(
            expected_linear_x=0.0,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )

        probe.acknowledge_quiescence()
        probe.request(SetControlAuthority.Request.RESUME_AUTONOMY)
        probe.assert_authority(ControlAuthorityState.AUTONOMY)
        probe.release_autonomy_quarantine()
        # voice 的 mux 优先级高于 Nav2；键盘在 AUTONOMY 下必须被正授权门拒绝。
        probe.assert_selected(
            expected_linear_x=0.22,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )

        probe.request(
            SetControlAuthority.Request.TAKE_KEYBOARD,
            requester="keyboard_teleop",
        )
        probe.assert_authority(ControlAuthorityState.KEYBOARD)
        probe.assert_selected(
            expected_linear_x=0.24,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )

        probe.request(
            SetControlAuthority.Request.RELEASE_KEYBOARD,
            requester="keyboard_teleop",
        )
        probe.assert_authority(ControlAuthorityState.HOLD)
        probe.assert_selected(
            expected_linear_x=0.0,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )
        probe.acknowledge_quiescence()

        probe.request(SetControlAuthority.Request.EMERGENCY_STOP)
        probe.assert_authority(ControlAuthorityState.ESTOP)
        probe.assert_selected(
            expected_linear_x=0.0,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )
        probe.request(SetControlAuthority.Request.RESET_EMERGENCY_STOP)
        probe.assert_authority(ControlAuthorityState.HOLD)
        probe.assert_selected(
            expected_linear_x=0.0,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )

        probe.acknowledge_quiescence()
        probe.request(SetControlAuthority.Request.RESUME_AUTONOMY)
        probe.release_autonomy_quarantine()
        probe.assert_selected(
            expected_linear_x=0.22,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )
        previous_epoch = probe.current_manager_epoch()
        probe.pause_original_manager()
        # 默认 lease=750 ms；越过租约后，稳定窗口的每一个输出都必须为零。
        probe.assert_selected(
            expected_linear_x=0.0,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
            settle_s=1.05,
        )

        probe.restart_manager()
        probe.wait_for_manager_epoch(
            previous_epoch=previous_epoch,
            expected_authority=ControlAuthorityState.HOLD,
        )
        probe.assert_selected(
            expected_linear_x=0.0,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )
        probe.acknowledge_quiescence()
        probe.request(SetControlAuthority.Request.RESUME_AUTONOMY)
        probe.release_autonomy_quarantine()
        probe.assert_selected(
            expected_linear_x=0.22,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )
        probe.request(SetControlAuthority.Request.ENTER_HOLD)
        probe.assert_selected(
            expected_linear_x=0.0,
            nav2_linear_x=0.11,
            voice_linear_x=0.22,
            keyboard_linear_x=0.24,
        )
        print(
            "PASS: HOLD zero -> AUTONOMY mux -> KEYBOARD owner -> "
            "HOLD/ESTOP zero -> manager lease zero -> fresh restart epoch"
        )
        return 0
    finally:
        probe.cleanup_replacement_manager()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
