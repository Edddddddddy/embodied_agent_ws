#!/usr/bin/env python3
"""Verify Agent Lifecycle gating, safe deactivate and reactivation."""

from __future__ import annotations

import argparse
import time

import rclpy
from embodied_agent_interfaces.msg import ComponentHealth, RobotCommand
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from std_msgs.msg import String


class AgentLifecycleProbe(Node):
    def __init__(self, agent_name: str):
        super().__init__("agent_lifecycle_probe")
        prefix = f"/{agent_name}"
        self._change = self.create_client(ChangeState, f"{prefix}/change_state")
        self._state = self.create_client(GetState, f"{prefix}/get_state")
        self._text = self.create_publisher(String, "/agent/text_input", 10)
        self.commands: list[RobotCommand] = []
        self.states: list[str] = []
        self.health: list[ComponentHealth] = []
        self.create_subscription(
            RobotCommand,
            "/agent/action_candidate",
            self.commands.append,
            10,
        )
        self.create_subscription(
            ComponentHealth,
            "/system/component_health",
            self.health.append,
            10,
        )
        self.create_subscription(
            String,
            "/agent/state",
            lambda message: self.states.append(message.data),
            10,
        )

    def wait_ready(self, timeout_s: float = 10.0) -> None:
        if not self._change.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("change_state service unavailable")
        if not self._state.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("get_state service unavailable")

    def transition(self, transition_id: int) -> None:
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self._change.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done() or future.result() is None:
            raise TimeoutError(f"transition {transition_id} timed out")
        if not future.result().success:
            raise RuntimeError(f"transition {transition_id} failed")

    def assert_state(self, expected: int) -> None:
        future = self._state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise TimeoutError("get_state timed out")
        actual = future.result().current_state.id
        if actual != expected:
            raise AssertionError(f"state={actual}, expected={expected}")

    def publish_until_move_count(self, expected: int, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._text.publish(String(data="向前走一秒"))
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.move_count() >= expected:
                return
        raise TimeoutError(f"move_count={self.move_count()}, expected={expected}")

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_until_state(self, expected: str, timeout_s: float = 3.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._text.publish(String(data="向前走一秒"))
            rclpy.spin_once(self, timeout_sec=0.05)
            if expected in self.states:
                return
        raise TimeoutError(f"state {expected!r} was not observed")

    def move_count(self) -> int:
        return sum(command.action_type == RobotCommand.MOVE for command in self.commands)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agent-name", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = AgentLifecycleProbe(args.agent_name)
    try:
        node.wait_ready()
        node.assert_state(State.PRIMARY_STATE_UNCONFIGURED)

        # 未 configure/activate 时即使 DDS 收到文本，也不能产生动作候选。
        node.publish_until_move_count(1, timeout_s=0.4)
    except TimeoutError as exc:
        if "move_count=0" not in str(exc):
            raise
    else:
        raise AssertionError("inactive Agent unexpectedly published MOVE")

    try:
        node.transition(Transition.TRANSITION_CONFIGURE)
        node.assert_state(State.PRIMARY_STATE_INACTIVE)
        node.transition(Transition.TRANSITION_ACTIVATE)
        node.assert_state(State.PRIMARY_STATE_ACTIVE)

        # 在模型仍流式输出时停用，运行时必须协作取消，而不是留下后台线程。
        node.publish_until_state("thinking")
        node.transition(Transition.TRANSITION_DEACTIVATE)
        node.assert_state(State.PRIMARY_STATE_INACTIVE)
        node.spin_for(0.3)
        assert node.move_count() == 0
        assert any(command.action_type == RobotCommand.STOP for command in node.commands)
        assert node.health[-1].state == ComponentHealth.STATE_STOPPED
        first_activation_moves = node.move_count()
        node._text.publish(String(data="向前走一秒"))
        node.spin_for(0.4)
        assert node.move_count() == first_activation_moves

        node.transition(Transition.TRANSITION_ACTIVATE)
        node.publish_until_move_count(first_activation_moves + 1)
        node.transition(Transition.TRANSITION_DEACTIVATE)
        node.transition(Transition.TRANSITION_ACTIVATE)
        node.publish_until_move_count(first_activation_moves + 2)
        node.transition(Transition.TRANSITION_DEACTIVATE)
        node.transition(Transition.TRANSITION_CLEANUP)
        node.assert_state(State.PRIMARY_STATE_UNCONFIGURED)

        # cleanup 后再次 configure，证明模型/provider 资源可被完整重建。
        node.transition(Transition.TRANSITION_CONFIGURE)
        node.transition(Transition.TRANSITION_ACTIVATE)
        node.publish_until_move_count(first_activation_moves + 3)
        node.transition(Transition.TRANSITION_DEACTIVATE)
        node.transition(Transition.TRANSITION_CLEANUP)
        node.assert_state(State.PRIMARY_STATE_UNCONFIGURED)
        print("PASS: inactive gate -> activate -> safe stop -> reactivate -> cleanup")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
