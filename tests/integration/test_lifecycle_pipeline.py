#!/usr/bin/env python3
"""Verify the public lifecycle contract of the Guard and simulation executor."""

import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from embodied_agent_interfaces.action import ExecuteRobotCommand
from embodied_agent_interfaces.msg import RobotCommand


class LifecycleProbe(Node):
    def __init__(self):
        super().__init__("lifecycle_pipeline_probe")
        self.candidate_pub = self.create_publisher(String, "/agent/action_candidate", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.typed_commands = []
        self.velocities = []
        self.create_subscription(
            RobotCommand,
            "/robot/action_command_typed",
            self.typed_commands.append,
            10,
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)
        self.action_client = ActionClient(
            self, ExecuteRobotCommand, "/robot/execute_command"
        )
        self.change_clients = {
            name: self.create_client(ChangeState, f"/{name}/change_state")
            for name in ("action_guard", "simulation_control")
        }
        self.state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in ("action_guard", "simulation_control")
        }

    def _on_velocity(self, message):
        self.velocities.append((message.linear.x, message.angular.z))

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_lifecycle_services(self):
        for name, client in self.change_clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                raise TimeoutError(f"{name} has no lifecycle change_state service")
        for name, client in self.state_clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                raise TimeoutError(f"{name} has no lifecycle get_state service")

    def transition(self, name, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change_clients[name].call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"{name} rejected transition {transition_id}")

    def state(self, name):
        future = self.state_clients[name].call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            raise RuntimeError(f"failed to read {name} lifecycle state")
        return response.current_state.id

    def publish_candidate(self):
        message = String()
        message.data = (
            '{"name":"move","arguments":{"linear_x":0.15,"duration_s":2.0}}'
        )
        self.candidate_pub.publish(message)

    def publish_scan(self, distance=2.0):
        scan = LaserScan()
        scan.angle_min = -3.14159265
        scan.angle_max = 3.14159265
        scan.angle_increment = 2.0 * 3.14159265 / 360.0
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [distance] * 360
        self.scan_pub.publish(scan)

    def start_move(self):
        command = RobotCommand()
        command.command_id = "lifecycle-deactivate"
        command.source = "test"
        command.action_type = RobotCommand.MOVE
        command.linear_x = 0.15
        command.duration_s = 2.0
        goal = ExecuteRobotCommand.Goal()
        goal.command = command
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("active simulation executor rejected a valid goal")
        return handle


def main():
    rclpy.init()
    probe = LifecycleProbe()
    try:
        probe.wait_for_lifecycle_services()
        for name in ("action_guard", "simulation_control"):
            if probe.state(name) != State.PRIMARY_STATE_UNCONFIGURED:
                raise RuntimeError(f"{name} did not start unconfigured")

        probe.publish_candidate()
        probe.spin_for(0.3)
        if probe.typed_commands:
            raise RuntimeError("unconfigured ActionGuard forwarded a command")

        for name in ("simulation_control", "action_guard"):
            probe.transition(name, Transition.TRANSITION_CONFIGURE)
            if probe.state(name) != State.PRIMARY_STATE_INACTIVE:
                raise RuntimeError(f"{name} did not become inactive")

        probe.publish_candidate()
        probe.spin_for(0.3)
        if probe.typed_commands:
            raise RuntimeError("inactive ActionGuard forwarded a command")

        for name in ("simulation_control", "action_guard"):
            probe.transition(name, Transition.TRANSITION_ACTIVATE)
            if probe.state(name) != State.PRIMARY_STATE_ACTIVE:
                raise RuntimeError(f"{name} did not become active")

        probe.publish_candidate()
        probe.spin_for(0.3)
        if not probe.typed_commands:
            raise RuntimeError("active ActionGuard did not forward a command")

        if not probe.action_client.wait_for_server(timeout_sec=3.0):
            raise TimeoutError("active simulation Action server was not discovered")
        handle = probe.start_move()
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not any(
            abs(linear) > 0.01 for linear, _ in probe.velocities
        ):
            probe.publish_scan()
            rclpy.spin_once(probe, timeout_sec=0.05)
        if not any(abs(linear) > 0.01 for linear, _ in probe.velocities):
            raise RuntimeError("active executor never produced motion")

        probe.transition("simulation_control", Transition.TRANSITION_DEACTIVATE)
        rclpy.spin_until_future_complete(probe, result_future, timeout_sec=3.0)
        if not result_future.done():
            raise TimeoutError("deactivate did not terminate the active Action")
        probe.spin_for(0.2)
        if not probe.velocities or any(abs(value) > 1e-6 for value in probe.velocities[-1]):
            raise RuntimeError(f"deactivate left non-zero velocity: {probe.velocities[-1:]}")

        probe.transition("action_guard", Transition.TRANSITION_DEACTIVATE)
        for name in ("action_guard", "simulation_control"):
            probe.transition(name, Transition.TRANSITION_CLEANUP)
            if probe.state(name) != State.PRIMARY_STATE_UNCONFIGURED:
                raise RuntimeError(f"{name} cleanup did not return to unconfigured")

        print("PASS: lifecycle gating -> active motion -> deactivate stop -> cleanup")
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
