#!/usr/bin/env python3
"""Exercise typed ROS Action and BehaviorTree behavior without Gazebo."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from embodied_agent_interfaces.action import ExecuteRobotCommand
from embodied_agent_interfaces.msg import BehaviorTreeStatus, RobotCommand
from embodied_online_agent.runtime_status_transport import behavior_tree_status_to_dict


class ActionProbe(Node):
    def __init__(self):
        super().__init__("typed_action_probe")
        self.client = ActionClient(
            self, ExecuteRobotCommand, "/robot/execute_command"
        )
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.velocities = []
        self.feedback = []
        self.bt_statuses = []
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)
        self.create_subscription(BehaviorTreeStatus, "/robot/bt_status", self._on_bt_status, 10)

    def _on_velocity(self, message):
        self.velocities.append((message.linear.x, message.angular.z))

    def publish_scan(self, distance=2.0):
        scan = LaserScan()
        scan.angle_min = -3.14159265
        scan.angle_max = 3.14159265
        scan.angle_increment = 2.0 * 3.14159265 / 360.0
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [distance] * 360
        self.scan_pub.publish(scan)

    def _on_feedback(self, message):
        self.feedback.append(message.feedback)

    def _on_bt_status(self, message):
        self.bt_statuses.append(behavior_tree_status_to_dict(message))

    def assert_bt_status(self, command_id, stage, outcome):
        def observed():
            return any(
                item.get("command_id") == command_id
                and item.get("stage") == stage
                and item.get("outcome") == outcome
                for item in self.bt_statuses
            )

        deadline = time.monotonic() + 1.0
        while not observed() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not observed():
            raise RuntimeError(
                f"missing BT status {command_id}/{stage}/{outcome}: "
                f"{self.bt_statuses}"
            )

    def send(self, command, timeout=5.0):
        goal = ExecuteRobotCommand.Goal()
        goal.command = command
        sent = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=timeout)
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("typed action goal was rejected")
        return handle

    def wait_result(self, handle, timeout=5.0, scan_distance=2.0):
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not result_future.done() and time.monotonic() < deadline:
            self.publish_scan(scan_distance)
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            raise TimeoutError("typed action did not return a result")
        return result_future.result().result

    def execute(self, command, timeout=5.0, scan_distance=2.0):
        return self.wait_result(
            self.send(command, timeout), timeout, scan_distance
        )

    def assert_stopped(self):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.velocities:
            raise RuntimeError("no cmd_vel was observed")
        linear, angular = self.velocities[-1]
        if abs(linear) > 1e-6 or abs(angular) > 1e-6:
            raise RuntimeError(
                f"Action terminated with non-zero velocity: {(linear, angular)}"
            )


def main():
    rclpy.init()
    node = ActionProbe()
    try:
        if not node.client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError("typed action server was not discovered")
        node.publish_scan()
        time.sleep(0.1)

        invalid_command = RobotCommand()
        invalid_command.command_id = "integration-rejected"
        invalid_command.source = "test"
        invalid_command.action_type = RobotCommand.UNKNOWN
        rejected = node.execute(invalid_command)
        if rejected.status != rejected.STATUS_REJECTED:
            raise RuntimeError(f"unexpected validation result: {rejected}")
        node.assert_bt_status("integration-rejected", "validate", "rejected")
        print("PASS: BT validation -> rejected Action result")

        command = RobotCommand()
        command.command_id = "integration-success"
        command.source = "test"
        command.action_type = RobotCommand.MOVE
        command.linear_x = 0.15
        command.duration_s = 0.3
        result = node.execute(command)
        for _ in range(5):
            node.publish_scan()
            rclpy.spin_once(node, timeout_sec=0.05)

        if not result.success or result.status != result.STATUS_SUCCEEDED:
            raise RuntimeError(f"unexpected success result: {result}")
        if not any(feedback.phase == feedback.PHASE_EXECUTING for feedback in node.feedback):
            raise RuntimeError("no executing feedback was observed")
        if not any(abs(linear) > 0.01 for linear, _ in node.velocities):
            raise RuntimeError("action never produced forward velocity")
        if not node.velocities or abs(node.velocities[-1][0]) > 1e-6:
            raise RuntimeError("robot did not stop after action completion")
        node.assert_bt_status("integration-success", "execute", "running")
        node.assert_bt_status("integration-success", "confirm", "succeeded")
        print("PASS: typed Action goal -> feedback -> success -> stop")

        cancel_command = RobotCommand()
        cancel_command.command_id = "integration-cancel"
        cancel_command.source = "test"
        cancel_command.action_type = RobotCommand.MOVE
        cancel_command.linear_x = 0.15
        cancel_command.duration_s = 2.0
        cancel_handle = node.send(cancel_command)
        cancel_deadline = time.monotonic() + 0.2
        while time.monotonic() < cancel_deadline:
            node.publish_scan()
            rclpy.spin_once(node, timeout_sec=0.05)
        cancel_future = cancel_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
        canceled = node.wait_result(cancel_handle)
        if canceled.status != canceled.STATUS_CANCELED:
            raise RuntimeError(f"unexpected cancel result: {canceled}")
        node.assert_stopped()
        node.assert_bt_status("integration-cancel", "execute", "canceled")
        print("PASS: typed Action cancellation -> canceled result -> stop")

        blocked_command = RobotCommand()
        blocked_command.command_id = "integration-blocked"
        blocked_command.source = "test"
        blocked_command.action_type = RobotCommand.MOVE
        blocked_command.linear_x = 0.15
        blocked_command.duration_s = 2.0
        blocked = node.execute(blocked_command, scan_distance=0.1)
        if blocked.status != blocked.STATUS_BLOCKED:
            raise RuntimeError(f"unexpected blocked result: {blocked}")
        node.assert_stopped()
        node.assert_bt_status("integration-blocked", "safety", "blocked")
        print("PASS: lidar safety stop -> blocked Action result")

        timeout_command = RobotCommand()
        timeout_command.command_id = "integration-timeout"
        timeout_command.source = "test"
        timeout_command.action_type = RobotCommand.TURN
        timeout_command.angular_z = 0.5
        timeout_command.duration_s = 2.0
        timed_out = node.execute(timeout_command)
        if timed_out.status != timed_out.STATUS_TIMED_OUT:
            raise RuntimeError(f"unexpected timeout result: {timed_out}")
        node.assert_stopped()
        node.assert_bt_status("integration-timeout", "execute", "timed_out")
        print("PASS: hard execution timeout -> timed_out Action result")

        preempted_command = RobotCommand()
        preempted_command.command_id = "integration-preempted"
        preempted_command.source = "test"
        preempted_command.action_type = RobotCommand.TURN
        preempted_command.angular_z = 0.5
        preempted_command.duration_s = 2.0
        preempted_handle = node.send(preempted_command)
        for _ in range(3):
            node.publish_scan()
            rclpy.spin_once(node, timeout_sec=0.05)

        stop_command = RobotCommand()
        stop_command.command_id = "integration-preempt-stop"
        stop_command.source = "test"
        stop_command.action_type = RobotCommand.STOP
        stop_result = node.execute(stop_command)
        preempted_result = node.wait_result(preempted_handle)
        if not stop_result.success:
            raise RuntimeError(f"preempting stop failed: {stop_result}")
        if preempted_result.status != preempted_result.STATUS_CANCELED:
            raise RuntimeError(f"old goal was not preempted: {preempted_result}")
        print("PASS: new Action goal preempted the active goal")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
