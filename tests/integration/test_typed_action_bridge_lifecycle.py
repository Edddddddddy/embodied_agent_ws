#!/usr/bin/env python3
"""向已激活的 typed bridge 发送一条命令并校验关联终态。"""

from __future__ import annotations

import argparse
import time

import rclpy
from embodied_agent_interfaces.msg import RobotCommand, RobotCommandResult
from rclpy.node import Node


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--command-id", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Node(f"typed_bridge_lifecycle_probe_{args.command_id.replace('-', '_')}")
    publisher = node.create_publisher(
        RobotCommand, "/robot/action_command_typed", 10
    )
    results: list[RobotCommandResult] = []
    node.create_subscription(
        RobotCommandResult, "/robot/action_result", results.append, 10
    )
    deadline = time.monotonic() + 8.0
    try:
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() < 1:
            raise TimeoutError("typed bridge command subscriber not discovered")

        command = RobotCommand()
        command.command_id = args.command_id
        command.source = "lifecycle_probe"
        command.action_type = RobotCommand.STOP
        command.priority = True
        publisher.publish(command)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            result = next(
                (item for item in results if item.command_id == args.command_id),
                None,
            )
            if result is not None:
                if not result.success:
                    raise RuntimeError(
                        f"command failed: status={result.status} message={result.message}"
                    )
                print(f"PASS: {args.command_id} -> {result.message}")
                return
        raise TimeoutError("typed bridge action result not received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
