#!/usr/bin/env python3
"""用有界 rclpy client 验证节点参数，避免 ros2cli daemon discovery 干扰 smoke。"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMessage
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


def _normalized(value: object) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("node")
    parser.add_argument("parameter")
    parser.add_argument("--expected")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("parameter_contract_probe")
    try:
        client = AsyncParameterClient(node, args.node)
        if not client.wait_for_services(timeout_sec=args.timeout):
            print(f"FAIL: parameter service unavailable: {args.node}", file=sys.stderr)
            return 2
        future = client.get_parameters([args.parameter])
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done() or future.result() is None:
            print(f"FAIL: parameter request timed out: {args.parameter}", file=sys.stderr)
            return 3
        response = future.result()
        if len(response.values) != 1:
            print(f"FAIL: parameter missing: {args.parameter}", file=sys.stderr)
            return 4
        value = Parameter.from_parameter_msg(
            ParameterMessage(name=args.parameter, value=response.values[0])
        ).value
        print(f"{args.node}.{args.parameter}={_normalized(value)}")
        if args.expected is not None and _normalized(value) != args.expected.lower():
            print(
                f"FAIL: expected {args.expected!r}, actual {_normalized(value)!r}",
                file=sys.stderr,
            )
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
