#!/usr/bin/env python3
"""Wait for the typed system readiness contract published by the launch profile."""

from __future__ import annotations

import argparse
import json
import time


def readiness_to_dict(message) -> dict:
    return {
        "profile": message.profile,
        "ready": bool(message.ready),
        "required_components": list(message.required_components),
        "ready_components": list(message.ready_components),
        "missing_components": list(message.missing_components),
        "degraded_components": list(message.degraded_components),
        "detail": message.detail,
    }


def format_readiness(payload: dict) -> str:
    status = "PASS" if payload.get("ready") else "BLOCKED"
    lines = [
        f"{status}: system readiness profile={payload.get('profile', '')}",
        f"  ready: {', '.join(payload.get('ready_components', [])) or '-'}",
        f"  missing: {', '.join(payload.get('missing_components', [])) or '-'}",
        f"  degraded: {', '.join(payload.get('degraded_components', [])) or '-'}",
        f"  detail: {payload.get('detail', '')}",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--profile", default="")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    import rclpy
    from embodied_agent_interfaces.msg import SystemReadiness
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = Node("system_readiness_check")
    latest = None

    def on_readiness(message):
        nonlocal latest
        if not args.profile or message.profile == args.profile:
            latest = message

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    subscription = node.create_subscription(
        SystemReadiness, "/system/readiness", on_readiness, qos
    )
    deadline = time.monotonic() + max(0.1, args.timeout)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest is not None and latest.ready:
                break
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    payload = readiness_to_dict(latest) if latest is not None else {
        "profile": args.profile,
        "ready": False,
        "required_components": [],
        "ready_components": [],
        "missing_components": ["system_readiness_status_missing"],
        "degraded_components": [],
        "detail": "system_readiness_status_missing",
    }
    print(
        json.dumps(payload, ensure_ascii=False, indent=2)
        if args.json else format_readiness(payload)
    )
    raise SystemExit(0 if payload["ready"] else 1)


if __name__ == "__main__":
    main()
