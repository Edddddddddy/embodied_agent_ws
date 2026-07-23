#!/usr/bin/env python3
"""Wait for the typed system readiness contract published by the launch profile."""

from __future__ import annotations

import argparse
import json
import sys
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


def format_readiness_progress(payload: dict, *, elapsed_s: float) -> str:
    """把“仍在等待”的原因打印出来，避免 GUI 已启动时被误认为程序卡死。"""

    missing = ", ".join(payload.get("missing_components", [])) or "readiness_status"
    degraded = ", ".join(payload.get("degraded_components", [])) or "-"
    return (
        f"WAIT: system readiness profile={payload.get('profile', '')} "
        f"elapsed={elapsed_s:.1f}s missing={missing} degraded={degraded}. "
        "若长期不变化，请检查缺失节点的 Lifecycle configure/activate 日志。"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--profile", default="")
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--progress-interval",
        type=float,
        default=5.0,
        help="未就绪时打印缺失组件的间隔；设为 0 可关闭",
    )
    args = parser.parse_args()

    import rclpy
    from embodied_agent_core.ros_qos import state_qos
    from embodied_agent_interfaces.msg import SystemReadiness
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    rclpy.init()
    node = Node("system_readiness_check")
    latest = None

    def on_readiness(message):
        nonlocal latest
        if not args.profile or message.profile == args.profile:
            latest = message

    subscription = node.create_subscription(
        SystemReadiness, "/system/readiness", on_readiness, state_qos()
    )
    started = time.monotonic()
    deadline = started + max(0.1, args.timeout)
    next_progress = started + max(0.0, args.progress_interval)
    interrupted = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest is not None and latest.ready:
                break
            now = time.monotonic()
            if args.progress_interval > 0.0 and now >= next_progress:
                payload = (
                    readiness_to_dict(latest)
                    if latest is not None
                    else {
                        "profile": args.profile,
                        "ready": False,
                        "missing_components": ["system_readiness_status_missing"],
                        "degraded_components": [],
                    }
                )
                print(
                    format_readiness_progress(payload, elapsed_s=now - started),
                    file=sys.stderr,
                    flush=True,
                )
                next_progress = now + args.progress_interval
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 由 launch/会话管理器统一回收；这里不再输出一整段 Python traceback。
        interrupted = True
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
    if interrupted:
        payload["ready"] = False
        payload["detail"] = "system_readiness_check_interrupted"
    print(
        json.dumps(payload, ensure_ascii=False, indent=2)
        if args.json else format_readiness(payload)
    )
    raise SystemExit(130 if interrupted else (0 if payload["ready"] else 1))


if __name__ == "__main__":
    main()
