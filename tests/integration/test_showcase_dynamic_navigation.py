#!/usr/bin/env python3
"""在已启动的 showcase 导航阶段单独验证动态障碍闭环。"""

from __future__ import annotations

import argparse
import json
import threading
from pathlib import Path

from lifecycle_msgs.msg import State
import rclpy

from test_voice_slam_session_orchestrator import (
    SessionProbe,
    lifecycle_states,
    run_showcase_dynamic_navigation,
    wait_until,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = SessionProbe()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.localization_tf_count > 0
            and node.amcl_pose_count > 0
            and node.latest_costmap is not None,
            90.0,
            "showcase localization/costmap did not become ready",
        )
        states = lifecycle_states(node)
        if len(states) != 4 or any(
            value != State.PRIMARY_STATE_ACTIVE for value in states.values()
        ):
            raise RuntimeError(f"Nav2 lifecycle nodes not active: {states}")
        evidence = run_showcase_dynamic_navigation(
            node, args.scenario, timeout_s=args.timeout
        )
        report = {
            "passed": evidence["passed"],
            "lifecycle_states": states,
            "localization_tf_count": node.localization_tf_count,
            "amcl_pose_count": node.amcl_pose_count,
            "dynamic_navigation": evidence,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
