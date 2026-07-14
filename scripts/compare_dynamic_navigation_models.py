#!/usr/bin/env python3
"""Compare four full Nav2 dynamic-obstacle reports without inventing a winner."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


EXPECTED = ("current_only", "constant_velocity", "kalman", "imm")


def compare(reports: list[dict[str, Any]]) -> dict[str, Any]:
    by_model = {item.get("motion_model"): item for item in reports}
    errors: list[str] = []
    if set(by_model) != set(EXPECTED):
        errors.append(f"expected exactly {list(EXPECTED)}")
    rows = []
    for model in EXPECTED:
        report = by_model.get(model)
        if report is None:
            continue
        if not report.get("passed"):
            errors.append(f"{model}: navigation report did not pass")
        checks = report.get("checks", {})
        required = (
            "future_cell_marked_lethal",
            "dynamic_plan_increased_clearance",
            "navigate_to_pose_succeeded",
            "cmd_vel_returned_to_zero",
        )
        for check in required:
            if checks.get(check) is not True:
                errors.append(f"{model}: missing successful check {check}")
        rows.append(
            {
                "model": model,
                "velocity_y_mps": report.get("track", {}).get("velocity_y_mps"),
                "predicted_y_m": report.get("prediction", {}).get("y"),
                "baseline_clearance_m": report.get("baseline_clearance_m"),
                "dynamic_clearance_m": report.get("dynamic_clearance_m"),
                "odom_traveled_distance_m": report.get("odom_traveled_distance_m"),
                "elapsed_s": report.get("elapsed_s"),
            }
        )
    return {
        "schema_version": 1,
        "passed": not errors,
        "models": rows,
        "errors": errors,
        "claim_boundary": (
            "Same Gazebo/Nav2 crossing scenario for all models; tracker benchmark report is the "
            "authoritative source for prediction RMSE ranking."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("reports", nargs=4, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = compare([json.loads(path.read_text(encoding="utf-8")) for path in args.reports])
    result["sources"] = [
        {
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        for path in args.reports
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
