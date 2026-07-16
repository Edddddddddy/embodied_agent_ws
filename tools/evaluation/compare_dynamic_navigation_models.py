#!/usr/bin/env python3
"""Compare four full Nav2 dynamic-obstacle reports without inventing a winner."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


EXPECTED = ("current_only", "constant_velocity", "kalman", "imm")
PROVENANCE_FIELDS = (
    "scenario_id",
    "scenario_sha256",
    "map_artifact_sha256",
    "nav2_params_sha256",
)


def compare(reports: list[dict[str, Any]]) -> dict[str, Any]:
    by_model = {item.get("motion_model"): item for item in reports}
    errors: list[str] = []
    if set(by_model) != set(EXPECTED):
        errors.append(f"expected exactly {list(EXPECTED)}")
    rows = []
    reference_provenance: dict[str, Any] | None = None
    for model in EXPECTED:
        report = by_model.get(model)
        if report is None:
            continue
        if not report.get("passed"):
            errors.append(f"{model}: navigation report did not pass")
        checks = report.get("checks", {})
        required = (
            "tracker_model_behavior",
            "future_cell_marked_lethal",
            "dynamic_plan_increased_clearance",
            "navigate_to_pose_succeeded",
            "robot_moved_at_least_1m",
            "cmd_vel_returned_to_zero",
        )
        for check in required:
            if checks.get(check) is not True:
                errors.append(f"{model}: missing successful check {check}")
        provenance = report.get("provenance", {})
        missing_provenance = [field for field in PROVENANCE_FIELDS if not provenance.get(field)]
        if missing_provenance:
            errors.append(f"{model}: missing provenance {missing_provenance}")
        elif reference_provenance is None:
            reference_provenance = {field: provenance[field] for field in PROVENANCE_FIELDS}
        else:
            for field in PROVENANCE_FIELDS:
                if provenance[field] != reference_provenance[field]:
                    errors.append(
                        f"{model}: {field} differs from the first model; ablation is not comparable"
                    )
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
        "schema_version": 2,
        "passed": not errors,
        "provenance": reference_provenance or {},
        "models": rows,
        "errors": errors,
        "claim_boundary": (
            "All hashes prove the same deterministic typed-detection schedule, map artifact and "
            "Nav2 parameters. This is a Gazebo/Nav2 closed loop, but the obstacle input is a "
            "synthetic PoseArray rather than a physical Gazebo actor. The tracker benchmark is "
            "the authoritative source for prediction RMSE ranking."
        ),
    }


def render_markdown(result: dict[str, Any]) -> str:
    def metric(value: Any, digits: int) -> str:
        return f"{float(value):.{digits}f}" if isinstance(value, (int, float)) else "n/a"

    status = "PASS" if result["passed"] else "FAIL"
    provenance = result.get("provenance", {})
    lines = [
        "# Dynamic-obstacle Gazebo/Nav2 ablation",
        "",
        f"- Status: **{status}**",
        f"- Scenario: `{provenance.get('scenario_id', 'missing')}`",
        f"- Scenario SHA256: `{provenance.get('scenario_sha256', 'missing')}`",
        f"- Map artifact SHA256: `{provenance.get('map_artifact_sha256', 'missing')}`",
        f"- Nav2 params SHA256: `{provenance.get('nav2_params_sha256', 'missing')}`",
        "",
        "| Model | Vy (m/s) | Predicted y (m) | Baseline clearance (m) | Dynamic clearance (m) | Travel (m) | Elapsed (s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in result["models"]:
        lines.append(
            f"| {row['model']} | {metric(row['velocity_y_mps'], 4)} | "
            f"{metric(row['predicted_y_m'], 4)} | "
            f"{metric(row['baseline_clearance_m'], 4)} | "
            f"{metric(row['dynamic_clearance_m'], 4)} | "
            f"{metric(row['odom_traveled_distance_m'], 4)} | "
            f"{metric(row['elapsed_s'], 3)} |"
        )
    lines.extend(["", "## Evidence boundary", "", result["claim_boundary"]])
    if result["errors"]:
        lines.extend(["", "## Errors", ""])
        lines.extend(f"- {error}" for error in result["errors"])
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("reports", nargs=4, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--markdown", type=Path)
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
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(render_markdown(result), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
