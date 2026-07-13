#!/usr/bin/env python3
"""Compare dead-reckoning and loop-corrected trajectory reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def compare(baseline: dict, corrected: dict) -> dict[str, object]:
    baseline_ate = float(baseline["ate_xy_m"]["rmse"])
    corrected_ate = float(corrected["ate_xy_m"]["rmse"])
    baseline_rpe = float(baseline["rpe"]["translation_m"]["rmse"])
    corrected_rpe = float(corrected["rpe"]["translation_m"]["rmse"])
    baseline_final = float(baseline["closure"]["final_pose_error_m"])
    corrected_final = float(corrected["closure"]["final_pose_error_m"])
    checks = {
        "reports_passed": bool(baseline.get("passed")) and bool(corrected.get("passed")),
        "comparable_match_count": abs(
            int(baseline["association"]["matched_poses"])
            - int(corrected["association"]["matched_poses"])
        )
        <= 1,
        "ate_improved": corrected_ate < baseline_ate,
        "rpe_improved": corrected_rpe < baseline_rpe,
        "final_drift_improved": corrected_final < baseline_final,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "baseline": {
            "ate_rmse_m": baseline_ate,
            "rpe_translation_rmse_m": baseline_rpe,
            "final_pose_error_m": baseline_final,
            "loop_recall": baseline["loop"]["recall"],
        },
        "corrected": {
            "ate_rmse_m": corrected_ate,
            "rpe_translation_rmse_m": corrected_rpe,
            "final_pose_error_m": corrected_final,
            "loop_recall": corrected["loop"]["recall"],
        },
        "improvement": {
            "ate_rmse_ratio": corrected_ate / max(baseline_ate, 1e-12),
            "rpe_translation_rmse_ratio": corrected_rpe / max(baseline_rpe, 1e-12),
            "final_pose_error_ratio": corrected_final / max(baseline_final, 1e-12),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--corrected", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("logs/slam_evaluation_comparison.json"))
    args = parser.parse_args()
    report = compare(
        json.loads(args.baseline.read_text(encoding="utf-8")),
        json.loads(args.corrected.read_text(encoding="utf-8")),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: loop-correction comparison")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
