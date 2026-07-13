#!/usr/bin/env python3
"""Compare Ceres and GTSAM reports produced from the same OpenLORIS replay."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def _metric(report: dict, *path: str) -> float:
    value: object = report
    for key in path:
        value = value[key]  # type: ignore[index]
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"non-finite metric at {'.'.join(path)}")
    return result


def compare(
    ceres: dict,
    gtsam: dict,
    ceres_degradation: dict | None = None,
    gtsam_degradation: dict | None = None,
) -> dict[str, object]:
    """Return comparable evidence without assuming either solver must win."""

    c_matches = int(ceres["association"]["matched_poses"])
    g_matches = int(gtsam["association"]["matched_poses"])
    max_matches = max(c_matches, g_matches, 1)
    c_ate = _metric(ceres, "ate_xy_m", "rmse")
    g_ate = _metric(gtsam, "ate_xy_m", "rmse")
    c_rpe = _metric(ceres, "rpe", "translation_m", "rmse")
    g_rpe = _metric(gtsam, "rpe", "translation_m", "rmse")
    c_coverage = _metric(ceres, "association", "reference_temporal_coverage_ratio")
    g_coverage = _metric(gtsam, "association", "reference_temporal_coverage_ratio")
    checks = {
        "ceres_report_passed": bool(ceres.get("passed")),
        "gtsam_report_passed": bool(gtsam.get("passed")),
        "same_sample_window": abs(c_matches - g_matches) / max_matches <= 0.02,
        "same_temporal_coverage": abs(c_coverage - g_coverage) <= 0.02,
    }
    # A/B 的门禁只检查输入可比性。后端优劣属于实验输出，不能反向写死为“GTSAM 必胜”。
    winner = "tie"
    if not math.isclose(c_ate, g_ate, rel_tol=0.01, abs_tol=1e-4):
        winner = "ceres" if c_ate < g_ate else "gtsam"
    motion_classes: dict[str, object] = {}
    if (ceres_degradation is None) != (gtsam_degradation is None):
        raise ValueError("both degradation reports are required for motion A/B")
    if ceres_degradation is not None and gtsam_degradation is not None:
        for label in ("straight", "turning", "stationary"):
            c_item = ceres_degradation["motion_classes"][label]
            g_item = gtsam_degradation["motion_classes"][label]
            same_samples = int(c_item["samples"]) == int(g_item["samples"])
            checks[f"same_{label}_samples"] = same_samples
            c_value = c_item["ate_rmse_m"]
            g_value = g_item["ate_rmse_m"]
            motion_classes[label] = {
                "samples": int(c_item["samples"]),
                "ceres_ate_rmse_m": c_value,
                "gtsam_ate_rmse_m": g_value,
                "gtsam_minus_ceres_m": (
                    round(float(g_value) - float(c_value), 6)
                    if c_value is not None and g_value is not None
                    else None
                ),
            }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "methodology": {
            "controlled_variable": "same bag, front-end parameters and evaluator",
            "changed_variable": "slam_toolbox pose-graph solver plugin",
            "winner_policy": "descriptive ATE only; no backend is preselected",
        },
        "ceres": {
            "matched_poses": c_matches,
            "temporal_coverage_ratio": c_coverage,
            "ate_rmse_m": c_ate,
            "rpe_translation_rmse_m": c_rpe,
            "loop_recall": ceres["loop"]["recall"],
        },
        "gtsam": {
            "matched_poses": g_matches,
            "temporal_coverage_ratio": g_coverage,
            "ate_rmse_m": g_ate,
            "rpe_translation_rmse_m": g_rpe,
            "loop_recall": gtsam["loop"]["recall"],
        },
        "delta": {
            "ate_gtsam_minus_ceres_m": round(g_ate - c_ate, 6),
            "rpe_gtsam_minus_ceres_m": round(g_rpe - c_rpe, 6),
        },
        "motion_classes": motion_classes,
        "lower_ate_backend": winner,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ceres", type=Path, required=True)
    parser.add_argument("--gtsam", type=Path, required=True)
    parser.add_argument("--ceres-degradation", type=Path)
    parser.add_argument("--gtsam-degradation", type=Path)
    parser.add_argument(
        "--output", type=Path, default=Path("logs/openloris_backend_comparison.json")
    )
    args = parser.parse_args()
    report = compare(
        json.loads(args.ceres.read_text(encoding="utf-8")),
        json.loads(args.gtsam.read_text(encoding="utf-8")),
        (
            json.loads(args.ceres_degradation.read_text(encoding="utf-8"))
            if args.ceres_degradation
            else None
        ),
        (
            json.loads(args.gtsam_degradation.read_text(encoding="utf-8"))
            if args.gtsam_degradation
            else None
        ),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: OpenLORIS backend A/B")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
