#!/usr/bin/env python3
"""比较同一前端、同一漂移输入下的 Ceres 与 GTSAM 建图证据。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def compare(ceres: dict, gtsam: dict) -> dict[str, object]:
    c_traj = ceres["trajectory"]
    g_traj = gtsam["trajectory"]
    c_area = float(ceres["map"]["known_area_m2"])
    g_area = float(gtsam["map"]["known_area_m2"])
    area_delta_ratio = abs(c_area - g_area) / max(c_area, g_area, 1e-9)
    checks = {
        "ceres_run_passed": bool(ceres.get("passed")) and ceres.get("solver") == "ceres",
        "gtsam_run_passed": bool(gtsam.get("passed")) and gtsam.get("solver") == "gtsam",
        "same_route_length": abs(
            float(c_traj["reference_path_length_m"])
            - float(g_traj["reference_path_length_m"])
        )
        <= 0.50,
        "same_controlled_drift_scale": abs(
            float(c_traj["raw_ate_rmse_m"]) - float(g_traj["raw_ate_rmse_m"])
        )
        <= 0.08,
        "ceres_reduces_ate": float(c_traj["corrected_ate_rmse_m"])
        < float(c_traj["raw_ate_rmse_m"]),
        "gtsam_reduces_ate": float(g_traj["corrected_ate_rmse_m"])
        < float(g_traj["raw_ate_rmse_m"]),
        # A/B 门禁检查工程可用性而非预设谁必胜；不同求解器允许合理数值差异。
        "gtsam_no_major_ate_regression": float(g_traj["corrected_ate_rmse_m"])
        <= float(c_traj["corrected_ate_rmse_m"]) * 1.5 + 0.03,
        "map_coverage_consistent": area_delta_ratio <= 0.20,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "ceres": {
            "elapsed_s": ceres["elapsed_s"],
            "raw_ate_rmse_m": c_traj["raw_ate_rmse_m"],
            "corrected_ate_rmse_m": c_traj["corrected_ate_rmse_m"],
            "raw_closure_error_m": c_traj["raw_closure_error_m"],
            "corrected_closure_error_m": c_traj["corrected_closure_error_m"],
            "known_area_m2": c_area,
        },
        "gtsam": {
            "elapsed_s": gtsam["elapsed_s"],
            "raw_ate_rmse_m": g_traj["raw_ate_rmse_m"],
            "corrected_ate_rmse_m": g_traj["corrected_ate_rmse_m"],
            "raw_closure_error_m": g_traj["raw_closure_error_m"],
            "corrected_closure_error_m": g_traj["corrected_closure_error_m"],
            "known_area_m2": g_area,
        },
        "delta": {
            "corrected_ate_gtsam_minus_ceres_m": round(
                float(g_traj["corrected_ate_rmse_m"])
                - float(c_traj["corrected_ate_rmse_m"]),
                4,
            ),
            "corrected_closure_gtsam_minus_ceres_m": round(
                float(g_traj["corrected_closure_error_m"])
                - float(c_traj["corrected_closure_error_m"]),
                4,
            ),
            "known_area_delta_ratio": round(area_delta_ratio, 4),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ceres", type=Path, default=Path("logs/slam_ceres_report.json"))
    parser.add_argument("--gtsam", type=Path, default=Path("logs/slam_gtsam_report.json"))
    parser.add_argument("--output", type=Path, default=Path("logs/slam_backend_comparison.json"))
    args = parser.parse_args()
    result = compare(
        json.loads(args.ceres.read_text(encoding="utf-8")),
        json.loads(args.gtsam.read_text(encoding="utf-8")),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("PASS: Ceres/GTSAM comparable backend evidence" if result["passed"] else "FAIL: backend comparison gate")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
