from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_slam_evaluations", ROOT / "scripts" / "compare_slam_evaluations.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(ate: float, rpe: float, final: float) -> dict:
    return {
        "passed": True,
        "association": {"matched_poses": 100},
        "ate_xy_m": {"rmse": ate},
        "rpe": {"translation_m": {"rmse": rpe}},
        "closure": {"final_pose_error_m": final},
        "loop": {"recall": 1.0},
    }


def test_comparison_requires_global_local_and_final_drift_improvements():
    result = MODULE.compare(_report(0.5, 0.2, 0.8), _report(0.1, 0.05, 0.1))
    assert result["passed"] is True
    assert result["improvement"]["ate_rmse_ratio"] == 0.2


def test_comparison_rejects_local_accuracy_regression():
    result = MODULE.compare(_report(0.5, 0.2, 0.8), _report(0.1, 0.3, 0.1))
    assert result["passed"] is False
    assert result["checks"]["rpe_improved"] is False
