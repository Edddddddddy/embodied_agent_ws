from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_slam_backends", ROOT / "scripts" / "compare_slam_backends.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(solver: str, corrected_ate: float, area: float = 23.5) -> dict:
    return {
        "passed": True,
        "solver": solver,
        "elapsed_s": 70.0,
        "trajectory": {
            "reference_path_length_m": 9.5,
            "raw_ate_rmse_m": 0.35,
            "corrected_ate_rmse_m": corrected_ate,
            "raw_closure_error_m": 0.75,
            "corrected_closure_error_m": 0.22,
        },
        "map": {"known_area_m2": area},
    }


def test_comparison_accepts_two_effective_backends_without_preselecting_a_winner():
    result = MODULE.compare(_report("ceres", 0.10), _report("gtsam", 0.11))
    assert result["passed"] is True
    assert result["checks"]["ceres_reduces_ate"] is True
    assert result["checks"]["gtsam_reduces_ate"] is True


def test_comparison_rejects_a_gtsam_regression_or_mismatched_scenario():
    result = MODULE.compare(_report("ceres", 0.10), _report("gtsam", 0.30, area=10.0))
    assert result["passed"] is False
    assert result["checks"]["gtsam_no_major_ate_regression"] is False
    assert result["checks"]["map_coverage_consistent"] is False
