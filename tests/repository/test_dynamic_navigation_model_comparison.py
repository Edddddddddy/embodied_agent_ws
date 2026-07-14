from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_dynamic_navigation_models",
    ROOT / "scripts" / "compare_dynamic_navigation_models.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def report(model: str, passed: bool = True) -> dict:
    return {
        "motion_model": model,
        "passed": passed,
        "checks": {
            "future_cell_marked_lethal": True,
            "dynamic_plan_increased_clearance": True,
            "navigate_to_pose_succeeded": True,
            "cmd_vel_returned_to_zero": True,
        },
        "track": {"velocity_y_mps": 0.4},
        "prediction": {"y": 0.2},
        "baseline_clearance_m": 0.01,
        "dynamic_clearance_m": 0.8,
        "odom_traveled_distance_m": 2.0,
        "elapsed_s": 45.0,
    }


def test_comparison_requires_all_models_and_closed_loop_checks() -> None:
    result = MODULE.compare([report(model) for model in MODULE.EXPECTED])
    assert result["passed"] is True
    assert [row["model"] for row in result["models"]] == list(MODULE.EXPECTED)


def test_comparison_preserves_failed_navigation_evidence() -> None:
    reports = [report(model) for model in MODULE.EXPECTED]
    reports[-1]["checks"]["cmd_vel_returned_to_zero"] = False
    reports[-1]["passed"] = False
    result = MODULE.compare(reports)
    assert result["passed"] is False
    assert any("cmd_vel_returned_to_zero" in error for error in result["errors"])
