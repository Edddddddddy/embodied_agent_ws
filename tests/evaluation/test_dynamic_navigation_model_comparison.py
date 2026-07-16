from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_dynamic_navigation_models",
    ROOT / "tools" / "evaluation" / "compare_dynamic_navigation_models.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def report(model: str, passed: bool = True) -> dict:
    return {
        "motion_model": model,
        "passed": passed,
        "checks": {
            "tracker_model_behavior": True,
            "future_cell_marked_lethal": True,
            "dynamic_plan_increased_clearance": True,
            "navigate_to_pose_succeeded": True,
            "robot_moved_at_least_1m": True,
            "cmd_vel_returned_to_zero": True,
        },
        "provenance": {
            "scenario_id": "typed_crossing_v1",
            "scenario_sha256": "scenario-sha",
            "map_artifact_sha256": "map-sha",
            "nav2_params_sha256": "params-sha",
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


def test_comparison_rejects_different_scenario_or_map() -> None:
    reports = [report(model) for model in MODULE.EXPECTED]
    reports[1]["provenance"]["scenario_sha256"] = "different-scenario"
    reports[2]["provenance"]["map_artifact_sha256"] = "different-map"
    result = MODULE.compare(reports)
    assert result["passed"] is False
    assert any("scenario_sha256 differs" in error for error in result["errors"])
    assert any("map_artifact_sha256 differs" in error for error in result["errors"])


def test_comparison_requires_provenance_and_all_closed_loop_checks() -> None:
    reports = [report(model) for model in MODULE.EXPECTED]
    reports[0].pop("provenance")
    reports[-1]["checks"].pop("robot_moved_at_least_1m")
    result = MODULE.compare(reports)
    assert result["passed"] is False
    assert any("missing provenance" in error for error in result["errors"])
    assert any("robot_moved_at_least_1m" in error for error in result["errors"])


def test_markdown_exposes_hashes_metrics_and_evidence_boundary() -> None:
    result = MODULE.compare([report(model) for model in MODULE.EXPECTED])
    markdown = MODULE.render_markdown(result)
    assert "Scenario SHA256: `scenario-sha`" in markdown
    assert "| current_only |" in markdown
    assert "synthetic PoseArray" in markdown


def test_markdown_preserves_partial_failure_report() -> None:
    reports = [report(model) for model in MODULE.EXPECTED]
    reports[-1]["track"] = {}
    reports[-1]["prediction"] = {}
    markdown = MODULE.render_markdown(MODULE.compare(reports))
    assert "| imm | n/a | n/a |" in markdown
