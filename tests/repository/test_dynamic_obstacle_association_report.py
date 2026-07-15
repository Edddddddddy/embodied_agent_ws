from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "verify_dynamic_obstacle_association",
    ROOT / "scripts" / "verify_dynamic_obstacle_association.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid_report() -> dict:
    return {
        "schema_version": 1,
        "scenario": "two_track_conflicting_gate_v1",
        "association_distance_m": 0.5,
        "association_metric": "euclidean",
        "association_nis_gate": 9.21,
        "motion_model": "current_only",
        "strategies": [
            {
                "strategy": "greedy_nearest",
                "total_tracks": 3,
                "matched_existing_tracks": 1,
                "fragment_tracks": 1,
                "identity_position_rmse_m": 0.29,
                "update_time_us": 2.0,
            },
            {
                "strategy": "global_nearest",
                "total_tracks": 2,
                "matched_existing_tracks": 2,
                "fragment_tracks": 0,
                "identity_position_rmse_m": 0.0,
                "update_time_us": 4.0,
            },
        ],
    }


def test_accepts_order_independent_global_assignment_evidence() -> None:
    assert MODULE.validate_report(valid_report()) == []


def test_rejects_fragmented_or_non_improving_global_assignment() -> None:
    report = valid_report()
    report["strategies"][1]["fragment_tracks"] = 1
    report["strategies"][1]["identity_position_rmse_m"] = 0.3
    errors = MODULE.validate_report(report)
    assert any("preserve exactly two" in error for error in errors)
    assert any("reduce identity-position RMSE" in error for error in errors)
