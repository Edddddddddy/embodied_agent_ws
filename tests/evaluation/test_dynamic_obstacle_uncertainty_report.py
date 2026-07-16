from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "verify_dynamic_obstacle_uncertainty",
    ROOT / "tools" / "evaluation" / "verify_dynamic_obstacle_uncertainty.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid_report() -> dict:
    return {
        "schema_version": 1,
        "scenario": "heteroscedastic_crossing_v1",
        "association_strategy": "global_nearest",
        "association_distance_m": 1.0,
        "association_nis_gate": 9.21,
        "measurement_noise_variance": 0.0025,
        "metrics": [
            {
                "metric": "euclidean",
                "correct_identity_matches": 0,
                "unmatched_tracks": 0,
                "identity_position_rmse_m": 0.3,
                "mean_assignment_time_us": 1.0,
            },
            {
                "metric": "mahalanobis",
                "correct_identity_matches": 2,
                "unmatched_tracks": 0,
                "identity_position_rmse_m": 0.0,
                "mean_assignment_time_us": 1.2,
            },
        ],
    }


def test_accepts_covariance_aware_identity_recovery() -> None:
    assert MODULE.validate_report(valid_report()) == []


def test_rejects_non_improving_or_unmatched_mahalanobis_result() -> None:
    report = valid_report()
    report["metrics"][1]["correct_identity_matches"] = 1
    report["metrics"][1]["unmatched_tracks"] = 1
    report["metrics"][1]["identity_position_rmse_m"] = 0.4
    errors = MODULE.validate_report(report)
    assert any("recover both identities" in error for error in errors)
    assert any("keep both tracks matched" in error for error in errors)
    assert any("reduce identity-position RMSE" in error for error in errors)
