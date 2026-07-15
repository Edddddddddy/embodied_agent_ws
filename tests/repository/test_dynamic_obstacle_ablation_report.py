from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "verify_dynamic_obstacle_ablation",
    ROOT / "scripts" / "verify_dynamic_obstacle_ablation.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid_report() -> dict:
    rows = []
    values = {
        "current_only": (0.06, 0.45, 0.34, 0.19, 0.04),
        "constant_velocity": (0.04, 0.21, 0.24, 0.065, 0.16),
        "kalman": (0.05, 0.25, 0.27, 0.16, 0.12),
        "imm": (0.038, 0.20, 0.22, 0.045, 0.039),
    }
    for model, metrics in values.items():
        rows.append(
            {
                "model": model,
                "samples": 91,
                "occlusion_samples": 6,
                "track_drop_count": 0,
                "position_rmse_m": metrics[0],
                "velocity_rmse_mps": metrics[1],
                "forecast_rmse_m": metrics[2],
                "occlusion_rmse_m": metrics[3],
                "stop_forecast_rmse_m": metrics[4],
                "mean_update_time_us": 1.0,
            }
        )
    return {
        "schema_version": 1,
        "scenario": "stop_turn_occlusion_v1",
        "sample_step_s": 0.1,
        "forecast_horizon_s": 0.75,
        "tracker_config": {
            "association_distance_m": 1.2,
            "association_strategy": "global_nearest",
            "association_metric": "euclidean",
            "association_nis_gate": 9.21,
            "track_timeout_s": 1.0,
            "velocity_smoothing": 0.5,
            "measurement_noise_variance": 0.0016,
            "process_noise_variance": 0.35,
            "imm_stationary_process_noise": 0.004,
            "imm_maneuver_process_noise": 1.2,
            "imm_stay_probability": 0.94,
            "imm_stationary_velocity_decay": 0.1,
        },
        "models": rows,
    }


def test_accepts_complete_comparative_evidence() -> None:
    assert MODULE.validate_report(valid_report()) == []


def test_rejects_track_drop_and_non_comparative_imm_result() -> None:
    report = valid_report()
    rows = {row["model"]: row for row in report["models"]}
    rows["kalman"]["track_drop_count"] = 1
    rows["imm"]["stop_forecast_rmse_m"] = 0.15
    errors = MODULE.validate_report(report)
    assert any("track_drop_count" in error for error in errors)
    assert any("stop overshoot" in error for error in errors)


def test_rejects_missing_model() -> None:
    report = valid_report()
    report["models"] = report["models"][:-1]
    assert "models must be exactly" in MODULE.validate_report(report)[0]
