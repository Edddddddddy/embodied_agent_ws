#!/usr/bin/env python3
"""Validate the C++ dynamic-obstacle model ablation report.

The benchmark deliberately runs the same measurements through every model.  This
verifier keeps structural/invariant failures separate from comparative claims so a
future model cannot pass merely by emitting a well-formed JSON file.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


EXPECTED_MODELS = {"current_only", "constant_velocity", "kalman", "imm"}
METRICS = (
    "position_rmse_m",
    "velocity_rmse_mps",
    "forecast_rmse_m",
    "occlusion_rmse_m",
    "stop_forecast_rmse_m",
    "mean_update_time_us",
)


def validate_report(report: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if report.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if report.get("scenario") != "stop_turn_occlusion_v1":
        errors.append("unexpected scenario")
    if report.get("sample_step_s") != 0.1 or report.get("forecast_horizon_s") != 0.75:
        errors.append("fixed scenario timing contract is missing")
    required_config = {
        "association_distance_m",
        "track_timeout_s",
        "velocity_smoothing",
        "measurement_noise_variance",
        "process_noise_variance",
        "imm_stationary_process_noise",
        "imm_maneuver_process_noise",
        "imm_stay_probability",
        "imm_stationary_velocity_decay",
    }
    if set(report.get("tracker_config", {})) != required_config:
        errors.append("tracker_config must record every ablation parameter")

    rows = report.get("models")
    if not isinstance(rows, list):
        return errors + ["models must be a list"]
    by_model = {
        row.get("model"): row for row in rows if isinstance(row, dict) and row.get("model")
    }
    if set(by_model) != EXPECTED_MODELS:
        errors.append(f"models must be exactly {sorted(EXPECTED_MODELS)}")
        return errors

    sample_counts = set()
    for model, row in by_model.items():
        samples = row.get("samples")
        sample_counts.add(samples)
        if not isinstance(samples, int) or samples < 80:
            errors.append(f"{model}: samples must be >= 80")
        if not isinstance(row.get("occlusion_samples"), int) or row["occlusion_samples"] < 5:
            errors.append(f"{model}: occlusion_samples must be >= 5")
        if row.get("track_drop_count") != 0:
            errors.append(f"{model}: track_drop_count must be zero")
        for metric in METRICS:
            value = row.get(metric)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0.0:
                errors.append(f"{model}: {metric} must be finite and non-negative")
    if len(sample_counts) != 1:
        errors.append("all models must be evaluated on the same number of samples")

    current = by_model["current_only"]
    cv = by_model["constant_velocity"]
    imm = by_model["imm"]
    # 这些门槛验证算法区别，而不是宣称一个模型在所有场景都占优。
    if cv["forecast_rmse_m"] >= 0.9 * current["forecast_rmse_m"]:
        errors.append("constant_velocity must improve forecast RMSE over current_only by >= 10%")
    if imm["forecast_rmse_m"] >= cv["forecast_rmse_m"]:
        errors.append("IMM must improve forecast RMSE over constant_velocity in this maneuver scenario")
    if imm["occlusion_rmse_m"] >= cv["occlusion_rmse_m"]:
        errors.append("IMM must improve short-occlusion RMSE over constant_velocity")
    if imm["stop_forecast_rmse_m"] >= 0.5 * cv["stop_forecast_rmse_m"]:
        errors.append("IMM must reduce stop overshoot by at least 50% versus constant_velocity")
    return errors


def render_markdown(report: dict[str, Any], passed: bool, errors: list[str]) -> str:
    lines = [
        "# Dynamic obstacle motion-model ablation",
        "",
        f"- Scenario: `{report.get('scenario', 'unknown')}`",
        f"- Status: **{'PASS' if passed else 'FAIL'}**",
        "- Evidence type: deterministic C++ tracker benchmark (not Gazebo navigation evidence)",
        "",
        "| Model | Position RMSE (m) | Velocity RMSE (m/s) | 0.75 s forecast RMSE (m) | Occlusion RMSE (m) | Stop forecast RMSE (m) | Mean update (us) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in report.get("models", []):
        lines.append(
            "| {model} | {position_rmse_m:.4f} | {velocity_rmse_mps:.4f} | "
            "{forecast_rmse_m:.4f} | {occlusion_rmse_m:.4f} | "
            "{stop_forecast_rmse_m:.4f} | {mean_update_time_us:.3f} |".format(**row)
        )
    if errors:
        lines.extend(["", "## Failures", ""] + [f"- {error}" for error in errors])
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "`current_only` does not extrapolate. `constant_velocity` improves motion prediction but overshoots after a stop. "
            "Kalman-CV filters noisy measurements. IMM interacts a low-motion and a maneuver model, then updates model "
            "probabilities from measurement likelihoods; its advantage here is specific to the fixed stop/turn/occlusion scenario.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    parser.add_argument("--markdown", type=Path)
    args = parser.parse_args()

    report = json.loads(args.report.read_text(encoding="utf-8"))
    errors = validate_report(report)
    markdown = render_markdown(report, not errors, errors)
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(markdown, encoding="utf-8")
    print(markdown)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
