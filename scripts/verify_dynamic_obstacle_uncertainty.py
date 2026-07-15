#!/usr/bin/env python3
"""Validate and render the covariance-aware association A/B report."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


EXPECTED_METRICS = {"euclidean", "mahalanobis"}


def validate_report(report: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if report.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if report.get("scenario") != "heteroscedastic_crossing_v1":
        errors.append("unexpected uncertainty association scenario")
    if report.get("association_strategy") != "global_nearest":
        errors.append("association strategy must remain global_nearest")
    if report.get("association_nis_gate") != 9.21:
        errors.append("NIS gate must remain the 2D 99% chi-square threshold")
    rows = report.get("metrics")
    if not isinstance(rows, list):
        return errors + ["metrics must be a list"]
    by_metric = {
        row.get("metric"): row
        for row in rows
        if isinstance(row, dict) and row.get("metric")
    }
    if set(by_metric) != EXPECTED_METRICS:
        return errors + [f"metrics must be exactly {sorted(EXPECTED_METRICS)}"]
    for metric, row in by_metric.items():
        for name in ("correct_identity_matches", "unmatched_tracks"):
            if not isinstance(row.get(name), int) or row[name] < 0:
                errors.append(f"{metric}: {name} must be a non-negative integer")
        for name in ("identity_position_rmse_m", "mean_assignment_time_us"):
            value = row.get(name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                errors.append(f"{metric}: {name} must be finite and non-negative")
    euclidean = by_metric["euclidean"]
    mahalanobis = by_metric["mahalanobis"]
    if euclidean["correct_identity_matches"] != 0:
        errors.append("Euclidean baseline must expose the fixed identity swap")
    if mahalanobis["correct_identity_matches"] != 2:
        errors.append("Mahalanobis association must recover both identities")
    if mahalanobis["unmatched_tracks"] != 0:
        errors.append("Mahalanobis association must keep both tracks matched")
    if mahalanobis["identity_position_rmse_m"] >= euclidean["identity_position_rmse_m"]:
        errors.append("Mahalanobis association must reduce identity-position RMSE")
    return errors


def render_markdown(report: dict[str, Any], errors: list[str]) -> str:
    lines = [
        "# Dynamic obstacle uncertainty-aware association",
        "",
        f"- Scenario: `{report.get('scenario', 'unknown')}`",
        f"- Status: **{'FAIL' if errors else 'PASS'}**",
        "- Evidence type: deterministic covariance-aware association A/B",
        "",
        "| Metric | Correct identities | Unmatched | Identity RMSE (m) | Assignment (us) |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for row in report.get("metrics", []):
        lines.append(
            "| {metric} | {correct_identity_matches} | {unmatched_tracks} | "
            "{identity_position_rmse_m:.4f} | {mean_assignment_time_us:.3f} |".format(
                **row
            )
        )
    if errors:
        lines.extend(["", "## Failures", ""] + [f"- {error}" for error in errors])
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "Euclidean matching treats confident and uncertain tracks equally. The NIS cost "
            "normalizes each innovation with `S = P_prediction + R_measurement`, so a confident "
            "track cannot cheaply jump to an inconsistent observation while an occluded, "
            "uncertain track keeps a wider statistically justified gate. A hard metre gate remains "
            "active even when covariance grows.",
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
    markdown = render_markdown(report, errors)
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(markdown, encoding="utf-8")
    print(markdown)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
