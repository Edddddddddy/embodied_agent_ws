#!/usr/bin/env python3
"""Validate and render the deterministic C++ association A/B report."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


EXPECTED_STRATEGIES = {"greedy_nearest", "global_nearest"}


def validate_report(report: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if report.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if report.get("scenario") != "two_track_conflicting_gate_v1":
        errors.append("unexpected association scenario")
    if report.get("association_distance_m") != 0.5:
        errors.append("association gate must remain fixed at 0.5 m")
    if report.get("association_metric") != "euclidean":
        errors.append("strategy benchmark must isolate assignment with euclidean cost")
    if report.get("association_nis_gate") != 9.21:
        errors.append("association benchmark must record the fixed NIS gate")
    rows = report.get("strategies")
    if not isinstance(rows, list):
        return errors + ["strategies must be a list"]
    by_strategy = {
        row.get("strategy"): row
        for row in rows
        if isinstance(row, dict) and row.get("strategy")
    }
    if set(by_strategy) != EXPECTED_STRATEGIES:
        return errors + [f"strategies must be exactly {sorted(EXPECTED_STRATEGIES)}"]
    for strategy, row in by_strategy.items():
        for metric in ("total_tracks", "matched_existing_tracks", "fragment_tracks"):
            if not isinstance(row.get(metric), int) or row[metric] < 0:
                errors.append(f"{strategy}: {metric} must be a non-negative integer")
        for metric in ("identity_position_rmse_m", "update_time_us"):
            value = row.get(metric)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0.0:
                errors.append(f"{strategy}: {metric} must be finite and non-negative")
    greedy = by_strategy["greedy_nearest"]
    global_nearest = by_strategy["global_nearest"]
    if greedy["fragment_tracks"] < 1:
        errors.append("greedy baseline must expose at least one fragmented track")
    if global_nearest["fragment_tracks"] != 0 or global_nearest["total_tracks"] != 2:
        errors.append("global assignment must preserve exactly two tracks")
    if global_nearest["matched_existing_tracks"] != 2:
        errors.append("global assignment must update both existing identities")
    if global_nearest["identity_position_rmse_m"] >= greedy["identity_position_rmse_m"]:
        errors.append("global assignment must reduce identity-position RMSE")
    return errors


def render_markdown(report: dict[str, Any], errors: list[str]) -> str:
    lines = [
        "# Dynamic obstacle association ablation",
        "",
        f"- Scenario: `{report.get('scenario', 'unknown')}`",
        f"- Status: **{'FAIL' if errors else 'PASS'}**",
        "- Evidence type: deterministic C++ association conflict (not perception accuracy)",
        "",
        "| Strategy | Tracks | Existing tracks updated | Fragments | Identity RMSE (m) | Update (us) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in report.get("strategies", []):
        lines.append(
            "| {strategy} | {total_tracks} | {matched_existing_tracks} | "
            "{fragment_tracks} | {identity_position_rmse_m:.4f} | {update_time_us:.3f} |".format(
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
            "`greedy_nearest` depends on track iteration order. `global_nearest` adds one private "
            "unmatched dummy per track and solves the gated rectangular assignment with the "
            "Hungarian algorithm, so one ambiguous track cannot consume another track's only "
            "feasible observation.",
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
