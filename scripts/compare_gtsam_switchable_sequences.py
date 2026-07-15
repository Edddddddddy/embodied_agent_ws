#!/usr/bin/env python3
"""Aggregate GTSAM switchable-constraint fixed-graph ablations across sequences."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


REQUIRED_VARIANTS = {
    "gaussian",
    "cauchy_loop",
    "switchable_gaussian",
    "switchable_cauchy",
}


def _variants(report: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rows = {
        str(item.get("name")): item
        for item in report.get("variants", [])
        if isinstance(item, dict) and item.get("name")
    }
    missing = REQUIRED_VARIANTS - rows.keys()
    if missing:
        raise ValueError(f"switchable report is missing variants: {sorted(missing)}")
    return rows


def _weighted_rmse(rows: list[tuple[int, float]]) -> float:
    samples = sum(count for count, _ in rows)
    if samples <= 0:
        raise ValueError("matched pose count must be positive")
    return math.sqrt(sum(count * value * value for count, value in rows) / samples)


def compare(reports: list[tuple[str, dict[str, Any]]]) -> dict[str, Any]:
    if len(reports) < 2:
        raise ValueError("at least two independent sequence reports are required")

    sequence_rows: list[dict[str, Any]] = []
    graph_hashes: set[str] = set()
    switch_configs: set[tuple[float, float]] = set()
    gaussian_metrics: list[tuple[int, float]] = []
    cauchy_metrics: list[tuple[int, float]] = []
    switchable_metrics: list[tuple[int, float]] = []
    for sequence, report in reports:
        rows = _variants(report)
        gaussian = rows["gaussian"]
        cauchy = rows["cauchy_loop"]
        switchable = rows["switchable_cauchy"]
        matched = int(report["matched_poses_per_variant"])
        graph_hashes.add(str(report["graph_sha256"]))
        switch_configs.add(
            (
                float(switchable["switch_prior_sigma"]),
                float(switchable["switch_suppression_threshold"]),
            )
        )
        gaussian_ate = float(gaussian["ate_rmse_m"])
        cauchy_ate = float(cauchy["ate_rmse_m"])
        switchable_ate = float(switchable["ate_rmse_m"])
        gaussian_metrics.append((matched, gaussian_ate))
        cauchy_metrics.append((matched, cauchy_ate))
        switchable_metrics.append((matched, switchable_ate))
        sequence_rows.append(
            {
                "sequence": sequence,
                "graph_sha256": report["graph_sha256"],
                "nodes": int(report["graph_nodes"]),
                "constraints": int(report["graph_constraints"]),
                "matched_poses": matched,
                "gaussian_ate_rmse_m": gaussian_ate,
                "cauchy_loop_ate_rmse_m": cauchy_ate,
                "switchable_cauchy_ate_rmse_m": switchable_ate,
                "switchable_vs_gaussian_pct": round(
                    100.0 * (switchable_ate - gaussian_ate) / max(gaussian_ate, 1e-12), 6
                ),
                "switchable_vs_cauchy_pct": round(
                    100.0 * (switchable_ate - cauchy_ate) / max(cauchy_ate, 1e-12), 6
                ),
                "switchable_constraints": int(switchable["switchable_constraints"]),
                "switch_suppressed_constraints": int(
                    switchable["switch_suppressed_constraints"]
                ),
                "minimum_switch_value": float(switchable["minimum_switch_value"]),
                "mean_switch_value": float(switchable["mean_switch_value"]),
                "report_passed": bool(report["passed"]),
            }
        )

    threshold = next(iter(switch_configs))[1] if len(switch_configs) == 1 else 0.5
    checks = {
        "at_least_two_sequences": len(sequence_rows) >= 2,
        "independent_graph_snapshots": len(graph_hashes) == len(sequence_rows),
        "identical_switch_configuration": len(switch_configs) == 1,
        "all_source_reports_passed": all(row["report_passed"] for row in sequence_rows),
        "no_per_sequence_ate_regression_vs_cauchy": all(
            row["switchable_cauchy_ate_rmse_m"]
            <= row["cauchy_loop_ate_rmse_m"] + 1e-6
            for row in sequence_rows
        ),
        "observed_suppressed_loop": any(
            row["switch_suppressed_constraints"] > 0 for row in sequence_rows
        ),
        "observed_retained_loop": any(
            row["minimum_switch_value"] > threshold for row in sequence_rows
        ),
    }
    aggregate = {
        "matched_poses": sum(row["matched_poses"] for row in sequence_rows),
        "gaussian_ate_rmse_m": round(_weighted_rmse(gaussian_metrics), 6),
        "cauchy_loop_ate_rmse_m": round(_weighted_rmse(cauchy_metrics), 6),
        "switchable_cauchy_ate_rmse_m": round(_weighted_rmse(switchable_metrics), 6),
        "switchable_constraints": sum(
            row["switchable_constraints"] for row in sequence_rows
        ),
        "switch_suppressed_constraints": sum(
            row["switch_suppressed_constraints"] for row in sequence_rows
        ),
    }
    aggregate["switchable_vs_gaussian_pct"] = round(
        100.0
        * (aggregate["switchable_cauchy_ate_rmse_m"] - aggregate["gaussian_ate_rmse_m"])
        / max(aggregate["gaussian_ate_rmse_m"], 1e-12),
        6,
    )
    aggregate["switchable_vs_cauchy_pct"] = round(
        100.0
        * (aggregate["switchable_cauchy_ate_rmse_m"] - aggregate["cauchy_loop_ate_rmse_m"])
        / max(aggregate["cauchy_loop_ate_rmse_m"], 1e-12),
        6,
    )
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "switch_configuration": {
            "prior_sigma": next(iter(switch_configs))[0] if len(switch_configs) == 1 else None,
            "suppression_threshold": threshold if len(switch_configs) == 1 else None,
        },
        "sequences": sequence_rows,
        "aggregate": aggregate,
        "interpretation_boundary": (
            "Switch values are latent backend weights on already accepted loop constraints. "
            "They do not label front-end loop closures and do not prove map-quality improvement "
            "outside the evaluated fixed graph snapshots."
        ),
    }


def render_markdown(report: dict[str, Any]) -> str:
    rows = "\n".join(
        "| {sequence} | {nodes} | {constraints} | {gaussian_ate_rmse_m:.4f} | "
        "{cauchy_loop_ate_rmse_m:.4f} | {switchable_cauchy_ate_rmse_m:.4f} | "
        "{switchable_vs_gaussian_pct:+.2f}% | {switchable_vs_cauchy_pct:+.2f}% | "
        "{switch_suppressed_constraints}/{switchable_constraints} | "
        "{minimum_switch_value:.4f} |".format(**item)
        for item in report["sequences"]
    )
    aggregate = report["aggregate"]
    return f"""# GTSAM 可切换回环约束多序列消融

- 门禁：**{'PASS' if report['passed'] else 'FAIL'}**
- switch prior sigma：{report['switch_configuration']['prior_sigma']}
- suppression threshold：{report['switch_configuration']['suppression_threshold']}
- 加权 ATE RMSE：Gaussian {aggregate['gaussian_ate_rmse_m']:.4f} m / Cauchy {aggregate['cauchy_loop_ate_rmse_m']:.4f} m / Switchable+Cauchy {aggregate['switchable_cauchy_ate_rmse_m']:.4f} m
- Switchable+Cauchy 相对 Gaussian：{aggregate['switchable_vs_gaussian_pct']:+.2f}%
- Switchable+Cauchy 相对 Cauchy：{aggregate['switchable_vs_cauchy_pct']:+.2f}%
- 被压低回环：{aggregate['switch_suppressed_constraints']}/{aggregate['switchable_constraints']}

| sequence | nodes | constraints | Gaussian ATE | Cauchy ATE | Switch+Cauchy ATE | vs Gaussian | vs Cauchy | switch off/all | min switch |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
{rows}

> 结论边界：switch 是后端对“前端已接受边”的潜变量权重，不是真值标签；本报告不能证明
> 前端回环 precision 提升，也不能外推到未评测地图。在线配置仍默认关闭。
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report", action="append", nargs=2, metavar=("SEQUENCE", "JSON"), required=True
    )
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    args = parser.parse_args()
    reports = [
        (sequence, json.loads(Path(path).read_text(encoding="utf-8")))
        for sequence, path in args.report
    ]
    result = compare(reports)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_markdown.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    args.output_markdown.write_text(render_markdown(result), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"{'PASS' if result['passed'] else 'FAIL'}: multi-sequence switchable constraints")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
