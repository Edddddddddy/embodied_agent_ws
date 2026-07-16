#!/usr/bin/env python3
"""Aggregate fixed-graph scan-overlap ablations across independent sequences."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any


REQUIRED_VARIANTS = {
    "cauchy_baseline",
    "naive_overlap",
    "innovation_gate",
    "dual_evidence",
}


def _finite(value: object) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"metric must be finite, got {value!r}")
    return number


def _variant_map(report: dict[str, Any]) -> dict[str, dict[str, Any]]:
    variants = report.get("variants")
    if not isinstance(variants, list):
        raise ValueError("report variants must be a list")
    rows = {
        str(item.get("name")): item
        for item in variants
        if isinstance(item, dict) and item.get("name")
    }
    missing = REQUIRED_VARIANTS - rows.keys()
    if missing:
        raise ValueError(f"report is missing variants: {sorted(missing)}")
    return rows


def build_summary(
    sequence_reports: list[tuple[str, dict[str, Any]]],
) -> dict[str, Any]:
    """Build a cross-sequence decision report without hiding per-sequence regressions."""

    if not sequence_reports:
        raise ValueError("at least one sequence report is required")
    sequence_names = [name for name, _report in sequence_reports]
    graph_hashes = [str(report.get("graph_sha256", "")) for _, report in sequence_reports]
    thresholds: set[tuple[float, float]] = set()
    rows: list[dict[str, Any]] = []

    for sequence, report in sequence_reports:
        variants = _variant_map(report)
        baseline = variants["cauchy_baseline"]
        naive = variants["naive_overlap"]
        innovation = variants["innovation_gate"]
        dual = variants["dual_evidence"]
        overlap = _finite(dual["minimum_scan_overlap_ratio"])
        innovation_threshold = _finite(dual["minimum_translation_residual_m"])
        thresholds.add((overlap, innovation_threshold))

        baseline_ate = _finite(baseline["ate_rmse_m"])
        baseline_p95 = _finite(baseline["ate_p95_m"])
        innovation_ate = _finite(innovation["ate_rmse_m"])
        dual_ate = _finite(dual["ate_rmse_m"])
        dual_p95 = _finite(dual["ate_p95_m"])
        rows.append(
            {
                "sequence": sequence,
                "graph_sha256": str(report.get("graph_sha256", "")),
                "graph_nodes": int(report["graph_nodes"]),
                "graph_constraints": int(report["graph_constraints"]),
                "minimum_scan_overlap_ratio": overlap,
                "minimum_translation_innovation_m": innovation_threshold,
                "baseline_ate_rmse_m": baseline_ate,
                "innovation_gate_ate_rmse_m": innovation_ate,
                "dual_evidence_ate_rmse_m": dual_ate,
                "baseline_ate_p95_m": baseline_p95,
                "dual_evidence_ate_p95_m": dual_p95,
                "dual_ate_change_vs_baseline_pct": round(
                    100.0 * (dual_ate - baseline_ate) / max(baseline_ate, 1e-12), 6
                ),
                "dual_ate_change_vs_innovation_pct": round(
                    100.0
                    * (dual_ate - innovation_ate)
                    / max(innovation_ate, 1e-12),
                    6,
                ),
                "dual_p95_change_vs_baseline_pct": round(
                    100.0 * (dual_p95 - baseline_p95) / max(baseline_p95, 1e-12),
                    6,
                ),
                "naive_overlap_rejected_constraints": int(
                    naive["scan_overlap_rejected_constraints"]
                ),
                "dual_evidence_rejected_constraints": int(
                    dual["scan_overlap_rejected_constraints"]
                ),
                "overlap_evidence_unavailable": int(
                    report.get("scan_evidence", {}).get(
                        "unavailable_nonlocal_constraints",
                        dual.get("scan_overlap_unavailable_constraints", 0),
                    )
                ),
                "source_report_passed": bool(report.get("passed", False)),
            }
        )

    checks = {
        "at_least_two_sequences": len(sequence_reports) >= 2,
        "unique_sequence_names": len(set(sequence_names)) == len(sequence_names),
        "distinct_fixed_graphs": (
            len(set(graph_hashes)) == len(graph_hashes)
            and all(len(digest) == 64 for digest in graph_hashes)
        ),
        "same_gate_thresholds": len(thresholds) == 1,
        "all_source_reports_passed": all(row["source_report_passed"] for row in rows),
        "all_overlap_evidence_available": all(
            row["overlap_evidence_unavailable"] == 0 for row in rows
        ),
    }
    dual_improves_all_ate = all(
        row["dual_ate_change_vs_baseline_pct"] < 0.0 for row in rows
    )
    # P95 允许 2% 数值抖动，但不能用平均值掩盖某条序列的明显长尾退化。
    dual_p95_no_material_regression = all(
        row["dual_p95_change_vs_baseline_pct"] <= 2.0 for row in rows
    )
    conservative_vs_naive = all(
        row["dual_evidence_rejected_constraints"]
        < row["naive_overlap_rejected_constraints"]
        for row in rows
    )
    recommended_default_enabled = (
        all(checks.values())
        and dual_improves_all_ate
        and dual_p95_no_material_regression
        and conservative_vs_naive
    )
    decision = (
        "eligible_for_default_enable"
        if recommended_default_enabled
        else "keep_disabled_collect_more_sequences"
    )
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "sequence_count": len(rows),
        "sequences": rows,
        "aggregate": {
            "mean_dual_ate_change_vs_baseline_pct": round(
                mean(row["dual_ate_change_vs_baseline_pct"] for row in rows), 6
            ),
            "mean_dual_ate_change_vs_innovation_pct": round(
                mean(row["dual_ate_change_vs_innovation_pct"] for row in rows), 6
            ),
            "dual_improves_all_sequence_ate": dual_improves_all_ate,
            "dual_p95_no_material_regression": dual_p95_no_material_regression,
            "dual_rejects_fewer_edges_than_naive_all_sequences": conservative_vs_naive,
        },
        "release_decision": {
            "recommended_default_enabled": recommended_default_enabled,
            "status": decision,
            "policy": (
                "Enable only when every independent fixed graph improves ATE, no sequence "
                "regresses ATE P95 by more than 2%, and dual evidence rejects fewer edges "
                "than the naive overlap-only gate."
            ),
        },
        "claim_boundary": (
            "This report evaluates already accepted Karto constraints on independent fixed "
            "graphs. It uses ground truth only for offline trajectory scoring; runtime overlap "
            "and innovation gates remain ground-truth-free. It does not measure candidate-level "
            "loop-closure precision or recall."
        ),
    }


def render_markdown(report: dict[str, Any]) -> str:
    rows = "\n".join(
        "| {sequence} | {graph_nodes} | {graph_constraints} | "
        "{baseline_ate_rmse_m:.4f} | {innovation_gate_ate_rmse_m:.4f} | "
        "{dual_evidence_ate_rmse_m:.4f} | {dual_ate_change_vs_baseline_pct:+.2f}% | "
        "{dual_p95_change_vs_baseline_pct:+.2f}% | {naive_overlap_rejected_constraints} | "
        "{dual_evidence_rejected_constraints} |".format(**row)
        for row in report["sequences"]
    )
    decision = report["release_decision"]
    return f"""# GTSAM 扫描重叠门控多序列验证

- 独立固定图：{report['sequence_count']} 条
- 公平性检查：**{'PASS' if report['passed'] else 'FAIL'}**
- 默认启用决策：**{decision['status']}**
- 双证据 ATE 平均变化：{report['aggregate']['mean_dual_ate_change_vs_baseline_pct']:+.2f}%

| sequence | nodes | constraints | baseline ATE | innovation ATE | dual ATE | dual ATE change | dual P95 change | naive reject | dual reject |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
{rows}

> 决策规则：{decision['policy']}

> 证据边界：该报告比较 Karto 已接受约束的独立固定图，真值只用于离线轨迹评分；运行时
> 扫描重叠和创新量不读取真值。它不能替代候选级回环 precision/recall 评测。
"""


def _parse_input(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("report must use SEQUENCE=PATH")
    sequence, raw_path = value.split("=", 1)
    if not sequence or not raw_path:
        raise argparse.ArgumentTypeError("report must use non-empty SEQUENCE=PATH")
    return sequence, Path(raw_path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report", action="append", type=_parse_input, required=True, metavar="SEQUENCE=PATH"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    args = parser.parse_args()
    loaded: list[tuple[str, dict[str, Any]]] = []
    for sequence, path in args.report:
        if not path.is_file():
            parser.error(f"missing sequence report: {path}")
        loaded.append((sequence, json.loads(path.read_text(encoding="utf-8"))))
    report = build_summary(loaded)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: multi-sequence scan-overlap validation")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
