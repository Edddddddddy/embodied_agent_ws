#!/usr/bin/env python3
"""Compare greedy single-track and multi-hypothesis LiDAR sequence gates."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _assignment(value: str) -> tuple[str, Path]:
    sequence, separator, raw_path = value.partition("=")
    if not separator or not sequence or not raw_path:
        raise argparse.ArgumentTypeError("report must use SEQUENCE=PATH")
    return sequence, Path(raw_path)


def _precision(profile: dict[str, object]) -> float:
    accepted = int(profile["accepted_pairs"])
    return int(profile["true_accepted_pairs"]) / accepted if accepted else 0.0


def compare(reports: list[tuple[str, Path]]) -> dict[str, object]:
    if len(reports) < 2:
        raise ValueError("sequence ablation requires at least two independent sequences")
    configurations: list[dict[str, object]] = []
    rows: list[dict[str, object]] = []
    totals = {
        "ranked": [0, 0],
        "single_track": [0, 0],
        "multi_hypothesis": [0, 0],
    }
    true_pairs = 0
    sources = []
    for sequence, path in reports:
        report = json.loads(path.read_text(encoding="utf-8"))
        if report.get("sequence") != sequence or not report.get("passed"):
            raise ValueError(f"invalid or failed sequence report: {sequence}")
        profiles = report["profiles"]
        ranked = profiles["cpp_ranked_single"]
        single = profiles["cpp_temporal"]
        multi = profiles["cpp_sequence_multi_hypothesis"]
        configuration = report["diagnostics"]["sequence_consistency"]["config"]
        configurations.append(configuration)
        for key, profile in (
            ("ranked", ranked),
            ("single_track", single),
            ("multi_hypothesis", multi),
        ):
            totals[key][0] += int(profile["accepted_pairs"])
            totals[key][1] += int(profile["true_accepted_pairs"])
        true_pairs += int(report["ground_truth"]["true_candidate_pairs"])
        rows.append(
            {
                "sequence": sequence,
                "ranked_single": ranked,
                "single_track": single,
                "multi_hypothesis": multi,
                "multi_precision_delta_vs_ranked": _precision(multi)
                - _precision(ranked),
                "multi_precision_delta_vs_single_track": _precision(multi)
                - _precision(single),
            }
        )
        sources.append(
            {"sequence": sequence, "report": path.name, "sha256": _sha256(path)}
        )
    if any(config != configurations[0] for config in configurations[1:]):
        raise ValueError("all sequence reports must use one fixed configuration")

    def aggregate(key: str) -> dict[str, float | int]:
        accepted, true = totals[key]
        return {
            "accepted_pairs": accepted,
            "true_accepted_pairs": true,
            "false_accepted_pairs": accepted - true,
            "micro_precision": true / accepted if accepted else 0.0,
            "conditional_pair_recall": true / true_pairs if true_pairs else 0.0,
        }

    ranked_total = aggregate("ranked")
    single_total = aggregate("single_track")
    multi_total = aggregate("multi_hypothesis")
    checks = {
        "at_least_two_sequences": len(rows) >= 2,
        "fixed_configuration": True,
        "precision_improved_vs_ranked_each_sequence": all(
            float(row["multi_precision_delta_vs_ranked"]) > 0.0 for row in rows
        ),
        "precision_improved_vs_single_track_each_sequence": all(
            float(row["multi_precision_delta_vs_single_track"]) > 0.0
            for row in rows
        ),
        "micro_precision_improved_vs_single_track": float(
            multi_total["micro_precision"]
        )
        > float(single_total["micro_precision"]),
        "false_acceptances_reduced_vs_single_track": int(
            multi_total["false_accepted_pairs"]
        )
        < int(single_total["false_accepted_pairs"]),
        "true_constraints_not_reduced_vs_single_track": int(
            multi_total["true_accepted_pairs"]
        )
        >= int(single_total["true_accepted_pairs"]),
    }
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "config": configurations[0],
        "aggregate": {
            "ranked_single": ranked_total,
            "single_track": single_total,
            "multi_hypothesis": multi_total,
            "precision_delta_vs_single_track": float(multi_total["micro_precision"])
            - float(single_total["micro_precision"]),
        },
        "sequences": rows,
        "sources": sources,
        "release_decision": {
            "direct_graph_edge_insertion_enabled": False,
            "status": "shadow_only_precision_improved",
            "reason": (
                "Cross-sequence precision improved, but absolute recall remains too low "
                "for unattended pose-graph insertion."
            ),
        },
        "boundary": (
            "The C++ multi-hypothesis gate consumes matcher outputs and timestamps only. "
            "OpenLORIS ground truth labels reports offline and never enters runtime decisions."
        ),
    }


def render_markdown(report: dict[str, object]) -> str:
    aggregate = report["aggregate"]
    lines = [
        "# Multi-sequence LiDAR multi-hypothesis sequence ablation",
        "",
        f"- 证据门禁：**{'PASS' if report['passed'] else 'FAIL'}**",
        f"- 固定参数：`{json.dumps(report['config'], ensure_ascii=False)}`",
        "",
        "| Sequence | Ranked precision | Single-track precision | Multi-hypothesis precision |",
        "| --- | ---: | ---: | ---: |",
    ]
    for row in report["sequences"]:
        lines.append(
            f"| {row['sequence']} | {_precision(row['ranked_single']):.2%} | "
            f"{_precision(row['single_track']):.2%} | "
            f"{_precision(row['multi_hypothesis']):.2%} |"
        )
    lines.extend(
        [
            "",
            f"- 聚合精度：{aggregate['single_track']['micro_precision']:.2%} → "
            f"{aggregate['multi_hypothesis']['micro_precision']:.2%}",
            f"- 假接受：{aggregate['single_track']['false_accepted_pairs']} → "
            f"{aggregate['multi_hypothesis']['false_accepted_pairs']}",
            f"- 保留真约束：{aggregate['single_track']['true_accepted_pairs']} → "
            f"{aggregate['multi_hypothesis']['true_accepted_pairs']}",
            "",
            "> 边界：仍为 shadow-only；低召回意味着尚不能开放无人值守图边写入。",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="append", type=_assignment, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.report)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    args.markdown.write_text(render_markdown(result), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"{'PASS' if result['passed'] else 'FAIL'}: multi-hypothesis sequence ablation")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
