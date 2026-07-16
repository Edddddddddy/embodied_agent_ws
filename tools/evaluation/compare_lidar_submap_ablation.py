#!/usr/bin/env python3
"""Compare scan-to-scan and scan-to-submap shadow evidence on identical pairs."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path


def _mode(report: dict[str, object]) -> str:
    methodology = report.get("methodology", {})
    if isinstance(methodology, dict) and methodology.get("matching_mode"):
        return str(methodology["matching_mode"])
    diagnostics = report.get("diagnostics", {})
    if isinstance(diagnostics, dict) and diagnostics.get("matching_mode"):
        return str(diagnostics["matching_mode"])
    # b4ae450 以前的已发布报告都是单帧模式，兼容它们只为建立一次可追溯基线。
    return "scan_to_scan"


def _sampling(report: dict[str, object]) -> tuple[int, int]:
    methodology = report.get("methodology", {})
    if isinstance(methodology, dict):
        sampling = methodology.get("geometry_sampling", {})
        if isinstance(sampling, dict) and sampling:
            return (
                int(sampling["effective_point_stride_per_scan"]),
                int(sampling["matcher_minimum_points"]),
            )
    # 兼容一次旧版基线；正式 runner 会在同一 binary 上重算 A/B 两侧。
    return (2, 30)


def _index(values: list[tuple[str, dict[str, object]]]) -> dict[str, dict[str, object]]:
    indexed = dict(values)
    if len(indexed) != len(values):
        raise ValueError("sequence names must be unique")
    return indexed


def build_summary(
    baselines: list[tuple[str, dict[str, object]]],
    submaps: list[tuple[str, dict[str, object]]],
) -> dict[str, object]:
    baseline_by_sequence = _index(baselines)
    submap_by_sequence = _index(submaps)
    if len(baseline_by_sequence) < 2:
        raise ValueError("at least two sequence A/B pairs are required")
    if baseline_by_sequence.keys() != submap_by_sequence.keys():
        raise ValueError("baseline and submap sequence sets differ")

    rows: list[dict[str, object]] = []
    contract_checks: list[bool] = []
    for sequence in sorted(baseline_by_sequence):
        baseline = baseline_by_sequence[sequence]
        submap = submap_by_sequence[sequence]
        baseline_metrics = baseline["profiles"]["cpp_default"]
        submap_metrics = submap["profiles"]["cpp_default"]
        same_source = all(
            baseline["source"].get(key) == submap["source"].get(key)
            for key in ("groundtruth_sha256", "candidates_sha256", "corpus_metadata_sha256")
        )
        same_pairs = (
            int(baseline["association"]["scored_pairs"])
            == int(submap["association"]["scored_pairs"])
            and int(baseline["ground_truth"]["true_candidate_pairs"])
            == int(submap["ground_truth"]["true_candidate_pairs"])
        )
        same_thresholds = all(
            baseline["ground_truth"][key] == submap["ground_truth"][key]
            for key in ("revisit_radius_m", "minimum_temporal_separation_s")
        )
        same_sampling = _sampling(baseline) == _sampling(submap)
        contract_ok = (
            bool(baseline["passed"])
            and bool(submap["passed"])
            and baseline.get("sequence") == sequence
            and submap.get("sequence") == sequence
            and _mode(baseline) == "scan_to_scan"
            and _mode(submap) == "scan_to_submap"
            and same_sampling
            and same_source
            and same_pairs
            and same_thresholds
        )
        contract_checks.append(contract_ok)
        baseline_translation = baseline_metrics["relative_translation_error_m"]["median"]
        submap_translation = submap_metrics["relative_translation_error_m"]["median"]
        rows.append(
            {
                "sequence": sequence,
                "contract_passed": contract_ok,
                "contract": {
                    "same_source": same_source,
                    "same_pairs": same_pairs,
                    "same_thresholds": same_thresholds,
                    "same_effective_sampling": same_sampling,
                },
                "scored_pairs": int(submap["association"]["scored_pairs"]),
                "true_candidate_pairs": int(submap["ground_truth"]["true_candidate_pairs"]),
                "scan_to_scan": {
                    "precision": float(baseline_metrics["precision"]),
                    "conditional_pair_recall": float(
                        baseline_metrics["conditional_pair_recall"]
                    ),
                    "median_translation_error_m": baseline_translation,
                },
                "scan_to_submap": {
                    "precision": float(submap_metrics["precision"]),
                    "conditional_pair_recall": float(
                        submap_metrics["conditional_pair_recall"]
                    ),
                    "median_translation_error_m": submap_translation,
                },
                "delta": {
                    "precision": float(submap_metrics["precision"])
                    - float(baseline_metrics["precision"]),
                    "conditional_pair_recall": float(
                        submap_metrics["conditional_pair_recall"]
                    )
                    - float(baseline_metrics["conditional_pair_recall"]),
                    "median_translation_error_m": (
                        None
                        if baseline_translation is None or submap_translation is None
                        else float(submap_translation) - float(baseline_translation)
                    ),
                },
            }
        )

    # 子地图即使优于单帧，也必须逐序列满足绝对质量线，才允许进入下一轮受保护写图消融。
    guarded_ablation_ready = all(contract_checks) and all(
        row["scan_to_submap"]["precision"] >= 0.80
        and row["scan_to_submap"]["conditional_pair_recall"] >= 0.15
        and row["scan_to_submap"]["median_translation_error_m"] is not None
        and row["scan_to_submap"]["median_translation_error_m"] <= 0.50
        for row in rows
    )
    return {
        "schema_version": 1,
        "passed": all(contract_checks),
        "checks": {
            "minimum_two_sequences": len(rows) >= 2,
            "all_ab_contracts_passed": all(contract_checks),
        },
        "sequence_count": len(rows),
        "sequences": rows,
        "aggregate": {
            "mean_precision_delta": statistics.fmean(
                row["delta"]["precision"] for row in rows
            ),
            "mean_conditional_pair_recall_delta": statistics.fmean(
                row["delta"]["conditional_pair_recall"] for row in rows
            ),
        },
        "release_decision": {
            "guarded_graph_edge_ablation_ready": guarded_ablation_ready,
            "direct_graph_edge_insertion_enabled": False,
            "status": (
                "ready_for_guarded_edge_ablation"
                if guarded_ablation_ready
                else "shadow_only_submap_quality_insufficient"
            ),
            "reason": (
                "Relative A/B improvement is not an enablement criterion; every real sequence "
                "must meet fixed absolute precision, recall, and transform-error thresholds."
            ),
        },
        "claim_boundary": (
            "The same candidates, timestamps, labels, and matcher thresholds are used for both "
            "modes. Ground truth is offline-only and direct pose-graph insertion remains disabled."
        ),
    }


def render_markdown(report: dict[str, object]) -> str:
    lines = [
        "# OpenLORIS scan-to-submap A/B",
        "",
        f"- A/B 数据契约：**{'PASS' if report['passed'] else 'FAIL'}**",
        f"- 发布决策：**{report['release_decision']['status']}**",
        f"- 平均 precision 变化：{report['aggregate']['mean_precision_delta']:+.2%}",
        f"- 平均 conditional recall 变化：{report['aggregate']['mean_conditional_pair_recall_delta']:+.2%}",
        "",
        "| sequence | scan precision | submap precision | Δ precision | scan recall | submap recall | Δ median trans. error |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in report["sequences"]:
        error_delta = row["delta"]["median_translation_error_m"]
        lines.append(
            f"| {row['sequence']} | {row['scan_to_scan']['precision']:.2%} | "
            f"{row['scan_to_submap']['precision']:.2%} | {row['delta']['precision']:+.2%} | "
            f"{row['scan_to_scan']['conditional_pair_recall']:.2%} | "
            f"{row['scan_to_submap']['conditional_pair_recall']:.2%} | "
            f"{'n/a' if error_delta is None else f'{error_delta:+.3f} m'} |"
        )
    lines.extend(
        [
            "",
            "> 结论边界：相对改善不等于可上线；所有结果仍为 shadow evidence，图边写入关闭。",
            "",
        ]
    )
    return "\n".join(lines)


def _parse(values: list[str]) -> list[tuple[str, dict[str, object]]]:
    result = []
    for value in values:
        if "=" not in value:
            raise ValueError("report arguments must use sequence=path")
        sequence, path = value.split("=", 1)
        result.append((sequence, json.loads(Path(path).read_text(encoding="utf-8"))))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", action="append", required=True, help="sequence=path")
    parser.add_argument("--submap", action="append", required=True, help="sequence=path")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    args = parser.parse_args()
    report = build_summary(_parse(args.baseline), _parse(args.submap))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: scan-to-submap A/B")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
