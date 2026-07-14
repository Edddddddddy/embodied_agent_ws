#!/usr/bin/env python3
"""Aggregate independent OpenLORIS LiDAR shadow scan-matching reports."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path


def build_summary(reports: list[tuple[str, dict[str, object]]]) -> dict[str, object]:
    if len(reports) < 2:
        raise ValueError("at least two sequence reports are required")
    names = [name for name, _ in reports]
    if len(set(names)) != len(names):
        raise ValueError("sequence names must be unique")

    rows: list[dict[str, object]] = []
    source_hashes: list[str] = []
    thresholds: list[tuple[float, float]] = []
    profile_sets: list[tuple[str, ...]] = []
    for name, report in reports:
        if report.get("sequence") != name:
            raise ValueError(f"sequence label mismatch for {name}")
        cpp = report["profiles"]["cpp_default"]
        events = report["event_recovery"]
        source_hashes.append(str(report["source"]["matches_sha256"]))
        thresholds.append(
            (
                float(report["ground_truth"]["minimum_temporal_separation_s"]),
                float(report["ground_truth"]["revisit_radius_m"]),
            )
        )
        profile_sets.append(tuple(sorted(report["profiles"])))
        rows.append(
            {
                "sequence": name,
                "source_report_passed": bool(report["passed"]),
                "scored_pairs": int(report["association"]["scored_pairs"]),
                "true_candidate_pairs": int(
                    report["ground_truth"]["true_candidate_pairs"]
                ),
                "accepted_pairs": int(cpp["accepted_pairs"]),
                "precision": float(cpp["precision"]),
                "conditional_pair_recall": float(cpp["conditional_pair_recall"]),
                "query_recall": float(cpp["eligible_query_recall"]),
                "median_translation_error_m": cpp["relative_translation_error_m"][
                    "median"
                ],
                "median_yaw_error_deg": cpp["relative_yaw_error_deg"]["median"],
                "recovered_events": int(events["recovered_events"]),
                "event_count": int(events["event_count"]),
            }
        )

    checks = {
        "minimum_two_sequences": len(rows) >= 2,
        "source_reports_passed": all(row["source_report_passed"] for row in rows),
        "distinct_match_outputs": len(set(source_hashes)) == len(source_hashes),
        "same_groundtruth_thresholds": len(set(thresholds)) == 1,
        "same_fixed_profiles": len(set(profile_sets)) == 1,
        "all_sequences_have_true_candidates": all(
            row["true_candidate_pairs"] > 0 for row in rows
        ),
    }
    # 只有所有独立序列同时达到保守质量线，下一阶段才允许做“受保护图边”消融。
    # 本阶段无论指标如何都不直接写图，避免用一次离线调参替代运行时安全证明。
    guarded_edge_ablation_ready = all(checks.values()) and all(
        row["precision"] >= 0.80
        and row["conditional_pair_recall"] >= 0.15
        and row["median_translation_error_m"] is not None
        and float(row["median_translation_error_m"]) <= 0.50
        for row in rows
    )
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "sequence_count": len(rows),
        "sequences": rows,
        "aggregate": {
            "mean_cpp_precision": statistics.fmean(
                float(row["precision"]) for row in rows
            ),
            "mean_conditional_pair_recall": statistics.fmean(
                float(row["conditional_pair_recall"]) for row in rows
            ),
            "recovered_events": sum(int(row["recovered_events"]) for row in rows),
            "event_count": sum(int(row["event_count"]) for row in rows),
        },
        "release_decision": {
            "guarded_graph_edge_ablation_ready": guarded_edge_ablation_ready,
            "direct_graph_edge_insertion_enabled": False,
            "status": (
                "ready_for_guarded_edge_ablation"
                if guarded_edge_ablation_ready
                else "shadow_only_improve_geometric_verification"
            ),
            "reason": (
                "All sequences must exceed the fixed precision, recall, and transform-error "
                "thresholds before a separate guarded pose-graph ablation is allowed."
            ),
        },
        "claim_boundary": (
            "C++ scan matching consumes candidate scans, descriptor yaw, and an optional "
            "odometry yaw prior. Official ground truth is offline-only; no result is inserted "
            "into Ceres/GTSAM in this stage."
        ),
    }


def render_markdown(report: dict[str, object]) -> str:
    lines = [
        "# OpenLORIS LiDAR 影子扫描匹配多序列验证",
        "",
        f"- 数据契约：**{'PASS' if report['passed'] else 'FAIL'}**",
        f"- 序列数：{report['sequence_count']}",
        f"- 发布决策：**{report['release_decision']['status']}**",
        f"- 平均 accepted precision：{report['aggregate']['mean_cpp_precision']:.2%}",
        f"- 平均 conditional recall：{report['aggregate']['mean_conditional_pair_recall']:.2%}",
        "",
        "| sequence | scored | true pair | accepted | precision | conditional recall | median trans. error | median yaw error |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in report["sequences"]:
        translation = row["median_translation_error_m"]
        yaw = row["median_yaw_error_deg"]
        lines.append(
            f"| {row['sequence']} | {row['scored_pairs']} | "
            f"{row['true_candidate_pairs']} | {row['accepted_pairs']} | "
            f"{row['precision']:.2%} | {row['conditional_pair_recall']:.2%} | "
            f"{'n/a' if translation is None else f'{translation:.3f} m'} | "
            f"{'n/a' if yaw is None else f'{yaw:.2f}°'} |"
        )
    lines.extend(
        [
            "",
            "> 结论边界：当前结果只作为 shadow evidence；直接图边写入始终关闭。",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="append", required=True, help="sequence=path")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    args = parser.parse_args()
    reports = []
    for value in args.report:
        if "=" not in value:
            parser.error("--report must use sequence=path")
        sequence, path = value.split("=", 1)
        reports.append((sequence, json.loads(Path(path).read_text(encoding="utf-8"))))
    summary = build_summary(reports)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(render_markdown(summary), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"{'PASS' if summary['passed'] else 'FAIL'}: multi-sequence shadow matching")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
