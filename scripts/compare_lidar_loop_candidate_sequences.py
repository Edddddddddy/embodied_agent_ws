#!/usr/bin/env python3
"""Aggregate independent OpenLORIS LiDAR loop-candidate retrieval reports."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path


def build_summary(reports: list[tuple[str, dict[str, object]]]) -> dict[str, object]:
    if len(reports) < 2:
        raise ValueError("at least two sequence reports are required")
    names = [name for name, _report in reports]
    if len(set(names)) != len(names):
        raise ValueError("sequence names must be unique")

    rows = []
    corpus_hashes: list[str] = []
    graph_hashes: list[str] = []
    thresholds: list[tuple[float, float]] = []
    source_reports_passed = True
    quality_checks_passed = True
    for name, report in reports:
        if report.get("sequence") != name:
            raise ValueError(f"sequence label mismatch for {name}")
        source_reports_passed &= bool(report.get("passed"))
        quality_checks_passed &= all(report.get("quality_checks", {}).values())
        corpus_hashes.append(str(report["source"]["corpus_sha256"]))
        graph_hashes.append(str(report["source"]["graph_sha256"]))
        thresholds.append(
            (
                float(report["ground_truth"]["minimum_temporal_separation_s"]),
                float(report["ground_truth"]["revisit_radius_m"]),
            )
        )
        primary = report["retrieval"]["ring_key_retrieval"]
        rerank = report["retrieval"]["similarity_rerank_ablation"]
        primary_at_10 = primary["top_k_metrics"]["at_10"]
        rerank_at_10 = rerank["top_k_metrics"]["at_10"]
        rows.append(
            {
                "sequence": name,
                "scan_rows": int(report["association"]["candidate_rows"]),
                "eligible_queries": int(
                    report["ground_truth"]["eligible_long_revisit_queries"]
                ),
                "ring_key_recall_at_10": float(primary_at_10["query_recall"]),
                "ring_key_precision_at_10": float(
                    primary_at_10["candidate_precision"]
                ),
                "similarity_rerank_recall_at_10": float(
                    rerank_at_10["query_recall"]
                ),
                "ring_key_mrr": float(primary["mean_reciprocal_rank"]),
                "event_recall_at_10": float(
                    report["event_recovery"]["event_recall_at_max_k"]
                ),
            }
        )

    checks = {
        "minimum_two_sequences": len(rows) >= 2,
        "source_reports_passed": source_reports_passed,
        "source_quality_checks_passed": quality_checks_passed,
        "distinct_scan_corpora": len(set(corpus_hashes)) == len(corpus_hashes),
        "distinct_fixed_graphs": len(set(graph_hashes)) == len(graph_hashes),
        "same_groundtruth_thresholds": len(set(thresholds)) == 1,
        "ring_key_beats_similarity_rerank_every_sequence": all(
            row["ring_key_recall_at_10"]
            > row["similarity_rerank_recall_at_10"]
            for row in rows
        ),
    }
    shadow_ready = (
        all(checks.values())
        and all(row["ring_key_recall_at_10"] >= 0.30 for row in rows)
        and all(row["event_recall_at_10"] >= 1.0 for row in rows)
    )
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "sequence_count": len(rows),
        "sequences": rows,
        "aggregate": {
            "mean_ring_key_recall_at_10": statistics.fmean(
                row["ring_key_recall_at_10"] for row in rows
            ),
            "mean_ring_key_precision_at_10": statistics.fmean(
                row["ring_key_precision_at_10"] for row in rows
            ),
            "mean_ring_key_mrr": statistics.fmean(row["ring_key_mrr"] for row in rows),
            "all_events_recovered_at_10": all(
                row["event_recall_at_10"] >= 1.0 for row in rows
            ),
        },
        "release_decision": {
            "shadow_scan_match_integration_ready": shadow_ready,
            "direct_graph_edge_insertion_enabled": False,
            "status": (
                "ready_for_shadow_scan_match_integration"
                if shadow_ready
                else "offline_only_collect_more_evidence"
            ),
            "reason": (
                "Candidate recall is sufficient for shadow-mode scan-matcher evaluation, but "
                "candidate precision is not an accepted-edge precision measurement."
            ),
        },
        "claim_boundary": (
            "The C++ retriever uses LaserScan and timestamps only. Official ground truth labels "
            "offline candidates; no candidate is inserted into the pose graph in this stage."
        ),
    }


def render_markdown(report: dict[str, object]) -> str:
    lines = [
        "# OpenLORIS LiDAR 回环候选多序列验证",
        "",
        f"- 数据与公平性检查：**{'PASS' if report['passed'] else 'FAIL'}**",
        f"- 序列数：{report['sequence_count']}",
        f"- 决策：**{report['release_decision']['status']}**",
        f"- 平均 Recall@10：{report['aggregate']['mean_ring_key_recall_at_10']:.2%}",
        f"- 平均 Precision@10：{report['aggregate']['mean_ring_key_precision_at_10']:.2%}",
        "",
        "| sequence | scans | eligible query | ring-key R@10 | rerank R@10 | P@10 | event recall |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in report["sequences"]:
        lines.append(
            f"| {row['sequence']} | {row['scan_rows']} | {row['eligible_queries']} | "
            f"{row['ring_key_recall_at_10']:.2%} | "
            f"{row['similarity_rerank_recall_at_10']:.2%} | "
            f"{row['ring_key_precision_at_10']:.2%} | {row['event_recall_at_10']:.2%} |"
        )
    lines.extend(
        [
            "",
            "> 结论边界：当前只允许进入 shadow scan-matcher 集成，不允许把候选直接写入位姿图。",
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
    print(f"{'PASS' if summary['passed'] else 'FAIL'}: multi-sequence loop retrieval")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
