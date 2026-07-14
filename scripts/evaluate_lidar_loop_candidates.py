#!/usr/bin/env python3
"""Score C++ LiDAR loop candidates against offline OpenLORIS ground truth."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import statistics
from pathlib import Path
from typing import Sequence


def _load_evaluator():
    path = Path(__file__).with_name("evaluate_slam_trajectory.py")
    spec = importlib.util.spec_from_file_location("slam_trajectory_evaluator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load evaluator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVALUATOR = _load_evaluator()


def _summary(values: Sequence[float]) -> dict[str, float | None]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "p95": None}
    ordered = sorted(values)
    position = 0.95 * (len(ordered) - 1)
    lower, upper = math.floor(position), math.ceil(position)
    ratio = position - lower
    p95 = ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": p95,
    }


def load_candidate_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    seen_ids: set[int] = set()
    previous_stamp = -math.inf
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        row = json.loads(raw)
        query_id = int(row["query_id"])
        stamp = float(row["query_stamp_s"])
        if query_id in seen_ids:
            raise ValueError(f"duplicate query id at line {line_number}")
        if stamp < previous_stamp:
            raise ValueError("candidate query timestamps are not monotonic")
        candidates = list(row["candidates"])
        ranks = [int(candidate["rank"]) for candidate in candidates]
        if ranks != list(range(1, len(candidates) + 1)):
            raise ValueError(f"non-contiguous candidate ranks at line {line_number}")
        seen_ids.add(query_id)
        previous_stamp = stamp
        rows.append(row)
    if not rows:
        raise ValueError("candidate file is empty")
    return rows


def _score_ranking(
    rows: list[dict[str, object]],
    eligible: dict[int, set[int]],
    query_poses: dict[int, object],
    top_ks: Sequence[int],
    *,
    similarity_rerank: bool,
) -> dict[str, object]:
    hit_queries = {top_k: set() for top_k in top_ks}
    true_pairs = {top_k: 0 for top_k in top_ks}
    scored_pairs = {top_k: 0 for top_k in top_ks}
    unscored_pairs = {top_k: 0 for top_k in top_ks}
    reciprocal_ranks: list[float] = []
    positive_scores: list[float] = []
    negative_scores: list[float] = []
    max_top_k = max(top_ks)

    for row in rows:
        query_id = int(row["query_id"])
        positives = eligible.get(query_id, set())
        candidates = list(row["candidates"])
        if similarity_rerank:
            candidates.sort(
                key=lambda candidate: (
                    -float(candidate["similarity"]),
                    float(candidate["ring_key_distance"]),
                    int(candidate["scan_id"]),
                )
            )
        else:
            candidates.sort(
                key=lambda candidate: (
                    float(candidate["ring_key_distance"]),
                    -float(candidate["similarity"]),
                    int(candidate["scan_id"]),
                )
            )
        first_true_rank: int | None = None
        for rank, candidate in enumerate(candidates[:max_top_k], 1):
            candidate_id = int(candidate["scan_id"])
            if candidate_id not in query_poses:
                continue
            score = (
                float(candidate["similarity"])
                if similarity_rerank
                else -float(candidate["ring_key_distance"])
            )
            is_true = candidate_id in positives
            (positive_scores if is_true else negative_scores).append(score)
            if is_true and first_true_rank is None:
                first_true_rank = rank
        if positives:
            reciprocal_ranks.append(0.0 if first_true_rank is None else 1.0 / first_true_rank)
        for top_k in top_ks:
            selected = candidates[:top_k]
            scored = [
                candidate
                for candidate in selected
                if int(candidate["scan_id"]) in query_poses
            ]
            scored_pairs[top_k] += len(scored)
            unscored_pairs[top_k] += len(selected) - len(scored)
            count = sum(int(candidate["scan_id"]) in positives for candidate in scored)
            true_pairs[top_k] += count
            if positives and count > 0:
                hit_queries[top_k].add(query_id)

    eligible_count = len(eligible)
    metrics = {}
    for top_k in top_ks:
        metrics[f"at_{top_k}"] = {
            "query_recall": len(hit_queries[top_k]) / eligible_count if eligible_count else 0.0,
            "candidate_precision": (
                true_pairs[top_k] / scored_pairs[top_k] if scored_pairs[top_k] else 0.0
            ),
            "hit_queries": len(hit_queries[top_k]),
            "true_pairs": true_pairs[top_k],
            "scored_pairs": scored_pairs[top_k],
            "unscored_outside_groundtruth_pairs": unscored_pairs[top_k],
        }
    return {
        "top_k_metrics": metrics,
        "mean_reciprocal_rank": statistics.fmean(reciprocal_ranks)
        if reciprocal_ranks
        else 0.0,
        "positive_score": _summary(positive_scores),
        "negative_score": _summary(negative_scores),
        "hit_queries": hit_queries,
    }


def evaluate(
    rows: list[dict[str, object]],
    reference,
    *,
    sequence: str,
    corpus_metadata: dict[str, object],
    top_ks: Sequence[int],
    minimum_temporal_separation_s: float,
    revisit_radius_m: float,
    maximum_time_diff_s: float,
    event_gap_s: float,
) -> dict[str, object]:
    if not top_ks or min(top_ks) <= 0 or list(top_ks) != sorted(set(top_ks)):
        raise ValueError("top-k values must be unique positive ascending integers")
    if minimum_temporal_separation_s <= 0.0 or revisit_radius_m <= 0.0:
        raise ValueError("revisit thresholds must be positive")

    reference_stamps = [pose.stamp for pose in reference]
    query_poses: dict[int, object] = {}
    query_stamps: dict[int, float] = {}
    unassociated_query_ids: list[int] = []
    for row in rows:
        query_id = int(row["query_id"])
        stamp = float(row["query_stamp_s"])
        pose = EVALUATOR._interpolate(
            reference, reference_stamps, stamp, maximum_time_diff_s
        )
        if pose is None:
            unassociated_query_ids.append(query_id)
            continue
        query_poses[query_id] = pose
        query_stamps[query_id] = stamp

    eligible: dict[int, set[int]] = {}
    ordered_ids = [int(row["query_id"]) for row in rows if int(row["query_id"]) in query_poses]
    for current_index, query_id in enumerate(ordered_ids):
        current = query_poses[query_id]
        positives: set[int] = set()
        for candidate_id in ordered_ids[:current_index]:
            candidate = query_poses[candidate_id]
            if current.stamp - candidate.stamp < minimum_temporal_separation_s:
                continue
            if math.hypot(current.x - candidate.x, current.y - candidate.y) <= revisit_radius_m:
                positives.add(candidate_id)
        if positives:
            eligible[query_id] = positives

    max_top_k = max(top_ks)
    ring_key_retrieval = _score_ranking(
        rows, eligible, query_poses, top_ks, similarity_rerank=False
    )
    similarity_rerank = _score_ranking(
        rows, eligible, query_poses, top_ks, similarity_rerank=True
    )

    config = EVALUATOR.EvaluationConfig(
        loop_radius_m=revisit_radius_m,
        loop_yaw_tolerance_deg=180.0,
        loop_min_separation_s=minimum_temporal_separation_s,
        loop_sample_interval_s=1.0,
        loop_event_gap_s=event_gap_s,
    )
    loop_catalog = EVALUATOR._loop_metrics(reference, reference, config)
    events = []
    for event in loop_catalog["events"]:
        matching_queries = {
            query_id
            for query_id, stamp in query_stamps.items()
            if float(event["start_stamp_s"]) - 1.0
            <= stamp
            <= float(event["end_stamp_s"]) + 1.0
        }
        recovered = bool(matching_queries & ring_key_retrieval["hit_queries"][max_top_k])
        events.append(
            {
                "event_id": int(event["event_id"]),
                "start_stamp_s": float(event["start_stamp_s"]),
                "end_stamp_s": float(event["end_stamp_s"]),
                "candidate_query_count": len(matching_queries),
                "recovered_at_max_k": recovered,
            }
        )
    recovered_events = sum(event["recovered_at_max_k"] for event in events)
    eligible_count = len(eligible)
    primary_metrics = ring_key_retrieval["top_k_metrics"]
    rerank_metrics = similarity_rerank["top_k_metrics"]

    expected_nodes = int(corpus_metadata["corpus"]["nodes"])
    association_coverage = len(query_poses) / len(rows)
    checks = {
        "corpus_metadata_passed": bool(corpus_metadata.get("passed")),
        "candidate_rows_match_corpus": len(rows) == expected_nodes,
        "minimum_95pct_query_time_coverage": association_coverage >= 0.95,
        "minimum_100_associated_queries": len(query_poses) >= 100,
        "has_long_revisit_queries": eligible_count > 0,
        "has_ground_truth_events": len(events) > 0,
    }
    quality = {
        "recall_at_max_k_nonzero": primary_metrics[f"at_{max_top_k}"]["query_recall"]
        > 0.0,
        "at_least_one_event_recovered": recovered_events > 0,
    }
    return EVALUATOR._round_floats(
        {
            "schema_version": 1,
            "passed": all(checks.values()),
            "checks": checks,
            "quality_checks": quality,
            "sequence": sequence,
            "source": {
                "groundtruth_sha256": hashlib.sha256(
                    Path(corpus_metadata.get("groundtruth_path", "")).read_bytes()
                ).hexdigest()
                if corpus_metadata.get("groundtruth_path")
                else None,
                "corpus_sha256": corpus_metadata["corpus"]["sha256"],
                "graph_sha256": corpus_metadata["source_graph"]["sha256"],
                "bag_sha256": corpus_metadata["source_bag"]["sha256"],
            },
            "association": {
                "candidate_rows": len(rows),
                "associated_queries": len(query_poses),
                "query_time_coverage_ratio": association_coverage,
                "unassociated_query_ids": unassociated_query_ids,
            },
            "ground_truth": {
                "eligible_long_revisit_queries": eligible_count,
                "event_count": len(events),
                "minimum_temporal_separation_s": minimum_temporal_separation_s,
                "revisit_radius_m": revisit_radius_m,
                "heading_policy": "position_only_360_lidar",
            },
            "retrieval": {
                "ring_key_retrieval": {
                    key: value
                    for key, value in ring_key_retrieval.items()
                    if key != "hit_queries"
                },
                "similarity_rerank_ablation": {
                    key: value
                    for key, value in similarity_rerank.items()
                    if key != "hit_queries"
                },
                "comparison_at_max_k": {
                    "ring_key_recall_delta_vs_similarity_rerank": primary_metrics[
                        f"at_{max_top_k}"
                    ]["query_recall"]
                    - rerank_metrics[f"at_{max_top_k}"]["query_recall"],
                    "ring_key_precision_delta_vs_similarity_rerank": primary_metrics[
                        f"at_{max_top_k}"
                    ]["candidate_precision"]
                    - rerank_metrics[f"at_{max_top_k}"]["candidate_precision"],
                },
            },
            "event_recovery": {
                "recovered_events_at_max_k": recovered_events,
                "event_recall_at_max_k": recovered_events / len(events) if events else 0.0,
                "events": events,
            },
            "methodology": {
                "runtime_input": "raw LaserScan polar occupancy descriptor + timestamp only",
                "offline_label": "official trajectory position within radius after temporal exclusion",
                "candidate_stage": (
                    "yaw-invariant ring-key Top-K retrieval; circular-shift descriptor alignment "
                    "provides similarity and yaw initialization without overriding retrieval order"
                ),
                "boundary": (
                    "This evaluates candidate retrieval before scan matching. Ground truth never "
                    "enters the C++ index, and a retrieved candidate is not an accepted graph edge."
                ),
            },
        }
    )


def render_markdown(report: dict[str, object]) -> str:
    retrieval = report["retrieval"]
    ring_key = retrieval["ring_key_retrieval"]
    rerank = retrieval["similarity_rerank_ablation"]
    lines = [
        f"# {report['sequence']} LiDAR 回环候选检索",
        "",
        f"- 数据契约：**{'PASS' if report['passed'] else 'FAIL'}**",
        f"- 长回访 query：{report['ground_truth']['eligible_long_revisit_queries']}",
        f"- 事件：{report['ground_truth']['event_count']}",
        f"- 环键检索 MRR：{ring_key['mean_reciprocal_rank']:.4f}",
        f"- 完整相似度重排 MRR：{rerank['mean_reciprocal_rank']:.4f}",
        "",
        "| K | ring-key recall | similarity-rerank recall | ring-key precision | rerank precision |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ]
    for name, row in ring_key["top_k_metrics"].items():
        top_k = name.removeprefix("at_")
        baseline = rerank["top_k_metrics"][name]
        lines.append(
            f"| {top_k} | {row['query_recall']:.2%} | {baseline['query_recall']:.2%} | "
            f"{row['candidate_precision']:.2%} | {baseline['candidate_precision']:.2%} |"
        )
    event = report["event_recovery"]
    lines.extend(
        [
            "",
            f"- 最大 K 事件召回：{event['recovered_events_at_max_k']}/"
            f"{report['ground_truth']['event_count']} ({event['event_recall_at_max_k']:.2%})",
            "",
            "> 边界：候选检索发生在 scan matcher 之前；候选命中不等于约束被接受，真值只用于离线评分。",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--groundtruth", type=Path, required=True)
    parser.add_argument("--corpus-metadata", type=Path, required=True)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    parser.add_argument("--top-k", default="1,5,10")
    parser.add_argument("--minimum-temporal-separation", type=float, default=60.0)
    parser.add_argument("--revisit-radius", type=float, default=1.0)
    parser.add_argument("--maximum-time-diff", type=float, default=0.05)
    parser.add_argument("--event-gap", type=float, default=2.0)
    args = parser.parse_args()
    top_ks = [int(value) for value in args.top_k.split(",")]
    metadata = json.loads(args.corpus_metadata.read_text(encoding="utf-8"))
    metadata["groundtruth_path"] = str(args.groundtruth.resolve())
    report = evaluate(
        load_candidate_rows(args.candidates),
        EVALUATOR.load_trajectory(args.groundtruth),
        sequence=args.sequence,
        corpus_metadata=metadata,
        top_ks=top_ks,
        minimum_temporal_separation_s=args.minimum_temporal_separation,
        revisit_radius_m=args.revisit_radius,
        maximum_time_diff_s=args.maximum_time_diff,
        event_gap_s=args.event_gap,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: LiDAR loop-candidate evaluation")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
