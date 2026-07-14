#!/usr/bin/env python3
"""Evaluate shadow-only 2D LiDAR scan matching with offline OpenLORIS labels."""

from __future__ import annotations

import argparse
import collections
import hashlib
import importlib.util
import json
import math
import statistics
from pathlib import Path
from typing import Callable, Sequence


def _load_evaluator():
    path = Path(__file__).with_name("evaluate_slam_trajectory.py")
    spec = importlib.util.spec_from_file_location("slam_trajectory_evaluator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load evaluator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVALUATOR = _load_evaluator()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _summary(values: Sequence[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
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
        "max": ordered[-1],
    }


def load_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    seen: set[tuple[int, int]] = set()
    matching_modes: set[str] = set()
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        row = json.loads(raw)
        # 旧版单帧结果没有显式模式字段；读入时归一化，避免 A/B 报告靠文件名猜测。
        row.setdefault("matching_mode", "scan_to_scan")
        mode = str(row["matching_mode"])
        if mode not in {"scan_to_scan", "scan_to_submap"}:
            raise ValueError(f"unsupported matching_mode at line {line_number}: {mode}")
        matching_modes.add(mode)
        key = (int(row["query_id"]), int(row["candidate_id"]))
        if key in seen:
            raise ValueError(f"duplicate shadow pair at line {line_number}: {key}")
        seen.add(key)
        for field in (
            "query_stamp_s",
            "candidate_stamp_s",
            "inlier_ratio",
            "bidirectional_overlap_ratio",
            "rmse_m",
            "observability_ratio",
        ):
            if not math.isfinite(float(row[field])):
                raise ValueError(f"non-finite {field} at line {line_number}")
        rows.append(row)
    if not rows:
        raise ValueError("shadow match file is empty")
    if len(matching_modes) != 1:
        raise ValueError("one evidence file must contain exactly one matching_mode")
    return rows


def _relative_pose(query, candidate) -> tuple[float, float, float]:
    world_x = candidate.x - query.x
    world_y = candidate.y - query.y
    cosine = math.cos(query.yaw)
    sine = math.sin(query.yaw)
    return (
        cosine * world_x + sine * world_y,
        -sine * world_x + cosine * world_y,
        EVALUATOR._wrap_angle(candidate.yaw - query.yaw),
    )


def _profile_metrics(
    scored: list[dict[str, object]],
    eligible_query_ids: set[int],
    predicate: Callable[[dict[str, object]], bool],
) -> dict[str, object]:
    accepted = [row for row in scored if predicate(row)]
    true_pairs = [row for row in scored if bool(row["is_true_loop"])]
    true_accepted = [row for row in accepted if bool(row["is_true_loop"])]
    accepted_queries = {int(row["query_id"]) for row in true_accepted}
    translation_errors = [float(row["translation_error_m"]) for row in true_accepted]
    yaw_errors = [float(row["yaw_error_deg"]) for row in true_accepted]
    return {
        "accepted_pairs": len(accepted),
        "true_accepted_pairs": len(true_accepted),
        "false_accepted_pairs": len(accepted) - len(true_accepted),
        "precision": len(true_accepted) / len(accepted) if accepted else 0.0,
        "conditional_pair_recall": len(true_accepted) / len(true_pairs) if true_pairs else 0.0,
        "eligible_query_recall": len(accepted_queries & eligible_query_ids)
        / len(eligible_query_ids)
        if eligible_query_ids
        else 0.0,
        "relative_translation_error_m": _summary(translation_errors),
        "relative_yaw_error_deg": _summary(yaw_errors),
    }


def evaluate(
    rows: list[dict[str, object]],
    reference,
    *,
    sequence: str,
    maximum_time_diff_s: float,
    revisit_radius_m: float,
    minimum_temporal_separation_s: float,
    event_gap_s: float,
    minimum_pair_time_coverage: float = 0.85,
) -> dict[str, object]:
    if not 0.0 < minimum_pair_time_coverage <= 1.0:
        raise ValueError("minimum pair time coverage must be in (0, 1]")
    reference_stamps = [pose.stamp for pose in reference]
    pose_cache: dict[float, object | None] = {}

    def pose_at(stamp: float):
        if stamp not in pose_cache:
            pose_cache[stamp] = EVALUATOR._interpolate(
                reference, reference_stamps, stamp, maximum_time_diff_s
            )
        return pose_cache[stamp]

    scored: list[dict[str, object]] = []
    unassociated = 0
    eligible_query_ids: set[int] = set()
    true_candidate_queries: set[int] = set()
    for row in rows:
        query = pose_at(float(row["query_stamp_s"]))
        candidate = pose_at(float(row["candidate_stamp_s"]))
        if query is None or candidate is None:
            unassociated += 1
            continue
        separation = query.stamp - candidate.stamp
        distance = math.hypot(query.x - candidate.x, query.y - candidate.y)
        is_true = separation >= minimum_temporal_separation_s and distance <= revisit_radius_m
        query_id = int(row["query_id"])
        if separation >= minimum_temporal_separation_s:
            eligible_query_ids.add(query_id)
        if is_true:
            true_candidate_queries.add(query_id)
        expected_x, expected_y, expected_yaw = _relative_pose(query, candidate)
        transform = row["target_to_source"]
        estimate_x = float(transform["x_m"])
        estimate_y = float(transform["y_m"])
        estimate_yaw = float(transform["yaw_rad"])
        enriched = dict(row)
        enriched.update(
            {
                "is_true_loop": is_true,
                "groundtruth_distance_m": distance,
                "translation_error_m": math.hypot(
                    estimate_x - expected_x, estimate_y - expected_y
                ),
                "yaw_error_deg": math.degrees(
                    abs(EVALUATOR._wrap_angle(estimate_yaw - expected_yaw))
                ),
            }
        )
        scored.append(enriched)

    if not scored:
        raise ValueError("no shadow matches overlap ground-truth timestamps")

    profiles: dict[str, Callable[[dict[str, object]], bool]] = {
        "cpp_default": lambda row: bool(row["accepted"]),
        "balanced_shadow": lambda row: (
            bool(row["available"])
            and bool(row["converged"])
            and not bool(row.get("yaw_ambiguous", False))
            and bool(row.get("odometry_prior_consistent", True))
            and math.hypot(
                float(row["target_to_source"]["x_m"]),
                float(row["target_to_source"]["y_m"]),
            )
            <= 2.0
            and float(row["inlier_ratio"]) >= 0.35
            and float(row["bidirectional_overlap_ratio"]) >= 0.55
            and float(row["rmse_m"]) <= 0.18
            and float(row["observability_ratio"]) >= 0.005
        ),
        "conservative_shadow": lambda row: (
            bool(row["available"])
            and bool(row["converged"])
            and not bool(row.get("yaw_ambiguous", False))
            and bool(row.get("odometry_prior_consistent", True))
            and math.hypot(
                float(row["target_to_source"]["x_m"]),
                float(row["target_to_source"]["y_m"]),
            )
            <= 1.5
            and float(row["inlier_ratio"]) >= 0.45
            and float(row["bidirectional_overlap_ratio"]) >= 0.65
            and float(row["rmse_m"]) <= 0.14
            and float(row["observability_ratio"]) >= 0.01
        ),
    }
    profile_metrics = {
        name: _profile_metrics(scored, true_candidate_queries, predicate)
        for name, predicate in profiles.items()
    }

    config = EVALUATOR.EvaluationConfig(
        loop_radius_m=revisit_radius_m,
        loop_yaw_tolerance_deg=180.0,
        loop_min_separation_s=minimum_temporal_separation_s,
        loop_sample_interval_s=1.0,
        loop_event_gap_s=event_gap_s,
    )
    loop_catalog = EVALUATOR._loop_metrics(reference, reference, config)
    balanced_queries = {
        int(row["query_id"])
        for row in scored
        if bool(row["is_true_loop"]) and profiles["balanced_shadow"](row)
    }
    query_stamp_by_id = {
        int(row["query_id"]): float(row["query_stamp_s"]) for row in scored
    }
    events = []
    for event in loop_catalog["events"]:
        event_queries = {
            query_id
            for query_id, stamp in query_stamp_by_id.items()
            if float(event["start_stamp_s"]) - 1.0
            <= stamp
            <= float(event["end_stamp_s"]) + 1.0
        }
        events.append(
            {
                "event_id": int(event["event_id"]),
                "candidate_query_count": len(event_queries),
                "recovered_by_balanced_shadow": bool(event_queries & balanced_queries),
            }
        )

    associated_ratio = len(scored) / len(rows)
    checks = {
        "minimum_pair_time_coverage": associated_ratio >= minimum_pair_time_coverage,
        "has_scored_pairs": len(scored) > 0,
        "has_true_candidate_pairs": any(bool(row["is_true_loop"]) for row in scored),
        "has_groundtruth_revisit_events": len(events) > 0,
    }
    reason_counts = collections.Counter(str(row["rejection_reason"]) for row in scored)
    matching_mode = str(scored[0].get("matching_mode", "scan_to_scan"))
    # 旧版报告使用 matcher 默认值 2/30；新结果会把等效采样步长写入每一行。
    effective_point_stride = int(scored[0].get("effective_point_stride", 2))
    matcher_minimum_points = int(scored[0].get("matcher_minimum_points", 30))
    return EVALUATOR._round_floats(
        {
            "schema_version": 1,
            "passed": all(checks.values()),
            "checks": checks,
            "sequence": sequence,
            "association": {
                "input_pairs": len(rows),
                "scored_pairs": len(scored),
                "pair_time_coverage_ratio": associated_ratio,
                "unassociated_pairs": unassociated,
                "required_pair_time_coverage_ratio": minimum_pair_time_coverage,
            },
            "ground_truth": {
                "true_candidate_pairs": sum(bool(row["is_true_loop"]) for row in scored),
                "true_candidate_queries": len(true_candidate_queries),
                "candidate_queries": len(eligible_query_ids),
                "revisit_radius_m": revisit_radius_m,
                "minimum_temporal_separation_s": minimum_temporal_separation_s,
            },
            "profiles": profile_metrics,
            "event_recovery": {
                "events": events,
                "recovered_events": sum(
                    bool(event["recovered_by_balanced_shadow"]) for event in events
                ),
                "event_count": len(events),
            },
            "diagnostics": {
                "matching_mode": matching_mode,
                "query_submap_scans": _summary(
                    [float(row.get("query_submap_scans", 1)) for row in scored]
                ),
                "candidate_submap_scans": _summary(
                    [float(row.get("candidate_submap_scans", 1)) for row in scored]
                ),
                "query_geometry_points": _summary(
                    [float(row.get("query_geometry_points", 0)) for row in scored]
                ),
                "candidate_geometry_points": _summary(
                    [float(row.get("candidate_geometry_points", 0)) for row in scored]
                ),
                "available_pairs": sum(bool(row["available"]) for row in scored),
                "converged_pairs": sum(bool(row["converged"]) for row in scored),
                "yaw_ambiguous_pairs": sum(
                    bool(row.get("yaw_ambiguous", False)) for row in scored
                ),
                "odometry_prior_pairs": sum(
                    bool(row.get("odometry_prior_used", False)) for row in scored
                ),
                "odometry_prior_inconsistent_pairs": sum(
                    bool(row.get("odometry_prior_used", False))
                    and not bool(row.get("odometry_prior_consistent", False))
                    for row in scored
                ),
                "positive_prior_consistent_pairs": sum(
                    bool(row.get("odometry_prior_consistent", False))
                    for row in scored
                    if row["is_true_loop"]
                ),
                "negative_prior_consistent_pairs": sum(
                    bool(row.get("odometry_prior_consistent", False))
                    for row in scored
                    if not row["is_true_loop"]
                ),
                "positive_prior_yaw_error_deg": _summary(
                    [
                        math.degrees(float(row["odometry_prior_yaw_error_rad"]))
                        for row in scored
                        if row["is_true_loop"]
                    ]
                ),
                "negative_prior_yaw_error_deg": _summary(
                    [
                        math.degrees(float(row["odometry_prior_yaw_error_rad"]))
                        for row in scored
                        if not row["is_true_loop"]
                    ]
                ),
                "cpp_rejection_reasons": dict(sorted(reason_counts.items())),
                "positive_inlier_ratio": _summary(
                    [float(row["inlier_ratio"]) for row in scored if row["is_true_loop"]]
                ),
                "negative_inlier_ratio": _summary(
                    [float(row["inlier_ratio"]) for row in scored if not row["is_true_loop"]]
                ),
                "positive_overlap_ratio": _summary(
                    [
                        float(row["bidirectional_overlap_ratio"])
                        for row in scored
                        if row["is_true_loop"]
                    ]
                ),
                "negative_overlap_ratio": _summary(
                    [
                        float(row["bidirectional_overlap_ratio"])
                        for row in scored
                        if not row["is_true_loop"]
                    ]
                ),
            },
            "methodology": {
                "matching_mode": matching_mode,
                "geometry_sampling": {
                    "effective_point_stride_per_scan": effective_point_stride,
                    "matcher_minimum_points": matcher_minimum_points,
                },
                "runtime_input": (
                    "candidate scan/submap pair + descriptor yaw seed + optional odometry yaw prior"
                ),
                "matcher": (
                    "C++17 trimmed coarse-to-fine point-to-point ICP with multi-yaw seeds"
                ),
                "offline_label": "official OpenLORIS trajectory; never passed to C++ matcher",
                "boundary": (
                    "Shadow results are scored but never inserted into the pose graph. "
                    "Profile sweeps are evidence, not runtime GT-assisted selection."
                ),
            },
        }
    )


def render_markdown(report: dict[str, object]) -> str:
    lines = [
        f"# {report['sequence']} LiDAR shadow scan matching",
        "",
        f"- 数据契约：**{'PASS' if report['passed'] else 'FAIL'}**",
        f"- 已评分 pair：{report['association']['scored_pairs']}",
        f"- 真回环候选 pair：{report['ground_truth']['true_candidate_pairs']}",
        "",
        "| Profile | Accepted | Precision | Conditional pair recall | Query recall | Median translation error |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for name, metrics in report["profiles"].items():
        median_error = metrics["relative_translation_error_m"]["median"]
        error_text = "n/a" if median_error is None else f"{median_error:.3f} m"
        lines.append(
            f"| {name} | {metrics['accepted_pairs']} | {metrics['precision']:.2%} | "
            f"{metrics['conditional_pair_recall']:.2%} | "
            f"{metrics['eligible_query_recall']:.2%} | {error_text} |"
        )
    events = report["event_recovery"]
    lines.extend(
        [
            "",
            f"- balanced shadow 事件恢复：{events['recovered_events']}/{events['event_count']}",
            "",
            "> 边界：配准结果只在 shadow 报告中评分，没有向 Ceres/GTSAM 位姿图写入边；真值仅用于离线标注。",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matches", type=Path, required=True)
    parser.add_argument("--groundtruth", type=Path, required=True)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    parser.add_argument("--candidates", type=Path)
    parser.add_argument("--corpus-metadata", type=Path)
    parser.add_argument("--maximum-time-diff", type=float, default=0.05)
    parser.add_argument("--revisit-radius", type=float, default=1.0)
    parser.add_argument("--minimum-temporal-separation", type=float, default=60.0)
    parser.add_argument("--event-gap", type=float, default=2.0)
    parser.add_argument("--minimum-pair-time-coverage", type=float, default=0.85)
    args = parser.parse_args()
    report = evaluate(
        load_rows(args.matches),
        EVALUATOR.load_trajectory(args.groundtruth),
        sequence=args.sequence,
        maximum_time_diff_s=args.maximum_time_diff,
        revisit_radius_m=args.revisit_radius,
        minimum_temporal_separation_s=args.minimum_temporal_separation,
        event_gap_s=args.event_gap,
        minimum_pair_time_coverage=args.minimum_pair_time_coverage,
    )
    report["source"] = {
        "matches_sha256": _sha256(args.matches),
        "groundtruth_sha256": _sha256(args.groundtruth),
        "candidates_sha256": _sha256(args.candidates) if args.candidates else None,
        "corpus_metadata_sha256": _sha256(args.corpus_metadata)
        if args.corpus_metadata
        else None,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: LiDAR shadow scan-match evaluation")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
