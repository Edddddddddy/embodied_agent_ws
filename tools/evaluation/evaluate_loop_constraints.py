#!/usr/bin/env python3
"""Evaluate accepted pose-graph loop constraints against timestamped ground truth."""

from __future__ import annotations

import argparse
import bisect
import importlib.util
import json
import math
from pathlib import Path
from typing import Sequence


def _load_trajectory_evaluator():
    path = Path(__file__).with_name("evaluate_slam_trajectory.py")
    spec = importlib.util.spec_from_file_location("slam_trajectory_evaluator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load evaluator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVALUATOR = _load_trajectory_evaluator()


def load_constraints(
    path: Path, *, include_all_accepted: bool = False
) -> list[dict[str, object]]:
    """Load the append-only JSONL emitted by the instrumented GTSAM ScanSolver."""

    constraints: list[dict[str, object]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            item = json.loads(raw)
            source_stamp = float(item["source_stamp_s"])
            target_stamp = float(item["target_stamp_s"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            raise ValueError(f"{path}:{line_number}: invalid loop constraint") from error
        if not math.isfinite(source_stamp) or not math.isfinite(target_stamp):
            raise ValueError(f"{path}:{line_number}: non-finite timestamp")
        if not item.get("accepted", False):
            continue
        if not include_all_accepted and item.get("constraint_kind") != "loop":
            continue
        constraints.append(
            {
                **item,
                "source_stamp_s": source_stamp,
                "target_stamp_s": target_stamp,
            }
        )
    return constraints


def load_frontend_closure_scan_ids(path: Path) -> set[int]:
    """Load Karto-confirmed closure scan ids from the structured frontend trace."""

    scan_ids: set[int] = set()
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            item = json.loads(raw)
        except json.JSONDecodeError as error:
            raise ValueError(f"{path}:{line_number}: invalid frontend event") from error
        if item.get("event") == "end_closure":
            try:
                scan_ids.add(int(item["scan_index"]))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    f"{path}:{line_number}: closure event lacks scan_index"
                ) from error
    return scan_ids


def select_frontend_confirmed_constraints(
    constraints: Sequence[dict[str, object]], closure_scan_ids: set[int]
) -> list[dict[str, object]]:
    """Select backend edges created by native Karto closure callbacks.

    A node-id gap alone cannot distinguish Karto's local graph links from loop
    closures.  The current scan is the greater monotonically allocated node id, so
    intersecting it with ``end_closure.scan_index`` recovers the actual closure edge.
    """

    selected: list[dict[str, object]] = []
    for constraint in constraints:
        try:
            current_id = max(
                int(constraint["source_id"]), int(constraint["target_id"])
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("accepted constraint lacks integer node ids") from error
        id_separation = abs(
            int(constraint["source_id"]) - int(constraint["target_id"])
        )
        # 每次加入新节点都会有一条相邻边；closure callback 同一时刻额外加入非相邻边。
        if current_id in closure_scan_ids and id_separation > 1:
            selected.append(constraint)
    return selected


def _nearest_pose(reference: Sequence, stamps: Sequence[float], stamp: float, tolerance_s: float):
    right = bisect.bisect_left(stamps, stamp)
    candidates = [index for index in (right - 1, right) if 0 <= index < len(reference)]
    if not candidates:
        return None
    index = min(candidates, key=lambda item: abs(stamps[item] - stamp))
    return reference[index] if abs(stamps[index] - stamp) <= tolerance_s else None


def evaluate_constraints(
    reference: Sequence,
    constraints: Sequence[dict[str, object]],
    *,
    config,
    timestamp_tolerance_s: float = 0.05,
    event_match_tolerance_s: float = 1.0,
    max_translation_residual_m: float = 0.75,
    max_yaw_residual_deg: float = 15.0,
    min_precision: float | None = None,
    min_event_recall: float | None = None,
) -> dict[str, object]:
    """Classify accepted loop edges and match them to ground-truth revisit events.

    Precision uses every accepted non-local graph edge. Recall uses time-clustered true
    revisit events, so changing the ground-truth sampling rate does not inflate the count.
    """

    stamps = [pose.stamp for pose in reference]
    truth = EVALUATOR._loop_metrics(reference, reference, config)
    classified: list[dict[str, object]] = []
    unassociated = 0
    outside_reference_coverage = 0
    reference_start = stamps[0]
    reference_end = stamps[-1]
    for constraint in constraints:
        source_stamp = float(constraint["source_stamp_s"])
        target_stamp = float(constraint["target_stamp_s"])
        # OpenLORIS 的真值可能晚于传感器包开始或早于其结束。覆盖区间外的边不可评价，
        # 但不能误报为时间关联器失败；区间内找不到近邻才属于数据契约错误。
        if (
            source_stamp < reference_start - timestamp_tolerance_s
            or source_stamp > reference_end + timestamp_tolerance_s
            or target_stamp < reference_start - timestamp_tolerance_s
            or target_stamp > reference_end + timestamp_tolerance_s
        ):
            outside_reference_coverage += 1
            continue
        source = _nearest_pose(
            reference, stamps, source_stamp, timestamp_tolerance_s
        )
        target = _nearest_pose(
            reference, stamps, target_stamp, timestamp_tolerance_s
        )
        if source is None or target is None:
            unassociated += 1
            continue
        try:
            measured_x = float(constraint["relative_x_m"])
            measured_y = float(constraint["relative_y_m"])
            measured_yaw = float(constraint["relative_yaw_rad"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("accepted constraint lacks a relative pose") from error
        # ScanSolver 约束表达 target 在 source 坐标系中的位姿。正确性应比较相对位姿残差，
        # 不能用端点是否落在 1 m 内替代；相距数米但扫描有重叠仍可能是合法闭环边。
        world_dx = target.x - source.x
        world_dy = target.y - source.y
        expected_x = math.cos(source.yaw) * world_dx + math.sin(source.yaw) * world_dy
        expected_y = -math.sin(source.yaw) * world_dx + math.cos(source.yaw) * world_dy
        expected_yaw = EVALUATOR._wrap_angle(target.yaw - source.yaw)
        translation_residual_m = math.hypot(
            measured_x - expected_x, measured_y - expected_y
        )
        yaw_residual_deg = math.degrees(
            abs(EVALUATOR._wrap_angle(measured_yaw - expected_yaw))
        )
        is_valid = (
            translation_residual_m <= max_translation_residual_m
            and yaw_residual_deg <= max_yaw_residual_deg
        )
        current, previous = (source, target) if source.stamp >= target.stamp else (target, source)
        separation_s = current.stamp - previous.stamp
        distance_m = math.hypot(current.x - previous.x, current.y - previous.y)
        yaw_delta_deg = math.degrees(
            abs(EVALUATOR._wrap_angle(current.yaw - previous.yaw))
        )
        classified.append(
            {
                "source_id": constraint.get("source_id"),
                "target_id": constraint.get("target_id"),
                "current_stamp_s": current.stamp,
                "previous_stamp_s": previous.stamp,
                "temporal_separation_s": separation_s,
                "ground_truth_distance_m": distance_m,
                "ground_truth_yaw_delta_deg": yaw_delta_deg,
                "measured_relative_pose": {
                    "x_m": measured_x,
                    "y_m": measured_y,
                    "yaw_deg": math.degrees(measured_yaw),
                },
                "ground_truth_relative_pose": {
                    "x_m": expected_x,
                    "y_m": expected_y,
                    "yaw_deg": math.degrees(expected_yaw),
                },
                "translation_residual_m": translation_residual_m,
                "yaw_residual_deg": yaw_residual_deg,
                "valid_constraint": is_valid,
                "true_loop": is_valid,
            }
        )

    true_constraints = [item for item in classified if item["valid_constraint"]]
    recovered_event_ids: set[int] = set()
    for item in true_constraints:
        current_stamp = float(item["current_stamp_s"])
        matching = [
            event
            for event in truth["events"]
            if float(event["start_stamp_s"]) - event_match_tolerance_s
            <= current_stamp
            <= float(event["end_stamp_s"]) + event_match_tolerance_s
        ]
        if matching:
            nearest = min(
                matching,
                key=lambda event: abs(
                    current_stamp
                    - 0.5
                    * (float(event["start_stamp_s"]) + float(event["end_stamp_s"]))
                ),
            )
            recovered_event_ids.add(int(nearest["event_id"]))

    accepted = len(classified)
    true_positive = len(true_constraints)
    false_positive = accepted - true_positive
    event_count = int(truth["event_count"])
    precision = true_positive / accepted if accepted else None
    event_recall = len(recovered_event_ids) / event_count if event_count else None
    checks = {
        # “真实回访存在但前端一个闭环都没接收”是 recall=0 的有效实验结果，
        # 不能因为结果难看而让报告缺失；只有显式门槛才把它判为性能失败。
        "has_ground_truth_revisit_events": event_count > 0,
        "all_in_coverage_constraints_time_associated": unassociated == 0,
    }
    if min_precision is not None:
        checks["precision_within_limit"] = precision is not None and precision >= min_precision
    if min_event_recall is not None:
        checks["event_recall_within_limit"] = (
            event_recall is not None and event_recall >= min_event_recall
        )
    return EVALUATOR._round_floats(
        {
            "schema_version": 1,
            "passed": all(checks.values()),
            "checks": checks,
            "ground_truth": {
                "revisit_events": event_count,
                "opportunity_samples": truth["opportunities"],
                "event_gap_s": truth["event_gap_s"],
                "radius_m": truth["radius_m"],
                "yaw_tolerance_deg": truth["yaw_tolerance_deg"],
                "heading_policy": truth["heading_policy"],
            },
            "accepted_constraints": {
                "input_loop_constraints": len(constraints),
                "associated": accepted,
                "unassociated": unassociated,
                "outside_reference_coverage": outside_reference_coverage,
                "true_positive": true_positive,
                "false_positive": false_positive,
                "precision": precision,
                "false_loop_rate": false_positive / accepted if accepted else None,
                "maximum_translation_residual_m": max_translation_residual_m,
                "maximum_yaw_residual_deg": max_yaw_residual_deg,
            },
            "event_recovery": {
                "recovered_events": len(recovered_event_ids),
                "event_recall": event_recall,
                "recovered_event_ids": sorted(recovered_event_ids),
                "match_tolerance_s": event_match_tolerance_s,
            },
            "constraints": classified,
            "methodology": {
                "scope": "frontend-confirmed accepted pose-graph closure constraints",
                "precision": "accepted constraints whose relative-pose residual is within declared translation/yaw limits",
                "recall": "time-clustered ground-truth revisit events with a true accepted edge",
                "boundary": (
                    "the ScanSolver API exposes accepted edges, not every rejected scan-matcher "
                    "candidate; rejected-candidate calibration is out of scope"
                ),
            },
        }
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--constraints", type=Path, required=True)
    parser.add_argument("--frontend-trace", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timestamp-tolerance", type=float, default=0.05)
    parser.add_argument("--loop-radius", type=float, default=0.50)
    parser.add_argument("--loop-yaw-tolerance-deg", type=float, default=30.0)
    parser.add_argument("--loop-min-separation", type=float, default=10.0)
    parser.add_argument("--loop-sample-interval", type=float, default=1.0)
    parser.add_argument("--loop-event-gap", type=float, default=2.0)
    parser.add_argument("--event-match-tolerance", type=float, default=1.0)
    parser.add_argument("--max-translation-residual", type=float, default=0.75)
    parser.add_argument("--max-yaw-residual-deg", type=float, default=15.0)
    parser.add_argument("--min-precision", type=float)
    parser.add_argument("--min-event-recall", type=float)
    args = parser.parse_args()
    config = EVALUATOR.EvaluationConfig(
        loop_radius_m=args.loop_radius,
        loop_yaw_tolerance_deg=args.loop_yaw_tolerance_deg,
        loop_min_separation_s=args.loop_min_separation,
        loop_sample_interval_s=args.loop_sample_interval,
        loop_event_gap_s=args.loop_event_gap,
    )
    all_constraints = load_constraints(
        args.constraints, include_all_accepted=args.frontend_trace is not None
    )
    if args.frontend_trace is not None:
        closure_scan_ids = load_frontend_closure_scan_ids(args.frontend_trace)
        constraints = select_frontend_confirmed_constraints(
            all_constraints, closure_scan_ids
        )
    else:
        closure_scan_ids = set()
        constraints = all_constraints
    report = evaluate_constraints(
        EVALUATOR.load_trajectory(args.reference),
        constraints,
        config=config,
        timestamp_tolerance_s=args.timestamp_tolerance,
        event_match_tolerance_s=args.event_match_tolerance,
        max_translation_residual_m=args.max_translation_residual,
        max_yaw_residual_deg=args.max_yaw_residual_deg,
        min_precision=args.min_precision,
        min_event_recall=args.min_event_recall,
    )
    report["constraint_selection"] = {
        "method": (
            "frontend_end_closure_scan_id"
            if args.frontend_trace is not None
            else "backend_id_separation_heuristic"
        ),
        "accepted_backend_edges": len(all_constraints),
        "frontend_closure_scan_ids": sorted(closure_scan_ids),
        "selected_closure_edges": len(constraints),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: loop-constraint evaluation")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
