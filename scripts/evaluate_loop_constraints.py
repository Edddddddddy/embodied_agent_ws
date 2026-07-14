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


def load_constraints(path: Path) -> list[dict[str, object]]:
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
        if item.get("constraint_kind") != "loop" or not item.get("accepted", False):
            continue
        constraints.append(
            {
                **item,
                "source_stamp_s": source_stamp,
                "target_stamp_s": target_stamp,
            }
        )
    return constraints


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
    for constraint in constraints:
        source = _nearest_pose(
            reference, stamps, float(constraint["source_stamp_s"]), timestamp_tolerance_s
        )
        target = _nearest_pose(
            reference, stamps, float(constraint["target_stamp_s"]), timestamp_tolerance_s
        )
        if source is None or target is None:
            unassociated += 1
            continue
        current, previous = (source, target) if source.stamp >= target.stamp else (target, source)
        separation_s = current.stamp - previous.stamp
        distance_m = math.hypot(current.x - previous.x, current.y - previous.y)
        yaw_delta_deg = math.degrees(
            abs(EVALUATOR._wrap_angle(current.yaw - previous.yaw))
        )
        is_true = (
            separation_s >= config.loop_min_separation_s
            and distance_m <= config.loop_radius_m
            and math.radians(yaw_delta_deg) <= config.loop_yaw_tolerance_rad
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
                "true_loop": is_true,
            }
        )

    true_constraints = [item for item in classified if item["true_loop"]]
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
        "all_constraints_time_associated": unassociated == 0,
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
            },
            "accepted_constraints": {
                "input_loop_constraints": len(constraints),
                "associated": accepted,
                "unassociated": unassociated,
                "true_positive": true_positive,
                "false_positive": false_positive,
                "precision": precision,
                "false_loop_rate": false_positive / accepted if accepted else None,
            },
            "event_recovery": {
                "recovered_events": len(recovered_event_ids),
                "event_recall": event_recall,
                "recovered_event_ids": sorted(recovered_event_ids),
                "match_tolerance_s": event_match_tolerance_s,
            },
            "constraints": classified,
            "methodology": {
                "scope": "accepted non-local pose-graph constraints",
                "precision": "accepted constraints consistent with ground-truth XY/yaw",
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
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timestamp-tolerance", type=float, default=0.05)
    parser.add_argument("--loop-radius", type=float, default=0.50)
    parser.add_argument("--loop-yaw-tolerance-deg", type=float, default=30.0)
    parser.add_argument("--loop-min-separation", type=float, default=10.0)
    parser.add_argument("--loop-sample-interval", type=float, default=1.0)
    parser.add_argument("--loop-event-gap", type=float, default=2.0)
    parser.add_argument("--event-match-tolerance", type=float, default=1.0)
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
    report = evaluate_constraints(
        EVALUATOR.load_trajectory(args.reference),
        load_constraints(args.constraints),
        config=config,
        timestamp_tolerance_s=args.timestamp_tolerance,
        event_match_tolerance_s=args.event_match_tolerance,
        min_precision=args.min_precision,
        min_event_recall=args.min_event_recall,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: loop-constraint evaluation")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
