#!/usr/bin/env python3
"""Evaluate a planar SLAM trajectory against timestamped ground truth.

The public interface is deliberately small: ``load_trajectory`` reads the
TUM/OpenLORIS text format and ``evaluate`` returns one serializable report.
Time association, interpolation, SE(2) alignment and all metrics remain
inside this module so rosbag adapters and tests do not duplicate math.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
from pathlib import Path
from typing import NamedTuple, Sequence


class Pose2(NamedTuple):
    stamp: float
    x: float
    y: float
    yaw: float


class EvaluationConfig:
    def __init__(
        self,
        *,
        max_time_diff_s: float = 0.05,
        rpe_delta_s: float = 1.0,
        segment_window_s: float = 10.0,
        loop_radius_m: float = 0.50,
        loop_yaw_tolerance_deg: float = 30.0,
        loop_min_separation_s: float = 10.0,
        loop_sample_interval_s: float = 1.0,
        loop_event_gap_s: float = 2.0,
        loop_recovery_tolerance_m: float = 0.50,
        min_match_ratio: float = 0.80,
        max_ate_rmse_m: float | None = None,
        max_rpe_translation_rmse_m: float | None = None,
    ) -> None:
        if max_time_diff_s <= 0 or rpe_delta_s <= 0:
            raise ValueError("time tolerances must be positive")
        if loop_sample_interval_s <= 0 or loop_event_gap_s <= 0:
            raise ValueError("loop sampling and event gap must be positive")
        if not 0.0 <= min_match_ratio <= 1.0:
            raise ValueError("min_match_ratio must be in [0, 1]")
        self.max_time_diff_s = max_time_diff_s
        self.rpe_delta_s = rpe_delta_s
        self.segment_window_s = segment_window_s
        self.loop_radius_m = loop_radius_m
        self.loop_yaw_tolerance_rad = math.radians(loop_yaw_tolerance_deg)
        self.loop_min_separation_s = loop_min_separation_s
        self.loop_sample_interval_s = loop_sample_interval_s
        self.loop_event_gap_s = loop_event_gap_s
        self.loop_recovery_tolerance_m = loop_recovery_tolerance_m
        self.min_match_ratio = min_match_ratio
        self.max_ate_rmse_m = max_ate_rmse_m
        self.max_rpe_translation_rmse_m = max_rpe_translation_rmse_m


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def load_trajectory(path: Path) -> list[Pose2]:
    """Read ``timestamp tx ty tz qx qy qz qw`` and return sorted planar poses."""

    by_stamp: dict[float, Pose2] = {}
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 columns, got {len(fields)}")
        try:
            stamp, x, y, _z, qx, qy, qz, qw = map(float, fields)
        except ValueError as exc:
            raise ValueError(f"{path}:{line_number}: non-numeric trajectory row") from exc
        values = (stamp, x, y, qx, qy, qz, qw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path}:{line_number}: non-finite value")
        by_stamp[stamp] = Pose2(stamp, x, y, _yaw_from_quaternion(qx, qy, qz, qw))
    poses = [by_stamp[stamp] for stamp in sorted(by_stamp)]
    if len(poses) < 3:
        raise ValueError(f"{path}: at least 3 poses are required")
    return poses


def _interpolate(
    reference: Sequence[Pose2], stamps: Sequence[float], stamp: float, max_diff: float
) -> Pose2 | None:
    right = bisect.bisect_left(stamps, stamp)
    if right < len(reference) and abs(reference[right].stamp - stamp) <= 1e-9:
        pose = reference[right]
        return Pose2(stamp, pose.x, pose.y, pose.yaw)
    if right == 0 or right == len(reference):
        nearest = reference[0 if right == 0 else -1]
        if abs(nearest.stamp - stamp) <= max_diff:
            return Pose2(stamp, nearest.x, nearest.y, nearest.yaw)
        return None
    left_pose, right_pose = reference[right - 1], reference[right]
    # 大片缺帧时禁止跨洞插值；否则漂亮的 ATE 会掩盖传感器时间轴故障。
    if right_pose.stamp - left_pose.stamp > 2.0 * max_diff:
        return None
    ratio = (stamp - left_pose.stamp) / (right_pose.stamp - left_pose.stamp)
    yaw_delta = _wrap_angle(right_pose.yaw - left_pose.yaw)
    return Pose2(
        stamp,
        left_pose.x + ratio * (right_pose.x - left_pose.x),
        left_pose.y + ratio * (right_pose.y - left_pose.y),
        _wrap_angle(left_pose.yaw + ratio * yaw_delta),
    )


def associate(
    reference: Sequence[Pose2], estimate: Sequence[Pose2], max_time_diff_s: float
) -> tuple[list[Pose2], list[Pose2]]:
    matched_reference: list[Pose2] = []
    matched_estimate: list[Pose2] = []
    stamps = [pose.stamp for pose in reference]
    for pose in estimate:
        interpolated = _interpolate(reference, stamps, pose.stamp, max_time_diff_s)
        if interpolated is not None:
            matched_reference.append(interpolated)
            matched_estimate.append(pose)
    return matched_reference, matched_estimate


def _align_se2(reference: Sequence[Pose2], estimate: Sequence[Pose2]) -> tuple[list[Pose2], dict]:
    ref_cx = statistics.fmean(pose.x for pose in reference)
    ref_cy = statistics.fmean(pose.y for pose in reference)
    est_cx = statistics.fmean(pose.x for pose in estimate)
    est_cy = statistics.fmean(pose.y for pose in estimate)
    cross = 0.0
    dot = 0.0
    for ref, est in zip(reference, estimate):
        ex, ey = est.x - est_cx, est.y - est_cy
        rx, ry = ref.x - ref_cx, ref.y - ref_cy
        cross += ex * ry - ey * rx
        dot += ex * rx + ey * ry
    rotation = math.atan2(cross, dot)
    cosine, sine = math.cos(rotation), math.sin(rotation)
    tx = ref_cx - (cosine * est_cx - sine * est_cy)
    ty = ref_cy - (sine * est_cx + cosine * est_cy)
    aligned = [
        Pose2(
            pose.stamp,
            cosine * pose.x - sine * pose.y + tx,
            sine * pose.x + cosine * pose.y + ty,
            _wrap_angle(pose.yaw + rotation),
        )
        for pose in estimate
    ]
    # 禁止估计尺度：轮径或里程计比例误差必须留在指标中，不能被对齐步骤“修好”。
    return aligned, {
        "mode": "se2_rigid_no_scale",
        "rotation_deg": math.degrees(rotation),
        "translation_x_m": tx,
        "translation_y_m": ty,
        "scale": 1.0,
    }


def associate_and_align(
    reference: Sequence[Pose2], estimate: Sequence[Pose2], max_time_diff_s: float
) -> tuple[list[Pose2], list[Pose2], dict[str, object]]:
    """Associate by time and apply one fixed-scale SE(2) alignment.

    公开这个窄接口，让全局评价和退化分段评价共享同一时间轴/对齐语义；调用方不能各自
    重新实现一套“更好看”的对齐方式，否则 A/B 指标将失去可比性。
    """

    matched_reference, matched_estimate = associate(
        reference, estimate, max_time_diff_s
    )
    if len(matched_reference) < 3:
        raise ValueError("fewer than 3 timestamp-associated poses")
    aligned_estimate, alignment = _align_se2(matched_reference, matched_estimate)
    return matched_reference, aligned_estimate, alignment


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def _summary(values: Sequence[float]) -> dict[str, float]:
    return {
        "rmse": math.sqrt(statistics.fmean(value * value for value in values)),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": _percentile(values, 0.95),
        "max": max(values),
    }


def _path_length(poses: Sequence[Pose2]) -> float:
    return sum(
        math.hypot(current.x - previous.x, current.y - previous.y)
        for previous, current in zip(poses, poses[1:])
    )


def _relative(start: Pose2, end: Pose2) -> tuple[float, float, float]:
    dx, dy = end.x - start.x, end.y - start.y
    cosine, sine = math.cos(start.yaw), math.sin(start.yaw)
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        _wrap_angle(end.yaw - start.yaw),
    )


def _rpe(
    reference: Sequence[Pose2], estimate: Sequence[Pose2], delta_s: float
) -> tuple[list[float], list[float]]:
    stamps = [pose.stamp for pose in reference]
    translation_errors: list[float] = []
    yaw_errors: list[float] = []
    tolerance = max(0.05, delta_s * 0.20)
    for index, pose in enumerate(reference):
        target = pose.stamp + delta_s
        candidate = bisect.bisect_left(stamps, target, lo=index + 1)
        options = [item for item in (candidate - 1, candidate) if index < item < len(stamps)]
        if not options:
            continue
        end_index = min(options, key=lambda item: abs(stamps[item] - target))
        if abs(stamps[end_index] - target) > tolerance:
            continue
        ref_dx, ref_dy, ref_dyaw = _relative(reference[index], reference[end_index])
        est_dx, est_dy, est_dyaw = _relative(estimate[index], estimate[end_index])
        translation_errors.append(math.hypot(est_dx - ref_dx, est_dy - ref_dy))
        yaw_errors.append(abs(_wrap_angle(est_dyaw - ref_dyaw)))
    return translation_errors, yaw_errors


def _worst_segment(poses: Sequence[Pose2], squared_errors: Sequence[float], window_s: float) -> dict:
    stamps = [pose.stamp for pose in poses]
    prefix = [0.0]
    for value in squared_errors:
        prefix.append(prefix[-1] + value)
    effective_window = min(window_s, stamps[-1] - stamps[0])
    best: tuple[float, int, int] | None = None
    for start in range(len(poses) - 1):
        end = bisect.bisect_right(stamps, stamps[start] + window_s, lo=start + 1) - 1
        if end <= start:
            continue
        if stamps[end] - stamps[start] < effective_window * 0.90:
            continue
        count = end - start + 1
        rmse = math.sqrt((prefix[end + 1] - prefix[start]) / count)
        if best is None or rmse > best[0]:
            best = (rmse, start, end)
    if best is None:
        return {"window_s": window_s, "available": False}
    rmse, start, end = best
    return {
        "window_s": effective_window,
        "available": True,
        "start_s": poses[start].stamp,
        "end_s": poses[end].stamp,
        "rmse_m": rmse,
    }


def _sample_indices(poses: Sequence[Pose2], interval_s: float) -> list[int]:
    indices = [0]
    next_stamp = poses[0].stamp + interval_s
    for index, pose in enumerate(poses[1:], 1):
        if pose.stamp >= next_stamp:
            indices.append(index)
            next_stamp = pose.stamp + interval_s
    if indices[-1] != len(poses) - 1:
        indices.append(len(poses) - 1)
    return indices


def _loop_metrics(
    reference: Sequence[Pose2], estimate: Sequence[Pose2], config: EvaluationConfig
) -> dict[str, object]:
    sampled = _sample_indices(reference, config.loop_sample_interval_s)
    opportunities: list[dict[str, object]] = []
    relation_errors: list[float] = []
    yaw_return_errors: list[float] = []
    for current_position, current_index in enumerate(sampled):
        current = reference[current_index]
        candidates: list[tuple[float, int]] = []
        for previous_index in sampled[:current_position]:
            previous = reference[previous_index]
            if current.stamp - previous.stamp < config.loop_min_separation_s:
                continue
            distance = math.hypot(current.x - previous.x, current.y - previous.y)
            yaw_delta = abs(_wrap_angle(current.yaw - previous.yaw))
            if distance <= config.loop_radius_m and yaw_delta <= config.loop_yaw_tolerance_rad:
                candidates.append((distance, previous_index))
        if not candidates:
            continue
        reference_distance, previous_index = min(candidates)
        estimated_distance = math.hypot(
            estimate[current_index].x - estimate[previous_index].x,
            estimate[current_index].y - estimate[previous_index].y,
        )
        estimated_yaw_delta = abs(
            _wrap_angle(estimate[current_index].yaw - estimate[previous_index].yaw)
        )
        relation_errors.append(estimated_distance)
        yaw_return_errors.append(estimated_yaw_delta)
        recovered = (
            estimated_distance <= config.loop_recovery_tolerance_m
            and estimated_yaw_delta <= config.loop_yaw_tolerance_rad
        )
        opportunities.append(
            {
                "current_stamp_s": current.stamp,
                "previous_stamp_s": reference[previous_index].stamp,
                "reference_distance_m": reference_distance,
                "reference_yaw_delta_deg": math.degrees(
                    abs(_wrap_angle(current.yaw - reference[previous_index].yaw))
                ),
                "estimated_distance_m": estimated_distance,
                "estimated_yaw_delta_deg": math.degrees(estimated_yaw_delta),
                "recovered": recovered,
            }
        )

    # 相邻采样点常属于同一次“进入旧区域”的回访过程。按时间聚合后再报告 event recall，
    # 避免提高采样率就人为放大回环机会数量。
    events: list[dict[str, object]] = []
    for opportunity in opportunities:
        if (
            not events
            or float(opportunity["current_stamp_s"])
            - float(events[-1]["end_stamp_s"])
            > config.loop_event_gap_s
        ):
            events.append(
                {
                    "event_id": len(events) + 1,
                    "start_stamp_s": opportunity["current_stamp_s"],
                    "end_stamp_s": opportunity["current_stamp_s"],
                    "opportunity_count": 1,
                    "recovered": bool(opportunity["recovered"]),
                }
            )
        else:
            events[-1]["end_stamp_s"] = opportunity["current_stamp_s"]
            events[-1]["opportunity_count"] = int(events[-1]["opportunity_count"]) + 1
            events[-1]["recovered"] = bool(events[-1]["recovered"]) or bool(
                opportunity["recovered"]
            )

    recovered_opportunities = sum(bool(item["recovered"]) for item in opportunities)
    recovered_events = sum(bool(item["recovered"]) for item in events)
    return {
        "opportunities": len(opportunities),
        "recovered": recovered_opportunities,
        "recall": (
            recovered_opportunities / len(opportunities) if opportunities else None
        ),
        "event_count": len(events),
        "events_recovered": recovered_events,
        "event_recall": recovered_events / len(events) if events else None,
        "event_gap_s": config.loop_event_gap_s,
        "events": events,
        "opportunity_samples": opportunities,
        "mean_estimated_return_distance_m": (
            statistics.fmean(relation_errors) if relation_errors else None
        ),
        "mean_estimated_return_yaw_deg": (
            math.degrees(statistics.fmean(yaw_return_errors)) if yaw_return_errors else None
        ),
        "definition": (
            "ground-truth return samples and time-clustered revisit events recovered by "
            "estimated XY distance and yaw"
        ),
    }


def _round_floats(value):
    if isinstance(value, float):
        return round(value, 6) if math.isfinite(value) else value
    if isinstance(value, dict):
        return {key: _round_floats(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_round_floats(item) for item in value]
    return value


def evaluate(
    reference: Sequence[Pose2], estimate: Sequence[Pose2], config: EvaluationConfig
) -> dict[str, object]:
    matched_ref, aligned_est, alignment = associate_and_align(
        reference, estimate, config.max_time_diff_s
    )
    position_errors = [
        math.hypot(est.x - ref.x, est.y - ref.y)
        for ref, est in zip(matched_ref, aligned_est)
    ]
    yaw_errors = [
        abs(_wrap_angle(est.yaw - ref.yaw)) for ref, est in zip(matched_ref, aligned_est)
    ]
    rpe_translation, rpe_yaw = _rpe(matched_ref, aligned_est, config.rpe_delta_s)
    reference_length = _path_length(matched_ref)
    estimate_length = _path_length(aligned_est)
    association_ratio = len(matched_ref) / len(estimate)
    temporal_coverage = (
        (matched_ref[-1].stamp - matched_ref[0].stamp)
        / max(reference[-1].stamp - reference[0].stamp, 1e-9)
    )
    checks = {
        "minimum_three_matches": len(matched_ref) >= 3,
        "estimate_match_ratio": association_ratio >= config.min_match_ratio,
    }
    ate = _summary(position_errors)
    if config.max_ate_rmse_m is not None:
        checks["ate_rmse_within_limit"] = ate["rmse"] <= config.max_ate_rmse_m
    rpe_translation_summary = _summary(rpe_translation) if rpe_translation else None
    if config.max_rpe_translation_rmse_m is not None:
        checks["rpe_translation_within_limit"] = bool(
            rpe_translation_summary
            and rpe_translation_summary["rmse"] <= config.max_rpe_translation_rmse_m
        )
    report = {
        "passed": all(checks.values()),
        "checks": checks,
        "association": {
            "reference_poses": len(reference),
            "estimate_poses": len(estimate),
            "matched_poses": len(matched_ref),
            "estimate_match_ratio": association_ratio,
            "reference_temporal_coverage_ratio": min(1.0, temporal_coverage),
            "max_time_diff_s": config.max_time_diff_s,
        },
        "alignment": alignment,
        "ate_xy_m": ate,
        "yaw_error_deg": _summary([math.degrees(value) for value in yaw_errors]),
        "rpe": {
            "delta_s": config.rpe_delta_s,
            "pairs": len(rpe_translation),
            "translation_m": rpe_translation_summary,
            "yaw_deg": (
                _summary([math.degrees(value) for value in rpe_yaw]) if rpe_yaw else None
            ),
        },
        "path": {
            "reference_length_m": reference_length,
            "estimate_length_m": estimate_length,
            "length_ratio": estimate_length / max(reference_length, 1e-9),
            "drift_per_reference_meter": ate["rmse"] / max(reference_length, 1e-9),
        },
        "closure": {
            "reference_start_end_m": math.hypot(
                matched_ref[-1].x - matched_ref[0].x,
                matched_ref[-1].y - matched_ref[0].y,
            ),
            "estimate_start_end_m": math.hypot(
                aligned_est[-1].x - aligned_est[0].x,
                aligned_est[-1].y - aligned_est[0].y,
            ),
            "final_pose_error_m": position_errors[-1],
        },
        "loop": _loop_metrics(matched_ref, aligned_est, config),
        "worst_segment": _worst_segment(
            matched_ref, [value * value for value in position_errors], config.segment_window_s
        ),
        "methodology": {
            "trajectory_format": "TUM: timestamp tx ty tz qx qy qz qw",
            "association": "reference interpolation at estimate timestamps",
            "alignment": "least-squares SE(2), scale fixed to 1",
            "scope": "planar XY/yaw evaluation",
        },
    }
    return _round_floats(report)


def render_markdown(report: dict[str, object], reference: Path, estimate: Path) -> str:
    ate = report["ate_xy_m"]
    rpe = report["rpe"]
    loop = report["loop"]
    path = report["path"]
    closure = report["closure"]
    return f"""# SLAM 轨迹定量评估

- 参考轨迹：`{reference}`
- 估计轨迹：`{estimate}`
- 结论：**{'PASS' if report['passed'] else 'FAIL'}**
- 对齐：SE(2) 刚体对齐，固定尺度 1.0

| 指标 | 结果 |
| --- | ---: |
| 匹配位姿 | {report['association']['matched_poses']} |
| ATE XY RMSE | {ate['rmse']:.4f} m |
| ATE XY P95 | {ate['p95']:.4f} m |
| RPE 平移 RMSE ({rpe['delta_s']:.1f}s) | {rpe['translation_m']['rmse'] if rpe['translation_m'] else 'N/A'} m |
| 路径长度比 | {path['length_ratio']:.4f} |
| 终点误差 | {closure['final_pose_error_m']:.4f} m |
| 回访采样点/恢复 | {loop['opportunities']} / {loop['recovered']} |
| 回访事件/恢复 | {loop['event_count']} / {loop['events_recovered']} |

> ATE 反映全局一致性，RPE 反映局部漂移；对齐过程不估计尺度，因此轮径/尺度误差不会被隐藏。
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--estimate", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("logs/slam_trajectory_report.json"))
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--max-time-diff", type=float, default=0.05)
    parser.add_argument("--rpe-delta", type=float, default=1.0)
    parser.add_argument("--segment-window", type=float, default=10.0)
    parser.add_argument("--loop-radius", type=float, default=0.50)
    parser.add_argument("--loop-min-separation", type=float, default=10.0)
    parser.add_argument("--loop-event-gap", type=float, default=2.0)
    parser.add_argument("--loop-recovery-tolerance", type=float, default=0.50)
    parser.add_argument("--min-match-ratio", type=float, default=0.80)
    parser.add_argument("--max-ate-rmse", type=float)
    parser.add_argument("--max-rpe-translation-rmse", type=float)
    args = parser.parse_args()
    config = EvaluationConfig(
        max_time_diff_s=args.max_time_diff,
        rpe_delta_s=args.rpe_delta,
        segment_window_s=args.segment_window,
        loop_radius_m=args.loop_radius,
        loop_min_separation_s=args.loop_min_separation,
        loop_event_gap_s=args.loop_event_gap,
        loop_recovery_tolerance_m=args.loop_recovery_tolerance,
        min_match_ratio=args.min_match_ratio,
        max_ate_rmse_m=args.max_ate_rmse,
        max_rpe_translation_rmse_m=args.max_rpe_translation_rmse,
    )
    report = evaluate(load_trajectory(args.reference), load_trajectory(args.estimate), config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    markdown = args.markdown or args.output.with_suffix(".md")
    markdown.parent.mkdir(parents=True, exist_ok=True)
    markdown.write_text(render_markdown(report, args.reference, args.estimate), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: SLAM trajectory evaluation")
    print(f"Reports: {args.output} {markdown}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
