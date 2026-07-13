#!/usr/bin/env python3
"""Break aligned SLAM error into motion classes and optional labelled intervals."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import statistics
from pathlib import Path
from typing import Sequence


def _load_evaluator():
    script = Path(__file__).with_name("evaluate_slam_trajectory.py")
    spec = importlib.util.spec_from_file_location("slam_trajectory_evaluator", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load evaluator: {script}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVALUATOR = _load_evaluator()


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _rmse(values: Sequence[float]) -> float | None:
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else None


def _p95(values: Sequence[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]


def _summarize(
    indices: list[int], reference, estimate, *, duration_s: float | None = None
) -> dict[str, object]:
    errors = [
        math.hypot(estimate[index].x - reference[index].x, estimate[index].y - reference[index].y)
        for index in indices
    ]
    return {
        "samples": len(indices),
        "duration_s": (
            duration_s
            if duration_s is not None
            else (
                reference[indices[-1]].stamp - reference[indices[0]].stamp
                if len(indices) >= 2
                else 0.0
            )
        ),
        "ate_rmse_m": _rmse(errors),
        "ate_p95_m": _p95(errors),
        "ate_max_m": max(errors) if errors else None,
    }


def analyze(
    reference,
    estimate,
    *,
    max_time_diff_s: float = 0.05,
    moving_speed_mps: float = 0.05,
    turning_rate_rad_s: float = 0.25,
    annotations: dict | None = None,
) -> dict[str, object]:
    """Return motion-conditioned errors using the evaluator's shared alignment."""

    matched_ref, aligned_est, alignment = EVALUATOR.associate_and_align(
        reference, estimate, max_time_diff_s
    )
    classes: dict[str, list[int]] = {"straight": [], "turning": [], "stationary": []}
    class_durations = {"straight": 0.0, "turning": 0.0, "stationary": 0.0}
    labels: list[str] = ["stationary"]
    for index in range(1, len(matched_ref)):
        previous, current = matched_ref[index - 1], matched_ref[index]
        dt = max(current.stamp - previous.stamp, 1e-9)
        speed = math.hypot(current.x - previous.x, current.y - previous.y) / dt
        yaw_rate = abs(_wrap(current.yaw - previous.yaw)) / dt
        if yaw_rate >= turning_rate_rad_s:
            label = "turning"
        elif speed >= moving_speed_mps:
            label = "straight"
        else:
            label = "stationary"
        labels.append(label)
        classes[label].append(index)
        # 类别样本不是连续区间，不能用最后时间减最初时间；逐段累加才能保持总时长守恒。
        class_durations[label] += dt

    runs: list[dict[str, object]] = []
    start = 0
    for index in range(1, len(labels) + 1):
        if index == len(labels) or labels[index] != labels[start]:
            indices = list(range(start, index))
            item = {"label": labels[start], **_summarize(indices, matched_ref, aligned_est)}
            item["start_s"] = matched_ref[start].stamp
            item["end_s"] = matched_ref[index - 1].stamp
            runs.append(item)
            start = index

    labelled: list[dict[str, object]] = []
    for interval in (annotations or {}).get("intervals", []):
        start_s = float(interval["start_s"])
        end_s = float(interval["end_s"])
        if end_s <= start_s:
            raise ValueError(f"invalid annotation interval: {interval}")
        indices = [
            index
            for index, pose in enumerate(matched_ref)
            if start_s <= pose.stamp <= end_s
        ]
        # 动态遮挡/走廊等语义只能来自人工复核区间，不能由速度阈值反向猜测场景含义。
        labelled.append(
            {
                "label": str(interval["label"]),
                "start_s": start_s,
                "end_s": end_s,
                **_summarize(indices, matched_ref, aligned_est),
            }
        )

    return {
        "passed": len(matched_ref) >= 3,
        "association": {
            "matched_poses": len(matched_ref),
            "max_time_diff_s": max_time_diff_s,
        },
        "alignment": alignment,
        "classification": {
            "moving_speed_mps": moving_speed_mps,
            "turning_rate_rad_s": turning_rate_rad_s,
            "warning": (
                "motion classes come from ground-truth kinematics; they are not semantic corridor "
                "or dynamic-occlusion labels"
            ),
        },
        "motion_classes": {
            label: _summarize(
                indices,
                matched_ref,
                aligned_est,
                duration_s=class_durations[label],
            )
            for label, indices in classes.items()
        },
        "worst_motion_runs": sorted(
            (item for item in runs if item["samples"] >= 3),
            key=lambda item: float(item["ate_rmse_m"] or 0.0),
            reverse=True,
        )[:10],
        "labelled_intervals": labelled,
        "dynamic_occlusion_evidence": {
            "available": any(item["label"] == "dynamic_occlusion" for item in labelled),
            "requirement": "provide manually reviewed timestamp intervals",
        },
    }


def render_markdown(report: dict[str, object]) -> str:
    """Render the compact interview-facing summary."""

    rows = []
    for label, item in report["motion_classes"].items():
        rows.append(
            f"| {label} | {item['samples']} | {item['duration_s']:.2f} | "
            f"{item['ate_rmse_m'] if item['ate_rmse_m'] is not None else 'N/A'} | "
            f"{item['ate_p95_m'] if item['ate_p95_m'] is not None else 'N/A'} |"
        )
    return """# SLAM 退化分段报告

| 运动类别 | 样本数 | 覆盖时长 (s) | ATE RMSE (m) | ATE P95 (m) |
| --- | ---: | ---: | ---: | ---: |
""" + "\n".join(rows) + "\n\n> 动态遮挡只有在提供人工复核时间区间时才单独统计。\n"


def main() -> int:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--estimate", type=Path, required=True)
    parser.add_argument("--annotations", type=Path)
    parser.add_argument("--max-time-diff", type=float, default=0.05)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path)
    args = parser.parse_args()
    annotations = (
        json.loads(args.annotations.read_text(encoding="utf-8"))
        if args.annotations
        else None
    )
    report = analyze(
        EVALUATOR.load_trajectory(args.reference),
        EVALUATOR.load_trajectory(args.estimate),
        max_time_diff_s=args.max_time_diff,
        annotations=annotations,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    markdown = args.markdown or args.output.with_suffix(".md")
    markdown.write_text(render_markdown(report), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"PASS: SLAM degradation analysis -> {args.output} {markdown}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
