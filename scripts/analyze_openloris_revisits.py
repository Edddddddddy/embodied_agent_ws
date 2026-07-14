#!/usr/bin/env python3
"""Audit whether an OpenLORIS ground-truth sequence contains real revisit events."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path


def _load_evaluator():
    path = Path(__file__).with_name("evaluate_slam_trajectory.py")
    spec = importlib.util.spec_from_file_location("slam_trajectory_evaluator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load evaluator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVALUATOR = _load_evaluator()


def analyze(
    reference_path: Path,
    config,
    *,
    min_events: int = 1,
    path_sample_interval_s: float = 0.1,
) -> dict[str, object]:
    if path_sample_interval_s <= 0:
        raise ValueError("path_sample_interval_s must be positive")
    poses = EVALUATOR.load_trajectory(reference_path)
    loop = EVALUATOR._loop_metrics(poses, poses, config)
    sampled = [
        poses[index]
        for index in EVALUATOR._sample_indices(poses, path_sample_interval_s)
    ]
    sampled_path_length = EVALUATOR._path_length(sampled)
    checks = {
        "minimum_revisit_events": int(loop["event_count"]) >= min_events,
        "nonzero_path": sampled_path_length > 0.0,
    }
    return EVALUATOR._round_floats(
        {
            "schema_version": 1,
            "passed": all(checks.values()),
            "checks": checks,
            "source": {
                "path": str(reference_path.resolve()),
                "sha256": hashlib.sha256(reference_path.read_bytes()).hexdigest(),
            },
            "trajectory": {
                "poses": len(poses),
                "duration_s": poses[-1].stamp - poses[0].stamp,
                # 高频 mocap 点间抖动会显著放大逐点累加路程；主结果先按 10 Hz 降采样，
                # 同时保留 raw 值，使评估方法和噪声敏感性都可审计。
                "path_length_m": sampled_path_length,
                "path_sample_interval_s": path_sample_interval_s,
                "raw_path_length_m": EVALUATOR._path_length(poses),
            },
            "revisit_catalog": {
                "event_count": loop["event_count"],
                "opportunity_samples": loop["opportunities"],
                # 真值目录不携带 self-evaluation 产生的 recovered=true，避免被误读为
                # SLAM 已检测/恢复这些事件；恢复结果只能来自 estimate 或 constraint 报告。
                "events": [
                    {key: value for key, value in event.items() if key != "recovered"}
                    for event in loop["events"]
                ],
                "radius_m": config.loop_radius_m,
                "yaw_tolerance_deg": math.degrees(config.loop_yaw_tolerance_rad),
                "minimum_separation_s": config.loop_min_separation_s,
                "sample_interval_s": config.loop_sample_interval_s,
                "event_gap_s": config.loop_event_gap_s,
            },
            "claim_boundary": (
                "this catalog proves ground-truth revisit opportunities only; it does not "
                "prove that a SLAM front-end detected or accepted a loop closure"
            ),
        }
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--loop-radius", type=float, default=0.50)
    parser.add_argument("--loop-yaw-tolerance-deg", type=float, default=30.0)
    parser.add_argument("--loop-min-separation", type=float, default=10.0)
    parser.add_argument("--loop-sample-interval", type=float, default=1.0)
    parser.add_argument("--loop-event-gap", type=float, default=2.0)
    parser.add_argument("--min-events", type=int, default=1)
    parser.add_argument("--path-sample-interval", type=float, default=0.1)
    args = parser.parse_args()
    config = EVALUATOR.EvaluationConfig(
        loop_radius_m=args.loop_radius,
        loop_yaw_tolerance_deg=args.loop_yaw_tolerance_deg,
        loop_min_separation_s=args.loop_min_separation,
        loop_sample_interval_s=args.loop_sample_interval,
        loop_event_gap_s=args.loop_event_gap,
    )
    report = analyze(
        args.reference,
        config,
        min_events=args.min_events,
        path_sample_interval_s=args.path_sample_interval,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: OpenLORIS revisit audit")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
