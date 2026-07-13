#!/usr/bin/env python3
"""Generate deterministic closed-loop trajectories for the SLAM evaluation gate."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def _square(samples_per_edge: int = 60, edge_m: float = 4.0) -> list[tuple[float, float, float]]:
    poses: list[tuple[float, float, float]] = []
    edges = (
        (0.0, 0.0, edge_m, 0.0, 0.0),
        (edge_m, 0.0, edge_m, edge_m, math.pi / 2.0),
        (edge_m, edge_m, 0.0, edge_m, math.pi),
        (0.0, edge_m, 0.0, 0.0, -math.pi / 2.0),
    )
    for start_x, start_y, end_x, end_y, yaw in edges:
        for index in range(samples_per_edge):
            ratio = index / samples_per_edge
            poses.append(
                (
                    start_x + ratio * (end_x - start_x),
                    start_y + ratio * (end_y - start_y),
                    yaw,
                )
            )
    poses.append((0.0, 0.0, 0.0))
    return poses


def _world_transform(x: float, y: float, yaw: float) -> tuple[float, float, float]:
    angle = 0.28
    cosine, sine = math.cos(angle), math.sin(angle)
    return (
        cosine * x - sine * y + 2.1,
        sine * x + cosine * y - 1.3,
        yaw + angle,
    )


def _write(path: Path, poses: list[tuple[float, float, float]], start: float, step: float) -> None:
    lines = ["# timestamp tx ty tz qx qy qz qw"]
    for index, (x, y, yaw) in enumerate(poses):
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        lines.append(f"{start + index * step:.6f} {x:.9f} {y:.9f} 0 0 0 {qz:.9f} {qw:.9f}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def generate(output_dir: Path) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    reference = _square()
    baseline: list[tuple[float, float, float]] = []
    corrected: list[tuple[float, float, float]] = []
    count = len(reference) - 1
    for index, (x, y, yaw) in enumerate(reference):
        progress = index / count
        # 基线模拟轮滑与航向累计漂移；尾点不能自然闭合。
        drift_x = 0.65 * progress
        drift_y = 0.42 * progress * progress
        drift_yaw = 0.12 * progress
        baseline.append(_world_transform(x + drift_x, y + drift_y, yaw + drift_yaw))
        # 校正轨迹只保留厘米级局部噪声，模拟后端加入正确回环约束后的结果。
        noise_x = 0.012 * math.sin(index * 0.17)
        noise_y = 0.010 * math.cos(index * 0.13)
        corrected.append(_world_transform(x + noise_x, y + noise_y, yaw))
    paths = {
        "reference": output_dir / "reference.tum",
        "baseline": output_dir / "dead_reckoning.tum",
        "corrected": output_dir / "loop_corrected.tum",
    }
    _write(paths["reference"], reference, start=1_700_000_000.0, step=0.1)
    _write(paths["baseline"], baseline, start=1_700_000_000.0, step=0.1)
    _write(paths["corrected"], corrected, start=1_700_000_000.0, step=0.1)
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("logs/slam_evaluation_fixture"))
    args = parser.parse_args()
    paths = generate(args.output_dir)
    for name, path in paths.items():
        print(f"{name}={path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
