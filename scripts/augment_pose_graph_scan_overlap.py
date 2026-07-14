#!/usr/bin/env python3
"""Attach raw-LaserScan overlap scores to an immutable pose-graph snapshot."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from embodied_slam_tools.bag_source import iter_events


@dataclass(frozen=True)
class ConstraintRow:
    line_index: int
    source_id: int
    target_id: int
    relative_pose: tuple[float, float, float]


Pose2 = tuple[float, float, float]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_graph(path: Path) -> tuple[list[str], dict[int, float], list[ConstraintRow]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    node_stamps: dict[int, float] = {}
    constraints: list[ConstraintRow] = []
    for index, line in enumerate(lines):
        fields = line.split()
        if not fields or fields[0].startswith("#"):
            continue
        if fields[0] == "N" and len(fields) == 6:
            node_stamps[int(fields[1])] = float(fields[2])
        elif fields[0] == "C" and len(fields) >= 15:
            constraints.append(
                ConstraintRow(
                    index,
                    int(fields[1]),
                    int(fields[2]),
                    (float(fields[3]), float(fields[4]), float(fields[5])),
                )
            )
        else:
            raise ValueError(f"unsupported graph row {index + 1}: {line[:120]}")
    if len(node_stamps) < 3 or not constraints:
        raise ValueError("graph must contain at least three nodes and one constraint")
    return lines, node_stamps, constraints


def normalize_frame(frame: str) -> str:
    return frame.lstrip("/")


def compose_pose(source_to_middle: Pose2, middle_to_target: Pose2) -> Pose2:
    """Compose two 2-D coordinate transforms as source -> middle -> target."""

    ax, ay, ayaw = source_to_middle
    bx, by, byaw = middle_to_target
    cosine, sine = math.cos(byaw), math.sin(byaw)
    return (
        bx + cosine * ax - sine * ay,
        by + sine * ax + cosine * ay,
        math.atan2(math.sin(ayaw + byaw), math.cos(ayaw + byaw)),
    )


def inverse_pose(source_to_target: Pose2) -> Pose2:
    x, y, yaw = source_to_target
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        -cosine * x - sine * y,
        sine * x - cosine * y,
        -yaw,
    )


def quaternion_yaw(rotation) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def resolve_scan_to_base_transform(
    transforms: list[object], scan_frame: str, base_frame: str
) -> Pose2:
    """Resolve a static TF chain that maps LaserScan points into the robot base frame."""

    graph: dict[str, list[tuple[str, Pose2]]] = {}
    for item in transforms:
        parent = normalize_frame(item.header.frame_id)
        child = normalize_frame(item.child_frame_id)
        translation = item.transform.translation
        # TF 中的 parent->child 数据把 child 坐标映射到 parent 坐标。
        child_to_parent = (
            float(translation.x),
            float(translation.y),
            quaternion_yaw(item.transform.rotation),
        )
        graph.setdefault(child, []).append((parent, child_to_parent))
        graph.setdefault(parent, []).append((child, inverse_pose(child_to_parent)))

    source = normalize_frame(scan_frame)
    target = normalize_frame(base_frame)
    if source == target:
        return (0.0, 0.0, 0.0)
    queue = deque([(source, (0.0, 0.0, 0.0))])
    visited = {source}
    while queue:
        frame, source_to_frame = queue.popleft()
        for neighbor, frame_to_neighbor in graph.get(frame, []):
            if neighbor in visited:
                continue
            source_to_neighbor = compose_pose(source_to_frame, frame_to_neighbor)
            if neighbor == target:
                return source_to_neighbor
            visited.add(neighbor)
            queue.append((neighbor, source_to_neighbor))
    raise RuntimeError(f"static TF cannot connect scan frame {source!r} to base frame {target!r}")


def scan_points(
    message,
    *,
    minimum_range: float,
    maximum_range: float,
    stride: int,
    scan_to_base: Pose2,
) -> np.ndarray:
    ranges = np.asarray(message.ranges, dtype=float)
    angles = message.angle_min + np.arange(ranges.size) * message.angle_increment
    # 与 Karto LaserRangeFinder 的有效量程判定保持严格开区间，确保离线与在线分数同义。
    valid = np.isfinite(ranges) & (ranges > minimum_range) & (ranges < maximum_range)
    points = np.column_stack(
        (ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]))
    )[::stride]
    x, y, yaw = scan_to_base
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return points @ np.asarray(((cosine, sine), (-sine, cosine))) + (x, y)


def bidirectional_overlap(
    source: np.ndarray,
    target: np.ndarray,
    relative_pose: tuple[float, float, float],
    match_distance: float,
) -> float:
    x, y, yaw = relative_pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    transformed_target = target @ np.asarray(((cosine, sine), (-sine, cosine))) + (x, y)
    distances_squared = np.sum(
        (source[:, np.newaxis, :] - transformed_target[np.newaxis, :, :]) ** 2, axis=2
    )
    threshold_squared = match_distance * match_distance
    return float(
        0.5
        * (
            np.mean(np.min(distances_squared, axis=1) < threshold_squared)
            + np.mean(np.min(distances_squared, axis=0) < threshold_squared)
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--loop-id-separation", type=int, default=20)
    parser.add_argument("--minimum-range", type=float, default=0.12)
    parser.add_argument("--maximum-range", type=float, default=30.0)
    parser.add_argument("--point-stride", type=int, default=2)
    parser.add_argument("--minimum-points", type=int, default=30)
    parser.add_argument("--match-distance", type=float, default=0.20)
    parser.add_argument("--maximum-association-delta", type=float, default=0.001)
    args = parser.parse_args()
    if args.point_stride <= 0 or args.minimum_points <= 0 or args.loop_id_separation < 2:
        parser.error("stride/minimum-points must be positive and loop separation >= 2")
    if not (0.0 < args.minimum_range < args.maximum_range) or args.match_distance <= 0.0:
        parser.error("range and match-distance parameters are invalid")
    if not args.graph.is_file() or not args.bag.exists():
        parser.error("graph and bag must exist")

    lines, node_stamps, constraints = parse_graph(args.graph)
    raw_scans: list[tuple[float, object]] = []
    static_transforms: list[object] = []
    scan_frame = ""
    for event in iter_events(args.bag, topics={args.scan_topic, "/tf_static"}):
        if event.topic == "/tf_static":
            static_transforms.extend(event.message.transforms)
            continue
        message = event.message
        current_frame = normalize_frame(message.header.frame_id)
        if scan_frame and current_frame != scan_frame:
            raise RuntimeError(f"scan frame changed from {scan_frame!r} to {current_frame!r}")
        scan_frame = current_frame
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        raw_scans.append((stamp, message))
    if not raw_scans:
        raise RuntimeError(f"bag has no {args.scan_topic} messages")
    scan_to_base = resolve_scan_to_base_transform(
        static_transforms, scan_frame, args.base_frame
    )
    scans: list[tuple[float, np.ndarray]] = []
    for stamp, message in raw_scans:
        scans.append(
            (
                stamp,
                scan_points(
                    message,
                    minimum_range=args.minimum_range,
                    maximum_range=args.maximum_range,
                    stride=args.point_stride,
                    scan_to_base=scan_to_base,
                ),
            )
        )
    scan_stamps = [item[0] for item in scans]

    associated: dict[int, np.ndarray] = {}
    association_deltas: list[float] = []
    for node_id, stamp in node_stamps.items():
        insertion = bisect.bisect_left(scan_stamps, stamp)
        candidate_indexes = range(max(0, insertion - 1), min(len(scans), insertion + 2))
        best = min(candidate_indexes, key=lambda index: abs(scan_stamps[index] - stamp))
        delta = abs(scan_stamps[best] - stamp)
        if delta > args.maximum_association_delta:
            raise RuntimeError(f"node {node_id} has no scan within association tolerance: {delta}")
        associated[node_id] = scans[best][1]
        association_deltas.append(delta)

    overlap_values: list[float] = []
    unavailable = 0
    for constraint in constraints:
        fields = lines[constraint.line_index].split()
        # v2 允许重复运行：先丢弃旧 overlap，仅保留固定的 C + 14 个基础字段。
        base_row = " ".join(fields[:15])
        if abs(constraint.source_id - constraint.target_id) < args.loop_id_separation:
            lines[constraint.line_index] = base_row
            continue
        source = associated[constraint.source_id]
        target = associated[constraint.target_id]
        if source.shape[0] < args.minimum_points or target.shape[0] < args.minimum_points:
            unavailable += 1
            lines[constraint.line_index] = base_row
            continue
        overlap = bidirectional_overlap(
            source, target, constraint.relative_pose, args.match_distance
        )
        overlap_values.append(overlap)
        lines[constraint.line_index] = f"{base_row} {overlap:.17g}"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    lines[0] = "# embodied_slam_pose_graph_v2 scan_overlap_ratio=optional"
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    values = np.asarray(overlap_values, dtype=float)
    metadata = {
        "schema_version": 1,
        "passed": unavailable == 0 and len(overlap_values) > 0,
        "source_graph_sha256": sha256(args.graph),
        "source_bag_sha256": sha256(args.bag),
        "output_graph_sha256": sha256(args.output),
        "nodes": len(node_stamps),
        "constraints": len(constraints),
        "scan_messages": len(scans),
        "scored_nonlocal_constraints": len(overlap_values),
        "unavailable_nonlocal_constraints": unavailable,
        "association_delta_s": {
            "maximum": round(max(association_deltas), 9),
            "p95": round(float(np.quantile(association_deltas, 0.95)), 9),
        },
        "configuration": {
            "scan_topic": args.scan_topic,
            "scan_frame": scan_frame,
            "base_frame": normalize_frame(args.base_frame),
            "scan_to_base": {
                "x_m": round(scan_to_base[0], 9),
                "y_m": round(scan_to_base[1], 9),
                "yaw_rad": round(scan_to_base[2], 9),
            },
            "loop_id_separation": args.loop_id_separation,
            "minimum_range_m": args.minimum_range,
            "maximum_range_m": args.maximum_range,
            "point_stride": args.point_stride,
            "minimum_points": args.minimum_points,
            "match_distance_m": args.match_distance,
        },
        "overlap_ratio": {
            "minimum": round(float(values.min()), 6),
            "median": round(float(np.median(values)), 6),
            "p95": round(float(np.quantile(values, 0.95)), 6),
            "maximum": round(float(values.max()), 6),
        },
        "methodology_boundary": (
            "Raw LaserScan is associated by timestamp and scored without ground truth. "
            "Single-frame overlap is supporting evidence, not a replacement for Karto chain matching."
        ),
    }
    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    print(f"{'PASS' if metadata['passed'] else 'FAIL'}: pose graph scan-overlap augmentation")
    return 0 if metadata["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
