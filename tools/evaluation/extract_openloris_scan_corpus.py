#!/usr/bin/env python3
"""Associate fixed-graph nodes with raw LaserScan points for C++ loop retrieval."""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path

import numpy as np

from augment_pose_graph_scan_overlap import (
    normalize_frame,
    parse_graph,
    resolve_scan_to_base_transform,
    scan_points,
    sha256,
)
from embodied_slam_tools.bag_source import iter_events


def write_corpus(rows: list[tuple[int, float, np.ndarray]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        stream.write("# embodied_lidar_scan_corpus_v1\n")
        for node_id, stamp, points in rows:
            fields = ["S", str(node_id), f"{stamp:.9f}", str(points.shape[0])]
            for x, y in points:
                fields.extend((f"{float(x):.6f}", f"{float(y):.6f}"))
            stream.write(" ".join(fields) + "\n")


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(orientation) -> float:
    siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy = 1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
    return math.atan2(siny, cosy)


def associate_odometry(
    rows: list[tuple[int, float, np.ndarray]],
    raw_odometry: list[tuple[float, float, float, float]],
    maximum_delta_s: float = 0.10,
) -> tuple[list[tuple[int, float, float, float, float]], list[int]]:
    if maximum_delta_s <= 0.0:
        raise ValueError("odometry association tolerance must be positive")
    # 某些 OpenLORIS 旧 bag 含重复 /odom 时间戳；后到样本覆盖前者，使插值轴严格单调。
    by_stamp = {sample[0]: sample for sample in raw_odometry}
    samples = [by_stamp[stamp] for stamp in sorted(by_stamp)]
    stamps = [sample[0] for sample in samples]
    associated: list[tuple[int, float, float, float, float]] = []
    missing: list[int] = []
    for scan_id, stamp, _points in rows:
        insertion = bisect.bisect_left(stamps, stamp)
        if insertion < len(samples) and abs(samples[insertion][0] - stamp) <= 1e-9:
            _, x, y, yaw = samples[insertion]
            associated.append((scan_id, stamp, x, y, yaw))
            continue
        if insertion == 0 or insertion == len(samples):
            nearest = samples[0 if insertion == 0 else -1]
            if abs(nearest[0] - stamp) > maximum_delta_s:
                missing.append(scan_id)
                continue
            associated.append((scan_id, stamp, nearest[1], nearest[2], nearest[3]))
            continue
        left, right = samples[insertion - 1], samples[insertion]
        if stamp - left[0] > maximum_delta_s or right[0] - stamp > maximum_delta_s:
            missing.append(scan_id)
            continue
        ratio = (stamp - left[0]) / (right[0] - left[0])
        associated.append(
            (
                scan_id,
                stamp,
                left[1] + ratio * (right[1] - left[1]),
                left[2] + ratio * (right[2] - left[2]),
                _wrap_angle(left[3] + ratio * _wrap_angle(right[3] - left[3])),
            )
        )
    return associated, missing


def write_odometry_priors(
    rows: list[tuple[int, float, float, float, float]], output: Path
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        stream.write("# embodied_lidar_odometry_priors_v1\n")
        for scan_id, stamp, x, y, yaw in rows:
            stream.write(f"O {scan_id} {stamp:.9f} {x:.9f} {y:.9f} {yaw:.9f}\n")


def extract(
    graph: Path,
    bag: Path,
    output: Path,
    metadata_path: Path,
    *,
    scan_topic: str = "/scan",
    base_frame: str = "base_link",
    minimum_range: float = 0.12,
    maximum_range: float = 20.0,
    point_stride: int = 2,
    minimum_points: int = 30,
    maximum_association_delta: float = 0.001,
    sample_interval_s: float = 0.5,
    odometry_output: Path | None = None,
    odometry_topic: str = "/odom",
) -> dict[str, object]:
    if point_stride <= 0 or minimum_points <= 0:
        raise ValueError("point stride and minimum points must be positive")
    if not 0.0 < minimum_range < maximum_range:
        raise ValueError("invalid scan range")
    if maximum_association_delta <= 0.0:
        raise ValueError("association tolerance must be positive")
    if sample_interval_s < 0.0:
        raise ValueError("sample interval cannot be negative")

    _lines, node_stamps, _constraints = parse_graph(graph)
    raw_scans: list[tuple[float, object]] = []
    raw_odometry: list[tuple[float, float, float, float]] = []
    static_transforms: list[object] = []
    scan_frame = ""
    topics = {scan_topic, "/tf_static"}
    if odometry_output is not None:
        topics.add(odometry_topic)
    for event in iter_events(bag, topics=topics):
        if event.topic == "/tf_static":
            static_transforms.extend(event.message.transforms)
            continue
        if event.topic == odometry_topic:
            message = event.message
            stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
            position = message.pose.pose.position
            raw_odometry.append(
                (stamp, float(position.x), float(position.y), _yaw_from_quaternion(message.pose.pose.orientation))
            )
            continue
        message = event.message
        current_frame = normalize_frame(message.header.frame_id)
        if scan_frame and current_frame != scan_frame:
            raise RuntimeError(
                f"scan frame changed from {scan_frame!r} to {current_frame!r}"
            )
        scan_frame = current_frame
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        raw_scans.append((stamp, message))
    if not raw_scans:
        raise RuntimeError(f"bag has no {scan_topic} messages")

    scan_to_base = resolve_scan_to_base_transform(
        static_transforms, scan_frame, base_frame
    )
    scans = [
        (
            stamp,
            scan_points(
                message,
                minimum_range=minimum_range,
                maximum_range=maximum_range,
                stride=point_stride,
                scan_to_base=scan_to_base,
            ),
        )
        for stamp, message in raw_scans
    ]
    scan_stamps = [item[0] for item in scans]

    rows: list[tuple[int, float, np.ndarray]] = []
    association_deltas: list[float] = []
    insufficient_nodes: list[int] = []
    if sample_interval_s > 0.0:
        # 前端候选检索必须覆盖原始扫描时间轴，不能被异步 Karto 是否插入图节点所支配。
        # 固定时间采样既控制计算量，也让不同后端共享完全相同的检索输入。
        last_stamp = -float("inf")
        for stamp, points in scans:
            if stamp - last_stamp + 1e-9 < sample_interval_s:
                continue
            node_id = len(rows)
            if points.shape[0] < minimum_points:
                insufficient_nodes.append(node_id)
            rows.append((node_id, stamp, points))
            last_stamp = stamp
        sampling_mode = "uniform_raw_scan_time"
    else:
        for node_id, stamp in sorted(node_stamps.items(), key=lambda item: item[1]):
            insertion = bisect.bisect_left(scan_stamps, stamp)
            indexes = range(max(0, insertion - 1), min(len(scans), insertion + 2))
            best = min(indexes, key=lambda index: abs(scan_stamps[index] - stamp))
            delta = abs(scan_stamps[best] - stamp)
            if delta > maximum_association_delta:
                raise RuntimeError(
                    f"node {node_id} has no scan within association tolerance: {delta}"
                )
            points = scans[best][1]
            if points.shape[0] < minimum_points:
                insufficient_nodes.append(node_id)
            rows.append((node_id, stamp, points))
            association_deltas.append(delta)
        sampling_mode = "fixed_graph_node_timestamp"

    write_corpus(rows, output)
    odometry_metadata: dict[str, object] | None = None
    if odometry_output is not None:
        if not raw_odometry:
            raise RuntimeError(f"bag has no {odometry_topic} messages")
        odometry_rows, missing_odometry = associate_odometry(rows, raw_odometry)
        write_odometry_priors(odometry_rows, odometry_output)
        odometry_metadata = {
            "path": str(odometry_output.resolve()),
            "sha256": sha256(odometry_output),
            "topic": odometry_topic,
            "messages": len(raw_odometry),
            "associated_scans": len(odometry_rows),
            "coverage_ratio": len(odometry_rows) / len(rows),
            "missing_scan_ids": missing_odometry,
            "claim_boundary": (
                "Wheel/visual odometry is a runtime prior for resolving scan symmetry; "
                "it is not OpenLORIS ground truth."
            ),
        }
    values = np.asarray(association_deltas, dtype=float)
    metadata: dict[str, object] = {
        "schema_version": 1,
        "passed": not insufficient_nodes
        and (
            len(rows) >= 100
            if sampling_mode == "uniform_raw_scan_time"
            else len(rows) == len(node_stamps)
        ),
        "source_graph": {
            "path": str(graph.resolve()),
            "sha256": sha256(graph),
        },
        "source_bag": {
            "path": str(bag.resolve()),
            "sha256": sha256(bag),
        },
        "corpus": {
            "path": str(output.resolve()),
            "sha256": sha256(output),
            "nodes": len(rows),
            "scan_messages": len(scans),
            "insufficient_point_nodes": insufficient_nodes,
            "sampling_mode": sampling_mode,
            "sample_interval_s": sample_interval_s if sample_interval_s > 0.0 else None,
        },
        "association_delta_s": {
            "maximum": round(float(values.max()), 9) if values.size else None,
            "p95": round(float(np.quantile(values, 0.95)), 9) if values.size else None,
        },
        "configuration": {
            "scan_topic": scan_topic,
            "scan_frame": scan_frame,
            "base_frame": normalize_frame(base_frame),
            "scan_to_base": {
                "x_m": round(scan_to_base[0], 9),
                "y_m": round(scan_to_base[1], 9),
                "yaw_rad": round(scan_to_base[2], 9),
            },
            "minimum_range_m": minimum_range,
            "maximum_range_m": maximum_range,
            "point_stride": point_stride,
            "minimum_points": minimum_points,
            "maximum_association_delta_s": maximum_association_delta,
            "sample_interval_s": sample_interval_s,
        },
        "odometry_prior": odometry_metadata,
        "claim_boundary": (
            "The corpus deterministically samples raw LaserScan geometry and binds the source "
            "graph/bag hashes. It contains no ground-truth pose and cannot score loop "
            "correctness by itself."
        ),
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return metadata


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--odometry-output", type=Path)
    parser.add_argument("--odometry-topic", default="/odom")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--minimum-range", type=float, default=0.12)
    parser.add_argument("--maximum-range", type=float, default=20.0)
    parser.add_argument("--point-stride", type=int, default=2)
    parser.add_argument("--minimum-points", type=int, default=30)
    parser.add_argument("--maximum-association-delta", type=float, default=0.001)
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=0.5,
        help="deterministically sample the raw scan timeline; use 0 for graph-node association",
    )
    args = parser.parse_args()
    if not args.graph.is_file() or not args.bag.exists():
        parser.error("graph and bag must exist")
    report = extract(
        args.graph,
        args.bag,
        args.output,
        args.metadata,
        scan_topic=args.scan_topic,
        base_frame=args.base_frame,
        minimum_range=args.minimum_range,
        maximum_range=args.maximum_range,
        point_stride=args.point_stride,
        minimum_points=args.minimum_points,
        maximum_association_delta=args.maximum_association_delta,
        sample_interval_s=args.sample_interval,
        odometry_output=args.odometry_output,
        odometry_topic=args.odometry_topic,
    )
    print(json.dumps(report, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: OpenLORIS scan corpus")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
