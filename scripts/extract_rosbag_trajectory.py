#!/usr/bin/env python3
"""Export Odometry, Pose or TF from a ROS 1/2 bag to TUM trajectory format.

This is the optional rosbag Adapter at the trajectory-evaluation seam. It uses
``rosbags`` so ROS 1 OpenLORIS bags can be read inside the ROS 2 workspace
without installing a complete ROS 1 distribution.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import NamedTuple


class TrajectoryRow(NamedTuple):
    stamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def _normalise_frame(frame: str) -> str:
    return frame.lstrip("/")


def _stamp_seconds(header, fallback_ns: int) -> float:
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_ns * 1e-9
    seconds = getattr(stamp, "sec", getattr(stamp, "secs", None))
    nanoseconds = getattr(stamp, "nanosec", getattr(stamp, "nsecs", None))
    if seconds is None or nanoseconds is None:
        return fallback_ns * 1e-9
    return float(seconds) + float(nanoseconds) * 1e-9


def _row(stamp: float, position, orientation) -> TrajectoryRow:
    values = (
        stamp,
        float(position.x),
        float(position.y),
        float(position.z),
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("bag message contains non-finite pose data")
    return TrajectoryRow(*values)


def rows_from_message(
    message,
    msgtype: str,
    bag_timestamp_ns: int,
    *,
    parent_frame: str = "",
    child_frame: str = "",
) -> list[TrajectoryRow]:
    """Convert supported ROS message shapes without depending on generated classes."""

    if msgtype.endswith("/Odometry"):
        return [
            _row(
                _stamp_seconds(message.header, bag_timestamp_ns),
                message.pose.pose.position,
                message.pose.pose.orientation,
            )
        ]
    if msgtype.endswith("/PoseStamped"):
        return [
            _row(
                _stamp_seconds(message.header, bag_timestamp_ns),
                message.pose.position,
                message.pose.orientation,
            )
        ]
    if msgtype.endswith("/PoseWithCovarianceStamped"):
        return [
            _row(
                _stamp_seconds(message.header, bag_timestamp_ns),
                message.pose.pose.position,
                message.pose.pose.orientation,
            )
        ]
    if msgtype.endswith("/TFMessage"):
        rows: list[TrajectoryRow] = []
        for transform in message.transforms:
            parent = _normalise_frame(transform.header.frame_id)
            child = _normalise_frame(transform.child_frame_id)
            if parent_frame and parent != _normalise_frame(parent_frame):
                continue
            if child_frame and child != _normalise_frame(child_frame):
                continue
            rows.append(
                _row(
                    _stamp_seconds(transform.header, bag_timestamp_ns),
                    transform.transform.translation,
                    transform.transform.rotation,
                )
            )
        return rows
    raise ValueError(f"unsupported pose message type: {msgtype}")


def extract(
    bag: Path,
    topic: str,
    output: Path,
    *,
    parent_frame: str = "",
    child_frame: str = "",
) -> int:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "optional dependency missing; run: pip install -r requirements-slam-eval.txt"
        ) from exc
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    rows: list[TrajectoryRow] = []
    with AnyReader([bag], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            topics = ", ".join(sorted({connection.topic for connection in reader.connections}))
            raise ValueError(f"topic {topic!r} not found; available: {topics}")
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            rows.extend(
                rows_from_message(
                    message,
                    connection.msgtype,
                    timestamp,
                    parent_frame=parent_frame,
                    child_frame=child_frame,
                )
            )
    unique = {row.stamp: row for row in rows}
    ordered = [unique[stamp] for stamp in sorted(unique)]
    if len(ordered) < 3:
        raise ValueError("fewer than 3 poses exported; check topic and TF frame filters")
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# timestamp tx ty tz qx qy qz qw"]
    lines.extend(" ".join(f"{value:.9f}" for value in row) for row in ordered)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return len(ordered)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--parent-frame", default="")
    parser.add_argument("--child-frame", default="")
    args = parser.parse_args()
    count = extract(
        args.bag,
        args.topic,
        args.output,
        parent_frame=args.parent_frame,
        child_frame=args.child_frame,
    )
    print(f"PASS: exported {count} poses -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
