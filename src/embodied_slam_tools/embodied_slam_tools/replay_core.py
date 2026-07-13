"""ROS-independent timing, frame and planar-pose logic for dataset replay."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def compose_pose(parent_to_child: Pose2, child_to_base: Pose2) -> Pose2:
    """Compose ``T_parent_child * T_child_base`` in SE(2)."""

    cosine = math.cos(parent_to_child.yaw)
    sine = math.sin(parent_to_child.yaw)
    return Pose2(
        x=parent_to_child.x + cosine * child_to_base.x - sine * child_to_base.y,
        y=parent_to_child.y + sine * child_to_base.x + cosine * child_to_base.y,
        yaw=wrap_angle(parent_to_child.yaw + child_to_base.yaw),
    )


class FrameMapper:
    """Map dataset frames into an isolated TF tree without leaking raw names."""

    def __init__(self, prefix: str = "dataset") -> None:
        clean = prefix.strip("/_")
        if not clean:
            raise ValueError("frame prefix must not be empty")
        self._prefix = clean
        self._known = {
            "base_odom": f"{clean}_odom",
            "odom": f"{clean}_odom",
            "base_link": f"{clean}_base_link",
            "laser": f"{clean}_laser",
        }

    def map(self, frame: str) -> str:
        clean = frame.strip().lstrip("/")
        if not clean:
            return ""
        if clean in self._known:
            return self._known[clean]
        if clean.startswith(f"{self._prefix}_"):
            return clean
        return f"{self._prefix}_{clean.replace('/', '_')}"


class TimestampDeduplicator:
    """Reject duplicate or backward sensor stamps while keeping bounded state."""

    def __init__(self) -> None:
        self._last_by_stream: dict[str, int] = {}

    def accept(self, stream: str, stamp_ns: int) -> bool:
        previous = self._last_by_stream.get(stream)
        if previous is not None and stamp_ns <= previous:
            return False
        self._last_by_stream[stream] = stamp_ns
        return True


class ReplayTimeline:
    """Translate bag timestamps into monotonic simulated time and wall delays."""

    def __init__(self, rate: float, max_wall_gap_s: float = 1.0) -> None:
        if rate <= 0.0:
            raise ValueError("replay rate must be positive")
        if max_wall_gap_s <= 0.0:
            raise ValueError("max_wall_gap_s must be positive")
        self._rate = rate
        self._max_wall_gap_s = max_wall_gap_s
        self._last_bag_ns: int | None = None
        self._last_clock_ns: int | None = None

    def advance(self, bag_stamp_ns: int) -> tuple[int, float]:
        # rosbag 中偶发的倒序写入不能让 /clock 回跳，否则 TF buffer 会整体失效。
        clock_ns = bag_stamp_ns
        if self._last_clock_ns is not None:
            clock_ns = max(clock_ns, self._last_clock_ns)
        wall_delay_s = 0.0
        if self._last_bag_ns is not None:
            bag_delta_s = max(0.0, (bag_stamp_ns - self._last_bag_ns) * 1e-9)
            wall_delay_s = min(self._max_wall_gap_s, bag_delta_s / self._rate)
        self._last_bag_ns = max(bag_stamp_ns, self._last_bag_ns or bag_stamp_ns)
        self._last_clock_ns = clock_ns
        return clock_ns, wall_delay_s


def stamp_to_ns(stamp, fallback_ns: int) -> int:
    seconds = getattr(stamp, "sec", getattr(stamp, "secs", None))
    nanoseconds = getattr(stamp, "nanosec", getattr(stamp, "nsecs", None))
    if seconds is None or nanoseconds is None:
        return fallback_ns
    return int(seconds) * 1_000_000_000 + int(nanoseconds)
