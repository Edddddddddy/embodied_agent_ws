"""线程安全的 ``/cmd_vel`` 终止证据。

本模块只回答两个问题：任务边界之后是否真的发生过运动，以及最后是否收到
边界之后的新零速。ROS subscription wiring 留在 SessionObserver。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import threading
import time


@dataclass(frozen=True, slots=True)
class MotionEvidenceSnapshot:
    linear_x: float
    angular_z: float
    sample_count: int
    nonzero_sample_count: int
    last_received_at_s: float
    final_stop_boundary_at_s: float
    samples_after_boundary: int
    nonzero_samples_after_boundary: int

    def as_dict(self) -> dict[str, float | int]:
        return asdict(self)

    def has_fresh_stop(self) -> bool:
        return bool(
            self.sample_count > 0
            and self.samples_after_boundary > 0
            and self.final_stop_boundary_at_s > 0.0
            and self.last_received_at_s >= self.final_stop_boundary_at_s
            and abs(self.linear_x) < 1e-6
            and abs(self.angular_z) < 1e-6
        )

    def has_fresh_motion_stop(self) -> bool:
        return self.nonzero_samples_after_boundary > 0 and self.has_fresh_stop()


class MotionEvidenceTracker:
    """拥有速度样本代际；调用方无需自己组合多个可变计数器。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._sample_count = 0
        self._nonzero_sample_count = 0
        self._last_received_at_s = 0.0
        self._boundary_at_s = 0.0
        self._boundary_sample_count = 0
        self._boundary_nonzero_count = 0

    def observe(
        self,
        linear_x: float,
        angular_z: float,
        *,
        received_at_s: float | None = None,
    ) -> None:
        with self._lock:
            self._linear_x = float(linear_x)
            self._angular_z = float(angular_z)
            self._sample_count += 1
            if abs(linear_x) >= 1e-3 or abs(angular_z) >= 1e-3:
                self._nonzero_sample_count += 1
            self._last_received_at_s = (
                time.monotonic() if received_at_s is None else received_at_s
            )

    def mark_boundary(
        self,
        *,
        boundary_at_s: float | None = None,
        only_if_unset: bool = False,
    ) -> float:
        with self._lock:
            if only_if_unset and self._boundary_at_s > 0.0:
                return self._boundary_at_s
            self._boundary_at_s = (
                time.monotonic() if boundary_at_s is None else boundary_at_s
            )
            self._boundary_sample_count = self._sample_count
            self._boundary_nonzero_count = self._nonzero_sample_count
            return self._boundary_at_s

    def snapshot(self) -> MotionEvidenceSnapshot:
        with self._lock:
            return MotionEvidenceSnapshot(
                linear_x=self._linear_x,
                angular_z=self._angular_z,
                sample_count=self._sample_count,
                nonzero_sample_count=self._nonzero_sample_count,
                last_received_at_s=self._last_received_at_s,
                final_stop_boundary_at_s=self._boundary_at_s,
                samples_after_boundary=max(
                    0, self._sample_count - self._boundary_sample_count
                ),
                nonzero_samples_after_boundary=max(
                    0,
                    self._nonzero_sample_count - self._boundary_nonzero_count,
                ),
            )
