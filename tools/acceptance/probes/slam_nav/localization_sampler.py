"""AMCL 与 Gazebo truth 的采样、去重和坐标适配。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading

from embodied_agent_interfaces.msg import SlamSessionState
from geometry_msgs.msg import PoseWithCovarianceStamped

from tools.acceptance.unknown_world_evidence import (
    LocalizationSample,
    MapWorldTransform,
)


__all__ = ("GazeboTruthConfig", "LocalizationSampler")


_NAVIGATION_PHASES = frozenset(
    {
        SlamSessionState.STARTING_NAVIGATION,
        SlamSessionState.NAVIGATING,
        SlamSessionState.AUTOMATIC_NAVIGATING,
        SlamSessionState.MISSION_COMPLETED,
    }
)
_MAPPING_PHASES = frozenset(
    {SlamSessionState.MAPPING, SlamSessionState.AUTOMATIC_MAPPING}
)
_POSITION_EQUAL_TOLERANCE_M = 1e-9


@dataclass(frozen=True, slots=True)
class GazeboTruthConfig:
    world_name: str
    robot_entity_name: str
    transform: MapWorldTransform

    @property
    def topic(self) -> str:
        return f"/world/{self.world_name}/dynamic_pose/info"


def _ros_timestamp_s(stamp) -> float:
    """转换 ROS builtin_interfaces/Time；字段名是 ``nanosec``。"""

    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _gazebo_timestamp_s(stamp) -> float:
    """转换 Gazebo protobuf Time；它使用 ``nsec`` 而不是 ``nanosec``。"""

    return float(stamp.sec) + float(stamp.nsec) / 1_000_000_000.0


class LocalizationSampler:
    """采集同一 navigation stage 内可比较的 AMCL/truth 样本。"""

    def __init__(self, config: GazeboTruthConfig | None = None) -> None:
        self.config = config
        self.amcl_pose_count = 0
        self._amcl_samples: list[LocalizationSample] = []
        self._gazebo_samples: list[LocalizationSample] = []
        self._amcl_by_timestamp: dict[float, tuple[float, float]] = {}
        self._gazebo_by_timestamp: dict[float, tuple[float, float]] = {}
        self._evidence_error = ""
        self._gazebo_entity_id: int | None = None
        self._sampling = False
        self._lock = threading.RLock()

    @property
    def amcl_samples(self) -> list[LocalizationSample]:
        return self._amcl_samples

    @property
    def gazebo_samples(self) -> list[LocalizationSample]:
        return self._gazebo_samples

    @property
    def evidence_error(self) -> str:
        return self._evidence_error

    @property
    def sampling(self) -> bool:
        return self._sampling

    def transition_phase(self, phase: int) -> None:
        with self._lock:
            if phase in _NAVIGATION_PHASES and not self._sampling:
                # mapping/navigation 会重启 Gazebo，仿真时钟和 entity id 都会重置；
                # 跨 stage 混样会制造看似优秀但实际错误的时间对齐结果。
                self._amcl_samples.clear()
                self._gazebo_samples.clear()
                self._amcl_by_timestamp.clear()
                self._gazebo_by_timestamp.clear()
                self._evidence_error = ""
                self._gazebo_entity_id = None
                self._sampling = True
            elif phase in _MAPPING_PHASES:
                self._sampling = False

    def observe_amcl(self, message: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self.amcl_pose_count += 1
            if not self._sampling:
                return
            timestamp = _ros_timestamp_s(message.header.stamp)
            if timestamp <= 0.0:
                return
            position = message.pose.pose.position
            sample = LocalizationSample(
                timestamp,
                float(position.x),
                float(position.y),
            )
            self._append_unique_locked(
                "AMCL",
                sample,
                self._amcl_samples,
                self._amcl_by_timestamp,
            )

    def observe_gazebo(self, message) -> None:
        with self._lock:
            if not self._sampling or self.config is None:
                return
            matches = [
                pose
                for pose in message.pose
                if str(pose.name) == self.config.robot_entity_name
            ]
            if not matches:
                return
            if len(matches) != 1:
                self._evidence_error = (
                    "Gazebo truth contains duplicate robot entity names"
                )
                return
            pose = matches[0]
            entity_id = int(pose.id)
            if self._gazebo_entity_id is None:
                self._gazebo_entity_id = entity_id
            elif self._gazebo_entity_id != entity_id:
                self._evidence_error = (
                    "Gazebo robot entity id changed within one navigation stage"
                )
                return
            timestamp = _gazebo_timestamp_s(message.header.stamp)
            if timestamp <= 0.0:
                return
            x_m, y_m = self.config.transform.world_to_map(
                float(pose.position.x),
                float(pose.position.y),
            )
            self._append_unique_locked(
                "Gazebo truth",
                LocalizationSample(timestamp, x_m, y_m),
                self._gazebo_samples,
                self._gazebo_by_timestamp,
            )

    def evidence(
        self,
    ) -> tuple[
        tuple[LocalizationSample, ...],
        tuple[LocalizationSample, ...],
        str,
    ]:
        with self._lock:
            return (
                tuple(self._amcl_samples),
                tuple(self._gazebo_samples),
                self._evidence_error,
            )

    def _append_unique_locked(
        self,
        source: str,
        sample: LocalizationSample,
        samples: list[LocalizationSample],
        index: dict[float, tuple[float, float]],
    ) -> None:
        previous_pose = index.get(sample.timestamp_s)
        current_pose = (sample.x_m, sample.y_m)
        if previous_pose is not None:
            if all(
                math.isclose(
                    previous,
                    current,
                    rel_tol=0.0,
                    abs_tol=_POSITION_EQUAL_TOLERANCE_M,
                )
                for previous, current in zip(previous_pose, current_pose)
            ):
                # DDS/Gazebo 可能重投递同一帧；保留一份即可，避免虚增统计样本量。
                return
            self._evidence_error = (
                f"{source} has conflicting poses at timestamp {sample.timestamp_s:.9f}"
            )
            return
        if samples and sample.timestamp_s < samples[-1].timestamp_s:
            self._evidence_error = (
                f"{source} timestamp moved backwards within navigation stage"
            )
            return
        index[sample.timestamp_s] = current_pose
        samples.append(sample)
