"""Nav2 sampled goal 与 ``/plan`` 的跨 topic 关联模块。

ROS 2 只保证单 topic 内的消息顺序。本模块把 mission 代际、目标生命周期、
路径终点和短暂乱序窗口收拢在一个深模块中，调用方只需提交 typed state/path。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

from embodied_agent_interfaces.msg import (
    SlamNavigationGoalEvidence,
    SlamSessionState,
)
from nav_msgs.msg import Path as NavPath


__all__ = ("SampledGoalPlanSnapshot", "SampledGoalPlanTracker")


_ACTIVE_SAMPLED_GOAL_STATUSES = frozenset(
    {
        SlamNavigationGoalEvidence.STATUS_REQUESTED,
        SlamNavigationGoalEvidence.STATUS_ACCEPTED,
        SlamNavigationGoalEvidence.STATUS_EXECUTING,
    }
)
_SAMPLED_PLAN_ENDPOINT_TOLERANCE_M = 0.35
_PENDING_SAMPLED_PLAN_LIMIT = 16
_PENDING_SAMPLED_PLAN_TTL_S = 4.0
_SYSTEM_CLOCK_EPOCH_FLOOR_S = 1_000_000_000.0


def _ros_timestamp_s(stamp) -> float:
    """转换 ROS builtin_interfaces/Time；字段名是 ``nanosec``。"""

    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _sampled_goal_message_to_dict(message) -> dict[str, object]:
    snapshot: dict[str, object] = {
        "sequence": int(message.sequence),
        "x": float(message.goal.pose.position.x),
        "y": float(message.goal.pose.position.y),
        "frame_id": str(message.goal.header.frame_id),
        "status": int(message.status),
        "nav2_status": int(message.nav2_status),
        "nav2_error_code": int(message.nav2_error_code),
        "plan_count": int(message.plan_count),
        "max_unknown_cell_count": int(message.max_unknown_cell_count),
        "max_occupied_cell_count": int(message.max_occupied_cell_count),
        "max_outside_map_cell_count": int(message.max_outside_map_cell_count),
        "all_plans_known_free": bool(message.all_plans_known_free),
        "detail": str(message.detail),
    }
    # 生命周期时间用于排除上一个目标的迟到 plan；兼容尚无字段的旧消息对象。
    if hasattr(message, "started_at"):
        snapshot["started_at"] = _ros_timestamp_s(message.started_at)
    if hasattr(message, "finished_at"):
        snapshot["finished_at"] = _ros_timestamp_s(message.finished_at)
    return snapshot


def _navigation_plan_endpoint(message: NavPath) -> tuple[float, float] | None:
    """返回可信的 map 路径终点；拒绝空路径和混合坐标系。"""

    if str(message.header.frame_id) != "map" or not message.poses:
        return None
    if any(str(pose.header.frame_id) not in {"", "map"} for pose in message.poses):
        return None
    endpoint = message.poses[-1].pose.position
    return float(endpoint.x), float(endpoint.y)


def _timestamps_share_clock_domain(left_s: float, right_s: float) -> bool:
    """区分 Unix 系统时钟和从零起步的 Gazebo 仿真时钟。"""

    if left_s <= 0.0 or right_s <= 0.0:
        return False
    # orchestrator 常驻于 Gazebo stage 外部，typed 生命周期使用 SYSTEM_TIME；
    # Nav2 启用 use_sim_time 后 Path stamp 则来自 /clock。两者数值不可排序，
    # 但仍可依靠 mission 代际、active 状态、endpoint 与有界乱序缓存做关联。
    return (
        left_s >= _SYSTEM_CLOCK_EPOCH_FLOOR_S
    ) == (
        right_s >= _SYSTEM_CLOCK_EPOCH_FLOOR_S
    )


@dataclass(frozen=True, slots=True)
class _PendingSampledPlan:
    path: NavPath
    endpoint_x: float
    endpoint_y: float
    mission_sequence: int
    received_at_s: float
    path_stamp_s: float


@dataclass(frozen=True, slots=True)
class SampledGoalPlanSnapshot:
    """一次加锁读取获得的自洽快照，供证据生成或诊断使用。"""

    mission_sequence: int
    goals: dict[int, dict[str, object]]
    status_history: dict[int, tuple[int, ...]]
    plans: dict[int, tuple[NavPath, ...]]
    navigation_plans: tuple[NavPath, ...]
    active_sequence: int | None
    evidence_error: str


class SampledGoalPlanTracker:
    """将 typed goal state 与无 request-id 的 Nav2 plan 安全关联。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._mission_sequence = 0
        self._goals: dict[int, dict[str, object]] = {}
        self._status_history: dict[int, list[int]] = {}
        self._plans: dict[int, list[NavPath]] = {}
        self._navigation_plans: list[NavPath] = []
        self._active_sequence: int | None = None
        self._evidence_error = ""
        self._pending: deque[_PendingSampledPlan] = deque(
            maxlen=_PENDING_SAMPLED_PLAN_LIMIT
        )

    @property
    def mission_sequence(self) -> int:
        with self._lock:
            return self._mission_sequence

    @property
    def goals(self) -> dict[int, dict[str, object]]:
        return self.snapshot().goals

    @property
    def status_history(self) -> dict[int, list[int]]:
        return {key: list(value) for key, value in self.snapshot().status_history.items()}

    @property
    def plans(self) -> dict[int, list[NavPath]]:
        return {key: list(value) for key, value in self.snapshot().plans.items()}

    @property
    def navigation_plans(self) -> list[NavPath]:
        return list(self.snapshot().navigation_plans)

    @property
    def active_sequence(self) -> int | None:
        with self._lock:
            return self._active_sequence

    @property
    def evidence_error(self) -> str:
        with self._lock:
            return self._evidence_error

    @property
    def pending_plans(self) -> deque[_PendingSampledPlan]:
        """仅为旧探针诊断兼容；业务调用应使用 :meth:`snapshot`."""

        return self._pending

    def update_state(self, message: SlamSessionState) -> None:
        incoming_mission = int(getattr(message, "mission_sequence", 0))
        sampled = tuple(getattr(message, "sampled_navigation_goals", ()))
        now_s = time.monotonic()
        with self._lock:
            previous_mission = self._mission_sequence
            if incoming_mission > 0 and incoming_mission != previous_mission:
                self._reset_mission_locked(previous_mission)
            self._mission_sequence = incoming_mission
            active: list[int] = []
            for item in sampled:
                goal = _sampled_goal_message_to_dict(item)
                goal["mission_sequence"] = incoming_mission
                sequence = int(goal["sequence"])
                previous = self._goals.get(sequence)
                if previous is None or previous["status"] != goal["status"]:
                    self._status_history.setdefault(sequence, []).append(
                        int(goal["status"])
                    )
                self._goals[sequence] = goal
                if int(goal["status"]) in _ACTIVE_SAMPLED_GOAL_STATUSES:
                    active.append(sequence)
            if len(active) > 1:
                self._evidence_error = (
                    "multiple sampled navigation goals active concurrently"
                )
            self._active_sequence = active[0] if len(active) == 1 else None
            # `/plan` 与 typed state 不共享 DDS 顺序；state 到达后必须在同一把锁
            # 下重放短缓存，否则并行 callback 可能把旧目标路径错绑到新目标。
            self._replay_pending_locked(now_s)

    def observe_plan(self, message: NavPath) -> bool:
        endpoint = _navigation_plan_endpoint(message)
        if endpoint is None:
            return False
        now_s = time.monotonic()
        path_stamp_s = _ros_timestamp_s(message.header.stamp)
        with self._lock:
            pending = _PendingSampledPlan(
                path=message,
                endpoint_x=endpoint[0],
                endpoint_y=endpoint[1],
                mission_sequence=self._mission_sequence,
                received_at_s=now_s,
                path_stamp_s=path_stamp_s,
            )
            self._navigation_plans.append(message)
            sequence = self._matching_active_sequence_locked(pending)
            if sequence is not None:
                self._plans.setdefault(sequence, []).append(message)
                return True
            # 无 request-id 的 `/plan` 不能见到 active goal 就盲绑；有界 TTL
            # 同时容忍正常跨 topic 乱序，并限制无关消息的内存与信任范围。
            self._prune_pending_locked(now_s)
            self._pending.append(pending)
            return True

    def snapshot(self) -> SampledGoalPlanSnapshot:
        with self._lock:
            return SampledGoalPlanSnapshot(
                mission_sequence=self._mission_sequence,
                goals={key: dict(value) for key, value in self._goals.items()},
                status_history={
                    key: tuple(value) for key, value in self._status_history.items()
                },
                plans={key: tuple(value) for key, value in self._plans.items()},
                navigation_plans=tuple(self._navigation_plans),
                active_sequence=self._active_sequence,
                evidence_error=self._evidence_error,
            )

    def _reset_mission_locked(self, previous_mission: int) -> None:
        self._goals.clear()
        self._status_history.clear()
        self._plans.clear()
        self._active_sequence = None
        self._evidence_error = ""
        # 启动时 mission=0 的 plan 可以先于第一条 state；真正跨 mission 时必须丢弃。
        if previous_mission > 0:
            self._pending.clear()

    def _prune_pending_locked(self, now_s: float) -> None:
        while (
            self._pending
            and now_s - self._pending[0].received_at_s
            > _PENDING_SAMPLED_PLAN_TTL_S
        ):
            self._pending.popleft()

    def _matching_active_sequence_locked(
        self, pending: _PendingSampledPlan
    ) -> int | None:
        sequence = self._active_sequence
        if sequence is None or self._mission_sequence <= 0:
            return None
        goal = self._goals.get(sequence)
        if goal is None or int(goal["status"]) not in _ACTIVE_SAMPLED_GOAL_STATUSES:
            return None
        if int(goal.get("mission_sequence", 0)) != self._mission_sequence:
            return None
        if pending.mission_sequence not in {0, self._mission_sequence}:
            return None
        if str(goal["frame_id"]) != "map":
            return None
        if math.hypot(
            pending.endpoint_x - float(goal["x"]),
            pending.endpoint_y - float(goal["y"]),
        ) > _SAMPLED_PLAN_ENDPOINT_TOLERANCE_M:
            return None
        started_at = float(goal.get("started_at", 0.0))
        if (
            started_at > 0.0
            and pending.path_stamp_s > 0.0
            and _timestamps_share_clock_domain(
                started_at, pending.path_stamp_s
            )
            and pending.path_stamp_s < started_at
        ):
            return None
        return sequence

    def _replay_pending_locked(self, now_s: float) -> None:
        self._prune_pending_locked(now_s)
        retained: deque[_PendingSampledPlan] = deque(
            maxlen=_PENDING_SAMPLED_PLAN_LIMIT
        )
        for pending in self._pending:
            sequence = self._matching_active_sequence_locked(pending)
            if sequence is not None:
                self._plans.setdefault(sequence, []).append(pending.path)
            elif pending.mission_sequence in {0, self._mission_sequence}:
                retained.append(pending)
        self._pending = retained
