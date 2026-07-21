"""未知环境建图返航的纯领域契约。

本模块不依赖 ROS，ROS 适配层只需把位姿、Action 终态和速度快照转换为这里的
不可变对象。这样返航验收既能在单元测试中独立运行，也不会把“请求已发送”误判为
“机器人已经安全回到起点”。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math


class ReturnActionKind(Enum):
    """返航使用的强类型动作；避免自由字符串拼写绕过契约。"""

    NAVIGATE_TO_POSE = "navigate_to_pose"


class ReturnActionStatus(Enum):
    """返航 Action 的任务层终态/中间态。"""

    UNSPECIFIED = "unspecified"
    ACCEPTED = "accepted"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    REJECTED = "rejected"
    ABORTED = "aborted"
    CANCELED = "canceled"
    TIMED_OUT = "timed_out"


@dataclass(frozen=True, slots=True)
class PlanarPose:
    """同一二维坐标系中的位姿快照。"""

    x: float
    y: float
    yaw: float
    frame_id: str
    observed_at_ns: int

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("planar pose values must be finite")
        if not self.frame_id.strip():
            raise ValueError("planar pose frame_id must not be empty")
        if self.observed_at_ns < 0:
            raise ValueError("planar pose timestamp must be non-negative")


@dataclass(frozen=True, slots=True)
class ReturnToStartSpec:
    """返航成功阈值；默认值面向 TurtleBot3 仿真验收。"""

    max_xy_error_m: float = 0.35
    max_yaw_error_rad: float = math.radians(20.0)
    zero_velocity_tolerance: float = 1.0e-3
    max_cmd_vel_age_s: float = 1.0

    def __post_init__(self) -> None:
        positive = (
            self.max_xy_error_m,
            self.max_yaw_error_rad,
            self.zero_velocity_tolerance,
            self.max_cmd_vel_age_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("return-to-start thresholds must be finite and positive")
        if self.max_yaw_error_rad > math.pi:
            raise ValueError("max_yaw_error_rad must not exceed pi")


@dataclass(frozen=True, slots=True)
class ReturnToStartEvidence:
    """一次返航事务的原始证据。

    ``evaluated_at_ns`` 与其余时间戳必须来自同一时钟域。速度使用最后一帧
    ``cmd_vel``，而不是执行器内部的期望速度。
    """

    start_pose: PlanarPose
    final_pose: PlanarPose | None
    action_kind: ReturnActionKind
    action_status: ReturnActionStatus
    action_command_id: str
    action_started_at_ns: int
    action_finished_at_ns: int
    map_saved_at_ns: int
    cmd_vel_linear_x: float
    cmd_vel_angular_z: float
    cmd_vel_observed_at_ns: int
    evaluated_at_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.action_kind, ReturnActionKind):
            raise TypeError("action_kind must be ReturnActionKind")
        if not isinstance(self.action_status, ReturnActionStatus):
            raise TypeError("action_status must be ReturnActionStatus")
        timestamps = (
            self.action_started_at_ns,
            self.action_finished_at_ns,
            self.map_saved_at_ns,
            self.cmd_vel_observed_at_ns,
            self.evaluated_at_ns,
        )
        if any(value < 0 for value in timestamps):
            raise ValueError("return-to-start timestamps must be non-negative")
        if not all(
            math.isfinite(value)
            for value in (self.cmd_vel_linear_x, self.cmd_vel_angular_z)
        ):
            raise ValueError("cmd_vel values must be finite")


@dataclass(frozen=True, slots=True)
class ReturnToStartDecision:
    """纯判定结果，便于直接写入验收报告。"""

    passed: bool
    checks: tuple[tuple[str, bool], ...]
    failed_checks: tuple[str, ...]
    xy_error_m: float | None
    yaw_error_rad: float | None
    cmd_vel_age_s: float | None

    def check(self, name: str) -> bool:
        return dict(self.checks)[name]


def wrapped_yaw_error_rad(expected: float, actual: float) -> float:
    """返回 ``[-pi, pi]`` 环绕后的最短角距离绝对值。"""

    if not math.isfinite(expected) or not math.isfinite(actual):
        raise ValueError("yaw values must be finite")
    return abs(math.atan2(math.sin(actual - expected), math.cos(actual - expected)))


def _within_limit(value: float, limit: float) -> bool:
    """包含阈值边界，同时吸收二进制浮点在十进制阈值处的微小舍入误差。"""

    return value <= limit or math.isclose(value, limit, rel_tol=0.0, abs_tol=1.0e-12)


def evaluate_return_to_start(
    evidence: ReturnToStartEvidence,
    spec: ReturnToStartSpec = ReturnToStartSpec(),
) -> ReturnToStartDecision:
    """按 fail-closed 原则判定返航事务是否完整成功。

    返航不是单一 Action result：还必须证明最终位姿靠近起点、地图保存发生在返航
    之后，并且验收时拿到了足够新的零速度。任一证据缺失都会返回失败。
    """

    final_pose = evidence.final_pose
    xy_error_m = (
        math.hypot(
            final_pose.x - evidence.start_pose.x,
            final_pose.y - evidence.start_pose.y,
        )
        if final_pose is not None
        else None
    )
    yaw_error_rad = (
        wrapped_yaw_error_rad(evidence.start_pose.yaw, final_pose.yaw)
        if final_pose is not None
        else None
    )

    timeline_valid = (
        evidence.start_pose.observed_at_ns <= evidence.action_started_at_ns
        < evidence.action_finished_at_ns
        < evidence.map_saved_at_ns
        <= evidence.evaluated_at_ns
    )
    final_pose_in_return_window = bool(
        final_pose is not None
        and evidence.action_finished_at_ns <= final_pose.observed_at_ns
        <= evidence.map_saved_at_ns
    )
    cmd_vel_time_valid = (
        evidence.action_finished_at_ns <= evidence.cmd_vel_observed_at_ns
        <= evidence.evaluated_at_ns
    )
    cmd_vel_age_s = (
        (evidence.evaluated_at_ns - evidence.cmd_vel_observed_at_ns) / 1.0e9
        if evidence.cmd_vel_observed_at_ns <= evidence.evaluated_at_ns
        else None
    )

    checks = (
        ("typed_action_command", bool(evidence.action_command_id.strip())),
        (
            "typed_action_succeeded",
            evidence.action_kind is ReturnActionKind.NAVIGATE_TO_POSE
            and evidence.action_status is ReturnActionStatus.SUCCEEDED,
        ),
        ("return_before_map_save", timeline_valid),
        (
            "same_pose_frame",
            final_pose is not None
            and final_pose.frame_id == evidence.start_pose.frame_id,
        ),
        ("final_pose_in_return_window", final_pose_in_return_window),
        (
            "xy_within_tolerance",
            xy_error_m is not None
            and _within_limit(xy_error_m, spec.max_xy_error_m),
        ),
        (
            "yaw_within_tolerance",
            yaw_error_rad is not None
            and _within_limit(yaw_error_rad, spec.max_yaw_error_rad),
        ),
        ("cmd_vel_after_return", cmd_vel_time_valid),
        (
            "final_cmd_vel_fresh",
            cmd_vel_age_s is not None
            and cmd_vel_age_s <= spec.max_cmd_vel_age_s,
        ),
        (
            "final_cmd_vel_zero",
            abs(evidence.cmd_vel_linear_x) <= spec.zero_velocity_tolerance
            and abs(evidence.cmd_vel_angular_z) <= spec.zero_velocity_tolerance,
        ),
    )
    failed_checks = tuple(name for name, passed in checks if not passed)
    return ReturnToStartDecision(
        passed=not failed_checks,
        checks=checks,
        failed_checks=failed_checks,
        xy_error_m=xy_error_m,
        yaw_error_rad=yaw_error_rad,
        cmd_vel_age_s=cmd_vel_age_s,
    )


def build_return_to_start_report(
    evidence: ReturnToStartEvidence,
    spec: ReturnToStartSpec = ReturnToStartSpec(),
) -> dict[str, object]:
    """生成稳定的验收报告结构，供 ROS Adapter 和离线 evaluator 共用。"""

    decision = evaluate_return_to_start(evidence, spec)
    return {
        "schema_version": 1,
        "passed": decision.passed,
        # evaluator 必须逐项复核 checks，不能只相信一个可手工写入的 passed 位。
        "checks": dict(decision.checks),
        "failed_checks": list(decision.failed_checks),
        "thresholds": {
            "max_xy_error_m": spec.max_xy_error_m,
            "max_yaw_error_rad": spec.max_yaw_error_rad,
            "zero_velocity_tolerance": spec.zero_velocity_tolerance,
            "max_cmd_vel_age_s": spec.max_cmd_vel_age_s,
        },
        "errors": {
            "xy_error_m": decision.xy_error_m,
            "yaw_error_rad": decision.yaw_error_rad,
        },
        "start_pose": asdict(evidence.start_pose),
        "final_pose": asdict(evidence.final_pose) if evidence.final_pose else None,
        "action": {
            "kind": evidence.action_kind.value,
            "status": evidence.action_status.value,
            "command_id": evidence.action_command_id,
            "started_at_ns": evidence.action_started_at_ns,
            "finished_at_ns": evidence.action_finished_at_ns,
        },
        "map_saved_at_ns": evidence.map_saved_at_ns,
        "final_cmd_vel": {
            "linear_x": evidence.cmd_vel_linear_x,
            "angular_z": evidence.cmd_vel_angular_z,
            "observed_at_ns": evidence.cmd_vel_observed_at_ns,
            "age_s": decision.cmd_vel_age_s,
        },
        "evaluated_at_ns": evidence.evaluated_at_ns,
    }
