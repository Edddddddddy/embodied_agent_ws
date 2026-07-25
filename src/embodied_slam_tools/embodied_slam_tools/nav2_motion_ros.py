"""Nav2MotionTransaction 的 ROS 2 生产 Adapter。"""

from __future__ import annotations

import math
import time
from typing import Callable

from nav2_msgs.action import BackUp, NavigateToPose

from .nav2_motion_transaction import (
    ActionTerminal,
    BackupGoal,
    GoalHandlePort,
    GoalResponseFuturePort,
    GoalResultFuturePort,
    NavigateGoal,
    Nav2ActionKind,
    RuntimeGoal,
)


class _RosResultFutureAdapter(GoalResultFuturePort):
    """把 rclpy result future 收紧为与 ROS 消息解耦的终态值。"""

    def __init__(self, future, *, kind: Nav2ActionKind) -> None:
        self._future = future
        self._kind = kind

    def done(self) -> bool:
        return bool(self._future.done())

    def result(self) -> ActionTerminal | None:
        wrapper = self._future.result()
        if wrapper is None:
            return None
        payload = getattr(wrapper, "result", None)
        if payload is None:
            # Future 完成不等于 Action 终态可审计；缺 payload 必须让事务
            # 进入 fail-safe，不能把协议层“完成”误判成业务成功。
            raise RuntimeError("Nav2 terminal result payload is missing")
        status = int(wrapper.status)
        default_error = (
            BackUp.Result.UNKNOWN
            if self._kind is Nav2ActionKind.BACK_UP
            else getattr(NavigateToPose.Result, "UNKNOWN", -1)
        )
        error_code = int(getattr(payload, "error_code", default_error))
        error_message = str(
            getattr(
                payload,
                "error_msg",
                getattr(payload, "error_message", ""),
            )
        ).strip()
        return ActionTerminal(
            status=status,
            error_code=error_code,
            error_message=error_message,
        )


class _RosGoalHandleAdapter(GoalHandlePort):
    def __init__(self, handle, *, kind: Nav2ActionKind) -> None:
        self._handle = handle
        self._kind = kind

    @property
    def accepted(self) -> bool:
        return bool(self._handle.accepted)

    def get_result_async(self) -> GoalResultFuturePort:
        return _RosResultFutureAdapter(
            self._handle.get_result_async(),
            kind=self._kind,
        )

    def cancel_goal_async(self) -> object:
        return self._handle.cancel_goal_async()


class _RosGoalResponseFutureAdapter(GoalResponseFuturePort):
    def __init__(self, future, *, kind: Nav2ActionKind) -> None:
        self._future = future
        self._kind = kind

    def done(self) -> bool:
        return bool(self._future.done())

    def result(self) -> GoalHandlePort | None:
        handle = self._future.result()
        if handle is None:
            return None
        return _RosGoalHandleAdapter(handle, kind=self._kind)


class RosNav2MotionAdapter:
    """连接领域事务与 rclpy ActionClient、安全停车及证据发布。

    这里是唯一理解 ROS goal/result 消息的 seam；事务 Module 和测试只依赖
    稳定的领域值，不需要构造 rclpy Node 或生成的 ROS 消息。
    """

    def __init__(
        self,
        *,
        navigate_to_pose_client,
        backup_client,
        force_priority_stop: Callable[[float], None],
        stop_navigation_stage: Callable[[], None],
        nav2_goal_started: Callable[[str], None],
        nav2_goal_terminal: Callable[[str], None],
        evidence_timestamp_ns: Callable[[], int],
        navigation_evidence_changed: Callable[[], None],
    ) -> None:
        callbacks = (
            force_priority_stop,
            stop_navigation_stage,
            nav2_goal_started,
            nav2_goal_terminal,
            evidence_timestamp_ns,
            navigation_evidence_changed,
        )
        if not all(callable(callback) for callback in callbacks):
            raise TypeError("Nav2 ROS Adapter callbacks must be callable")
        self._navigate_to_pose_client = navigate_to_pose_client
        self._backup_client = backup_client
        self._force_priority_stop = force_priority_stop
        self._stop_navigation_stage = stop_navigation_stage
        self._nav2_goal_started = nav2_goal_started
        self._nav2_goal_terminal = nav2_goal_terminal
        self._evidence_timestamp_ns = evidence_timestamp_ns
        self._navigation_evidence_changed = navigation_evidence_changed

    @staticmethod
    def monotonic() -> float:
        return time.monotonic()

    @staticmethod
    def sleep(seconds: float) -> None:
        time.sleep(seconds)

    def evidence_timestamp_ns(self) -> int:
        return int(self._evidence_timestamp_ns())

    def wait_for_server(
        self,
        kind: Nav2ActionKind,
        *,
        timeout_s: float,
    ) -> bool:
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("Nav2 server wait timeout must be finite and non-negative")
        client = self._client_for(kind)
        return bool(client.wait_for_server(timeout_sec=float(timeout_s)))

    def send_goal(self, goal: RuntimeGoal) -> GoalResponseFuturePort:
        if isinstance(goal, NavigateGoal):
            ros_goal = NavigateToPose.Goal()
            ros_goal.pose.header.frame_id = goal.frame_id
            # map-frame 目标使用 zero/latest TF，避免低实时率仿真中目标 stamp
            # 短暂领先 map->odom，导致 Nav2 因 extrapolation 拒绝目标。
            ros_goal.pose.pose.position.x = float(goal.x)
            ros_goal.pose.pose.position.y = float(goal.y)
            ros_goal.pose.pose.orientation.z = math.sin(goal.yaw * 0.5)
            ros_goal.pose.pose.orientation.w = math.cos(goal.yaw * 0.5)
            kind = Nav2ActionKind.NAVIGATE_TO_POSE
            future = self._navigate_to_pose_client.send_goal_async(ros_goal)
        elif isinstance(goal, BackupGoal):
            ros_goal = BackUp.Goal()
            # Jazzy BackUp 接收正的距离/速度幅值；Behavior Server 内部转换
            # 成机器人负 X 运动，并持续用 local costmap 做碰撞预测。
            ros_goal.target.x = float(goal.distance_m)
            ros_goal.target.y = 0.0
            ros_goal.target.z = 0.0
            ros_goal.speed = float(goal.speed_mps)
            seconds = int(goal.time_allowance_s)
            nanoseconds = int(
                round((goal.time_allowance_s - seconds) * 1_000_000_000)
            )
            if nanoseconds >= 1_000_000_000:
                seconds += 1
                nanoseconds -= 1_000_000_000
            ros_goal.time_allowance.sec = seconds
            ros_goal.time_allowance.nanosec = nanoseconds
            kind = Nav2ActionKind.BACK_UP
            future = self._backup_client.send_goal_async(ros_goal)
        else:
            raise TypeError(f"unsupported Nav2 runtime goal: {type(goal)!r}")
        return _RosGoalResponseFutureAdapter(future, kind=kind)

    def force_priority_stop(self, *, timeout_s: float) -> None:
        self._force_priority_stop(float(timeout_s))

    def stop_navigation_stage(self) -> None:
        self._stop_navigation_stage()

    def nav2_goal_started(self, token: str) -> None:
        self._nav2_goal_started(token)

    def nav2_goal_terminal(self, token: str) -> None:
        self._nav2_goal_terminal(token)

    def navigation_evidence_changed(self) -> None:
        self._navigation_evidence_changed()

    def _client_for(self, kind: Nav2ActionKind):
        if kind is Nav2ActionKind.NAVIGATE_TO_POSE:
            return self._navigate_to_pose_client
        if kind is Nav2ActionKind.BACK_UP:
            return self._backup_client
        raise ValueError(f"unsupported Nav2 action kind: {kind!r}")
