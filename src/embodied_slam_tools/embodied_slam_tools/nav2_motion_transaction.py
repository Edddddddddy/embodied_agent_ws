"""Nav2 运动 Action 的安全事务；领域实现不依赖 rclpy 或 ROS 消息。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
import threading
from typing import Callable, Protocol, TypeAlias

from .mapping_evidence import NavigationGoalLedger, NavigationGoalStatus
from .mapping_return import PlanarPose
from .mission_executor import AutomaticMissionCancelled


class ActionStatus(IntEnum):
    """与 action_msgs/GoalStatus 数值保持一致的 ROS 无关快照。"""

    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    CANCELING = 3
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6


class Nav2ActionKind(str, Enum):
    NAVIGATE_TO_POSE = "navigate_to_pose"
    BACK_UP = "backup"


@dataclass(frozen=True, slots=True)
class SampledNavigate:
    """执行一个已经完成 preflight 并登记为 REQUESTED 的采样目标。"""

    sequence: int
    goal_xy: tuple[float, float]

    def __post_init__(self) -> None:
        if self.sequence <= 0:
            raise ValueError("sampled navigation sequence must be positive")
        if not all(math.isfinite(value) for value in self.goal_xy):
            raise ValueError("sampled navigation goal must be finite")


@dataclass(frozen=True, slots=True)
class MappingReturn:
    """在 mapping stage 内返回动态捕获的起点。"""

    goal_pose: PlanarPose


@dataclass(frozen=True, slots=True)
class RecoveryBackup:
    """用 Nav2 BackUp 行为执行一次局部恢复。"""

    distance_m: float
    speed_mps: float
    time_allowance_s: float

    def __post_init__(self) -> None:
        values = (self.distance_m, self.speed_mps, self.time_allowance_s)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("Nav2 backup values must be finite and positive")


MotionIntent: TypeAlias = SampledNavigate | MappingReturn | RecoveryBackup


@dataclass(frozen=True, slots=True)
class NavigateGoal:
    """供生产 Adapter 转换为 nav2_msgs/NavigateToPose.Goal。"""

    frame_id: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True, slots=True)
class BackupGoal:
    """供生产 Adapter 转换为 nav2_msgs/BackUp.Goal。"""

    distance_m: float
    speed_mps: float
    time_allowance_s: float


RuntimeGoal: TypeAlias = NavigateGoal | BackupGoal


@dataclass(frozen=True, slots=True)
class ActionTerminal:
    """生产 Adapter 从 ROS Action result wrapper 收紧出的稳定终态。"""

    status: int
    error_code: int
    error_message: str = ""


class GoalResultFuturePort(Protocol):
    def done(self) -> bool: ...

    def result(self) -> ActionTerminal | None: ...


class GoalHandlePort(Protocol):
    @property
    def accepted(self) -> bool: ...

    def get_result_async(self) -> GoalResultFuturePort: ...

    def cancel_goal_async(self) -> object: ...


class GoalResponseFuturePort(Protocol):
    def done(self) -> bool: ...

    def result(self) -> GoalHandlePort | None: ...


class RuntimePort(Protocol):
    """唯一生产 Adapter seam；Node 在此封装 ROS、时钟与安全停车。"""

    def monotonic(self) -> float: ...

    def evidence_timestamp_ns(self) -> int: ...

    def sleep(self, seconds: float) -> None: ...

    def wait_for_server(
        self,
        kind: Nav2ActionKind,
        *,
        timeout_s: float,
    ) -> bool: ...

    def send_goal(self, goal: RuntimeGoal) -> GoalResponseFuturePort: ...

    def force_priority_stop(self, *, timeout_s: float) -> None:
        """发布独立、未取消的 typed STOP，并等待新鲜零速度。"""

    def stop_navigation_stage(self) -> None: ...

    def nav2_goal_started(self, token: str) -> None: ...

    def nav2_goal_terminal(self, token: str) -> None: ...

    def navigation_evidence_changed(self) -> None: ...


class Nav2GoalRejected(RuntimeError):
    """Nav2 在执行前明确拒绝目标。"""


class Nav2MotionFailed(RuntimeError):
    """Action 已有可审计终态，但业务结果没有成功。"""


class Nav2SafetyFailure(RuntimeError):
    """无法证明运动 goal 已静默；navigation stage 已进入 fail-safe。"""

    def __init__(
        self,
        message: str,
        *,
        terminal: ActionTerminal | None = None,
    ) -> None:
        super().__init__(message)
        # cleanup 失败不等于 Action 终态未知。保留已经读到的 wrapper，
        # 让 ledger 同时表达“任务安全失败”和“Nav2 实际如何结束”。
        self.terminal = terminal


class Nav2TransactionBusy(RuntimeError):
    """同一个事务 Module 同时只允许一个运动 goal。"""


class Nav2MotionTransaction:
    """用一个 ``execute`` Interface 收口三类 Nav2 运动 Action。

    preflight 与 ``/plan`` 适配仍由 Node 负责；本 Module 隐藏 goal response、
    迟到 accepted、取消、priority STOP、terminal 证明、quiescence 和 sampled
    ledger 终态。
    """

    _TERMINAL_STATUSES = frozenset(
        {
            ActionStatus.SUCCEEDED,
            ActionStatus.CANCELED,
            ActionStatus.ABORTED,
        }
    )

    def __init__(
        self,
        runtime: RuntimePort,
        ledger: NavigationGoalLedger,
        *,
        safety_timeout_s: float = 10.0,
        poll_interval_s: float = 0.05,
        late_plan_drain_s: float = 0.3,
    ) -> None:
        positive = (safety_timeout_s, poll_interval_s)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("Nav2 transaction timeouts must be finite and positive")
        if not math.isfinite(late_plan_drain_s) or late_plan_drain_s < 0.0:
            raise ValueError(
                "late navigation plan drain must be finite and non-negative"
            )
        self._runtime = runtime
        self._ledger = ledger
        self._safety_timeout_s = float(safety_timeout_s)
        self._poll_interval_s = float(poll_interval_s)
        self._late_plan_drain_s = float(late_plan_drain_s)
        self._execution_lock = threading.Lock()
        self._token_lock = threading.Lock()
        self._token_sequence = 0

    def execute(
        self,
        intent: MotionIntent,
        *,
        is_cancelled: Callable[[], bool],
        deadline_monotonic: float,
    ) -> None:
        """执行一个运动事务；仅在业务成功且安全终态可证明时返回。"""

        if not isinstance(
            intent,
            (SampledNavigate, MappingReturn, RecoveryBackup),
        ):
            raise TypeError(f"unsupported Nav2 motion intent: {type(intent)!r}")
        if not callable(is_cancelled):
            raise TypeError("is_cancelled must be callable")
        if not math.isfinite(deadline_monotonic):
            raise ValueError("Nav2 motion deadline must be finite")
        if not self._execution_lock.acquire(blocking=False):
            raise Nav2TransactionBusy("another Nav2 motion transaction is active")
        try:
            if isinstance(intent, SampledNavigate):
                self._execute_sampled(
                    intent,
                    is_cancelled=is_cancelled,
                    deadline_monotonic=deadline_monotonic,
                )
            elif isinstance(intent, MappingReturn):
                terminal = self._execute_motion(
                    intent,
                    is_cancelled=is_cancelled,
                    deadline_monotonic=deadline_monotonic,
                )
                self._require_success(terminal, label="mapping return")
            else:
                terminal = self._execute_motion(
                    intent,
                    is_cancelled=is_cancelled,
                    deadline_monotonic=deadline_monotonic,
                )
                if not self._terminal_succeeded(terminal):
                    # BackUp 插件已经终止仍要补一次 typed STOP，避免异常终态
                    # 遗留最后一帧速度；这不是 Nav2 terminal 的替代证据。
                    reason = (
                        "Nav2 BackUp failed: "
                        f"status={terminal.status} "
                        f"error={terminal.error_code} "
                        f"message={terminal.error_message or 'unknown'}"
                    )
                    try:
                        self._runtime.force_priority_stop(
                            timeout_s=self._safety_timeout_s
                        )
                    except Exception as exc:
                        # 业务失败是主错误；STOP 失败属于 cleanup 证据，
                        # 两者必须同时留在诊断中并触发 stage fail-safe。
                        self._fail_safety(
                            reason,
                            [f"typed stop failed: {exc}"],
                            terminal=terminal,
                        )
                    raise Nav2MotionFailed(reason)
        finally:
            self._execution_lock.release()

    def _execute_sampled(
        self,
        intent: SampledNavigate,
        *,
        is_cancelled: Callable[[], bool],
        deadline_monotonic: float,
    ) -> None:
        record = self._ledger.get(intent.sequence)
        if record.goal_xy != intent.goal_xy:
            raise ValueError(
                "sampled navigation intent does not match planned ledger goal"
            )
        if record.status is not NavigationGoalStatus.REQUESTED:
            raise ValueError(
                "sampled navigation goal must be REQUESTED before execution"
            )

        try:
            terminal = self._execute_motion(
                intent,
                is_cancelled=is_cancelled,
                deadline_monotonic=deadline_monotonic,
            )
            succeeded = self._terminal_succeeded(terminal)
            if succeeded:
                # 成功没有待保护的业务异常，保持正常发布语义；发布失败应直接
                # 暴露给调用方，而不是伪装成一次 Nav2 业务失败。
                self._transition_sampled(
                    intent.sequence,
                    NavigationGoalStatus.SUCCEEDED,
                    nav2_status=int(terminal.status),
                    nav2_error_code=int(terminal.error_code),
                    detail="NavigateToPose succeeded",
                )
                return

            terminal_status = (
                NavigationGoalStatus.CANCELED
                if int(terminal.status) == int(ActionStatus.CANCELED)
                else NavigationGoalStatus.ABORTED
            )
            primary = Nav2MotionFailed(
                "navigation goal failed: "
                f"goal={intent.goal_xy} "
                f"status={terminal.status} "
                f"error={terminal.error_code}"
            )
            self._transition_sampled_preserving(
                primary,
                intent.sequence,
                terminal_status,
                nav2_status=int(terminal.status),
                nav2_error_code=int(terminal.error_code),
                detail=(
                    "NavigateToPose failed "
                    f"status={terminal.status} "
                    f"error={terminal.error_code}"
                ),
            )
            raise primary
        except AutomaticMissionCancelled as exc:
            self._transition_sampled_preserving(
                exc,
                intent.sequence,
                NavigationGoalStatus.CANCELED,
                detail="mission canceled during navigation",
            )
            raise
        except Nav2GoalRejected as exc:
            self._transition_sampled_preserving(
                exc,
                intent.sequence,
                NavigationGoalStatus.REJECTED,
                detail=str(exc),
            )
            raise
        except TimeoutError as exc:
            self._transition_sampled_preserving(
                exc,
                intent.sequence,
                NavigationGoalStatus.TIMED_OUT,
                detail=str(exc),
            )
            raise
        except Exception as exc:
            terminal = (
                exc.terminal
                if isinstance(exc, Nav2SafetyFailure)
                else None
            )
            self._transition_sampled_preserving(
                exc,
                intent.sequence,
                NavigationGoalStatus.ABORTED,
                nav2_status=(
                    int(terminal.status)
                    if terminal is not None
                    else int(ActionStatus.UNKNOWN)
                ),
                nav2_error_code=(
                    int(terminal.error_code)
                    if terminal is not None
                    else 0
                ),
                detail=str(exc),
            )
            raise

    def _execute_motion(
        self,
        intent: MotionIntent,
        *,
        is_cancelled: Callable[[], bool],
        deadline_monotonic: float,
    ) -> ActionTerminal:
        kind, goal, token_label = self._goal_for_intent(intent)
        remaining = max(0.0, deadline_monotonic - self._runtime.monotonic())
        if is_cancelled():
            raise AutomaticMissionCancelled(
                self._pending_reason(intent, canceled=True)
            )
        if remaining <= 0.0:
            raise TimeoutError(self._pending_reason(intent, canceled=False))
        wait_cap_s = 5.0 if kind is Nav2ActionKind.BACK_UP else 10.0
        if not self._runtime.wait_for_server(
            kind,
            timeout_s=min(wait_cap_s, remaining),
        ):
            raise TimeoutError(self._server_unavailable_message(intent))
        # Action server 的等待本身会消耗 deadline。发送前再检查一次，避免
        # “已经超时却仍新建运动 goal”的竞态。
        if is_cancelled():
            raise AutomaticMissionCancelled(
                self._pending_reason(intent, canceled=True)
            )
        if self._runtime.monotonic() >= deadline_monotonic:
            raise TimeoutError(self._pending_reason(intent, canceled=False))

        token = self._begin_quiescence(token_label)
        try:
            response = self._runtime.send_goal(goal)
        except Exception as exc:
            # send_goal 抛异常时无法知道中间件是否已经发送成功，不能把
            # quiescence 误记为 terminal；先发独立 STOP，再停止 stage。
            self._fail_unknown_goal_safely(
                self._response_read_reason(intent),
                [f"Nav2 goal dispatch failed: {exc}"],
                cleanup_deadline=self._new_cleanup_deadline(),
            )

        while not self._response_done(
            response,
            reason=self._response_read_reason(intent),
        ):
            try:
                canceled = bool(is_cancelled())
            except Exception as exc:
                reason = (
                    "cannot evaluate mission cancellation while waiting for "
                    f"Nav2 goal response: {exc}"
                )
                terminal = self._resolve_pending_goal_safely(
                    response,
                    reason=reason,
                    token=token,
                    cleanup_deadline=self._new_cleanup_deadline(),
                )
                primary = Nav2MotionFailed(reason)
                if isinstance(intent, SampledNavigate) and terminal is not None:
                    self._transition_sampled_preserving(
                        primary,
                        intent.sequence,
                        NavigationGoalStatus.ABORTED,
                        nav2_status=int(terminal.status),
                        nav2_error_code=int(terminal.error_code),
                        detail=reason,
                    )
                raise primary
            expired = self._runtime.monotonic() >= deadline_monotonic
            if canceled or expired:
                reason = self._pending_reason(intent, canceled=canceled)
                # 不能注册迟到回调后直接释放外层任务锁。必须同步拿到迟到
                # handle，并在 accepted 时完成 cancel/STOP/terminal 全事务。
                terminal = self._resolve_pending_goal_safely(
                    response,
                    reason=reason,
                    token=token,
                    cleanup_deadline=self._new_cleanup_deadline(),
                )
                primary: BaseException = (
                    AutomaticMissionCancelled(reason)
                    if canceled
                    else TimeoutError(reason)
                )
                if isinstance(intent, SampledNavigate) and terminal is not None:
                    self._transition_sampled_preserving(
                        primary,
                        intent.sequence,
                        (
                            NavigationGoalStatus.CANCELED
                            if canceled
                            else NavigationGoalStatus.TIMED_OUT
                        ),
                        nav2_status=int(terminal.status),
                        nav2_error_code=int(terminal.error_code),
                        detail=reason,
                    )
                raise primary
            self._runtime.sleep(self._poll_interval_s)

        try:
            handle = response.result()
        except Exception as exc:
            self._fail_unknown_goal_safely(
                self._response_read_reason(intent),
                [f"Nav2 goal response failed: {exc}"],
                cleanup_deadline=self._new_cleanup_deadline(),
            )
        if handle is None:
            self._finish_quiescence(
                token,
                reason=self._rejected_message(intent),
            )
            raise Nav2GoalRejected(self._rejected_message(intent))
        try:
            accepted = bool(handle.accepted)
        except Exception as exc:
            self._fail_unknown_goal_safely(
                self._response_read_reason(intent),
                [f"Nav2 goal acceptance is unreadable: {exc}"],
                cleanup_deadline=self._new_cleanup_deadline(),
            )
        if not accepted:
            self._finish_quiescence(
                token,
                reason=self._rejected_message(intent),
            )
            raise Nav2GoalRejected(self._rejected_message(intent))

        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            reason = self._missing_result_future_reason(intent, exc)
            self._cancel_stop_and_prove_terminal(
                handle,
                result_future=None,
                reason=reason,
                token=token,
                cleanup_deadline=self._new_cleanup_deadline(),
            )
            raise Nav2MotionFailed(reason)

        if isinstance(intent, SampledNavigate):
            try:
                self._transition_sampled(
                    intent.sequence,
                    NavigationGoalStatus.ACCEPTED,
                    nav2_status=int(ActionStatus.ACCEPTED),
                    detail="NavigateToPose accepted",
                )
                self._transition_sampled(
                    intent.sequence,
                    NavigationGoalStatus.EXECUTING,
                    nav2_status=int(ActionStatus.EXECUTING),
                    detail="NavigateToPose executing",
                )
            except Exception as exc:
                # handle 已 accepted 后，证据发布也属于可能失败的外围 I/O。
                # 任何异常都必须先关闭真实运动，不能只把 ledger 改成 ABORTED
                # 后释放事务锁，让 Nav2 goal 在后台继续运行。
                reason = (
                    "cannot record sampled navigation acceptance: "
                    f"{exc}"
                )
                self._fail_accepted_sampled_safely(
                    intent,
                    handle,
                    result_future=result_future,
                    reason=reason,
                    token=token,
                )

        while not self._result_done(result_future):
            try:
                canceled = bool(is_cancelled())
            except Exception as exc:
                reason = (
                    "cannot evaluate mission cancellation during Nav2 motion: "
                    f"{exc}"
                )
                terminal = self._cancel_stop_and_prove_terminal(
                    handle,
                    result_future=result_future,
                    reason=reason,
                    token=token,
                    cleanup_deadline=self._new_cleanup_deadline(),
                )
                primary = Nav2MotionFailed(reason)
                if isinstance(intent, SampledNavigate):
                    self._transition_sampled_preserving(
                        primary,
                        intent.sequence,
                        NavigationGoalStatus.ABORTED,
                        nav2_status=int(terminal.status),
                        nav2_error_code=int(terminal.error_code),
                        detail=reason,
                    )
                raise primary
            if canceled:
                reason = self._active_cancel_reason(intent)
                terminal = self._cancel_stop_and_prove_terminal(
                    handle,
                    result_future=result_future,
                    reason=reason,
                    token=token,
                    cleanup_deadline=self._new_cleanup_deadline(),
                )
                primary = AutomaticMissionCancelled(reason)
                if isinstance(intent, SampledNavigate):
                    self._transition_sampled_preserving(
                        primary,
                        intent.sequence,
                        NavigationGoalStatus.CANCELED,
                        nav2_status=int(terminal.status),
                        nav2_error_code=int(terminal.error_code),
                        detail=reason,
                    )
                raise primary
            if isinstance(intent, SampledNavigate):
                try:
                    record = self._ledger.get(intent.sequence)
                except Exception as exc:
                    reason = (
                        "cannot audit sampled navigation ledger after "
                        f"acceptance: {exc}"
                    )
                    self._fail_accepted_sampled_safely(
                        intent,
                        handle,
                        result_future=result_future,
                        reason=reason,
                        token=token,
                    )
                if record.plan_count > 0 and not record.all_plans_known_free:
                    reason = (
                        "unsafe runtime navigation plan for goal "
                        f"{intent.sequence}"
                    )
                    terminal = self._cancel_stop_and_prove_terminal(
                        handle,
                        result_future=result_future,
                        reason=reason,
                        token=token,
                        cleanup_deadline=self._new_cleanup_deadline(),
                    )
                    primary = Nav2MotionFailed(reason)
                    self._transition_sampled_preserving(
                        primary,
                        intent.sequence,
                        NavigationGoalStatus.ABORTED,
                        nav2_status=int(terminal.status),
                        nav2_error_code=int(terminal.error_code),
                        detail=reason,
                    )
                    raise primary
            if self._runtime.monotonic() >= deadline_monotonic:
                reason = self._active_timeout_reason(intent)
                terminal = self._cancel_stop_and_prove_terminal(
                    handle,
                    result_future=result_future,
                    reason=reason,
                    token=token,
                    cleanup_deadline=self._new_cleanup_deadline(),
                )
                primary = TimeoutError(reason)
                if isinstance(intent, SampledNavigate):
                    self._transition_sampled_preserving(
                        primary,
                        intent.sequence,
                        NavigationGoalStatus.TIMED_OUT,
                        nav2_status=int(terminal.status),
                        nav2_error_code=int(terminal.error_code),
                        detail=reason,
                    )
                raise primary
            self._runtime.sleep(self._poll_interval_s)

        terminal, terminal_error = self._read_terminal(result_future)
        if terminal_error is not None:
            reason = self._invalid_terminal_reason(intent, terminal_error)
            terminal = self._cancel_stop_and_prove_terminal(
                handle,
                result_future=result_future,
                reason=reason,
                token=token,
                cleanup_deadline=self._new_cleanup_deadline(),
            )
            primary = Nav2MotionFailed(reason)
            if isinstance(intent, SampledNavigate):
                self._transition_sampled_preserving(
                    primary,
                    intent.sequence,
                    NavigationGoalStatus.ABORTED,
                    nav2_status=int(terminal.status),
                    nav2_error_code=int(terminal.error_code),
                    detail=reason,
                )
            raise primary
        assert terminal is not None

        if isinstance(intent, SampledNavigate):
            # `/plan` 与 Action result 没有跨 topic 全局顺序。保持短排空窗口，
            # 并在 handle 仍可用于安全取消时检查最后一条迟到 replan。
            # 该观测窗口仍属于业务执行，不能在 Action terminal 后绕过公开
            # deadline 再固定赠送一段时间。
            drain_s = min(
                self._late_plan_drain_s,
                max(
                    0.0,
                    deadline_monotonic - self._runtime.monotonic(),
                ),
            )
            if drain_s > 0.0:
                self._runtime.sleep(drain_s)
            try:
                record = self._ledger.get(intent.sequence)
            except Exception as exc:
                # Action terminal 可读并不代表路径证据已经完成。最终 audit
                # 无法读取时仍按业务失败保守停车，同时在 ledger 中保留真实
                # Nav2 terminal，避免把协议成功误报成已验证导航成功。
                reason = (
                    "cannot audit sampled navigation ledger after terminal: "
                    f"{exc}"
                )
                self._fail_accepted_sampled_safely(
                    intent,
                    handle,
                    result_future=result_future,
                    reason=reason,
                    token=token,
                )
            if record.plan_count == 0 or not record.all_plans_known_free:
                reason = (
                    "unsafe or missing runtime plan for goal "
                    f"{intent.sequence}"
                )
                terminal = self._cancel_stop_and_prove_terminal(
                    handle,
                    result_future=result_future,
                    reason=reason,
                    token=token,
                    cleanup_deadline=self._new_cleanup_deadline(),
                )
                primary = Nav2MotionFailed(reason)
                self._transition_sampled_preserving(
                    primary,
                    intent.sequence,
                    NavigationGoalStatus.ABORTED,
                    nav2_status=int(terminal.status),
                    nav2_error_code=int(terminal.error_code),
                    detail=reason,
                )
                raise primary

        self._finish_quiescence(
            token,
            reason=f"cannot close Nav2 transaction {token}",
            terminal=terminal,
        )
        return terminal

    def _resolve_pending_goal_safely(
        self,
        response: GoalResponseFuturePort,
        *,
        reason: str,
        token: str,
        cleanup_deadline: float,
    ) -> ActionTerminal | None:
        while (
            not self._response_done(
                response,
                reason=reason,
                cleanup_deadline=cleanup_deadline,
            )
            and self._runtime.monotonic() < cleanup_deadline
        ):
            self._runtime.sleep(
                min(
                    self._poll_interval_s,
                    self._remaining_cleanup_s(cleanup_deadline),
                )
            )
        if not self._response_done(
            response,
            reason=reason,
            cleanup_deadline=cleanup_deadline,
        ):
            self._fail_unknown_goal_safely(
                reason,
                ["Nav2 goal response did not arrive within safety budget"],
                cleanup_deadline=cleanup_deadline,
            )
        try:
            handle = response.result()
        except Exception as exc:
            self._fail_unknown_goal_safely(
                reason,
                [f"Nav2 goal response failed: {exc}"],
                cleanup_deadline=cleanup_deadline,
            )
        if handle is None:
            self._finish_quiescence(token, reason=reason)
            return None
        try:
            accepted = bool(handle.accepted)
        except Exception as exc:
            self._fail_unknown_goal_safely(
                reason,
                [f"Nav2 goal acceptance is unreadable: {exc}"],
                cleanup_deadline=cleanup_deadline,
            )
        if not accepted:
            self._finish_quiescence(token, reason=reason)
            return None
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            result_future = None
            reason = f"{reason}; cannot get Nav2 result future: {exc}"
        return self._cancel_stop_and_prove_terminal(
            handle,
            result_future=result_future,
            reason=reason,
            token=token,
            cleanup_deadline=cleanup_deadline,
        )

    def _cancel_stop_and_prove_terminal(
        self,
        handle: GoalHandlePort,
        *,
        result_future: GoalResultFuturePort | None,
        reason: str,
        token: str,
        cleanup_deadline: float,
    ) -> ActionTerminal:
        cleanup_errors: list[str] = []
        try:
            # cancel 请求先发出，但绝不等待它返回后才停车；否则故障路径仍
            # 可能在数秒 cancel timeout 内继续输出速度。
            handle.cancel_goal_async()
        except Exception as exc:
            cleanup_errors.append(f"cancel request failed: {exc}")
        try:
            # 即使 late response 已耗尽预算也要发 STOP；timeout=0 表示只做
            # 非阻塞发布/即时检查，不能因为没有等待预算而跳过停车命令。
            self._runtime.force_priority_stop(
                timeout_s=self._remaining_cleanup_s(cleanup_deadline)
            )
        except Exception as exc:
            cleanup_errors.append(f"typed stop failed: {exc}")

        if result_future is not None:
            while self._runtime.monotonic() < cleanup_deadline:
                try:
                    if result_future.done():
                        break
                except Exception:
                    break
                self._runtime.sleep(
                    min(
                        self._poll_interval_s,
                        self._remaining_cleanup_s(cleanup_deadline),
                    )
                )

        # Future 容器 done 仍不够；wrapper status 和 payload 必须由 Adapter
        # 收紧为 ActionTerminal，才允许 quiescence 把 goal 记为 terminal。
        terminal, terminal_error = self._read_terminal(result_future)
        if terminal_error is None:
            try:
                self._runtime.nav2_goal_terminal(token)
            except Exception as exc:
                cleanup_errors.append(
                    f"Nav2 quiescence terminal update failed: {exc}"
                )
        else:
            cleanup_errors.append(terminal_error)
        if cleanup_errors:
            self._fail_safety(
                reason,
                cleanup_errors,
                terminal=terminal,
            )
        assert terminal is not None
        return terminal

    def _read_terminal(
        self,
        result_future: GoalResultFuturePort | None,
    ) -> tuple[ActionTerminal | None, str | None]:
        if result_future is None:
            return None, "Nav2 result future unavailable"
        try:
            if not result_future.done():
                return None, "Nav2 goal did not reach terminal after cancel"
        except Exception as exc:
            return None, f"Nav2 result readiness check failed: {exc}"
        try:
            terminal = result_future.result()
        except Exception as exc:
            return None, f"Nav2 result read failed: {exc}"
        if terminal is None:
            return None, "Nav2 result future returned no terminal"
        if not isinstance(terminal, ActionTerminal):
            return None, "Nav2 result future returned invalid terminal payload"
        try:
            status = ActionStatus(int(terminal.status))
        except (TypeError, ValueError) as exc:
            return None, f"Nav2 terminal status is unreadable: {exc}"
        if status not in self._TERMINAL_STATUSES:
            return None, f"Nav2 terminal has non-terminal status: {int(status)}"
        try:
            error_code = int(terminal.error_code)
        except (TypeError, ValueError) as exc:
            return None, f"Nav2 terminal error code is unreadable: {exc}"
        return (
            ActionTerminal(
                status=int(status),
                error_code=error_code,
                error_message=str(terminal.error_message),
            ),
            None,
        )

    def _response_done(
        self,
        response: GoalResponseFuturePort,
        *,
        reason: str,
        cleanup_deadline: float | None = None,
    ) -> bool:
        try:
            return bool(response.done())
        except Exception as exc:
            self._fail_unknown_goal_safely(
                reason,
                [f"Nav2 goal response readiness check failed: {exc}"],
                cleanup_deadline=(
                    self._new_cleanup_deadline()
                    if cleanup_deadline is None
                    else cleanup_deadline
                ),
            )

    def _result_done(self, result_future: GoalResultFuturePort) -> bool:
        try:
            return bool(result_future.done())
        except Exception:
            # 调用方仍持有 accepted handle，统一走 cancel/STOP/terminal
            # 路径；具体 future 错误由 _read_terminal 生成稳定诊断。
            return True

    def _begin_quiescence(self, label: str) -> str:
        with self._token_lock:
            self._token_sequence += 1
            token = f"{label}:{self._token_sequence}"
        self._runtime.nav2_goal_started(token)
        return token

    def _new_cleanup_deadline(self) -> float:
        """首次故障只创建一次安全预算，后续步骤不得重新续期。"""

        return self._runtime.monotonic() + self._safety_timeout_s

    def _remaining_cleanup_s(self, cleanup_deadline: float) -> float:
        return max(0.0, cleanup_deadline - self._runtime.monotonic())

    def _finish_quiescence(
        self,
        token: str,
        *,
        reason: str,
        terminal: ActionTerminal | None = None,
    ) -> None:
        try:
            self._runtime.nav2_goal_terminal(token)
        except Exception as exc:
            self._fail_safety(
                reason,
                [f"Nav2 quiescence terminal update failed: {exc}"],
                terminal=terminal,
            )

    def _fail_unknown_goal_safely(
        self,
        reason: str,
        cleanup_errors: list[str],
        *,
        cleanup_deadline: float,
    ) -> None:
        """acceptance 未知时先停车，再停止 stage 并保留 unresolved token。"""

        errors = list(cleanup_errors)
        try:
            # 无 handle 时无法定点 cancel，但仍能通过控制面发布 priority STOP。
            # 即使上一步已经耗尽预算，timeout=0 也必须执行一次非阻塞发布。
            self._runtime.force_priority_stop(
                timeout_s=self._remaining_cleanup_s(cleanup_deadline)
            )
        except Exception as exc:
            errors.append(f"typed stop failed: {exc}")
        self._fail_safety(reason, errors)

    def _fail_accepted_sampled_safely(
        self,
        intent: SampledNavigate,
        handle: GoalHandlePort,
        *,
        result_future: GoalResultFuturePort,
        reason: str,
        token: str,
    ) -> None:
        """关闭 accepted sampled goal，并保留触发清理的主错误。"""

        terminal = self._cancel_stop_and_prove_terminal(
            handle,
            result_future=result_future,
            reason=reason,
            token=token,
            cleanup_deadline=self._new_cleanup_deadline(),
        )
        failure = Nav2MotionFailed(reason)
        self._transition_sampled_preserving(
            failure,
            intent.sequence,
            NavigationGoalStatus.ABORTED,
            nav2_status=int(terminal.status),
            nav2_error_code=int(terminal.error_code),
            detail=reason,
        )
        raise failure

    def _transition_sampled(
        self,
        sequence: int,
        status: NavigationGoalStatus,
        *,
        nav2_status: int = 0,
        nav2_error_code: int = 0,
        detail: str,
    ) -> None:
        self._ledger.transition(
            sequence,
            status,
            timestamp_ns=self._runtime.evidence_timestamp_ns(),
            nav2_status=nav2_status,
            nav2_error_code=nav2_error_code,
            detail=detail,
        )
        self._runtime.navigation_evidence_changed()

    def _transition_sampled_if_open(
        self,
        sequence: int,
        status: NavigationGoalStatus,
        *,
        nav2_status: int = 0,
        nav2_error_code: int = 0,
        detail: str,
    ) -> None:
        if self._ledger.is_terminal(sequence):
            return
        self._transition_sampled(
            sequence,
            status,
            nav2_status=nav2_status,
            nav2_error_code=nav2_error_code,
            detail=detail,
        )

    def _transition_sampled_preserving(
        self,
        primary_error: BaseException,
        sequence: int,
        status: NavigationGoalStatus,
        *,
        nav2_status: int = 0,
        nav2_error_code: int = 0,
        detail: str,
    ) -> None:
        """失败证据尽力写入；发布异常不能覆盖已经确定的主错误。"""

        try:
            self._transition_sampled_if_open(
                sequence,
                status,
                nav2_status=nav2_status,
                nav2_error_code=nav2_error_code,
                detail=detail,
            )
        except Exception as evidence_exc:
            add_note = getattr(primary_error, "add_note", None)
            if callable(add_note):
                add_note(
                    "cannot publish sampled navigation failure evidence: "
                    f"{evidence_exc}"
                )

    def _fail_safety(
        self,
        reason: str,
        cleanup_errors: list[str],
        *,
        terminal: ActionTerminal | None = None,
    ) -> None:
        try:
            self._runtime.stop_navigation_stage()
            shutdown_detail = "navigation stage stopped by fail-safe"
        except Exception as exc:
            shutdown_detail = f"navigation stage stop failed: {exc}"
        raise Nav2SafetyFailure(
            (
                f"{reason}; navigation safety cleanup failed: "
                + "; ".join((*cleanup_errors, shutdown_detail))
            ),
            terminal=terminal,
        )

    @staticmethod
    def _terminal_succeeded(terminal: ActionTerminal) -> bool:
        return (
            int(terminal.status) == int(ActionStatus.SUCCEEDED)
            and int(terminal.error_code) == 0
        )

    @classmethod
    def _require_success(
        cls,
        terminal: ActionTerminal,
        *,
        label: str,
    ) -> None:
        if not cls._terminal_succeeded(terminal):
            raise Nav2MotionFailed(
                f"{label} NavigateToPose failed: "
                f"status={terminal.status} error={terminal.error_code}"
            )

    @staticmethod
    def _goal_for_intent(
        intent: MotionIntent,
    ) -> tuple[Nav2ActionKind, RuntimeGoal, str]:
        if isinstance(intent, SampledNavigate):
            return (
                Nav2ActionKind.NAVIGATE_TO_POSE,
                NavigateGoal(
                    frame_id="map",
                    x=float(intent.goal_xy[0]),
                    y=float(intent.goal_xy[1]),
                    yaw=0.0,
                ),
                f"sampled-{intent.sequence}",
            )
        if isinstance(intent, MappingReturn):
            pose = intent.goal_pose
            return (
                Nav2ActionKind.NAVIGATE_TO_POSE,
                NavigateGoal(
                    frame_id=pose.frame_id,
                    x=float(pose.x),
                    y=float(pose.y),
                    yaw=float(pose.yaw),
                ),
                "mapping-return",
            )
        return (
            Nav2ActionKind.BACK_UP,
            BackupGoal(
                distance_m=float(intent.distance_m),
                speed_mps=float(intent.speed_mps),
                time_allowance_s=float(intent.time_allowance_s),
            ),
            "backup",
        )

    @staticmethod
    def _server_unavailable_message(intent: MotionIntent) -> str:
        if isinstance(intent, RecoveryBackup):
            return "Nav2 BackUp Action server unavailable"
        if isinstance(intent, MappingReturn):
            return "NavigateToPose Action server unavailable for return"
        return "NavigateToPose Action server unavailable"

    @staticmethod
    def _response_read_reason(intent: MotionIntent) -> str:
        if isinstance(intent, RecoveryBackup):
            return "cannot read Nav2 BackUp goal response"
        if isinstance(intent, MappingReturn):
            return "cannot read mapping return goal response"
        return f"cannot read navigation goal response for {intent.goal_xy}"

    @staticmethod
    def _pending_reason(
        intent: MotionIntent,
        *,
        canceled: bool,
    ) -> str:
        if isinstance(intent, RecoveryBackup):
            return (
                "automatic mission canceled during recovery backup"
                if canceled
                else "Nav2 BackUp goal response timeout"
            )
        if isinstance(intent, MappingReturn):
            return (
                "automatic mission canceled before return goal response"
                if canceled
                else "mapping return goal response timeout"
            )
        return (
            "automatic mission canceled before goal response"
            if canceled
            else f"navigation goal response timeout: {intent.goal_xy}"
        )

    @staticmethod
    def _active_cancel_reason(intent: MotionIntent) -> str:
        if isinstance(intent, RecoveryBackup):
            return "automatic mission canceled during recovery backup"
        if isinstance(intent, MappingReturn):
            return "automatic mission canceled during mapping return"
        return "automatic mission canceled during navigation"

    @staticmethod
    def _active_timeout_reason(intent: MotionIntent) -> str:
        if isinstance(intent, RecoveryBackup):
            return "Nav2 BackUp result timeout"
        if isinstance(intent, MappingReturn):
            return "mapping return goal result timeout"
        return f"navigation goal result timeout: {intent.goal_xy}"

    @staticmethod
    def _rejected_message(intent: MotionIntent) -> str:
        if isinstance(intent, RecoveryBackup):
            return "Nav2 BackUp goal rejected"
        if isinstance(intent, MappingReturn):
            return "mapping return goal rejected"
        return f"navigation goal rejected: {intent.goal_xy}"

    @staticmethod
    def _missing_result_future_reason(
        intent: MotionIntent,
        error: BaseException,
    ) -> str:
        if isinstance(intent, RecoveryBackup):
            return f"cannot get Nav2 BackUp result future: {error}"
        if isinstance(intent, MappingReturn):
            return f"cannot get mapping return result future: {error}"
        return (
            "cannot get Nav2 result future for goal "
            f"{intent.goal_xy}: {error}"
        )

    @staticmethod
    def _invalid_terminal_reason(
        intent: MotionIntent,
        terminal_error: str,
    ) -> str:
        if isinstance(intent, RecoveryBackup):
            return f"invalid Nav2 BackUp terminal result: {terminal_error}"
        if isinstance(intent, MappingReturn):
            return f"invalid mapping return terminal: {terminal_error}"
        return f"invalid NavigateToPose terminal result: {terminal_error}"
