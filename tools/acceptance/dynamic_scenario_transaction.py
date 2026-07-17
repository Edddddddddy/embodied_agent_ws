"""动态导航场景的 ROS-free 资源事务。

运行时 Adapter 把 Nav2 取消、Gazebo 归位和 tracker 清理封装成回调；本模块只
拥有清理顺序、幂等性和异常优先级，因此失败语义可以在普通 CI 中完整测试。
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from types import TracebackType
from typing import Protocol


Callback = Callable[[], None]
WarningSink = Callable[[str], None]
Sleep = Callable[[float], None]
WaitUntil = Callable[[Callable[[], bool], float, str], None]


class DoneFuture(Protocol):
    def done(self) -> bool: ...


class CancelableGoal(Protocol):
    def cancel_goal_async(self) -> DoneFuture: ...


@dataclass(frozen=True)
class CleanupFailure:
    step: str
    detail: str


class DynamicScenarioCleanupError(RuntimeError):
    """场景本身成功，但清理后置条件未满足。"""

    def __init__(self, failures: tuple[CleanupFailure, ...]) -> None:
        self.failures = failures
        detail = "; ".join(
            f"{failure.step}: {failure.detail}" for failure in failures
        )
        super().__init__(f"dynamic scenario cleanup failed: {detail}")


def cancel_pending_navigation(
    *,
    goal_handle: CancelableGoal,
    result_future: DoneFuture,
    wait_until: WaitUntil,
    timeout_s: float,
) -> None:
    """取消未终态 goal，并证明 Action result 已经进入终态。"""

    if result_future.done():
        return
    cancel_future = goal_handle.cancel_goal_async()
    wait_until(
        cancel_future.done,
        timeout_s,
        "dynamic NavigateToPose cancel response timeout",
    )
    wait_until(
        result_future.done,
        timeout_s,
        "dynamic NavigateToPose did not reach a terminal result after cancel",
    )


class DynamicScenarioTransaction:
    """确保动态场景以固定顺序释放跨进程资源。

    清理顺序刻意固定为：取消活动导航 → 障碍归位 → 清空检测 → 验证停车。
    若业务已经失败，清理异常只写 warning，不能掩盖最先发生、最有诊断价值的
    原始异常；若业务成功而清理失败，则门禁必须失败。
    """

    def __init__(
        self,
        *,
        park_obstacle: Callback,
        clear_detection: Callback,
        verify_scene_cleared: Callback,
        verify_stopped: Callback,
        warning: WarningSink,
        clear_attempts: int,
        clear_settle_s: float,
        clear_interval_s: float,
        sleep: Sleep,
    ) -> None:
        if clear_attempts < 1:
            raise ValueError("clear_attempts must be at least 1")
        if clear_interval_s < 0.0:
            raise ValueError("clear_interval_s cannot be negative")
        if clear_settle_s < 0.0:
            raise ValueError("clear_settle_s cannot be negative")
        self._park_obstacle = park_obstacle
        self._clear_detection = clear_detection
        self._verify_scene_cleared = verify_scene_cleared
        self._verify_stopped = verify_stopped
        self._warning = warning
        self._clear_attempts = clear_attempts
        self._clear_settle_s = clear_settle_s
        self._clear_interval_s = clear_interval_s
        self._sleep = sleep
        self._cancel_navigation: Callback | None = None
        self._obstacle_parked = False
        self._closed = False

    def __enter__(self) -> DynamicScenarioTransaction:
        return self

    def __exit__(
        self,
        _exception_type: type[BaseException] | None,
        exception: BaseException | None,
        _traceback: TracebackType | None,
    ) -> bool:
        self.close(primary_error=exception)
        return False

    def set_cancel_navigation(self, callback: Callback) -> None:
        """在 Nav2 goal 被接受后登记取消动作。"""

        self._cancel_navigation = callback

    def mark_obstacle_parked(self) -> None:
        """正常路径已归位时，避免 finally 再发送一次 Gazebo set_pose。"""

        self._obstacle_parked = True

    def close(self, *, primary_error: BaseException | None = None) -> None:
        """幂等执行清理；仅在没有原始异常时抛出清理错误。"""

        if self._closed:
            return
        self._closed = True
        failures: list[CleanupFailure] = []

        def attempt(step: str, callback: Callback) -> None:
            try:
                callback()
            except BaseException as error:
                # 清理阶段必须继续后续动作；即便 Ctrl-C 恰好落在回调中，也不能
                # 跳过归位/停车，更不能覆盖已经存在的业务异常。
                failures.append(CleanupFailure(step=step, detail=str(error)))

        if self._cancel_navigation is not None:
            attempt("cancel_navigation", self._cancel_navigation)
        if not self._obstacle_parked:
            attempt("park_obstacle", self._park_obstacle)
        if self._clear_settle_s > 0.0:
            attempt(
                "wait_before_clear",
                lambda: self._sleep(self._clear_settle_s),
            )
        for index in range(self._clear_attempts):
            attempt("clear_detection", self._clear_detection)
            if index + 1 < self._clear_attempts and self._clear_interval_s > 0.0:
                # 等待本身也属于清理事务。若计时器被打断，仍要继续发布后续
                # 空检测和停车验证，避免一个辅助步骤阻断真正的安全收口。
                attempt(
                    "wait_between_clears",
                    lambda: self._sleep(self._clear_interval_s),
                )
        attempt("verify_scene_cleared", self._verify_scene_cleared)
        attempt("verify_stopped", self._verify_stopped)

        if not failures:
            return
        cleanup_error = DynamicScenarioCleanupError(tuple(failures))
        try:
            self._warning(str(cleanup_error))
        except BaseException as warning_error:
            failures.append(
                CleanupFailure(step="warning", detail=str(warning_error))
            )
            cleanup_error = DynamicScenarioCleanupError(tuple(failures))
        if primary_error is not None and hasattr(primary_error, "add_note"):
            primary_error.add_note(str(cleanup_error))
        if primary_error is None:
            raise cleanup_error
