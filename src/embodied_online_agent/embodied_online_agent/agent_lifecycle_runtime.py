"""在线/离线 Agent 共用的 Lifecycle 资源与安全停机编排。"""

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class LifecycleQuiescence:
    """一次停机是否真正让输入与执行线程都静默。"""

    input_quiesced: bool
    execution_quiesced: bool
    errors: tuple[str, ...] = ()

    @property
    def quiesced(self) -> bool:
        return self.input_quiesced and self.execution_quiesced and not self.errors


class AgentLifecycleRuntime:
    """组合式拥有 endpoint/execution，并固化 Agent 的安全状态迁移顺序。

    ROS LifecycleNode 仍负责调用 ``super().on_*`` 来激活 managed publisher；
    本对象只管理业务资源，因而在线/离线 provider 不需要共享继承基类。
    """

    def __init__(
        self,
        *,
        control,
        action_sequencer,
        ros_io,
        events,
        publish_priority_stop: Callable[[], None],
        deactivate_timeout_s: float,
    ) -> None:
        self._control = control
        self._action_sequencer = action_sequencer
        self._ros_io = ros_io
        self._events = events
        self._publish_priority_stop = publish_priority_stop
        self._deactivate_timeout_s = max(0.0, float(deactivate_timeout_s))
        self._execution = None
        self._endpoint = None
        self._active = False
        self._stopping = False

    @property
    def active(self) -> bool:
        return self._active

    @property
    def stopping(self) -> bool:
        return self._stopping

    @property
    def configured(self) -> bool:
        return self._execution is not None and self._endpoint is not None

    @property
    def execution(self):
        return self._execution

    @property
    def endpoint(self):
        return self._endpoint

    def bind(self, *, execution, endpoint) -> None:
        """configure 成功后一次性移交运行时所有权。"""

        if self._execution is not None or self._endpoint is not None:
            raise RuntimeError("Agent lifecycle runtime is already configured")
        self._execution = execution
        self._endpoint = endpoint
        self._stopping = False

    def activate(self, start_input: Callable[[], None]) -> None:
        """managed publisher active 后再开放输入并启动 command worker。"""

        if not self.configured:
            raise RuntimeError("Agent lifecycle runtime is not configured")
        if self._active:
            return
        self._ros_io.set_lifecycle_active(True)
        self._active = True
        # 若任一步抛错，调用方继续走 deactivate；保留 active=True 才能发布安全 STOP。
        start_input()
        if not self._execution.start():
            raise RuntimeError("previous Agent turn did not quiesce")

    def deactivate(self, stop_input: Callable[[], bool]) -> LifecycleQuiescence:
        """按固定安全顺序关输入、取消任务、发 STOP，再等待所有 worker。"""

        self._active = False
        if self._endpoint is not None:
            self._endpoint.cancel_pending()
        self._action_sequencer.cancel("lifecycle_deactivated")
        self._control.command_queue.clear()
        self._events.publish_session_event(self._control.reset_session("lifecycle"))
        # 此时 managed publisher 仍 active，STOP 必须先于线程和 publisher 停用。
        self._publish_priority_stop()

        errors: list[str] = []
        try:
            input_quiesced = bool(stop_input())
        except Exception as exc:  # provider shutdown 失败也必须继续停止 execution。
            input_quiesced = False
            errors.append(f"input_stop:{exc}")
        execution_quiesced = self._execution is None or self._execution.stop(
            self._deactivate_timeout_s
        )
        result = LifecycleQuiescence(
            input_quiesced=input_quiesced,
            execution_quiesced=execution_quiesced,
            errors=tuple(errors),
        )
        if result.quiesced:
            self._events.publish_state("inactive")
            self._events.publish_stopped("lifecycle_inactive")
            self._ros_io.set_lifecycle_active(False)
        else:
            self._events.publish_state("deactivate_timeout")
            self._events.publish_stopped("deactivate_timeout")
        return result

    def release(self, stop_input: Callable[[], bool]) -> LifecycleQuiescence:
        """cleanup 时关闭 endpoint/worker；静默失败时保留引用以便再次停止。"""

        self._active = False
        self._ros_io.set_lifecycle_active(False)
        if self._endpoint is not None:
            self._endpoint.close()
        errors: list[str] = []
        try:
            input_quiesced = bool(stop_input())
        except Exception as exc:
            input_quiesced = False
            errors.append(f"input_stop:{exc}")
        execution_quiesced = self._execution is None or self._execution.stop(
            self._deactivate_timeout_s
        )
        result = LifecycleQuiescence(
            input_quiesced=input_quiesced,
            execution_quiesced=execution_quiesced,
            errors=tuple(errors),
        )
        if result.quiesced:
            self._execution = None
            self._endpoint = None
        return result

    def begin_shutdown(self) -> bool:
        """进程退出只执行一次；active 时仍利用 managed publisher 下发 STOP。"""

        if self._stopping:
            return False
        self._stopping = True
        was_active = self._active
        self._active = False
        self._action_sequencer.cancel("shutdown")
        self._control.command_queue.clear()
        if self._endpoint is not None:
            self._endpoint.cancel_pending()
        if was_active:
            self._publish_priority_stop()
        return True

    def mark_error_inactive(self) -> None:
        """configure 阶段失败时没有 active publisher，只关闭输入门和心跳。"""

        self._active = False
        self._ros_io.set_lifecycle_active(False)
