"""在线/离线 Agent 共用的应用层编排。

该模块位于 provider 数据面与 ROS 2 transport 之间：它拥有一次 transcript 从会话门控、
记忆命令、连续队列到动作批次的统一语义，但不创建 ROS publisher，也不依赖某一种
ASR/LLM/TTS。节点通过组合和少量回调注入差异，避免形成脆弱的 Online/Offline 继承树。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable

from .agent_execution_runtime import AgentExecutionRuntime
from .continuous_voice import QueueSnapshot
from .types import ActionCommand
from .user_context_runtime import UserContextRuntime, UserContextSnapshot
from .user_preferences import apply_user_preferences


@dataclass(frozen=True)
class AgentApplicationCallbacks:
    """provider/transport 差异的最小注入面。"""

    run_model_turn: Callable[[str, object, UserContextSnapshot], None]
    speak_memory_response: Callable[[str], None]
    publish_action_candidate: Callable[[ActionCommand], None]
    publish_enroll_request: Callable[[object], None]
    publish_response_delta: Callable[[str], None]
    publish_response: Callable[[str], None]
    capture_turn_context: Callable[[], object] = lambda: None
    prepare_turn_context: Callable[[object], None] = lambda _context: None
    finish_preparsed_turn: Callable[[object], None] = lambda _context: None


class AgentApplicationRuntime:
    """组合会话、用户上下文、队列和动作序列的应用层深模块。

    ``AgentControlPlane`` 负责纯领域决策，``AgentExecutionRuntime`` 负责线程所有权；
    本类把二者和记忆/动作批次连成稳定用例。在线与离线节点因此只保留音频和模型差异。
    """

    def __init__(
        self,
        *,
        source: str,
        control,
        lifecycle_runtime,
        action_sequencer,
        conversation_memory,
        user_context: UserContextRuntime,
        events,
        callbacks: AgentApplicationCallbacks,
        logger,
        wake_word_required: bool,
        continuous_enabled: bool,
    ) -> None:
        self._source = source
        self._control = control
        self._lifecycle = lifecycle_runtime
        self._action_sequencer = action_sequencer
        self._memory = conversation_memory
        self._user_context = user_context
        self._events = events
        self._callbacks = callbacks
        self._logger = logger
        self._wake_word_required = bool(wake_word_required)
        self._continuous_enabled = bool(continuous_enabled)

    @property
    def execution(self) -> AgentExecutionRuntime | None:
        return self._lifecycle.execution

    def handle_wake_event(self, event) -> None:
        """处理外部 KWS 的 wake/sleep；sleep 必须取消动作、清队列并发 STOP。"""

        if event.kind == "wake":
            session_event = self._control.voice_session.external_wake(
                event.provider, event.transcript
            )
            self._events.publish_session_event(session_event)
            self._events.publish_state("session_awake")
            self._logger.info(
                f"external wake event accepted from provider={event.provider}"
            )
            return

        dropped = self._control.command_queue.clear()
        self._events.publish_queue(
            self._control.queue_event(
                "clear",
                "",
                QueueSnapshot(True, self._control.command_queue.size(), dropped),
            )
        )
        self._action_sequencer.cancel("external_sleep")
        self.publish_actions([ActionCommand("stop", {}, priority=True)])
        session_event = self._control.voice_session.external_sleep(event.provider)
        self._events.publish_session_event(session_event)
        self._events.publish_state("sleeping")
        self._logger.info(
            f"external sleep event accepted from provider={event.provider}"
        )

    def clear_memory(self) -> None:
        self._memory.clear()
        self._user_context.clear_current()
        self._logger.info("conversation memory cleared")

    def notify_action_result(
        self, command_id: str, success: bool, message: str, *, status: int = 0
    ) -> None:
        self._action_sequencer.notify_result(
            command_id, success, message, status=status
        )

    def update_speaker_identity(self, identity) -> None:
        self._user_context.update_identity(identity)
        if identity.usable:
            self._logger.info(
                "speaker identity accepted: "
                f"speaker_id={identity.speaker_id}, confidence={identity.confidence:.3f}"
            )
        else:
            self._logger.debug("speaker identity ignored or unknown")

    def accept_transcript(self, transcript: str) -> None:
        """执行 transcript 的唯一应用入口，统一 online/offline 的边界行为。"""

        decision = self._control.accept_transcript(
            transcript, wake_word_required=self._wake_word_required
        )
        self._events.publish_control_decision(decision)
        if decision.directive in {"sleep", "priority"}:
            # stop/cancel 不能排在普通命令后面，控制面已清队列，这里再抢占当前序列。
            self._action_sequencer.cancel(decision.cancel_reason)
            self._logger.info(
                f"{decision.cancel_reason} received; cancelling current sequence"
            )
            if decision.dropped:
                self._logger.info(
                    f"cleared {decision.dropped} queued command(s)"
                )
            self.publish_actions(
                [ActionCommand(decision.priority_action, {}, priority=True)]
            )
            return
        if decision.directive != "command":
            return

        command = decision.command
        if self._handle_memory_command(command):
            self._events.publish_state("listening")
            return
        if self._continuous_enabled:
            self.enqueue_continuous_command(command)
            return

        execution = self.execution
        if execution is None:
            return
        turn_context = self._callbacks.capture_turn_context()
        user_context = self._user_context.snapshot()
        if not execution.start_background_turn(
            self.run_direct_turn, command, turn_context, user_context
        ):
            self._logger.warning(
                f"{self._source} agent busy; dropping overlapping utterance"
            )

    def _handle_memory_command(self, command: str) -> bool:
        result = self._user_context.handle_command(command)
        if result is None:
            return False
        if result.enroll_request is not None:
            self._callbacks.publish_enroll_request(result.enroll_request)
        self._callbacks.publish_response_delta(result.response)
        self._callbacks.publish_response(result.response)
        execution = self.execution
        if execution is not None and not execution.start_background_turn(
            self._callbacks.speak_memory_response, result.response
        ):
            self._logger.debug("memory response TTS skipped while Agent is busy")
        return True

    def run_direct_turn(
        self,
        user_text: str,
        turn_context: object,
        user_context: UserContextSnapshot,
    ) -> None:
        self._callbacks.prepare_turn_context(turn_context)
        self._callbacks.run_model_turn(user_text, turn_context, user_context)

    def run_queued_turn(self, item) -> None:
        """恢复入队时冻结的用户/延迟上下文，保证长队列中 turn 不串用户。"""

        context = item.context if isinstance(item.context, dict) else {}
        user_context = context.get("user_context")
        if not isinstance(user_context, UserContextSnapshot):
            user_context = self._user_context.snapshot()
        turn_context = context.get("turn_context")
        if turn_context is None:
            turn_context = self._callbacks.capture_turn_context()
        self._callbacks.prepare_turn_context(turn_context)

        actions = self._control.preparsed_actions(context)
        if actions:
            self._run_preparsed_turn(item.text, actions, turn_context, user_context)
            return
        self._callbacks.run_model_turn(item.text, turn_context, user_context)

    def _run_preparsed_turn(
        self,
        user_text: str,
        actions: Iterable[ActionCommand],
        turn_context: object,
        user_context: UserContextSnapshot,
    ) -> None:
        action_list = tuple(actions)
        self._events.publish_state("thinking")
        response = "好的，按顺序执行：" + "，".join(
            action.name for action in action_list
        ) + "。"
        self._callbacks.publish_response_delta(response)
        self._callbacks.publish_response(response)
        report = self.publish_actions(action_list, user_context)
        self._user_context.record_interaction(
            user_context,
            user_text=user_text,
            assistant_text=response,
            actions=[action.as_dict() for action in action_list],
            success=not report.failed,
        )
        self._callbacks.finish_preparsed_turn(turn_context)

    def enqueue_continuous_command(self, command: str) -> None:
        user_context = self._user_context.snapshot()
        private_context = {
            "turn_context": self._callbacks.capture_turn_context(),
            "user_context": user_context,
        }
        decision = self._control.enqueue_command(
            command,
            context_extras=private_context,
            fallback_context=private_context,
        )
        self._events.publish_enqueue_decision(decision)
        if decision.status == "queued":
            self._logger.info(
                f"continuous command queued: size={decision.queue_size}, text={command}"
            )
        elif decision.status == "retry":
            self._logger.warning(
                f"incomplete voice command: reason={decision.reason}, text={command}"
            )
        elif decision.status == "rejected":
            self._logger.warning(
                f"continuous command queue rejected input: {decision.reason}"
            )

    def publish_actions(
        self,
        actions: Iterable[ActionCommand],
        user_context: UserContextSnapshot | None = None,
    ):
        """应用用户偏好并发布动作；C++ ActionGuard 仍是最终安全边界。"""

        context = user_context or self._user_context.snapshot()
        action_list = apply_user_preferences(actions, context.preferences)
        execution = self.execution
        wait_for_results = (
            len(action_list) > 1
            if execution is None
            else execution.should_wait_for_action_results(len(action_list))
        )
        report = self._action_sequencer.publish(
            action_list,
            self._callbacks.publish_action_candidate,
            wait_for_results=wait_for_results,
        )
        if report.failed:
            self._logger.warning(
                "action sequence stopped after "
                f"{report.completed} completed step(s): {report.reason}"
            )
        return report
