"""在线 LLM/TTS 全流式 turn 数据面。"""

import json
import queue
import threading

from embodied_agent_core.agent_execution_runtime import AgentExecutionCancelled
from embodied_agent_core.streaming_turn import StreamingTurnRuntime


class OnlineStreamingTurnRuntime:
    """并行消费 LLM token 与 TTS 文本块，节点只负责 provider 装配。"""

    def __init__(
        self,
        *,
        llm,
        tts,
        memory,
        user_context,
        prompt_context,
        metrics,
        ros_io,
        events,
        lifecycle_runtime,
        publish_actions,
        publish_metrics,
        param,
        logger,
    ) -> None:
        self._llm = llm
        self._tts = tts
        self._memory = memory
        self._user_context = user_context
        self._prompt_context = prompt_context
        self._metrics = metrics
        self._ros_io = ros_io
        self._events = events
        self._lifecycle = lifecycle_runtime
        self._publish_actions = publish_actions
        self._publish_metrics = publish_metrics
        self._param = param
        self._logger = logger

    def run(self, user_text, user_context) -> None:
        self._events.publish_state("thinking")
        text_queue = queue.Queue()
        tts_errors = []
        first_tts_text = False

        def text_chunks():
            while True:
                item = text_queue.get()
                if item is None:
                    return
                yield item

        def run_tts():
            try:
                self._tts.synthesize(text_chunks(), self._on_tts_audio)
            except Exception as exc:
                tts_errors.append(exc)
                self._logger.error(f"TTS failed: {exc}")

        tts_thread = threading.Thread(target=run_tts, daemon=True)
        tts_thread.start()

        def enqueue_tts_text(text: str) -> None:
            nonlocal first_tts_text
            if not first_tts_text:
                self._metrics.mark_tts_requested()
                first_tts_text = True
                self._events.publish_state("speaking")
            text_queue.put(text)

        turn = StreamingTurnRuntime(
            max_chunk_chars=self._param("tts_chunk_max_chars"),
            on_first_token=self._metrics.mark_llm_first_token,
            on_speech_delta=self._ros_io.publish_response_delta,
            on_speakable=enqueue_tts_text,
            on_protocol_error=self._logger.warning,
        )

        try:
            prompt = self._prompt_context.build(user_text, user_context)
            # 只记录 source_id、耗时和字符预算，不把知识正文或用户问题复制进日志。
            self._logger.info(
                "prompt context: "
                + json.dumps(prompt.metrics(), ensure_ascii=False)
            )
            # 在线模型同样使用显式预算，避免云端隐式截断系统安全约束。
            prompt.require_budget()
            self._metrics.mark_llm_requested()
            for token in self._llm.stream(prompt.messages):
                self._lifecycle.execution.raise_if_stopping()
                turn.feed(token)

            self._lifecycle.execution.raise_if_stopping()
            result = turn.finish(
                user_text,
                # 普通聊天同样不能授权动作；只有本地 NLU 判定的控制快通道开放 action。
                allow_actions=prompt.actions_allowed,
            )
            if result.action_source == "context_blocked" and result.model_actions:
                self._logger.warning(
                    "non-control turn emitted action tags; all actions were blocked"
                )
            if result.action_source == "blocked" and result.model_actions:
                self._logger.warning("model actions blocked by semantic safety policy")
            citation_report = prompt.citation_metrics(result.assistant_text)
            if not citation_report["citation_valid"]:
                self._logger.warning(
                    "RAG response omitted all valid source_id citations"
                )
            action_report = self._publish_actions(result.actions, user_context)
            text_queue.put(None)
            tts_thread.join(timeout=35.0)
            if tts_thread.is_alive():
                raise TimeoutError("TTS worker did not stop")
            if tts_errors:
                raise tts_errors[0]

            self._ros_io.publish_response(result.assistant_text)
            self._memory.append_turn(
                user_text,
                result.assistant_text,
                model_output=prompt.model_output_for_history(
                    assistant_text=result.assistant_text,
                    model_output=result.model_output,
                ),
            )
            self._user_context.record_interaction(
                user_context,
                user_text=user_text,
                assistant_text=result.assistant_text,
                # 被语义安全策略挡住的模型动作不能被学习成用户习惯。
                actions=[action.as_dict() for action in result.actions],
                success=not action_report.failed,
            )
            self._publish_metrics()
        except AgentExecutionCancelled:
            text_queue.put(None)
            tts_thread.join(timeout=2.0)
            self._logger.info("agent turn cancelled by lifecycle transition")
            raise
        except Exception as exc:
            text_queue.put(None)
            tts_thread.join(timeout=2.0)
            self._logger.error(f"agent turn failed: {exc}")
            self._events.publish_state("error")
            raise
        finally:
            if not self._lifecycle.stopping and self._lifecycle.active:
                self._events.publish_state("listening")

    def _on_tts_audio(self, pcm16: bytes) -> None:
        self._metrics.mark_tts_first_audio()
        self._ros_io.publish_tts_audio(pcm16)
