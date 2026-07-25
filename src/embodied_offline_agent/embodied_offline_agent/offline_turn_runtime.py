"""离线 llama.cpp + 伪流式 TTS turn 数据面。"""

import json

from embodied_agent_core.agent_execution_runtime import AgentExecutionCancelled
from embodied_agent_core.streaming_turn import StreamingTurnRuntime

from .pseudo_streaming_tts import PseudoStreamingTtsPipeline


class OfflineStreamingTurnRuntime:
    """拥有离线双缓冲 TTS 管线，并输出可分层定位的延迟报告。"""

    def __init__(
        self,
        *,
        llm,
        tts,
        memory,
        user_context,
        ros_io,
        events,
        lifecycle_runtime,
        publish_actions,
        publish_metrics,
        prompt_context,
        tts_sample_rate,
        param,
        logger,
    ) -> None:
        self._llm = llm
        self._tts = tts
        self._memory = memory
        self._user_context = user_context
        self._ros_io = ros_io
        self._events = events
        self._lifecycle = lifecycle_runtime
        self._publish_actions = publish_actions
        self._publish_metrics = publish_metrics
        self._prompt_context = prompt_context
        self._tts_sample_rate = tts_sample_rate
        self._param = param
        self._logger = logger

    def run(self, user_text, latency, user_context) -> None:
        tts_pipeline = PseudoStreamingTtsPipeline(
            synthesize=self._tts.synthesize,
            publish_audio=self._ros_io.publish_tts_audio,
            sample_rate=self._tts_sample_rate(),
            pcm_chunk_ms=int(self._param("tts_pcm_chunk_ms")),
            on_first_audio=latency.mark_first_audio,
        )
        tts_pipeline.start()

        def enqueue_tts_text(text):
            self._events.publish_state("speaking")
            if not tts_pipeline.put_text(text):
                raise TimeoutError("message double buffer remained full")

        turn = StreamingTurnRuntime(
            max_chunk_chars=self._param("tts_chunk_max_chars"),
            on_first_token=latency.mark_first_token,
            on_speech_delta=self._ros_io.publish_response_delta,
            on_speakable=enqueue_tts_text,
            on_protocol_error=self._logger.warning,
        )
        self._events.publish_state("thinking")
        try:
            prompt = self._prompt_context.build(user_text, user_context)
            self._logger.info(
                "prompt context: "
                + json.dumps(prompt.metrics(), ensure_ascii=False)
            )
            # 预算无法满足时 fail-closed，不能把已知超窗请求交给 llama-server 截断。
            prompt.require_budget()
            latency.mark_llm_start()
            for token in self._llm.stream(prompt.messages):
                self._lifecycle.execution.raise_if_stopping()
                turn.feed(token)
            self._lifecycle.execution.raise_if_stopping()
            result = turn.finish(
                user_text,
                # 只有本地 NLU 明确认定为控制意图时，模型 action 才有资格进入候选层。
                # 普通聊天与 RAG 问答都 fail-closed，防止幻觉或历史知识注入触发运动。
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
            tts_metrics = tts_pipeline.close_and_wait(timeout_s=60.0)
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
                actions=[action.as_dict() for action in result.actions],
                success=not action_report.failed,
            )
            latency.finish()
            report = latency.report(
                tts_metrics.message_buffer_dropped,
                tts_metrics.audio_buffer_dropped,
            )
            # provider 与管线指标分开，才能判断瓶颈来自 llama.cpp 还是 TTS 缓冲。
            report["llm_provider"] = getattr(self._llm, "last_metrics", {})
            report["tts_pipeline"] = tts_metrics.as_dict()
            report["prompt_context"] = prompt.metrics()
            report["citations"] = citation_report
            self._publish_metrics(report)
        except AgentExecutionCancelled:
            tts_pipeline.abort()
            self._logger.info("offline turn cancelled by lifecycle transition")
            raise
        except Exception as exc:
            tts_pipeline.abort()
            self._logger.error(f"offline turn failed: {exc}")
            self._events.publish_state("error")
            raise
        finally:
            if not self._lifecycle.stopping and self._lifecycle.active:
                self._events.publish_state("listening")
