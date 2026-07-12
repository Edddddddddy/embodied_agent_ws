import json
import os
import queue
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from embodied_agent_interfaces.msg import (
    RobotCommandResult,
    SpeakerIdentity as SpeakerIdentityMessage,
    WakeEvent as WakeEventMessage,
)
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from embodied_online_agent.memory import ConversationMemory
from embodied_online_agent.action_sequence import SequentialActionPublisher
from embodied_online_agent.agent_control_plane import (
    AgentControlPlane,
    AgentControlPlaneConfig,
)
from embodied_online_agent.agent_execution_runtime import (
    AgentExecutionCancelled,
    AgentExecutionRuntime,
)
from embodied_online_agent.agent_parameters import declare_agent_parameters
from embodied_online_agent.agent_ros_io import AgentRosCallbacks, AgentRosIo
from embodied_online_agent.asr_endpoint_runtime import AsrEndpointRuntime
from embodied_online_agent.continuous_voice import QueueSnapshot
from embodied_online_agent.ros_action_transport import (
    action_command_to_message,
    command_message_to_dict,
)
from embodied_online_agent.ros_event_transport import wake_event_message_to_domain
from embodied_online_agent.ros_topics import AgentTopicContract
from embodied_online_agent.speaker_transport import (
    enroll_request_to_message,
    identity_message_to_domain,
)
from embodied_online_agent.streaming_turn import StreamingTurnRuntime
from embodied_online_agent.types import ActionCommand
from embodied_online_agent.user_context_runtime import (
    UserContextRuntime,
    UserContextSnapshot,
)
from embodied_online_agent.user_memory import UserMemoryStore
from embodied_online_agent.user_preferences import apply_user_preferences

from .latency import OfflineLatency
from .pseudo_streaming_tts import PseudoStreamingTtsPipeline
from .providers.mock import MockOfflineAsr, MockOfflineLlm, MockOfflineTts


class OfflineAgentNode(LifecycleNode):
    def __init__(self):
        super().__init__("offline_agent")
        # 在线/离线共享同一控制面参数契约，离线模型参数则由 offline profile 扩展。
        # `_parameters` 由 rclpy.Node 自己维护，领域配置快照必须使用独立名称。
        self._agent_parameters = declare_agent_parameters(self, "offline")
        self._mode = self._param("mode")
        self._stopping = False
        self._lifecycle_active = False
        self._continuous_enabled = bool(self._param("continuous_control_enabled"))
        self._latency = OfflineLatency()
        self._asr_events: queue.Queue[tuple[str, bytes | None]] = queue.Queue(maxsize=64)

        self._memory = ConversationMemory(
            self._param("memory_path"), self._param("memory_max_turns")
        )
        self._user_context = UserContextRuntime(
            UserMemoryStore(
                self._param("user_memory_dir"),
                max_recent=int(self._param("user_memory_max_recent")),
                retention_s=float(self._param("user_memory_retention_days"))
                * 86400.0,
            )
        )
        self._control = AgentControlPlane(
            AgentControlPlaneConfig.from_parameters(
                "offline", self._param, self._command_normalization_path()
            )
        )
        self._action_sequencer = SequentialActionPublisher(
            self._param("action_sequence_wait_timeout_s")
        )
        prompt_path = Path(
            get_package_share_directory("embodied_online_agent")
        ) / "prompts" / "system_prompt_zh.txt"
        configured_prompt = self._param("system_prompt_path")
        self._system_prompt = Path(os.path.expanduser(configured_prompt)).read_text(encoding="utf-8") if configured_prompt else prompt_path.read_text(encoding="utf-8")

        self._ros_io = AgentRosIo(
            self,
            AgentRosCallbacks(
                text_input=self._on_text,
                wake_event_input=self._on_wake_event_input,
                speaker_identity=self._on_speaker_identity,
                clear_memory=self._on_clear,
                action_result=self._on_action_result,
                clean_audio=self._on_audio,
                silence_timeout=self._on_silence,
                speech_started=self._on_speech_started,
                speech_ended=self._on_speech_ended,
            ),
            microphone_enabled=bool(self._param("microphone_enabled")),
            external_wake_event_enabled=bool(
                self._param("external_wake_event_enabled")
            ),
            topics=AgentTopicContract().with_metrics("/offline_agent/metrics"),
        )
        self._events = self._ros_io.events
        self._asr = None
        self._llm = None
        self._tts = None
        self._execution = None
        self._asr_endpoint = None
        self._asr_thread = None

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """加载离线模型并创建运行时；模型失败不会进入 inactive/active。"""

        managed = super().on_configure(_state)
        if managed != TransitionCallbackReturn.SUCCESS:
            return managed
        try:
            self._asr, self._llm, self._tts = self._create_providers()
            if self._mode == "offline" and self._param("runtime_warmup_enabled"):
                self._warmup_runtime()
            self._execution = AgentExecutionRuntime(
                self._control,
                execute_item=self._run_queued_turn,
                publish_execution=self._events.publish_execution,
                publish_queue=self._events.publish_queue,
                on_error=lambda error: self.get_logger().error(
                    f"continuous command failed: {error}"
                ),
            )
            self._asr_endpoint = AsrEndpointRuntime(
                delay_ms=self._param("asr_commit_delay_ms"),
                blocked=lambda: self._is_busy()
                and not self._continuous_enabled,
                commit=lambda: self._enqueue_asr(("commit", None), preserve=True),
                on_endpoint=self._on_asr_endpoint,
                on_commit=self._events.publish_asr_commit,
                on_duplicate=lambda source: self.get_logger().debug(
                    f"ignored duplicate ASR commit from {source}"
                ),
                on_error=lambda error: self.get_logger().error(
                    f"ASR endpoint commit failed: {error}"
                ),
            )
            self.get_logger().info("offline agent configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"offline agent configure failed: {exc}")
            self._release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if self._execution is None or self._asr_endpoint is None or self._asr is None:
            self.get_logger().error("offline agent activate requested before configure")
            return TransitionCallbackReturn.FAILURE
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        try:
            self._lifecycle_active = True
            self._ros_io.set_lifecycle_active(True)
            if self._param("microphone_enabled"):
                self._drain_asr_events()
                if hasattr(self._asr, "reset"):
                    self._asr.reset()
                self._asr.start(self._on_asr_partial, self._on_asr_final)
                self._asr_thread = threading.Thread(
                    target=self._run_asr, daemon=True
                )
                self._asr_thread.start()
            if not self._execution.start():
                raise RuntimeError("previous Agent turn did not quiesce")
            self._publish_state("listening")
            self._events.publish_ready(
                f"provider_mode={self._mode};"
                f"microphone={self._param('microphone_enabled')}"
            )
            self.get_logger().info(
                f"offline agent active: mode={self._mode}, "
                f"microphone={self._param('microphone_enabled')}"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"offline agent activate failed: {exc}")
            self.on_deactivate(state)
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._lifecycle_active = False
        if self._asr_endpoint is not None:
            self._asr_endpoint.cancel_pending()
        self._action_sequencer.cancel("lifecycle_deactivated")
        self._control.command_queue.clear()
        self._events.publish_session_event(
            self._control.reset_session("lifecycle")
        )
        self._publish_actions([ActionCommand("stop", {}, priority=True)])
        asr_quiesced = self._stop_asr_worker()
        execution_quiesced = self._execution is None or self._execution.stop(
            self._param("agent_deactivate_timeout_s")
        )
        if not asr_quiesced or not execution_quiesced:
            self._publish_state("deactivate_timeout")
            self._events.publish_stopped("deactivate_timeout")
            self.get_logger().error("offline agent deactivate timed out")
            return TransitionCallbackReturn.FAILURE
        self._publish_state("inactive")
        self._events.publish_stopped("lifecycle_inactive")
        self._ros_io.set_lifecycle_active(False)
        result = super().on_deactivate(state)
        self.get_logger().info("offline agent inactive")
        return result

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        self._lifecycle_active = False
        self._ros_io.set_lifecycle_active(False)
        if not self._release_resources():
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("offline agent cleaned up")
        return super().on_cleanup(_state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self.shutdown()
        return super().on_shutdown(_state)

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self._lifecycle_active = False
        self._ros_io.set_lifecycle_active(False)
        self._release_resources()
        self.get_logger().error("offline agent recovered to unconfigured after error")
        return super().on_error(state)

    def autostart_if_enabled(self) -> None:
        if not self._param("agent_lifecycle_autostart"):
            return
        if self.trigger_configure() != TransitionCallbackReturn.SUCCESS:
            raise RuntimeError("offline agent lifecycle configure failed")
        if self.trigger_activate() != TransitionCallbackReturn.SUCCESS:
            raise RuntimeError("offline agent lifecycle activate failed")

    def _stop_asr_worker(self) -> bool:
        if self._asr_thread is None:
            return True
        self._enqueue_asr(("stop", None), preserve=True)
        self._asr_thread.join(
            timeout=float(self._param("agent_deactivate_timeout_s"))
        )
        stopped = not self._asr_thread.is_alive()
        if stopped:
            self._asr_thread = None
            self._drain_asr_events()
        return stopped

    def _drain_asr_events(self) -> None:
        while True:
            try:
                self._asr_events.get_nowait()
                self._asr_events.task_done()
            except queue.Empty:
                return

    def _release_resources(self) -> bool:
        if self._asr_endpoint is not None:
            self._asr_endpoint.close()
            self._asr_endpoint = None
        execution_quiesced = self._execution is None or self._execution.stop(
            self._param("agent_deactivate_timeout_s")
        )
        asr_quiesced = self._stop_asr_worker()
        if not execution_quiesced or not asr_quiesced:
            self.get_logger().error("offline Agent threads are still running")
            return False
        self._execution = None
        if self._tts is not None and hasattr(self._tts, "close"):
            self._tts.close()
        self._asr = None
        self._llm = None
        self._tts = None
        return True

    def _param(self, name):
        return self._agent_parameters.get(name)

    def _command_normalization_path(self) -> Path | str:
        configured = self._param("command_normalization_path")
        if configured:
            return Path(os.path.expanduser(configured))
        return (
            Path(get_package_share_directory("embodied_online_agent"))
            / "config"
            / "command_normalization_zh.yaml"
        )

    def _create_providers(self):
        if self._mode == "mock":
            return MockOfflineAsr(
                self._mock_asr_finals(), self._mock_asr_partials()
            ), MockOfflineLlm(self._param("mock_token_delay_s")), MockOfflineTts(self._param("tts_sample_rate"))
        if self._mode != "offline":
            raise ValueError("mode must be 'mock' or 'offline'")
        # Native model wheels are intentionally optional in mock mode.
        from .providers.llama_cpp import LlamaCppLlm
        from .providers.sherpa_asr import SherpaZipformerAsr

        return (
            SherpaZipformerAsr(
                self._param("asr_model_dir"),
                self._param("audio_sample_rate"),
                self._param("asr_num_threads"),
                decoding_method=self._param("asr_decoding_method"),
                hotwords_file=self._hotwords_file(),
                hotwords_score=self._param("asr_hotwords_score"),
                max_active_paths=self._param("asr_max_active_paths"),
                modeling_unit=self._param("asr_modeling_unit"),
            ),
            LlamaCppLlm(
                self._param("llm_base_url"), self._param("llm_model"),
                self._param("llm_temperature"), self._param("llm_max_tokens"),
                self._param("llm_seed"),
                timeout_s=self._param("llm_timeout_s"),
                max_retries=self._param("llm_max_retries"),
                first_token_warn_ms=self._param("llm_first_token_warn_ms"),
            ),
            self._create_tts_provider(),
        )

    def _create_tts_provider(self):
        provider = str(self._param("tts_provider") or "sherpa").lower()
        if provider == "sherpa":
            from .providers.sherpa_tts import SherpaVitsTts

            return SherpaVitsTts(
                self._param("tts_model_dir"),
                self._param("tts_num_threads"),
                self._param("tts_speaker_id"),
                self._param("tts_speed"),
            )
        if provider == "summer":
            from .providers.summer_tts import SummerTts

            return SummerTts(
                self._param("summer_tts_binary"),
                self._param("summer_tts_model"),
                timeout_s=float(self._param("summer_tts_timeout_s")),
            )
        if provider == "summer_ros":
            from .providers.summer_tts_ros import SummerTtsRosClient

            return SummerTtsRosClient(
                self,
                service_name=self._param("summer_tts_service_name"),
                timeout_s=float(self._param("summer_tts_service_timeout_s")),
                speaker_id=int(self._param("summer_tts_service_speaker_id")),
                length_scale=float(self._param("summer_tts_service_length_scale")),
            )
        raise ValueError("tts_provider must be 'sherpa', 'summer', or 'summer_ros'")

    def _tts_sample_rate(self):
        return int(getattr(self._tts, "sample_rate", self._param("tts_sample_rate")))

    def _warmup_runtime(self):
        """在 ready 前预热真实 system+history 前缀，避免首条语音承担 prefill。"""
        started = time.perf_counter()
        llm_report = self._llm.warmup(
            self._llm_messages("只回复：就绪。", self._user_context.snapshot())
        )
        tts_started = time.perf_counter()
        warmup_pcm = self._tts.synthesize("好。")
        report = {
            "total_ms": round((time.perf_counter() - started) * 1000.0, 2),
            "llm": llm_report.get("metrics", {}),
            "tts_ms": round((time.perf_counter() - tts_started) * 1000.0, 2),
            "tts_pcm_bytes": len(warmup_pcm),
        }
        self.get_logger().info(
            "offline runtime warmup complete: "
            + json.dumps(report, ensure_ascii=False)
        )

    def _llm_messages(self, user_text, user_context: UserContextSnapshot):
        # ConversationMemory 中保存原始模型协议输出，使该列表能与 server slot 的
        # token 前缀精确对齐；不要在这里只给当前 user 临时追加不同的后缀。
        return self._memory.prompt_messages(
            user_context.system_prompt(self._system_prompt) + "\n/no_think",
            user_text,
        )

    def _mock_asr_finals(self):
        scripted = str(self._param("mock_asr_finals") or "")
        return [item.strip() for item in scripted.split("|") if item.strip()]

    def _mock_asr_partials(self):
        scripted = str(self._param("mock_asr_partials") or "")
        if not scripted:
            return []
        return ["" if item.strip() in {"", "-"} else item.strip() for item in scripted.split("|")]

    def _hotwords_file(self):
        configured = self._param("asr_hotwords_file")
        if configured:
            return str(Path(os.path.expanduser(configured)))
        return str(
            Path(get_package_share_directory("embodied_offline_agent"))
            / "config"
            / "hotwords_zh.txt"
        )

    def _on_audio(self, message):
        if not self._lifecycle_active or self._asr is None:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._enqueue_asr(("audio", bytes(message.data)))

    def _on_silence(self, _message):
        if not self._lifecycle_active:
            return
        self._commit_asr_endpoint("silence_timeout")

    def _on_speech_started(self, _message):
        if not self._lifecycle_active:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._control.transcript_stabilizer.clear()
        self._publish_state("speech_detected")

    def _on_speech_ended(self, _message):
        if not self._lifecycle_active:
            return
        if not self._param("speech_endpoint_events_enabled"):
            return
        self._commit_asr_endpoint("speech_ended")

    def _commit_asr_endpoint(self, source):
        if self._asr_endpoint is not None:
            self._asr_endpoint.request(source)

    def _on_asr_endpoint(self, source, delay_ms):
        # 每个 utterance 建立独立延迟对象，端点时刻是离线 E2E 的统一起点。
        self._latency = OfflineLatency()
        self._latency.mark_silence()
        self._events.publish_asr_endpoint(source, delay_ms)

    def _enqueue_asr(self, event, preserve=False):
        try:
            self._asr_events.put_nowait(event)
        except queue.Full:
            try:
                self._asr_events.get_nowait()
                self._asr_events.task_done()
                self._asr_events.put_nowait(event)
                self.get_logger().warning("ASR input backlog: dropped oldest audio frame")
            except queue.Empty:
                if preserve:
                    self.get_logger().error("ASR commit event could not be queued")

    def _run_asr(self):
        while not self._stopping:
            kind, payload = self._asr_events.get()
            try:
                if kind == "stop":
                    return
                if (
                    kind in {"audio", "commit"}
                    and self._is_busy()
                    and not self._continuous_enabled
                ):
                    continue
                if kind == "audio" and payload is not None:
                    self._asr.push_audio(payload)
                elif kind == "commit":
                    self._asr.commit()
            except Exception as exc:
                self.get_logger().error(f"offline ASR failed: {exc}")
            finally:
                self._asr_events.task_done()

    def _is_busy(self):
        return self._execution is not None and self._execution.is_busy()

    def _on_asr_partial(self, text):
        if not self._lifecycle_active:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._control.transcript_stabilizer.observe_partial(text)
        self._ros_io.publish_asr_partial(text)

    def _on_asr_final(self, text):
        if not self._lifecycle_active:
            return
        if self._is_busy() and not self._continuous_enabled:
            self._control.transcript_stabilizer.clear()
            self.get_logger().warning(
                "offline agent busy; suppressing overlapping ASR final"
            )
            return
        stabilized = self._control.transcript_stabilizer.finalize(text)
        if stabilized.recovered:
            self._publish_recognition_payload(stabilized.feedback_dict())
            self.get_logger().info(
                f"recovered ASR final from partial: '{text}' -> '{stabilized.text}'"
            )
        text = stabilized.text
        self._latency.mark_asr_final()
        self._ros_io.publish_asr_final(text)
        self._accept_transcript(text)

    def _on_text(self, message):
        if not self._lifecycle_active:
            self.get_logger().debug("ignored text input while Agent is inactive")
            return
        self._control.transcript_stabilizer.clear()
        self._latency = OfflineLatency()
        self._latency.mark_silence()
        self._latency.mark_asr_final()
        self._ros_io.publish_asr_final(message.data)
        self._accept_transcript(message.data)

    def _on_wake_event_input(self, message):
        if not self._lifecycle_active:
            return
        try:
            event = wake_event_message_to_domain(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        if event.kind == "wake":
            session_event = self._control.voice_session.external_wake(
                event.provider, event.transcript
            )
            self._publish_session_event(session_event)
            self._publish_state("session_awake")
            self.get_logger().info(
                f"external wake event accepted from provider={event.provider}"
            )
            return

        dropped = self._control.command_queue.clear()
        self._publish_queue_event(
            "clear", "", QueueSnapshot(True, self._control.command_queue.size(), dropped)
        )
        self._action_sequencer.cancel("external_sleep")
        self._publish_actions([ActionCommand("stop", {}, priority=True)])
        session_event = self._control.voice_session.external_sleep(event.provider)
        self._publish_session_event(session_event)
        self._publish_state("sleeping")
        self.get_logger().info(
            f"external sleep event accepted from provider={event.provider}"
        )

    def _on_clear(self, _message):
        self._memory.clear()
        self._user_context.clear_current()

    def _on_action_result(self, message):
        self._action_sequencer.notify_result(
            message.command_id,
            message.success,
            message.message,
            status=message.status,
        )

    def _on_speaker_identity(self, message):
        identity = identity_message_to_domain(
            message,
            min_confidence=float(self._param("speaker_identity_min_confidence")),
        )
        self._user_context.update_identity(identity)
        if identity.usable:
            self.get_logger().info(
                f"speaker identity accepted: speaker_id={identity.speaker_id}, confidence={identity.confidence:.3f}"
            )

    def _accept_transcript(self, transcript):
        decision = self._control.accept_transcript(
            transcript,
            wake_word_required=bool(self._param("wake_word_enabled")),
        )
        self._events.publish_control_decision(decision)
        if decision.directive in {"sleep", "priority"}:
            self._action_sequencer.cancel(decision.cancel_reason)
            self.get_logger().info(
                f"{decision.cancel_reason} received; cancelling current sequence"
            )
            if decision.dropped:
                self.get_logger().info(
                    f"cleared {decision.dropped} queued command(s)"
                )
            self._publish_actions(
                [ActionCommand(decision.priority_action, {}, priority=True)]
            )
            return
        if decision.directive != "command":
            return

        command = decision.command
        if self._handle_memory_command(command):
            self._publish_state("listening")
            return
        if self._continuous_enabled:
            self._enqueue_continuous_command(command)
            return
        if self._execution is None:
            return
        turn_latency = self._latency
        user_context = self._user_context.snapshot()
        if not self._execution.start_background_turn(
            self._run_turn, command, turn_latency, user_context
        ):
            self.get_logger().warning("offline agent busy; overlapping utterance dropped")
            return

    def _handle_memory_command(self, command):
        result = self._user_context.handle_command(command)
        if result is None:
            return False
        if result.enroll_request is not None:
            self._ros_io.publish_speaker_enroll_request(
                enroll_request_to_message(
                    result.enroll_request, stamp=self.get_clock().now()
                )
            )
        self._ros_io.publish_response_delta(result.response)
        self._ros_io.publish_response(result.response)
        if self._execution is not None and not self._execution.start_background_turn(
            self._speak_memory_response, result.response
        ):
            self.get_logger().debug("memory response TTS skipped while Agent is busy")
        return True

    def _speak_memory_response(self, response):
        try:
            self._execution.raise_if_stopping()
            self._ros_io.publish_tts_audio(self._tts.synthesize(response))
            self._execution.raise_if_stopping()
        except AgentExecutionCancelled:
            raise
        except Exception as exc:
            self.get_logger().warning(f"memory response TTS failed: {exc}")

    def _run_turn(self, user_text, latency, user_context: UserContextSnapshot):
        tts_pipeline = PseudoStreamingTtsPipeline(
            synthesize=self._tts.synthesize,
            publish_audio=self._ros_io.publish_tts_audio,
            sample_rate=self._tts_sample_rate(),
            pcm_chunk_ms=int(self._param("tts_pcm_chunk_ms")),
            on_first_audio=latency.mark_first_audio,
        )
        tts_pipeline.start()

        def enqueue_tts_text(text):
            self._publish_state("speaking")
            if not tts_pipeline.put_text(text):
                raise TimeoutError("message double buffer remained full")

        turn = StreamingTurnRuntime(
            max_chunk_chars=self._param("tts_chunk_max_chars"),
            on_first_token=latency.mark_first_token,
            on_speech_delta=self._ros_io.publish_response_delta,
            on_speakable=enqueue_tts_text,
            on_protocol_error=self.get_logger().warning,
        )
        self._publish_state("thinking")
        try:
            messages = self._llm_messages(user_text, user_context)
            latency.mark_llm_start()
            for token in self._llm.stream(messages):
                self._execution.raise_if_stopping()
                turn.feed(token)
            self._execution.raise_if_stopping()
            result = turn.finish(user_text)
            if result.action_source == "blocked" and result.model_actions:
                self.get_logger().warning(
                    "model actions blocked by semantic safety policy"
                )
            action_report = self._publish_actions(result.actions, user_context)
            tts_metrics = tts_pipeline.close_and_wait(timeout_s=60.0)
            assistant_text = result.assistant_text
            self._ros_io.publish_response(assistant_text)
            self._memory.append_turn(
                user_text,
                assistant_text,
                model_output=result.model_output,
            )
            self._user_context.record_interaction(
                user_context,
                user_text=user_text,
                assistant_text=assistant_text,
                # 被语义安全策略挡住的动作不能污染用户行为画像。
                actions=[action.as_dict() for action in result.actions],
                success=not action_report.failed,
            )
            latency.finish()
            report = latency.report(
                tts_metrics.message_buffer_dropped,
                tts_metrics.audio_buffer_dropped,
            )
            # llama.cpp 的吞吐和首 token 指标与端到端延迟分开记录；
            # 这样验收时能判断是 ASR、LLM 还是 TTS/动作链路导致慢。
            report["llm_provider"] = getattr(self._llm, "last_metrics", {})
            report["tts_pipeline"] = tts_metrics.as_dict()
            self._ros_io.publish_metrics(json.dumps(report, ensure_ascii=False))
            self.get_logger().info(f"offline latency: {report}")
        except AgentExecutionCancelled:
            tts_pipeline.abort()
            self.get_logger().info(
                "offline turn cancelled by lifecycle transition"
            )
            raise
        except Exception as exc:
            tts_pipeline.abort()
            self.get_logger().error(f"offline turn failed: {exc}")
            self._publish_state("error")
            raise
        finally:
            if not self._stopping and self._lifecycle_active:
                self._publish_state("listening")

    def _run_queued_turn(self, item):
        context = item.context if isinstance(item.context, dict) else {}
        user_context = context.get("user_context")
        if not isinstance(user_context, UserContextSnapshot):
            user_context = self._user_context.snapshot()
        actions = self._control.preparsed_actions(context)
        latency = context.get("latency")
        if not isinstance(latency, OfflineLatency):
            latency = item.context if isinstance(item.context, OfflineLatency) else OfflineLatency()
        if actions:
            self._run_preparsed_turn(
                item.text,
                actions,
                latency,
                user_context,
            )
            return
        self._run_turn(item.text, latency, user_context)

    def _run_preparsed_turn(
        self,
        user_text,
        actions,
        latency,
        user_context: UserContextSnapshot,
    ):
        self._publish_state("thinking")
        response = "好的，按顺序执行：" + "，".join(action.name for action in actions) + "。"
        self._ros_io.publish_response_delta(response)
        self._ros_io.publish_response(response)
        report = self._publish_actions(actions, user_context)
        self._user_context.record_interaction(
            user_context,
            user_text=user_text,
            assistant_text=response,
            actions=[action.as_dict() for action in actions],
            success=not report.failed,
        )
        latency.finish()
        report = latency.report(0, 0)
        self._ros_io.publish_metrics(json.dumps(report, ensure_ascii=False))

    def _enqueue_continuous_command(self, command):
        user_context = self._user_context.snapshot()
        private_context = {
            "latency": self._latency,
            "user_context": user_context,
        }
        decision = self._control.enqueue_command(
            command,
            context_extras=private_context,
            fallback_context=private_context,
        )
        self._events.publish_enqueue_decision(decision)
        if decision.status == "queued":
            self.get_logger().info(
                f"continuous command queued: size={decision.queue_size}, text={command}"
            )
        elif decision.status == "retry":
            self.get_logger().warning(
                f"incomplete voice command: reason={decision.reason}, text={command}"
            )
        elif decision.status == "rejected":
            self.get_logger().warning(
                f"continuous command queue rejected input: {decision.reason}"
            )

    def _publish_actions(
        self,
        actions,
        user_context: UserContextSnapshot | None = None,
    ):
        context = user_context or self._user_context.snapshot()
        action_list = apply_user_preferences(actions, context.preferences)

        def publish_payload(action: ActionCommand):
            message = action_command_to_message(action, source="offline_agent")
            self._ros_io.publish_action_candidate(message)
            self.get_logger().info(
                "action candidate: "
                + json.dumps(command_message_to_dict(message), ensure_ascii=False)
            )

        report = self._action_sequencer.publish(
            action_list,
            publish_payload,
            wait_for_results=self._should_wait_for_action_results(action_list),
        )
        if report.failed:
            self.get_logger().warning(
                f"action sequence stopped after {report.completed} completed step(s): {report.reason}"
            )
        return report

    def _should_wait_for_action_results(self, action_list):
        if self._execution is None:
            return len(action_list) > 1
        return self._execution.should_wait_for_action_results(
            len(action_list)
        )

    def _publish_state(self, state):
        self._events.publish_state(state)

    def _publish_session_event(self, event):
        self._events.publish_session_event(event)

    def _publish_queue_event(
        self,
        event,
        text,
        snapshot,
        *,
        priority_stop=False,
    ):
        payload = self._control.queue_event(
            event, text, snapshot, priority_stop=priority_stop
        )
        self._events.publish_queue(payload)

    def _publish_recognition_payload(self, payload):
        self._events.publish_recognition(payload)

    def shutdown(self):
        if self._stopping:
            return
        self._stopping = True
        self._lifecycle_active = False
        self._action_sequencer.cancel("shutdown")
        self._release_resources()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OfflineAgentNode()
        node.autostart_if_enabled()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
