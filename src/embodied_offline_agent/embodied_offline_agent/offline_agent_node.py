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

from embodied_agent_core.memory import ConversationMemory
from embodied_agent_core.action_sequence import SequentialActionPublisher
from embodied_agent_core.agent_application_runtime import (
    AgentApplicationCallbacks,
    AgentApplicationRuntime,
)
from embodied_agent_core.agent_control_plane import (
    AgentControlPlane,
    AgentControlPlaneConfig,
)
from embodied_agent_core.agent_execution_runtime import (
    AgentExecutionCancelled,
    AgentExecutionRuntime,
)
from embodied_agent_core.agent_lifecycle_runtime import AgentLifecycleRuntime
from embodied_agent_core.agent_parameters import declare_agent_parameters
from embodied_agent_core.agent_ros_io import AgentRosCallbacks, AgentRosIo
from embodied_agent_core.asr_endpoint_runtime import AsrEndpointRuntime
from embodied_agent_core.metrics_transport import agent_turn_metrics_to_message
from embodied_agent_core.ros_action_transport import action_command_to_message
from embodied_agent_core.ros_event_transport import wake_event_message_to_domain
from embodied_agent_core.speaker_transport import (
    enroll_request_to_message,
    identity_message_to_domain,
)
from embodied_agent_core.types import ActionCommand
from embodied_agent_core.user_context_runtime import (
    UserContextRuntime,
    UserContextSnapshot,
)
from embodied_agent_core.user_memory import UserMemoryStore

from .latency import OfflineLatency
from .providers.mock import MockOfflineAsr, MockOfflineLlm, MockOfflineTts
from .offline_turn_runtime import OfflineStreamingTurnRuntime


class OfflineAgentNode(LifecycleNode):
    def __init__(self):
        super().__init__("offline_agent")
        # 在线/离线共享同一控制面参数契约，离线模型参数则由 offline profile 扩展。
        # `_parameters` 由 rclpy.Node 自己维护，领域配置快照必须使用独立名称。
        self._agent_parameters = declare_agent_parameters(self, "offline")
        self._mode = self._param("mode")
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
            self._param("action_sequence_wait_timeout_s"),
            self._param("long_action_result_timeout_s"),
        )
        prompt_path = Path(
            get_package_share_directory("embodied_agent_core")
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
        )
        self._events = self._ros_io.events
        self._runtime = AgentLifecycleRuntime(
            control=self._control,
            action_sequencer=self._action_sequencer,
            ros_io=self._ros_io,
            events=self._events,
            publish_priority_stop=self._publish_priority_stop,
            deactivate_timeout_s=self._param("agent_deactivate_timeout_s"),
        )
        self._application = AgentApplicationRuntime(
            source="offline",
            control=self._control,
            lifecycle_runtime=self._runtime,
            action_sequencer=self._action_sequencer,
            conversation_memory=self._memory,
            user_context=self._user_context,
            events=self._events,
            callbacks=AgentApplicationCallbacks(
                run_model_turn=self._run_turn,
                speak_memory_response=self._speak_memory_response,
                publish_action_candidate=self._publish_action_candidate,
                publish_enroll_request=self._publish_enroll_request,
                publish_response_delta=self._ros_io.publish_response_delta,
                publish_response=self._ros_io.publish_response,
                capture_turn_context=self._capture_turn_context,
                finish_preparsed_turn=self._finish_preparsed_turn,
            ),
            logger=self.get_logger(),
            wake_word_required=bool(self._param("wake_word_enabled")),
            continuous_enabled=self._continuous_enabled,
        )
        self._asr = None
        self._llm = None
        self._tts = None
        self._asr_thread = None
        self._turn_runner = None

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """加载离线模型并创建运行时；模型失败不会进入 inactive/active。"""

        managed = super().on_configure(_state)
        if managed != TransitionCallbackReturn.SUCCESS:
            return managed
        try:
            self._asr, self._llm, self._tts = self._create_providers()
            if self._mode == "offline" and self._param("runtime_warmup_enabled"):
                self._warmup_runtime()
            execution = AgentExecutionRuntime(
                self._control,
                execute_item=self._application.run_queued_turn,
                publish_execution=self._events.publish_execution,
                publish_queue=self._events.publish_queue,
                on_error=lambda error: self.get_logger().error(
                    f"continuous command failed: {error}"
                ),
            )
            endpoint = AsrEndpointRuntime(
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
            self._runtime.bind(execution=execution, endpoint=endpoint)
            self._turn_runner = OfflineStreamingTurnRuntime(
                llm=self._llm,
                tts=self._tts,
                memory=self._memory,
                user_context=self._user_context,
                ros_io=self._ros_io,
                events=self._events,
                lifecycle_runtime=self._runtime,
                publish_actions=self._application.publish_actions,
                publish_metrics=self._publish_offline_metrics,
                llm_messages=self._llm_messages,
                tts_sample_rate=self._tts_sample_rate,
                param=self._param,
                logger=self.get_logger(),
            )
            self.get_logger().info("offline agent configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"offline agent configure failed: {exc}")
            self._release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if not self._runtime.configured or self._asr is None:
            self.get_logger().error("offline agent activate requested before configure")
            return TransitionCallbackReturn.FAILURE
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        try:
            self._runtime.activate(self._start_input)
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
        quiescence = self._runtime.deactivate(self._stop_asr_worker)
        if not quiescence.quiesced:
            self.get_logger().error(
                "offline agent deactivate timed out: "
                f"input={quiescence.input_quiesced}, "
                f"execution={quiescence.execution_quiesced}, "
                f"errors={quiescence.errors}"
            )
            return TransitionCallbackReturn.FAILURE
        result = super().on_deactivate(state)
        self.get_logger().info("offline agent inactive")
        return result

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        if not self._release_resources():
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("offline agent cleaned up")
        return super().on_cleanup(_state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self.shutdown()
        return super().on_shutdown(_state)

    def on_error(self, state: State) -> TransitionCallbackReturn:
        if self._runtime.active:
            self._runtime.deactivate(self._stop_asr_worker)
        else:
            self._runtime.mark_error_inactive()
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
        quiescence = self._runtime.release(self._stop_asr_worker)
        if not quiescence.quiesced:
            self.get_logger().error(
                "offline Agent resources are still running: "
                f"input={quiescence.input_quiesced}, "
                f"execution={quiescence.execution_quiesced}, "
                f"errors={quiescence.errors}"
            )
            return False
        if self._tts is not None and hasattr(self._tts, "close"):
            self._tts.close()
        self._turn_runner = None
        self._asr = None
        self._llm = None
        self._tts = None
        return True

    def _start_input(self) -> None:
        if not self._param("microphone_enabled"):
            return
        self._drain_asr_events()
        if hasattr(self._asr, "reset"):
            self._asr.reset()
        self._asr.start(self._on_asr_partial, self._on_asr_final)
        self._asr_thread = threading.Thread(target=self._run_asr, daemon=True)
        self._asr_thread.start()

    def _publish_priority_stop(self) -> None:
        self._application.publish_actions(
            [ActionCommand("stop", {}, priority=True)]
        )

    def _param(self, name):
        return self._agent_parameters.get(name)

    def _command_normalization_path(self) -> Path | str:
        configured = self._param("command_normalization_path")
        if configured:
            return Path(os.path.expanduser(configured))
        return (
            Path(get_package_share_directory("embodied_agent_core"))
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
        if not self._runtime.active or self._asr is None:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._enqueue_asr(("audio", bytes(message.data)))

    def _on_silence(self, _message):
        if not self._runtime.active:
            return
        self._commit_asr_endpoint("silence_timeout")

    def _on_speech_started(self, _message):
        if not self._runtime.active:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._control.transcript_stabilizer.clear()
        self._publish_state("speech_detected")

    def _on_speech_ended(self, _message):
        if not self._runtime.active:
            return
        if not self._param("speech_endpoint_events_enabled"):
            return
        self._commit_asr_endpoint("speech_ended")

    def _commit_asr_endpoint(self, source):
        if self._runtime.endpoint is not None:
            self._runtime.endpoint.request(source)

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
        while not self._runtime.stopping:
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
        return (
            self._runtime.execution is not None
            and self._runtime.execution.is_busy()
        )

    def _on_asr_partial(self, text):
        if not self._runtime.active:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._control.transcript_stabilizer.observe_partial(text)
        self._ros_io.publish_asr_partial(text)

    def _on_asr_final(self, text):
        if not self._runtime.active:
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
        if not self._runtime.active:
            self.get_logger().debug("ignored text input while Agent is inactive")
            return
        self._control.transcript_stabilizer.clear()
        self._latency = OfflineLatency()
        self._latency.mark_silence()
        self._latency.mark_asr_final()
        self._ros_io.publish_asr_final(message.data)
        self._accept_transcript(message.data)

    def _on_wake_event_input(self, message):
        if not self._runtime.active:
            return
        try:
            event = wake_event_message_to_domain(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._application.handle_wake_event(event)

    def _on_clear(self, _message):
        self._application.clear_memory()

    def _on_action_result(self, message):
        self._application.notify_action_result(
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
        self._application.update_speaker_identity(identity)

    def _accept_transcript(self, transcript):
        self._application.accept_transcript(transcript)

    def _publish_enroll_request(self, request):
        self._ros_io.publish_speaker_enroll_request(
            enroll_request_to_message(request, stamp=self.get_clock().now())
        )

    def _speak_memory_response(self, response):
        try:
            self._runtime.execution.raise_if_stopping()
            self._ros_io.publish_tts_audio(self._tts.synthesize(response))
            self._runtime.execution.raise_if_stopping()
        except AgentExecutionCancelled:
            raise
        except Exception as exc:
            self.get_logger().warning(f"memory response TTS failed: {exc}")

    def _capture_turn_context(self):
        """冻结当前 utterance 的延迟对象，避免排队期间被下一句 ASR 覆盖。"""

        return self._latency

    def _run_turn(self, user_text, latency, user_context: UserContextSnapshot):
        if self._turn_runner is None:
            raise RuntimeError("offline turn runtime is not configured")
        self._turn_runner.run(user_text, latency, user_context)

    def _publish_offline_metrics(self, report):
        self._ros_io.publish_metrics(
            agent_turn_metrics_to_message(
                "offline", report, stamp=self.get_clock().now().to_msg()
            )
        )
        self.get_logger().info(f"offline latency: {report}")

    def _finish_preparsed_turn(self, latency):
        """NLU 已直接产出动作时没有 LLM/TTS，仍发布同一份端到端指标。"""

        if not isinstance(latency, OfflineLatency):
            return
        latency.finish()
        report = latency.report(0, 0)
        self._publish_offline_metrics(report)

    def _publish_action_candidate(self, action: ActionCommand):
        """离线模型只产出领域动作；ROS typed msg 转换集中在这一处。"""

        message = action_command_to_message(action, source="offline_agent")
        self._ros_io.publish_action_candidate(message)
        self.get_logger().info(
            f"action candidate: type={message.action_type}, "
            f"command_id={message.command_id}, priority={message.priority}"
        )

    def _publish_state(self, state):
        self._events.publish_state(state)

    def _publish_recognition_payload(self, payload):
        self._events.publish_recognition(payload)

    def shutdown(self):
        if not self._runtime.begin_shutdown():
            return
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
