import json
import os
import queue
import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from embodied_agent_interfaces.msg import (
    RobotCommand,
    RobotCommandResult,
    SpeakerEnrollRequest as SpeakerEnrollRequestMessage,
    SpeakerIdentity as SpeakerIdentityMessage,
    WakeEvent as WakeEventMessage,
)
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray

from .memory import ConversationMemory
from .user_memory import UserMemoryStore
from .action_sequence import SequentialActionPublisher
from .agent_control_plane import (
    AgentControlPlane,
    AgentControlPlaneConfig,
)
from .agent_execution_runtime import (
    AgentExecutionCancelled,
    AgentExecutionRuntime,
)
from .agent_parameters import declare_agent_parameters
from .asr_endpoint_runtime import AsrEndpointRuntime
from .continuous_voice import QueueSnapshot
from .metrics import LatencyTracker
from .ros_action_transport import action_command_to_message, command_message_to_dict
from .ros_agent_events import RosAgentEventPublisher
from .ros_event_transport import wake_event_message_to_domain
from .speaker_transport import enroll_request_to_message, identity_message_to_domain
from .streaming_turn import StreamingTurnRuntime
from .types import ActionCommand
from .user_context_runtime import UserContextRuntime, UserContextSnapshot
from .user_preferences import apply_user_preferences
from .providers.mock import MockAsr, MockLlm, MockTts
from .providers.openai_compatible_llm import OpenAiCompatibleLlm
from .providers.qwen_asr import QwenRealtimeAsr
from .providers.qwen_tts import QwenRealtimeTts


class OnlineAgentNode(LifecycleNode):
    def __init__(self):
        super().__init__("online_agent")
        # 参数在连接云端 provider 前完成声明和校验，避免错误配置到运行中才暴露。
        # 不使用 `_parameters`：它是 rclpy.Node 的内部参数表，覆盖后会让参数服务崩溃。
        self._agent_parameters = declare_agent_parameters(self, "online")
        self.mode = self._param("mode")
        self._stopping = False
        self._lifecycle_active = False
        self._continuous_enabled = bool(self._param("continuous_control_enabled"))
        self.metrics = LatencyTracker()
        self._control = AgentControlPlane(
            AgentControlPlaneConfig.from_parameters(
                "online", self._param, self._command_normalization_path()
            )
        )
        self.action_sequencer = SequentialActionPublisher(
            self._param("action_sequence_wait_timeout_s")
        )
        self.memory = ConversationMemory(
            self._param("memory_path"),
            max_turns=self._param("memory_max_turns"),
        )
        self._user_context = UserContextRuntime(
            UserMemoryStore(
                self._param("user_memory_dir"),
                max_recent=int(self._param("user_memory_max_recent")),
                retention_s=float(self._param("user_memory_retention_days"))
                * 86400.0,
            )
        )
        self.system_prompt = self._load_system_prompt()

        self.asr_partial_pub = self.create_lifecycle_publisher(
            String, "/agent/asr_partial", 10
        )
        self.asr_final_pub = self.create_lifecycle_publisher(
            String, "/agent/asr_final", 10
        )
        self.response_pub = self.create_lifecycle_publisher(
            String, "/agent/response_text", 10
        )
        self.response_delta_pub = self.create_lifecycle_publisher(
            String, "/agent/response_delta", 10
        )
        self.action_candidate_pub = self.create_lifecycle_publisher(
            RobotCommand, "/agent/action_candidate", 10
        )
        self._events = RosAgentEventPublisher(
            self, publisher_factory=self.create_lifecycle_publisher
        )
        self.speaker_enroll_request_pub = self.create_lifecycle_publisher(
            SpeakerEnrollRequestMessage, "/agent/speaker_enroll_request", 10
        )
        self.metrics_pub = self.create_lifecycle_publisher(
            String, "/agent/metrics", 10
        )
        self.create_subscription(String, "/agent/text_input", self._on_text_input, 10)
        if self._param("external_wake_event_enabled"):
            self.create_subscription(
                WakeEventMessage,
                "/agent/wake_event_input",
                self._on_wake_event_input,
                10,
            )
        self.create_subscription(
            SpeakerIdentityMessage,
            "/agent/speaker_identity",
            self._on_speaker_identity,
            10,
        )
        self.create_subscription(Empty, "/agent/clear_memory", self._on_clear_memory, 10)
        self.create_subscription(
            RobotCommandResult, "/robot/action_result", self._on_action_result, 10
        )
        audio_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.tts_audio_pub = self.create_lifecycle_publisher(
            UInt8MultiArray, "/audio/tts_pcm", audio_qos
        )
        self.asr = None
        self.llm = None
        self.tts = None
        self._execution = None
        self._asr_endpoint = None
        self.audio_subscription = None
        self.silence_subscription = None

        if self._param("microphone_enabled"):
            self.audio_subscription = self.create_subscription(
                UInt8MultiArray,
                "/audio/clean_pcm",
                self._on_clean_audio,
                audio_qos,
            )
            self.silence_subscription = self.create_subscription(
                Empty,
                "/audio/silence_timeout",
                self._on_silence_timeout,
                10,
            )
            self.create_subscription(
                Empty,
                "/audio/speech_started",
                self._on_speech_started,
                10,
            )
            self.create_subscription(
                Empty,
                "/audio/speech_ended",
                self._on_speech_ended,
                10,
            )
    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """创建 provider 和并发运行时；失败时节点保持 unconfigured。"""

        managed = super().on_configure(_state)
        if managed != TransitionCallbackReturn.SUCCESS:
            return managed
        try:
            self.asr, self.llm, self.tts = self._create_providers()
            if self.mode == "online" and self._param("online_warmup_enabled"):
                if hasattr(self.tts, "connect"):
                    self.tts.connect()
                if hasattr(self.llm, "warmup"):
                    self.llm.warmup()
            self._execution = AgentExecutionRuntime(
                self._control,
                execute_item=self._run_queued_turn,
                publish_execution=self._events.publish_execution,
                publish_queue=self._events.publish_queue,
                on_error=lambda error: self.get_logger().error(
                    f"continuous command failed: {error}"
                ),
                before_execute=lambda _item: self._prepare_queued_turn(),
            )
            self._asr_endpoint = AsrEndpointRuntime(
                delay_ms=self._param("asr_commit_delay_ms"),
                blocked=lambda: self._is_busy()
                and not self._continuous_enabled,
                commit=self.asr.commit,
                on_endpoint=self._events.publish_asr_endpoint,
                on_commit=self._events.publish_asr_commit,
                on_duplicate=lambda source: self.get_logger().debug(
                    f"ignored duplicate ASR commit from {source}"
                ),
                on_error=lambda error: self.get_logger().error(
                    f"ASR endpoint commit failed: {error}"
                ),
            )
            self.get_logger().info("online agent configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"online agent configure failed: {exc}")
            self._release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if self._execution is None or self._asr_endpoint is None or self.asr is None:
            self.get_logger().error("online agent activate requested before configure")
            return TransitionCallbackReturn.FAILURE
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        try:
            self._lifecycle_active = True
            if self._param("microphone_enabled"):
                self.asr.start(self._on_asr_partial, self._on_asr_final)
            if not self._execution.start():
                raise RuntimeError("previous Agent turn did not quiesce")
            self._publish_state("listening")
            self._events.publish_ready(
                f"provider_mode={self.mode};"
                f"microphone={self._param('microphone_enabled')}"
            )
            self.get_logger().info(
                f"online agent active: mode={self.mode}, "
                f"microphone={self._param('microphone_enabled')}"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"online agent activate failed: {exc}")
            self.on_deactivate(state)
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._lifecycle_active = False
        if self._asr_endpoint is not None:
            self._asr_endpoint.cancel_pending()
        self.action_sequencer.cancel("lifecycle_deactivated")
        self._control.command_queue.clear()
        self._events.publish_session_event(
            self._control.reset_session("lifecycle")
        )
        # publisher 仍处于 active 时先发安全 STOP，再停 provider 和工作线程。
        self._publish_actions([ActionCommand("stop", {}, priority=True)])
        if self.asr is not None:
            self.asr.stop()
        quiesced = self._execution is None or self._execution.stop(
            self._param("agent_deactivate_timeout_s")
        )
        if not quiesced:
            self._publish_state("deactivate_timeout")
            self._events.publish_stopped("deactivate_timeout")
            self.get_logger().error("online agent deactivate timed out")
            return TransitionCallbackReturn.FAILURE
        self._publish_state("inactive")
        self._events.publish_stopped("lifecycle_inactive")
        result = super().on_deactivate(state)
        self.get_logger().info("online agent inactive")
        return result

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        self._lifecycle_active = False
        if not self._release_resources():
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("online agent cleaned up")
        return super().on_cleanup(_state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self.shutdown()
        return super().on_shutdown(_state)

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self._lifecycle_active = False
        self._release_resources()
        self.get_logger().error("online agent recovered to unconfigured after error")
        return super().on_error(state)

    def autostart_if_enabled(self) -> None:
        """保留 ros2 run 的易用性；组合 launch 则交给 lifecycle manager。"""

        if not self._param("agent_lifecycle_autostart"):
            return
        if self.trigger_configure() != TransitionCallbackReturn.SUCCESS:
            raise RuntimeError("online agent lifecycle configure failed")
        if self.trigger_activate() != TransitionCallbackReturn.SUCCESS:
            raise RuntimeError("online agent lifecycle activate failed")

    def _release_resources(self) -> bool:
        if self._asr_endpoint is not None:
            self._asr_endpoint.close()
            self._asr_endpoint = None
        if self.asr is not None:
            self.asr.stop()
        if self._execution is not None and not self._execution.stop(
            self._param("agent_deactivate_timeout_s")
        ):
            self.get_logger().error("online Agent threads are still running")
            return False
        self._execution = None
        self.asr = None
        if self.tts is not None:
            self.tts.close()
            self.tts = None
        self.llm = None
        return True

    def _param(self, name):
        return self._agent_parameters.get(name)

    def _load_system_prompt(self) -> str:
        configured = self._param("system_prompt_path")
        if configured:
            path = Path(os.path.expanduser(configured))
        else:
            path = Path(get_package_share_directory("embodied_online_agent")) / "prompts" / "system_prompt_zh.txt"
        return path.read_text(encoding="utf-8")

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
        if self.mode == "mock":
            return (
                MockAsr(self._mock_asr_finals(), self._mock_asr_partials()),
                MockLlm(self._param("mock_token_delay_s")),
                MockTts(self._param("tts_sample_rate")),
            )
        if self.mode != "online":
            raise ValueError("mode must be 'mock' or 'online'")
        return (
            QwenRealtimeAsr(
                model=self._param("asr_model"),
                url=self._param("asr_url"),
                sample_rate=self._param("audio_sample_rate"),
                language=self._param("asr_language"),
            ),
            OpenAiCompatibleLlm(
                model=self._param("llm_model"),
                base_url=self._param("llm_base_url"),
                temperature=self._param("llm_temperature"),
            ),
            QwenRealtimeTts(
                model=self._param("tts_model"),
                voice=self._param("tts_voice"),
                url=self._param("tts_url"),
                language=self._param("tts_language"),
            ),
        )

    def _mock_asr_finals(self):
        scripted = str(self._param("mock_asr_finals") or "")
        return [item.strip() for item in scripted.split("|") if item.strip()]

    def _mock_asr_partials(self):
        scripted = str(self._param("mock_asr_partials") or "")
        if not scripted:
            return []
        # “-”保留与 finals 的位置对齐，但表示本轮没有 partial。
        return ["" if item.strip() in {"", "-"} else item.strip() for item in scripted.split("|")]

    def _on_asr_partial(self, text: str):
        if not self._lifecycle_active:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._control.transcript_stabilizer.observe_partial(text)
        self.asr_partial_pub.publish(String(data=text))

    def _on_clean_audio(self, message: UInt8MultiArray):
        if not self._lifecycle_active or self.asr is None:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self.asr.push_audio(bytes(message.data))

    def _on_silence_timeout(self, _message: Empty):
        if not self._lifecycle_active:
            return
        self._commit_asr_endpoint("silence_timeout")

    def _on_speech_started(self, _message: Empty):
        if not self._lifecycle_active:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        # speech_started 是 utterance 边界，先清掉异常遗留的上一句 partial。
        self._control.transcript_stabilizer.clear()
        self._publish_state("speech_detected")

    def _on_speech_ended(self, _message: Empty):
        if not self._lifecycle_active:
            return
        if not self._param("speech_endpoint_events_enabled"):
            return
        self._commit_asr_endpoint("speech_ended")

    def _commit_asr_endpoint(self, source: str):
        if self._asr_endpoint is not None:
            self._asr_endpoint.request(source)

    def _is_busy(self):
        return self._execution is not None and self._execution.is_busy()

    def _on_asr_final(self, text: str):
        if not self._lifecycle_active:
            return
        if self._is_busy() and not self._continuous_enabled:
            self._control.transcript_stabilizer.clear()
            self.get_logger().warning("agent is busy; suppressing overlapping ASR final")
            return
        stabilized = self._control.transcript_stabilizer.finalize(text)
        if stabilized.recovered:
            self._publish_recognition_payload(stabilized.feedback_dict())
            self.get_logger().info(
                f"recovered ASR final from partial: '{text}' -> '{stabilized.text}'"
            )
        text = stabilized.text
        self.asr_final_pub.publish(String(data=text))
        self._accept_transcript(text)

    def _on_text_input(self, message: String):
        if not self._lifecycle_active:
            self.get_logger().debug("ignored text input while Agent is inactive")
            return
        self._control.transcript_stabilizer.clear()
        self.asr_final_pub.publish(String(data=message.data))
        self._accept_transcript(message.data)

    def _on_wake_event_input(self, message: WakeEventMessage):
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
        self.action_sequencer.cancel("external_sleep")
        self._publish_actions([ActionCommand("stop", {}, priority=True)])
        session_event = self._control.voice_session.external_sleep(event.provider)
        self._publish_session_event(session_event)
        self._publish_state("sleeping")
        self.get_logger().info(
            f"external sleep event accepted from provider={event.provider}"
        )

    def _on_clear_memory(self, _message: Empty):
        self.memory.clear()
        self._user_context.clear_current()
        self.get_logger().info("conversation memory cleared")

    def _on_action_result(self, message: RobotCommandResult):
        self.action_sequencer.notify_result(
            message.command_id,
            message.success,
            message.message,
            status=message.status,
        )

    def _on_speaker_identity(self, message: SpeakerIdentityMessage):
        identity = identity_message_to_domain(
            message,
            min_confidence=float(self._param("speaker_identity_min_confidence")),
        )
        self._user_context.update_identity(identity)
        if identity.usable:
            self.get_logger().info(
                f"speaker identity accepted: speaker_id={identity.speaker_id}, confidence={identity.confidence:.3f}"
            )
        else:
            self.get_logger().debug("speaker identity ignored or unknown")

    def _accept_transcript(self, transcript: str):
        decision = self._control.accept_transcript(
            transcript,
            wake_word_required=bool(self._param("wake_word_enabled")),
        )
        self._events.publish_control_decision(decision)
        if decision.directive in {"sleep", "priority"}:
            self.action_sequencer.cancel(decision.cancel_reason)
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
        user_context = self._user_context.snapshot()
        if not self._execution.start_background_turn(
            self._run_direct_turn, command, user_context
        ):
            self.get_logger().warning("agent is busy; dropping overlapping utterance")
            return

    def _handle_memory_command(self, command: str) -> bool:
        result = self._user_context.handle_command(command)
        if result is None:
            return False
        if result.enroll_request is not None:
            self.speaker_enroll_request_pub.publish(
                enroll_request_to_message(
                    result.enroll_request, stamp=self.get_clock().now()
                )
            )
        self.response_delta_pub.publish(String(data=result.response))
        self.response_pub.publish(String(data=result.response))
        if self._execution is not None and not self._execution.start_background_turn(
            self._speak_memory_response, result.response
        ):
            self.get_logger().debug("memory response TTS skipped while Agent is busy")
        return True

    def _speak_memory_response(self, response: str) -> None:
        try:
            self._execution.raise_if_stopping()
            self.tts.synthesize([response], self._on_tts_audio)
            self._execution.raise_if_stopping()
        except AgentExecutionCancelled:
            raise
        except Exception as exc:
            self.get_logger().warning(f"memory response TTS failed: {exc}")

    def _prepare_queued_turn(self):
        self.metrics.reset()
        self.metrics.mark_asr_final()

    def _run_direct_turn(
        self, user_text: str, user_context: UserContextSnapshot
    ) -> None:
        self._prepare_queued_turn()
        self._run_turn(user_text, user_context)

    def _run_turn(self, user_text: str, user_context: UserContextSnapshot):
        self._publish_state("thinking")
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
                self.tts.synthesize(text_chunks(), self._on_tts_audio)
            except Exception as exc:
                tts_errors.append(exc)
                self.get_logger().error(f"TTS failed: {exc}")

        tts_thread = threading.Thread(target=run_tts, daemon=True)
        tts_thread.start()

        def enqueue_tts_text(text: str) -> None:
            nonlocal first_tts_text
            if not first_tts_text:
                self.metrics.mark_tts_requested()
                first_tts_text = True
                self._publish_state("speaking")
            text_queue.put(text)

        turn = StreamingTurnRuntime(
            max_chunk_chars=self._param("tts_chunk_max_chars"),
            on_first_token=self.metrics.mark_llm_first_token,
            on_speech_delta=lambda delta: self.response_delta_pub.publish(
                String(data=delta)
            ),
            on_speakable=enqueue_tts_text,
            on_protocol_error=self.get_logger().warning,
        )

        try:
            messages = self.memory.prompt_messages(
                user_context.system_prompt(self.system_prompt), user_text
            )
            self.metrics.mark_llm_requested()
            for token in self.llm.stream(messages):
                self._execution.raise_if_stopping()
                turn.feed(token)

            self._execution.raise_if_stopping()
            result = turn.finish(user_text)
            if result.action_source == "blocked" and result.model_actions:
                self.get_logger().warning("model actions blocked by semantic safety policy")
            action_report = self._publish_actions(result.actions, user_context)
            text_queue.put(None)
            tts_thread.join(timeout=35.0)
            if tts_thread.is_alive():
                raise TimeoutError("TTS worker did not stop")
            if tts_errors:
                raise tts_errors[0]

            assistant_text = result.assistant_text
            self.response_pub.publish(String(data=assistant_text))
            self.memory.append_turn(
                user_text,
                assistant_text,
                model_output=result.model_output,
            )
            self._user_context.record_interaction(
                user_context,
                user_text=user_text,
                assistant_text=assistant_text,
                # 只记录真正通过确定性/语义安全选择的动作，不能把被拦截动作写成习惯。
                actions=[action.as_dict() for action in result.actions],
                success=not action_report.failed,
            )
            self._publish_metrics()
        except AgentExecutionCancelled:
            text_queue.put(None)
            tts_thread.join(timeout=2.0)
            self.get_logger().info("agent turn cancelled by lifecycle transition")
            raise
        except Exception as exc:
            text_queue.put(None)
            tts_thread.join(timeout=2.0)
            self.get_logger().error(f"agent turn failed: {exc}")
            self._publish_state("error")
            # worker 与非连续 turn 都由 AgentExecutionRuntime 拥有，异常统一交回运行时收口。
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
        if actions:
            self._run_preparsed_turn(item.text, actions, user_context)
            return
        self._run_turn(item.text, user_context)

    def _run_preparsed_turn(
        self,
        user_text: str,
        actions,
        user_context: UserContextSnapshot,
    ):
        self._publish_state("thinking")
        summary = "，".join(action.name for action in actions)
        response = f"好的，按顺序执行：{summary}。"
        self.response_delta_pub.publish(String(data=response))
        self.response_pub.publish(String(data=response))
        report = self._publish_actions(actions, user_context)
        self._user_context.record_interaction(
            user_context,
            user_text=user_text,
            assistant_text=response,
            actions=[action.as_dict() for action in actions],
            success=not report.failed,
        )

    def _enqueue_continuous_command(self, command: str):
        user_context = self._user_context.snapshot()
        private_context = {"user_context": user_context}
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
            message = action_command_to_message(action, source="online_agent")
            self.action_candidate_pub.publish(message)
            self.get_logger().info(
                "action candidate: "
                + json.dumps(command_message_to_dict(message), ensure_ascii=False)
            )

        report = self.action_sequencer.publish(
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

    def _on_tts_audio(self, pcm16: bytes):
        self.metrics.mark_tts_first_audio()
        self.tts_audio_pub.publish(UInt8MultiArray(data=list(pcm16)))

    def _publish_metrics(self):
        snapshot = self.metrics.snapshot().as_dict()
        llm_ms = snapshot["llm_first_token_ms"]
        tts_ms = snapshot["tts_first_audio_ms"]
        snapshot["llm_target_met"] = (
            llm_ms is not None and llm_ms < self._param("llm_first_token_target_ms")
        )
        snapshot["tts_target_met"] = (
            tts_ms is not None and tts_ms < self._param("tts_first_audio_target_ms")
        )
        self.metrics_pub.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))
        self.get_logger().info(f"latency: {snapshot}")

    def _publish_state(self, state: str):
        self._events.publish_state(state)

    def _publish_session_event(self, event):
        self._events.publish_session_event(event)

    def _publish_queue_event(
        self,
        event: str,
        text: str,
        snapshot: QueueSnapshot,
        *,
        priority_stop: bool = False,
    ):
        payload = self._control.queue_event(
            event, text, snapshot, priority_stop=priority_stop
        )
        self._events.publish_queue(payload)

    def _publish_recognition_payload(self, payload: dict):
        self._events.publish_recognition(payload)

    def shutdown(self):
        if self._stopping:
            return
        self._stopping = True
        self._lifecycle_active = False
        self.action_sequencer.cancel("shutdown")
        self._release_resources()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OnlineAgentNode()
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
