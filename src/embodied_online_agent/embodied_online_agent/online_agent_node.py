import os
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from embodied_agent_interfaces.msg import (
    RobotCommandResult,
    SpeakerIdentity as SpeakerIdentityMessage,
    WakeEvent as WakeEventMessage,
)
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import Empty, String, UInt8MultiArray

from embodied_agent_core.memory import ConversationMemory
from embodied_agent_core.user_memory import UserMemoryStore
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
from embodied_agent_core.metrics import LatencyTracker
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
from .providers.mock import MockAsr, MockLlm, MockTts
from .providers.openai_compatible_llm import OpenAiCompatibleLlm
from .providers.qwen_asr import QwenRealtimeAsr
from .providers.qwen_tts import QwenRealtimeTts
from .online_turn_runtime import OnlineStreamingTurnRuntime


class OnlineAgentNode(LifecycleNode):
    def __init__(self):
        super().__init__("online_agent")
        # 参数在连接云端 provider 前完成声明和校验，避免错误配置到运行中才暴露。
        # 不使用 `_parameters`：它是 rclpy.Node 的内部参数表，覆盖后会让参数服务崩溃。
        self._agent_parameters = declare_agent_parameters(self, "online")
        self.mode = self._param("mode")
        self._continuous_enabled = bool(self._param("continuous_control_enabled"))
        self.metrics = LatencyTracker()
        self._control = AgentControlPlane(
            AgentControlPlaneConfig.from_parameters(
                "online", self._param, self._command_normalization_path()
            )
        )
        self.action_sequencer = SequentialActionPublisher(
            self._param("action_sequence_wait_timeout_s"),
            self._param("long_action_result_timeout_s"),
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

        self._ros_io = AgentRosIo(
            self,
            AgentRosCallbacks(
                text_input=self._on_text_input,
                wake_event_input=self._on_wake_event_input,
                speaker_identity=self._on_speaker_identity,
                clear_memory=self._on_clear_memory,
                action_result=self._on_action_result,
                clean_audio=self._on_clean_audio,
                silence_timeout=self._on_silence_timeout,
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
            action_sequencer=self.action_sequencer,
            ros_io=self._ros_io,
            events=self._events,
            publish_priority_stop=self._publish_priority_stop,
            deactivate_timeout_s=self._param("agent_deactivate_timeout_s"),
        )
        self._application = AgentApplicationRuntime(
            source="online",
            control=self._control,
            lifecycle_runtime=self._runtime,
            action_sequencer=self.action_sequencer,
            conversation_memory=self.memory,
            user_context=self._user_context,
            events=self._events,
            callbacks=AgentApplicationCallbacks(
                run_model_turn=lambda text, _context, user: self._run_turn(text, user),
                speak_memory_response=self._speak_memory_response,
                publish_action_candidate=self._publish_action_candidate,
                publish_enroll_request=self._publish_enroll_request,
                publish_response_delta=self._ros_io.publish_response_delta,
                publish_response=self._ros_io.publish_response,
                prepare_turn_context=self._prepare_turn_context,
            ),
            logger=self.get_logger(),
            wake_word_required=bool(self._param("wake_word_enabled")),
            continuous_enabled=self._continuous_enabled,
        )
        self.asr = None
        self.llm = None
        self.tts = None
        self._turn_runner = None

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
            self._runtime.bind(execution=execution, endpoint=endpoint)
            self._turn_runner = OnlineStreamingTurnRuntime(
                llm=self.llm,
                tts=self.tts,
                memory=self.memory,
                user_context=self._user_context,
                system_prompt=self.system_prompt,
                metrics=self.metrics,
                ros_io=self._ros_io,
                events=self._events,
                lifecycle_runtime=self._runtime,
                publish_actions=self._application.publish_actions,
                publish_metrics=self._publish_metrics,
                param=self._param,
                logger=self.get_logger(),
            )
            self.get_logger().info("online agent configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"online agent configure failed: {exc}")
            self._release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if not self._runtime.configured or self.asr is None:
            self.get_logger().error("online agent activate requested before configure")
            return TransitionCallbackReturn.FAILURE
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        try:
            self._runtime.activate(self._start_input)
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
        quiescence = self._runtime.deactivate(self._stop_input)
        if not quiescence.quiesced:
            self.get_logger().error(
                "online agent deactivate timed out: "
                f"input={quiescence.input_quiesced}, "
                f"execution={quiescence.execution_quiesced}, "
                f"errors={quiescence.errors}"
            )
            return TransitionCallbackReturn.FAILURE
        result = super().on_deactivate(state)
        self.get_logger().info("online agent inactive")
        return result

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        if not self._release_resources():
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("online agent cleaned up")
        return super().on_cleanup(_state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self.shutdown()
        return super().on_shutdown(_state)

    def on_error(self, state: State) -> TransitionCallbackReturn:
        if self._runtime.active:
            self._runtime.deactivate(self._stop_input)
        else:
            self._runtime.mark_error_inactive()
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
        quiescence = self._runtime.release(self._stop_input)
        if not quiescence.quiesced:
            self.get_logger().error(
                "online Agent resources are still running: "
                f"input={quiescence.input_quiesced}, "
                f"execution={quiescence.execution_quiesced}, "
                f"errors={quiescence.errors}"
            )
            return False
        self.asr = None
        self._turn_runner = None
        if self.tts is not None:
            self.tts.close()
            self.tts = None
        self.llm = None
        return True

    def _start_input(self) -> None:
        if self._param("microphone_enabled"):
            self.asr.start(self._on_asr_partial, self._on_asr_final)

    def _stop_input(self) -> bool:
        if self.asr is not None:
            self.asr.stop()
        return True

    def _publish_priority_stop(self) -> None:
        self._application.publish_actions(
            [ActionCommand("stop", {}, priority=True)]
        )

    def _param(self, name):
        return self._agent_parameters.get(name)

    def _load_system_prompt(self) -> str:
        configured = self._param("system_prompt_path")
        if configured:
            path = Path(os.path.expanduser(configured))
        else:
            path = (
                Path(get_package_share_directory("embodied_agent_core"))
                / "prompts"
                / "system_prompt_zh.txt"
            )
        return path.read_text(encoding="utf-8")

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
        if not self._runtime.active:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._control.transcript_stabilizer.observe_partial(text)
        self._ros_io.publish_asr_partial(text)

    def _on_clean_audio(self, message: UInt8MultiArray):
        if not self._runtime.active or self.asr is None:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self.asr.push_audio(bytes(message.data))

    def _on_silence_timeout(self, _message: Empty):
        if not self._runtime.active:
            return
        # 主端点启用时忽略兼容 silence_timeout，避免二次切句；关闭时仍作回退。
        if self._param("speech_endpoint_events_enabled"):
            return
        self._commit_asr_endpoint("silence_timeout")

    def _on_speech_started(self, _message: Empty):
        if not self._runtime.active:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        # 短停顿后恢复说话时取消延迟提交，避免长句被提前 final。
        if self._runtime.endpoint is not None:
            self._runtime.endpoint.resume_utterance()
        # speech_started 是 utterance 边界，再清掉异常遗留的上一句 partial。
        self._control.transcript_stabilizer.clear()
        self._publish_state("speech_detected")

    def _on_speech_ended(self, _message: Empty):
        if not self._runtime.active:
            return
        if not self._param("speech_endpoint_events_enabled"):
            return
        self._commit_asr_endpoint("speech_ended")

    def _commit_asr_endpoint(self, source: str):
        if self._runtime.endpoint is not None:
            self._runtime.endpoint.request(source)

    def _is_busy(self):
        return (
            self._runtime.execution is not None
            and self._runtime.execution.is_busy()
        )

    def _on_asr_final(self, text: str):
        if not self._runtime.active:
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
        self._ros_io.publish_asr_final(text)
        self._application.accept_transcript(text)

    def _on_text_input(self, message: String):
        if not self._runtime.active:
            self.get_logger().debug("ignored text input while Agent is inactive")
            return
        self._control.transcript_stabilizer.clear()
        self._ros_io.publish_asr_final(message.data)
        self._application.accept_transcript(message.data)

    def _on_wake_event_input(self, message: WakeEventMessage):
        if not self._runtime.active:
            return
        try:
            event = wake_event_message_to_domain(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._application.handle_wake_event(event)

    def _on_clear_memory(self, _message: Empty):
        self._application.clear_memory()

    def _on_action_result(self, message: RobotCommandResult):
        self._application.notify_action_result(
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
        self._application.update_speaker_identity(identity)

    def _publish_enroll_request(self, request) -> None:
        self._ros_io.publish_speaker_enroll_request(
            enroll_request_to_message(request, stamp=self.get_clock().now())
        )

    def _speak_memory_response(self, response: str) -> None:
        try:
            self._runtime.execution.raise_if_stopping()
            # 记忆回复不是 LLM turn，不应借用 OnlineStreamingTurnRuntime 的私有
            # metrics callback；直接复用 ROS I/O Facade，避免节点重构后保留悬空方法。
            self.tts.synthesize([response], self._ros_io.publish_tts_audio)
            self._runtime.execution.raise_if_stopping()
        except AgentExecutionCancelled:
            raise
        except Exception as exc:
            self.get_logger().warning(f"memory response TTS failed: {exc}")

    def _prepare_turn_context(self, _context):
        self.metrics.reset()
        self.metrics.mark_asr_final()

    def _run_turn(self, user_text: str, user_context: UserContextSnapshot):
        if self._turn_runner is None:
            raise RuntimeError("online turn runtime is not configured")
        self._turn_runner.run(user_text, user_context)

    def _publish_action_candidate(self, action: ActionCommand) -> None:
        """应用层只发布领域动作；这里是 online → ROS typed msg 的唯一 seam。"""

        message = action_command_to_message(action, source="online_agent")
        self._ros_io.publish_action_candidate(message)
        self.get_logger().info(
            f"action candidate: type={message.action_type}, "
            f"command_id={message.command_id}, priority={message.priority}"
        )

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
        self._ros_io.publish_metrics(
            agent_turn_metrics_to_message(
                "online", snapshot, stamp=self.get_clock().now().to_msg()
            )
        )
        self.get_logger().info(f"latency: {snapshot}")

    def _publish_state(self, state: str):
        self._events.publish_state(state)

    def _publish_recognition_payload(self, payload: dict):
        self._events.publish_recognition(payload)

    def shutdown(self):
        if not self._runtime.begin_shutdown():
            return
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
