import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Iterable

import rclpy
from ament_index_python.packages import get_package_share_directory
from embodied_agent_interfaces.msg import (
    RobotCommand,
    RobotCommandResult,
    SpeakerEnrollRequest as SpeakerEnrollRequestMessage,
    SpeakerIdentity as SpeakerIdentityMessage,
    WakeEvent as WakeEventMessage,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray

from .memory import ConversationMemory
from .memory_command_service import MemoryCommandService
from .user_memory import (
    LowConfidenceSpeakerError,
    SpeakerIdentity,
    UserMemoryStore,
)
from .action_sequence import SequentialActionPublisher
from .agent_control_plane import (
    AgentControlPlane,
    AgentControlPlaneConfig,
)
from .agent_parameters import declare_agent_parameters
from .command_fallback import parse_fallback_actions, should_block_model_actions
from .continuous_voice import QueueSnapshot
from .metrics import LatencyTracker
from .protocol import SentenceChunker, TaggedStreamParser
from .ros_action_transport import action_command_to_message, command_message_to_dict
from .ros_agent_events import RosAgentEventPublisher
from .ros_event_transport import wake_event_message_to_domain
from .speaker_transport import enroll_request_to_message, identity_message_to_domain
from .types import ActionCommand
from .user_preferences import apply_user_preferences
from .providers.mock import MockAsr, MockLlm, MockTts
from .providers.openai_compatible_llm import OpenAiCompatibleLlm
from .providers.qwen_asr import QwenRealtimeAsr
from .providers.qwen_tts import QwenRealtimeTts


class OnlineAgentNode(Node):
    def __init__(self):
        super().__init__("online_agent")
        # 参数在连接云端 provider 前完成声明和校验，避免错误配置到运行中才暴露。
        # 不使用 `_parameters`：它是 rclpy.Node 的内部参数表，覆盖后会让参数服务崩溃。
        self._agent_parameters = declare_agent_parameters(self, "online")
        self.mode = self._param("mode")
        self._state_lock = threading.Lock()
        self._busy = False
        self._stopping = False
        self._last_asr_commit_monotonic = 0.0
        self._continuous_enabled = bool(self._param("continuous_control_enabled"))
        self._command_worker_thread = None
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
        self._current_speaker = SpeakerIdentity()
        self.user_memory = UserMemoryStore(
            self._param("user_memory_dir"),
            max_recent=int(self._param("user_memory_max_recent")),
            retention_s=float(self._param("user_memory_retention_days")) * 86400.0,
        )
        self._memory_commands = MemoryCommandService(self.user_memory)
        self.system_prompt = self._load_system_prompt()

        self.asr_partial_pub = self.create_publisher(String, "/agent/asr_partial", 10)
        self.asr_final_pub = self.create_publisher(String, "/agent/asr_final", 10)
        self.response_pub = self.create_publisher(String, "/agent/response_text", 10)
        self.response_delta_pub = self.create_publisher(String, "/agent/response_delta", 10)
        self.action_candidate_pub = self.create_publisher(
            RobotCommand, "/agent/action_candidate", 10
        )
        self._events = RosAgentEventPublisher(self)
        self.speaker_enroll_request_pub = self.create_publisher(
            SpeakerEnrollRequestMessage, "/agent/speaker_enroll_request", 10
        )
        self.metrics_pub = self.create_publisher(String, "/agent/metrics", 10)
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
        if self._continuous_enabled:
            self._command_worker_thread = threading.Thread(
                target=self._run_command_worker, daemon=True
            )
            self._command_worker_thread.start()

        audio_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.tts_audio_pub = self.create_publisher(
            UInt8MultiArray, "/audio/tts_pcm", audio_qos
        )
        self.asr, self.llm, self.tts = self._create_providers()
        if self.mode == "online" and self._param("online_warmup_enabled"):
            if hasattr(self.tts, "connect"):
                self.tts.connect()
            if hasattr(self.llm, "warmup"):
                self.llm.warmup()
        self.audio_subscription = None
        self.silence_subscription = None

        if self._param("microphone_enabled"):
            self.asr.start(self._on_asr_partial, self._on_asr_final)
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

        self._publish_state("listening")
        self._events.publish_ready(
            f"provider_mode={self.mode};microphone={self._param('microphone_enabled')}"
        )
        self.get_logger().info(
            f"online agent ready: mode={self.mode}, microphone={self._param('microphone_enabled')}"
        )

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
        if self._is_busy() and not self._continuous_enabled:
            return
        self._control.transcript_stabilizer.observe_partial(text)
        self.asr_partial_pub.publish(String(data=text))

    def _on_clean_audio(self, message: UInt8MultiArray):
        if self._is_busy() and not self._continuous_enabled:
            return
        self.asr.push_audio(bytes(message.data))

    def _on_silence_timeout(self, _message: Empty):
        self._commit_asr_endpoint("silence_timeout")

    def _on_speech_started(self, _message: Empty):
        if self._is_busy() and not self._continuous_enabled:
            return
        # speech_started 是 utterance 边界，先清掉异常遗留的上一句 partial。
        self._control.transcript_stabilizer.clear()
        self._publish_state("speech_detected")

    def _on_speech_ended(self, _message: Empty):
        if not self._param("speech_endpoint_events_enabled"):
            return
        self._commit_asr_endpoint("speech_ended")

    def _commit_asr_endpoint(self, source: str):
        if self._is_busy() and not self._continuous_enabled:
            return
        now = time.monotonic()
        if now - self._last_asr_commit_monotonic < 0.05:
            self.get_logger().debug(f"ignored duplicate ASR commit from {source}")
            return
        self._last_asr_commit_monotonic = now
        delay_ms = max(0, int(self._param("asr_commit_delay_ms")))
        self._publish_asr_endpoint_feedback(source, delay_ms)
        if delay_ms <= 0:
            self._perform_asr_commit(source)
            return
        # VAD 刚判定 speech_ended 时，云端 ASR 的 partial 可能还没稳定。
        # 延迟少量时间再 commit，可以换取更完整的“左转90度/前进一秒”尾部识别。
        timer = threading.Timer(
            delay_ms / 1000.0, self._perform_asr_commit, args=(source,)
        )
        timer.daemon = True
        timer.start()

    def _perform_asr_commit(self, source: str):
        if self._stopping:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._publish_asr_commit_feedback(source)
        self.asr.commit()

    def _is_busy(self):
        with self._state_lock:
            return self._busy

    def _on_asr_final(self, text: str):
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
        self._control.transcript_stabilizer.clear()
        self.asr_final_pub.publish(String(data=message.data))
        self._accept_transcript(message.data)

    def _on_wake_event_input(self, message: WakeEventMessage):
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
        self._try_user_memory_write(self.user_memory.clear, self._current_speaker)
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
        self._current_speaker = identity
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
        with self._state_lock:
            if self._busy:
                self.get_logger().warning("agent is busy; dropping overlapping utterance")
                return
            self._busy = True
        self.metrics.reset()
        self.metrics.mark_asr_final()
        threading.Thread(target=self._run_turn, args=(command,), daemon=True).start()

    def _handle_memory_command(self, command: str) -> bool:
        result = self._memory_commands.handle(command, self._current_speaker)
        if result is None:
            return False
        self._current_speaker = result.identity
        if result.enroll_request is not None:
            self.speaker_enroll_request_pub.publish(
                enroll_request_to_message(
                    result.enroll_request, stamp=self.get_clock().now()
                )
            )
        self.response_delta_pub.publish(String(data=result.response))
        self.response_pub.publish(String(data=result.response))
        threading.Thread(
            target=self._speak_memory_response, args=(result.response,), daemon=True
        ).start()
        return True

    def _speak_memory_response(self, response: str) -> None:
        try:
            self.tts.synthesize([response], self._on_tts_audio)
        except Exception as exc:
            self.get_logger().warning(f"memory response TTS failed: {exc}")

    def _run_command_worker(self):
        while not self._stopping:
            try:
                item = self._control.command_queue.get(
                    timeout=0.1,
                    on_stale=self._publish_stale_command_event,
                )
            except queue.Empty:
                continue
            try:
                with self._state_lock:
                    self._busy = True
                # started/finished 事件是现场演示的“可观测性锚点”，monitor 依赖它判断是否卡住。
                self._publish_execution_event(
                    self._control.execution_started(item)
                )
                self.metrics.reset()
                self.metrics.mark_asr_final()
                self._run_queued_turn(item)
                self._publish_execution_event(
                    self._control.execution_finished(
                        item, success=True, reason="completed"
                    )
                )
            except Exception as exc:
                self._publish_execution_event(
                    self._control.execution_finished(
                        item, success=False, reason=str(exc)
                    )
                )
                raise
            finally:
                self._control.command_queue.task_done()

    def _publish_stale_command_event(self, item):
        self._publish_command_queue_payload(
            self._control.queue_expired(item)
        )

    def _run_turn(self, user_text: str):
        self._publish_state("thinking")
        text_queue = queue.Queue()
        tts_errors = []
        first_tts_text = False

        def text_chunks() -> Iterable[str]:
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
        parser = TaggedStreamParser()
        chunker = SentenceChunker(self._param("tts_chunk_max_chars"))
        spoken_parts = []
        model_actions = []
        raw_output_parts = []
        protocol_errors = []
        first_token = True

        try:
            messages = self.memory.prompt_messages(
                self._system_prompt_with_user_memory(), user_text
            )
            self.metrics.mark_llm_requested()
            for token in self.llm.stream(messages):
                raw_output_parts.append(token)
                if first_token:
                    self.metrics.mark_llm_first_token()
                    first_token = False
                events = parser.feed(token)
                for delta in events.speech:
                    spoken_parts.append(delta)
                    self.response_delta_pub.publish(String(data=delta))
                    for speakable in chunker.feed(delta):
                        if not first_tts_text:
                            self.metrics.mark_tts_requested()
                            first_tts_text = True
                            self._publish_state("speaking")
                        text_queue.put(speakable)
                model_actions.extend(events.actions)
                protocol_errors.extend(events.errors)
                for error in events.errors:
                    self.get_logger().warning(error)

            final_events = parser.finish()
            model_actions.extend(final_events.actions)
            protocol_errors.extend(final_events.errors)
            fallback_actions = parse_fallback_actions(user_text)
            if fallback_actions:
                self._publish_actions(fallback_actions)
            elif should_block_model_actions(user_text):
                if model_actions:
                    self.get_logger().warning("model actions blocked by semantic safety policy")
            else:
                self._publish_actions(model_actions)
            for error in final_events.errors:
                self.get_logger().warning(error)
            if not spoken_parts:
                fallback = "抱歉，回复格式解析失败，请再说一次。"
                spoken_parts.append(fallback)
                self.response_delta_pub.publish(String(data=fallback))
                for speakable in chunker.feed(fallback):
                    if not first_tts_text:
                        self.metrics.mark_tts_requested()
                        first_tts_text = True
                        self._publish_state("speaking")
                    text_queue.put(speakable)
            for speakable in chunker.finish():
                if not first_tts_text:
                    self.metrics.mark_tts_requested()
                    first_tts_text = True
                text_queue.put(speakable)
            text_queue.put(None)
            tts_thread.join(timeout=35.0)
            if tts_thread.is_alive():
                raise TimeoutError("TTS worker did not stop")
            if tts_errors:
                raise tts_errors[0]

            assistant_text = "".join(spoken_parts).strip()
            self.response_pub.publish(String(data=assistant_text))
            raw_output = "".join(raw_output_parts).strip()
            cached_output = (
                raw_output
                if raw_output and not protocol_errors
                else f"<speech>{assistant_text}</speech>"
            )
            self.memory.append_turn(
                user_text,
                assistant_text,
                model_output=cached_output,
            )
            self._record_user_interaction(
                self._current_speaker,
                user_text=user_text,
                assistant_text=assistant_text,
                actions=[action.as_dict() for action in (fallback_actions or model_actions)],
                success=True,
            )
            self._publish_metrics()
        except Exception as exc:
            text_queue.put(None)
            self.get_logger().error(f"agent turn failed: {exc}")
            self._publish_state("error")
        finally:
            with self._state_lock:
                self._busy = False
            if not self._stopping:
                self._publish_state("listening")

    def _run_queued_turn(self, item):
        context = item.context if isinstance(item.context, dict) else {}
        actions = self._actions_from_context(context)
        if actions:
            self._run_preparsed_turn(item.text, actions, context)
            return
        self._run_turn(item.text)

    def _run_preparsed_turn(self, user_text: str, actions, context: dict):
        self._publish_state("thinking")
        summary = "，".join(action.name for action in actions)
        response = f"好的，按顺序执行：{summary}。"
        self.response_delta_pub.publish(String(data=response))
        self.response_pub.publish(String(data=response))
        report = self._publish_actions(actions)
        self._record_user_interaction(
            self._current_speaker,
            user_text=user_text,
            assistant_text=response,
            actions=[action.as_dict() for action in actions],
            success=not report.failed,
        )

    @staticmethod
    def _actions_from_context(context: dict):
        raw_actions = context.get("preparsed_actions") or []
        actions = []
        for raw in raw_actions:
            if isinstance(raw, dict) and isinstance(raw.get("name"), str):
                actions.append(
                    ActionCommand(raw["name"], dict(raw.get("arguments") or {}))
                )
        return actions

    def _try_user_memory_write(self, operation, *args, **kwargs):
        try:
            return operation(*args, **kwargs)
        except LowConfidenceSpeakerError as exc:
            self.get_logger().debug(f"skipped user memory write: {exc}")
            return None

    def _record_user_interaction(self, identity, **kwargs):
        return self._try_user_memory_write(
            self.user_memory.record_interaction,
            identity,
            **kwargs,
        )

    def _enqueue_continuous_command(self, command: str):
        nlu_result = self._control.command_nlu.parse(command)
        if nlu_result.accepted:
            batch_id = self._control.next_nlu_batch_id()
            self._publish_nlu_feedback(command, nlu_result, batch_id)
            for index, parsed in enumerate(nlu_result.commands, start=1):
                metadata = {
                    "batch_id": batch_id,
                    "batch_index": index,
                    "batch_size": len(nlu_result.commands),
                    "source_text": command,
                    "nlu_intent": parsed.intent,
                    "nlu_confidence": round(parsed.confidence, 3),
                }
                context = {
                    **metadata,
                    "preparsed_actions": [action.as_dict() for action in parsed.actions],
                }
                snapshot = self._control.command_queue.put(
                    parsed.span_text, context=context, metadata=metadata
                )
                self._publish_queue_event("enqueue", parsed.span_text, snapshot)
                if not snapshot.accepted:
                    self._publish_queue_rejected_recognition(parsed.span_text, snapshot)
                    self._publish_state("queue_full")
                    return
            self._publish_state("queued")
            return

        if nlu_result.retry_prompt:
            # 控制命令缺少必需槽位时不交给云端 LLM 猜测，避免错误动作并缩短重试路径。
            self._publish_nlu_retry(command, nlu_result)
            self._publish_state("retry_listening")
            return

        snapshot = self._control.command_queue.put(command)
        self._publish_queue_event("enqueue", command, snapshot)
        if snapshot.accepted:
            self.get_logger().info(
                f"continuous command queued: size={snapshot.size}, text={command}"
            )
            self._publish_state("queued")
        else:
            self.get_logger().warning(
                f"continuous command queue rejected input: {snapshot.reason}"
            )
            self._publish_queue_rejected_recognition(command, snapshot)
            self._publish_state("queue_full")

    def _publish_nlu_retry(self, command: str, nlu_result):
        payload = {
            "status": "retry",
            "reason": nlu_result.reason,
            "transcript": command,
            "prompt": nlu_result.retry_prompt,
        }
        self._publish_recognition_payload(payload)
        self.get_logger().warning(
            f"incomplete voice command: reason={nlu_result.reason}, text={command}"
        )

    def _publish_nlu_feedback(self, command: str, nlu_result, batch_id: str):
        self._events.publish_nlu(
            command, nlu_result, batch_id, source="online"
        )

    def _publish_actions(self, actions):
        action_list = apply_user_preferences(actions, self._current_user_preferences())

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

    def _current_user_preferences(self) -> dict:
        # 偏好只有在说话人身份可靠时才参与动作策略；未知用户继续使用系统默认参数。
        if not self._current_speaker.usable:
            return {}
        return dict(self.user_memory.profile(self._current_speaker).preferences)

    def _should_wait_for_action_results(self, action_list):
        if len(action_list) > 1:
            return True
        # 连续控制的 worker 必须等单动作 result 后再消费下一条命令。
        # 否则真人连续说“前进、左转、后退”时，后一个 ROS Action goal 会抢占前一个，
        # 表面看像“识别到了但机器人没完整执行”。急停/退出控制通常在 ROS 回调线程里发布，
        # 不能在这里同步等待，否则单线程 executor 无法处理 /robot/action_result 回调。
        return (
            bool(action_list)
            and self._continuous_enabled
            and self._command_worker_thread is not None
            and threading.current_thread() is self._command_worker_thread
        )

    def _system_prompt_with_user_memory(self) -> str:
        summary = self.user_memory.prompt_summary(self._current_speaker)
        if not summary:
            return self.system_prompt
        return (
            self.system_prompt
            + "\n\n[用户画像记忆]\n"
            + summary
            + "\n请仅把用户画像作为偏好参考，所有动作仍必须遵守输出格式和安全限幅。"
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
        self._publish_command_queue_payload(payload)

    def _publish_command_queue_payload(self, payload):
        self._events.publish_queue(payload)

    def _publish_execution_event(self, event):
        self._events.publish_execution(event)

    def _publish_ignored_recognition(self, transcript: str, reason: str):
        self._events.publish_ignored(transcript, reason)

    def _publish_queue_rejected_recognition(
        self, transcript: str, snapshot: QueueSnapshot
    ):
        self._events.publish_queue_rejected(transcript, snapshot)

    def _publish_asr_endpoint_feedback(self, source: str, delay_ms: int):
        self._events.publish_asr_endpoint(source, delay_ms)

    def _publish_asr_commit_feedback(self, source: str):
        self._events.publish_asr_commit(source)

    def _publish_session_timeout_feedback(self, transcript: str):
        self._events.publish_session_timeout(transcript)

    def _publish_recognition_payload(self, payload: dict):
        self._events.publish_recognition(payload)

    def shutdown(self):
        self._stopping = True
        self.asr.stop()
        self.tts.close()
        if self._command_worker_thread:
            self._command_worker_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OnlineAgentNode()
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
