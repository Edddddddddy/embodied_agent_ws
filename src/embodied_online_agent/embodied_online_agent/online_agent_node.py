import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Iterable

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray

from .memory import ConversationMemory
from .user_memory import (
    LowConfidenceSpeakerError,
    SpeakerIdentity,
    UserMemoryStore,
    parse_memory_command,
)
from .action_sequence import SequentialActionPublisher
from .command_completion import CommandCompleter
from .command_fallback import parse_fallback_actions, should_block_model_actions
from .command_nlu import CommandNLU
from .continuous_voice import ContinuousCommandQueue, ContinuousVoiceSession
from .continuous_voice import CommandExecutionTracker, QueueSnapshot
from .command_normalizer import CommandNormalizer
from .metrics import LatencyTracker
from .navigation_phrases import is_navigation_cancel
from .protocol import SentenceChunker, TaggedStreamParser
from .recognition_retry import RecognitionRetryTracker
from .types import ActionCommand
from .user_preferences import apply_user_preferences
from .wake_event_input import parse_external_wake_event
from .providers.mock import MockAsr, MockLlm, MockTts
from .providers.openai_compatible_llm import OpenAiCompatibleLlm
from .providers.qwen_asr import QwenRealtimeAsr
from .providers.qwen_tts import QwenRealtimeTts
from .wakeword import WakeWordGate


class OnlineAgentNode(Node):
    def __init__(self):
        super().__init__("online_agent")
        self._declare_parameters()
        self.mode = self._param("mode")
        self._state_lock = threading.Lock()
        self._busy = False
        self._stopping = False
        self._last_asr_commit_monotonic = 0.0
        self._continuous_enabled = bool(self._param("continuous_control_enabled"))
        self._nlu_batch_sequence = 0
        self._command_queue = ContinuousCommandQueue(
            int(self._param("continuous_command_queue_size")),
            max_age_s=float(self._param("continuous_command_max_age_s")),
        )
        self._command_tracker = CommandExecutionTracker("online")
        self._command_worker_thread = None
        self.metrics = LatencyTracker()
        self.wake_gate = WakeWordGate(
            self._param("wake_words"),
            aliases=self._param("wake_word_aliases"),
            enabled=self._param("wake_word_enabled"),
            active_timeout_s=(
                self._param("voice_session_timeout_s")
                if self._continuous_enabled
                else self._param("wake_active_timeout_s")
            ),
        )
        self.voice_session = ContinuousVoiceSession(
            self.wake_gate,
            enabled=self._continuous_enabled,
            duplicate_window_s=float(self._param("continuous_duplicate_window_s")),
        )
        self.retry_tracker = RecognitionRetryTracker(
            self._param("recognition_max_retries")
        )
        self.command_normalizer = CommandNormalizer(
            fuzzy_threshold=float(self._param("command_normalization_fuzzy_threshold")),
            rules_path=self._command_normalization_path(),
        )
        self.command_completer = CommandCompleter(
            enabled=bool(self._param("command_completion_enabled"))
        )
        self.command_nlu = CommandNLU(
            enabled=bool(self._param("command_nlu_enabled")),
            min_confidence=float(self._param("command_nlu_min_confidence")),
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
        )
        self.system_prompt = self._load_system_prompt()

        self.asr_partial_pub = self.create_publisher(String, "/agent/asr_partial", 10)
        self.asr_final_pub = self.create_publisher(String, "/agent/asr_final", 10)
        self.response_pub = self.create_publisher(String, "/agent/response_text", 10)
        self.response_delta_pub = self.create_publisher(String, "/agent/response_delta", 10)
        self.action_candidate_pub = self.create_publisher(
            String, "/agent/action_candidate", 10
        )
        self.state_pub = self.create_publisher(String, "/agent/state", 10)
        self.wake_event_pub = self.create_publisher(String, "/agent/wake_event", 10)
        self.session_state_pub = self.create_publisher(String, "/agent/session_state", 10)
        self.command_queue_pub = self.create_publisher(String, "/agent/command_queue", 10)
        self.command_execution_pub = self.create_publisher(
            String, "/agent/command_execution", 10
        )
        self.recognition_feedback_pub = self.create_publisher(
            String, "/agent/recognition_feedback", 10
        )
        self.speaker_enroll_request_pub = self.create_publisher(
            String, "/agent/speaker_enroll_request", 10
        )
        self.metrics_pub = self.create_publisher(String, "/agent/metrics", 10)
        self.create_subscription(String, "/agent/text_input", self._on_text_input, 10)
        if self._param("external_wake_event_enabled"):
            self.create_subscription(
                String, "/agent/wake_event_input", self._on_wake_event_input, 10
            )
        self.create_subscription(
            String, "/agent/speaker_identity", self._on_speaker_identity, 10
        )
        self.create_subscription(Empty, "/agent/clear_memory", self._on_clear_memory, 10)
        self.create_subscription(String, "/robot/action_result", self._on_action_result, 10)
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
        self.get_logger().info(
            f"online agent ready: mode={self.mode}, microphone={self._param('microphone_enabled')}"
        )

    def _declare_parameters(self):
        defaults = {
            "mode": "mock",
            "microphone_enabled": False,
            "audio_sample_rate": 16000,
            "tts_sample_rate": 24000,
            "wake_word_enabled": True,
            "wake_words": ["小智", "你好小智"],
            "wake_word_aliases": ["小志", "小治", "晓智", "晓志"],
            "wake_active_timeout_s": 10.0,
            "recognition_max_retries": 3,
            "command_normalization_enabled": True,
            "command_normalization_feedback_enabled": True,
            "command_normalization_fuzzy_threshold": 0.82,
            "command_normalization_path": "",
            "command_completion_enabled": True,
            "command_nlu_enabled": True,
            "command_nlu_min_confidence": 0.18,
            "memory_path": "~/.ros/embodied_agent/memory.json",
            "memory_max_turns": 10,
            "user_memory_dir": "~/.ros/embodied_agent/users",
            "user_memory_max_recent": 8,
            "speaker_identity_min_confidence": 0.55,
            "system_prompt_path": "",
            "llm_model": "qwen-plus",
            "llm_base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
            "llm_temperature": 0.0,
            "asr_model": "qwen3-asr-flash-realtime",
            "asr_url": "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
            "asr_language": "zh",
            "tts_model": "qwen3-tts-flash-realtime",
            "tts_voice": "Cherry",
            "tts_url": "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
            "tts_language": "Chinese",
            "tts_chunk_max_chars": 32,
            "llm_first_token_target_ms": 1000.0,
            "tts_first_audio_target_ms": 300.0,
            "mock_token_delay_s": 0.0,
            "mock_asr_finals": "",
            "online_warmup_enabled": True,
            "action_sequence_wait_timeout_s": 12.0,
            "continuous_control_enabled": False,
            "voice_session_timeout_s": 60.0,
            "continuous_command_queue_size": 8,
            "continuous_command_max_age_s": 30.0,
            "continuous_duplicate_window_s": 1.2,
            "speech_endpoint_events_enabled": True,
            "asr_commit_delay_ms": 0,
            "external_wake_event_enabled": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _param(self, name):
        return self.get_parameter(name).value

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
                MockAsr(self._mock_asr_finals()),
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

    def _on_asr_partial(self, text: str):
        if self._is_busy() and not self._continuous_enabled:
            return
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
            self.get_logger().warning("agent is busy; suppressing overlapping ASR final")
            return
        self.asr_final_pub.publish(String(data=text))
        self._accept_transcript(text)

    def _on_text_input(self, message: String):
        self.asr_final_pub.publish(String(data=message.data))
        self._accept_transcript(message.data)

    def _on_wake_event_input(self, message: String):
        event = parse_external_wake_event(message.data)
        if event is None:
            self.get_logger().warning(
                f"ignored invalid external wake event payload: {message.data!r}"
            )
            return
        if event.kind == "wake":
            session_event = self.voice_session.external_wake(
                event.provider, event.transcript
            )
            self._publish_session_event(session_event)
            self._publish_state("session_awake")
            self.get_logger().info(
                f"external wake event accepted from provider={event.provider}"
            )
            return

        dropped = self._command_queue.clear()
        self._publish_queue_event(
            "clear", "", QueueSnapshot(True, self._command_queue.size(), dropped)
        )
        self.action_sequencer.cancel("external_sleep")
        self._publish_actions([ActionCommand("stop", {})])
        session_event = self.voice_session.external_sleep(event.provider)
        self._publish_session_event(session_event)
        self._publish_state("sleeping")
        self.get_logger().info(
            f"external sleep event accepted from provider={event.provider}"
        )

    def _on_clear_memory(self, _message: Empty):
        self.memory.clear()
        self._try_user_memory_write(self.user_memory.clear, self._current_speaker)
        self.get_logger().info("conversation memory cleared")

    def _on_action_result(self, message: String):
        self.action_sequencer.notify_result(message.data)

    def _on_speaker_identity(self, message: String):
        identity = SpeakerIdentity.from_json(
            message.data,
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
        transcript = self._normalize_transcript(transcript)
        decision = self.voice_session.accept(transcript)
        self._publish_session_event(decision.event)
        if not decision.accepted or decision.command is None:
            if decision.reason == "session_sleep":
                dropped = self._command_queue.clear()
                self._publish_queue_event(
                    "clear", "", QueueSnapshot(True, self._command_queue.size(), dropped)
                )
                self.action_sequencer.cancel("session_sleep")
                self._publish_actions([ActionCommand("stop", {})])
                self._publish_state("sleeping")
                self.get_logger().info("continuous voice session sleeping")
            elif decision.reason == "session_awake":
                self._publish_state("session_awake")
                self.get_logger().info("continuous voice session awake")
            elif decision.reason in {"filler", "duplicate_command"}:
                self._publish_ignored_recognition(transcript, decision.reason)
                self._publish_state("listening")
            elif decision.reason == "session_timeout":
                feedback = self.retry_tracker.failed(transcript)
                self.recognition_feedback_pub.publish(String(data=feedback.to_json()))
                self._publish_session_timeout_feedback(transcript)
                self.get_logger().warning(
                    "voice session timed out; waiting for a new wake word"
                )
                self._publish_state("retry_listening")
            elif self._param("wake_word_enabled") and not self.wake_gate.active:
                feedback = self.retry_tracker.failed(transcript)
                self.recognition_feedback_pub.publish(String(data=feedback.to_json()))
                self.get_logger().warning(
                    f"wake word not detected ({feedback.attempt}/{feedback.max_attempts}); retrying"
                )
                self._publish_state("retry_listening")
            return
        self.retry_tracker.succeeded()
        command = decision.command
        if not decision.priority_stop:
            # 正常控制命令允许做“短命令补全”；停下/急停保持原样，保证安全指令最快抢占。
            command = self._complete_command(command)
        if self._handle_memory_command(command):
            self._publish_state("listening")
            return
        if self._continuous_enabled:
            priority_navigation_cancel = is_navigation_cancel(command)
            if decision.priority_stop or priority_navigation_cancel:
                cancel_reason = (
                    "priority_stop"
                    if decision.priority_stop
                    else "priority_navigation_cancel"
                )
                dropped = self._command_queue.clear()
                self._publish_queue_event(
                    "clear",
                    command,
                    QueueSnapshot(
                        True, self._command_queue.size(), dropped, cancel_reason
                    ),
                    priority_stop=decision.priority_stop,
                )
                self.action_sequencer.cancel(cancel_reason)
                self.get_logger().info(
                    f"{cancel_reason} received; cancelling current sequence"
                )
                if dropped:
                    self.get_logger().info(f"cleared {dropped} queued command(s)")
                priority_action = "stop" if decision.priority_stop else "cancel_navigation"
                # 取消导航属于控制面指令：必须绕过 FIFO 和 Action result 等待，
                # 否则它会排在当前导航之后，语义上等于“导航结束后再取消”。
                self._publish_actions([ActionCommand(priority_action, {})])
                self._publish_state("listening")
                return
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
        memory_command = parse_memory_command(command)
        if memory_command is None:
            return False
        identity = self._current_speaker
        if memory_command.kind == "whoami":
            if identity.usable:
                profile = self.user_memory.profile(identity)
                name = profile.display_name or identity.display_name or identity.speaker_id
                response = f"我识别到当前用户是：{name}。"
            else:
                response = "我还没有可靠识别到当前用户，可以先说“记住我，我是某某”。"
        elif memory_command.kind == "clear":
            if identity.usable:
                self._try_user_memory_write(self.user_memory.clear, identity)
                response = "已清除当前用户的本地行为记忆。"
            else:
                response = "我还没有可靠识别当前用户，无法清除个人记忆。"
        elif memory_command.kind == "enroll_request":
            if identity.usable:
                self._publish_speaker_enroll_request(
                    identity.speaker_id, identity.display_name or identity.speaker_id
                )
                response = "已开始声纹录入，请连续说三句短句用于采集样本。"
            else:
                response = "请先说“记住我，我是某某”，我会用这个名字开始声纹录入。"
        elif memory_command.kind == "enroll_name":
            if not identity.usable:
                # 没有真实声纹 identity 时提供演示兜底；真实部署仍以 /agent/speaker_identity 为准。
                identity = SpeakerIdentity(
                    speaker_id=str(memory_command.value),
                    confidence=1.0,
                    enrolled=True,
                    model="text-enroll-fallback",
                    display_name=str(memory_command.value),
                    updated_at=time.time(),
                )
                self._current_speaker = identity
            profile = self.user_memory.enroll(identity, str(memory_command.value))
            self._publish_speaker_enroll_request(identity.speaker_id, profile.display_name)
            response = (
                f"好的，我记住你是 {profile.display_name or profile.speaker_id}。"
                "如果已启动声纹 sidecar，请继续说三句短句完成样本采集。"
            )
        elif memory_command.kind == "preference":
            if not identity.usable:
                response = "我还没有可靠识别当前用户，先说“记住我，我是某某”后再记录偏好。"
            else:
                profile = self.user_memory.profile(identity)
                for key, value in dict(memory_command.value).items():
                    profile = self.user_memory.set_preference(identity, key, value)
                response = "已记录你的偏好：" + "，".join(
                    f"{key}={value}" for key, value in sorted(profile.preferences.items())
                )
        else:
            return False
        self.response_delta_pub.publish(String(data=response))
        self.response_pub.publish(String(data=response))
        threading.Thread(
            target=self._speak_memory_response, args=(response,), daemon=True
        ).start()
        self._record_user_interaction(
            identity,
            user_text=command,
            assistant_text=response,
            actions=[],
            success=True,
        )
        return True

    def _speak_memory_response(self, response: str) -> None:
        try:
            self.tts.synthesize([response], self._on_tts_audio)
        except Exception as exc:
            self.get_logger().warning(f"memory response TTS failed: {exc}")

    def _publish_speaker_enroll_request(self, speaker_id: str, display_name: str) -> None:
        payload = {
            "speaker_id": speaker_id,
            "display_name": display_name or speaker_id,
            "samples_required": 3,
        }
        self.speaker_enroll_request_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _normalize_transcript(self, transcript: str) -> str:
        if not self._param("command_normalization_enabled"):
            return transcript
        result = self.command_normalizer.normalize(transcript)
        if result.changed:
            self.get_logger().info(
                f"normalized ASR command: '{result.original}' -> '{result.text}'"
            )
            if self._param("command_normalization_feedback_enabled"):
                self.recognition_feedback_pub.publish(
                    String(data=result.to_feedback_json())
                )
        return result.text

    def _complete_command(self, command: str) -> str:
        result = self.command_completer.complete(command)
        if result.changed:
            self.get_logger().info(
                f"completed short ASR command: '{result.original}' -> '{result.text}'"
            )
            self.recognition_feedback_pub.publish(
                String(data=result.to_feedback_json())
            )
        return result.text

    def _run_command_worker(self):
        while not self._stopping:
            try:
                item = self._command_queue.get(
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
                    self._command_tracker.execution_started(item)
                )
                self.metrics.reset()
                self.metrics.mark_asr_final()
                self._run_queued_turn(item)
                self._publish_execution_event(
                    self._command_tracker.execution_finished(
                        item, success=True, reason="completed"
                    )
                )
            except Exception as exc:
                self._publish_execution_event(
                    self._command_tracker.execution_finished(
                        item, success=False, reason=str(exc)
                    )
                )
                raise
            finally:
                self._command_queue.task_done()

    def _publish_stale_command_event(self, item):
        self._publish_command_queue_payload(
            self._command_tracker.queue_expired(item, size=self._command_queue.size())
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
        first_token = True

        try:
            messages = [{"role": "system", "content": self._system_prompt_with_user_memory()}]
            messages.extend(self.memory.messages())
            messages.append({"role": "user", "content": user_text})
            self.metrics.mark_llm_requested()
            for token in self.llm.stream(messages):
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
                for error in events.errors:
                    self.get_logger().warning(error)

            final_events = parser.finish()
            model_actions.extend(final_events.actions)
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
            self.memory.append_turn(user_text, assistant_text)
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
        nlu_result = self.command_nlu.parse(command)
        if nlu_result.accepted:
            self._nlu_batch_sequence += 1
            batch_id = f"online-nlu-{self._nlu_batch_sequence}"
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
                snapshot = self._command_queue.put(
                    parsed.span_text, context=context, metadata=metadata
                )
                self._publish_queue_event("enqueue", parsed.span_text, snapshot)
                if not snapshot.accepted:
                    self._publish_queue_rejected_recognition(parsed.span_text, snapshot)
                    self._publish_state("queue_full")
                    return
            self._publish_state("queued")
            return

        snapshot = self._command_queue.put(command)
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

    def _publish_nlu_feedback(self, command: str, nlu_result, batch_id: str):
        payload = {
            "status": "nlu_parsed",
            "reason": "command_nlu",
            "transcript": command,
            "batch_id": batch_id,
            "commands": [
                {
                    "intent": parsed.intent,
                    "span_text": parsed.span_text,
                    "confidence": round(parsed.confidence, 3),
                    "actions": [action.as_dict() for action in parsed.actions],
                }
                for parsed in nlu_result.commands
            ],
        }
        self.recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_actions(self, actions):
        action_list = apply_user_preferences(actions, self._current_user_preferences())

        def publish_payload(payload: str):
            self.action_candidate_pub.publish(String(data=payload))
            self.get_logger().info(f"action candidate: {payload}")

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
        self.state_pub.publish(String(data=state))

    def _publish_session_event(self, event):
        self.wake_event_pub.publish(
            String(data=json.dumps(event.wake_event.as_dict(), ensure_ascii=False))
        )
        self.session_state_pub.publish(String(data=event.session_state))

    def _publish_queue_event(
        self,
        event: str,
        text: str,
        snapshot: QueueSnapshot,
        *,
        priority_stop: bool = False,
    ):
        payload = self._command_tracker.queue_event(
            event, text, snapshot, priority_stop=priority_stop
        )
        self._publish_command_queue_payload(payload)

    def _publish_command_queue_payload(self, payload):
        self.command_queue_pub.publish(
            String(data=json.dumps(payload.as_dict(), ensure_ascii=False))
        )

    def _publish_execution_event(self, event):
        self.command_execution_pub.publish(
            String(data=json.dumps(event.as_dict(), ensure_ascii=False))
        )

    def _publish_ignored_recognition(self, transcript: str, reason: str):
        payload = {
            "status": "ignored",
            "reason": reason,
            "transcript": transcript,
        }
        self.recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self.get_logger().info(f"ignored ASR final: reason={reason}, text={transcript}")

    def _publish_queue_rejected_recognition(
        self, transcript: str, snapshot: QueueSnapshot
    ):
        payload = {
            "status": "queue_rejected",
            "reason": snapshot.reason,
            "transcript": transcript,
            "queue_size": snapshot.size,
        }
        self.recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_asr_endpoint_feedback(self, source: str, delay_ms: int):
        payload = {
            "status": "asr_endpoint",
            "source": source,
            "delay_ms": delay_ms,
        }
        self.recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_asr_commit_feedback(self, source: str):
        payload = {
            "status": "asr_commit",
            "source": source,
        }
        self.recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_session_timeout_feedback(self, transcript: str):
        payload = {
            "status": "session_timeout",
            "reason": "voice_session_timeout",
            "transcript": transcript,
            "prompt": "会话已超时，请先说小智",
        }
        self.recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

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
