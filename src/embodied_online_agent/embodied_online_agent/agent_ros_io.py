"""Agent 共享 ROS 2 I/O Facade：集中 typed topic、QoS 与消息封装。"""

from dataclasses import dataclass
from typing import Callable

from embodied_agent_interfaces.msg import (
    RobotCommand,
    RobotCommandResult,
    SpeakerEnrollRequest,
    SpeakerIdentity,
    WakeEvent,
)
from std_msgs.msg import Empty, String, UInt8MultiArray

from .ros_agent_events import RosAgentEventPublisher
from .ros_qos import audio_stream_qos, command_event_qos
from .ros_topics import AgentTopicContract


MessageCallback = Callable[[object], None]


@dataclass(frozen=True)
class AgentRosCallbacks:
    """节点业务回调集合；ROS 接线层不依赖具体在线/离线实现。"""

    text_input: MessageCallback
    wake_event_input: MessageCallback
    speaker_identity: MessageCallback
    clear_memory: MessageCallback
    action_result: MessageCallback
    clean_audio: MessageCallback
    silence_timeout: MessageCallback
    speech_started: MessageCallback
    speech_ended: MessageCallback


class AgentRosIo:
    """为在线/离线 Agent 提供一个稳定、小而完整的 ROS 接口。

    节点只调用语义化 publish 方法，不再知道消息构造、Topic 字符串或
    QoS 细节。实时 PCM 使用 best-effort，控制事件使用 reliable，避免
    DDS 可靠传输在音频高频链路上制造反压。
    """

    def __init__(
        self,
        node,
        callbacks: AgentRosCallbacks,
        *,
        microphone_enabled: bool,
        external_wake_event_enabled: bool,
        topics: AgentTopicContract | None = None,
    ) -> None:
        self._node = node
        self.topics = topics or AgentTopicContract()
        event_qos = command_event_qos(depth=10)
        audio_qos = audio_stream_qos(depth=20)

        create = node.create_lifecycle_publisher
        self._asr_partial = create(String, self.topics.asr_partial, event_qos)
        self._asr_final = create(String, self.topics.asr_final, event_qos)
        self._response = create(String, self.topics.response_text, event_qos)
        self._response_delta = create(String, self.topics.response_delta, event_qos)
        self._action_candidate = create(
            RobotCommand, self.topics.action_candidate, event_qos
        )
        self._speaker_enroll_request = create(
            SpeakerEnrollRequest, self.topics.speaker_enroll_request, event_qos
        )
        self._metrics = create(String, self.topics.metrics, event_qos)
        self._tts_audio = create(UInt8MultiArray, self.topics.tts_pcm, audio_qos)
        self.events = RosAgentEventPublisher(
            node,
            publisher_factory=node.create_lifecycle_publisher,
            topics=self.topics,
        )

        # 保存 subscription 引用，使该 Facade 明确拥有完整 ROS 接线生命周期。
        self._subscriptions = [
            node.create_subscription(
                String, self.topics.text_input, callbacks.text_input, event_qos
            ),
            node.create_subscription(
                SpeakerIdentity,
                self.topics.speaker_identity,
                callbacks.speaker_identity,
                event_qos,
            ),
            node.create_subscription(
                Empty, self.topics.clear_memory, callbacks.clear_memory, event_qos
            ),
            node.create_subscription(
                RobotCommandResult,
                self.topics.action_result,
                callbacks.action_result,
                event_qos,
            ),
        ]
        if external_wake_event_enabled:
            self._subscriptions.append(
                node.create_subscription(
                    WakeEvent,
                    self.topics.wake_event_input,
                    callbacks.wake_event_input,
                    event_qos,
                )
            )
        if microphone_enabled:
            self._subscriptions.extend(
                [
                    node.create_subscription(
                        UInt8MultiArray,
                        self.topics.clean_audio,
                        callbacks.clean_audio,
                        audio_qos,
                    ),
                    node.create_subscription(
                        Empty,
                        self.topics.silence_timeout,
                        callbacks.silence_timeout,
                        event_qos,
                    ),
                    node.create_subscription(
                        Empty,
                        self.topics.speech_started,
                        callbacks.speech_started,
                        event_qos,
                    ),
                    node.create_subscription(
                        Empty,
                        self.topics.speech_ended,
                        callbacks.speech_ended,
                        event_qos,
                    ),
                ]
            )

    def publish_asr_partial(self, text: str) -> None:
        self._asr_partial.publish(String(data=text))

    def publish_asr_final(self, text: str) -> None:
        self._asr_final.publish(String(data=text))

    def publish_response(self, text: str) -> None:
        self._response.publish(String(data=text))

    def publish_response_delta(self, text: str) -> None:
        self._response_delta.publish(String(data=text))

    def publish_action_candidate(self, message: RobotCommand) -> None:
        self._action_candidate.publish(message)

    def publish_speaker_enroll_request(self, message: SpeakerEnrollRequest) -> None:
        self._speaker_enroll_request.publish(message)

    def publish_metrics(self, payload: str) -> None:
        self._metrics.publish(String(data=payload))

    def publish_tts_audio(self, pcm16: bytes) -> None:
        self._tts_audio.publish(UInt8MultiArray(data=list(pcm16)))

    def set_lifecycle_active(self, active: bool) -> None:
        self.events.set_lifecycle_active(active)
