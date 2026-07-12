"""ROS 2 sidecar node for optional WebRTC VAD endpoint detection."""

from __future__ import annotations

import rclpy
from embodied_agent_interfaces.msg import VadEvent
from rclpy.node import Node
from std_msgs.msg import Empty, UInt8MultiArray

from embodied_agent_core.ros_qos import audio_qos, event_qos
from embodied_agent_core.runtime_status_transport import vad_event_to_message

from .silero_vad_sidecar import (
    StreamingVadEndpoint,
    VadEventName,
    WebRtcVadProvider,
    WebRtcVadUnavailableError,
)


class WebRtcVadNode(Node):
    """订阅 /audio/clean_pcm，发布标准 speech endpoint 事件。

    WebRTC VAD 适合 WSL/笔记本麦克风演示里的“轻量成熟默认方案”：它比能量阈值更不容易
    被键盘声/底噪触发，又比 Silero 少了 torch/onnxruntime 这类较重依赖。缺点是没有概率，
    所以本节点把二分类结果映射为 1.0/0.0，再交给 StreamingVadEndpoint 做最小时长、
    静音超时和最大句长控制。
    """

    def __init__(self):
        super().__init__("webrtc_vad")
        self._declare_parameters()
        self._enabled = bool(self.get_parameter("enabled").value)

        event_profile = event_qos(depth=10)
        self._speech_started_pub = self.create_publisher(
            Empty, "/audio/speech_started", event_profile
        )
        self._speech_ended_pub = self.create_publisher(
            Empty, "/audio/speech_ended", event_profile
        )
        self._legacy_silence_pub = self.create_publisher(
            Empty, "/audio/silence_timeout", event_profile
        )
        self._event_pub = self.create_publisher(
            VadEvent, "/audio/vad_event", event_profile
        )
        audio_profile = audio_qos(
            depth=int(self.get_parameter("input_queue_depth").value)
        )

        self._endpoint = None
        if self._enabled:
            try:
                provider = WebRtcVadProvider(
                    aggressiveness=int(self.get_parameter("aggressiveness").value)
                )
            except WebRtcVadUnavailableError:
                raise
            except Exception as error:
                raise WebRtcVadUnavailableError(f"WebRTC VAD 初始化失败: {error}") from error

            self._endpoint = StreamingVadEndpoint(
                provider,
                sample_rate=int(self.get_parameter("sample_rate").value),
                frame_ms=int(self.get_parameter("frame_ms").value),
                threshold=0.5,
                speech_start_ms=float(self.get_parameter("speech_start_ms").value),
                speech_end_silence_s=float(
                    self.get_parameter("speech_end_silence_s").value
                ),
                min_utterance_ms=float(self.get_parameter("min_utterance_ms").value),
                max_utterance_s=float(self.get_parameter("max_utterance_s").value),
            )

        self.create_subscription(
            UInt8MultiArray, "/audio/clean_pcm", self._on_audio, audio_profile
        )

        self.get_logger().info(
            "webrtc vad sidecar ready: enabled=%s sample_rate=%s aggressiveness=%s"
            % (
                self._enabled,
                self.get_parameter("sample_rate").value,
                self.get_parameter("aggressiveness").value,
            )
        )

    def _declare_parameters(self):
        defaults = {
            "enabled": True,
            "sample_rate": 16000,
            "frame_ms": 20,
            "aggressiveness": 2,
            "speech_start_ms": 60.0,
            "speech_end_silence_s": 0.7,
            "min_utterance_ms": 100.0,
            "max_utterance_s": 12.0,
            "input_queue_depth": 20,
            "publish_legacy_silence_timeout": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_audio(self, message: UInt8MultiArray):
        if not self._enabled or self._endpoint is None:
            return
        for event in self._endpoint.process_pcm(bytes(message.data)):
            self._publish_event(event)

    def _publish_event(self, event):
        # WebRTC 与 Silero 共享同一消息契约，上游实现可替换、下游无需分支解析。
        self._event_pub.publish(
            vad_event_to_message(event, provider="webrtc", stamp=self.get_clock().now())
        )
        if event.name == VadEventName.SPEECH_STARTED:
            self._speech_started_pub.publish(Empty())
        elif event.name == VadEventName.SPEECH_ENDED:
            self._speech_ended_pub.publish(Empty())
            if bool(self.get_parameter("publish_legacy_silence_timeout").value):
                self._legacy_silence_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(WebRtcVadNode())
    except WebRtcVadUnavailableError as error:
        rclpy.logging.get_logger("webrtc_vad").fatal(str(error))
        raise SystemExit(1) from error
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
