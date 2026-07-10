"""ROS 2 sidecar node for optional Silero VAD endpoint detection."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray

from .silero_vad_sidecar import (
    SileroVadProvider,
    SileroVadUnavailableError,
    StreamingVadEndpoint,
    VadEventName,
)


class SileroVadNode(Node):
    """订阅 /audio/clean_pcm，发布标准 speech endpoint 事件。

    这个节点是 sidecar：AudioFrontend 仍然负责声卡、AEC 和 clean PCM 发布；
    SileroVadNode 只接管“什么时候开始/结束一句话”。这样默认 energy VAD 和
    可选 Silero VAD 可以通过 launch 参数平滑切换。
    """

    def __init__(self):
        super().__init__("silero_vad")
        self._declare_parameters()
        self._enabled = bool(self.get_parameter("enabled").value)

        self._speech_started_pub = self.create_publisher(Empty, "/audio/speech_started", 10)
        self._speech_ended_pub = self.create_publisher(Empty, "/audio/speech_ended", 10)
        self._legacy_silence_pub = self.create_publisher(Empty, "/audio/silence_timeout", 10)
        self._event_pub = self.create_publisher(String, "/audio/vad_event", 10)

        audio_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self.get_parameter("input_queue_depth").value),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._endpoint = None
        if self._enabled:
            try:
                provider = SileroVadProvider(
                    use_onnx=bool(self.get_parameter("use_onnx").value),
                    model_path=str(self.get_parameter("model_path").value),
                )
            except SileroVadUnavailableError:
                raise
            except Exception as error:
                raise SileroVadUnavailableError(f"Silero VAD 初始化失败: {error}") from error

            self._endpoint = StreamingVadEndpoint(
                provider,
                sample_rate=int(self.get_parameter("sample_rate").value),
                frame_ms=int(self.get_parameter("frame_ms").value),
                threshold=float(self.get_parameter("threshold").value),
                speech_start_ms=float(self.get_parameter("speech_start_ms").value),
                speech_end_threshold=float(
                    self.get_parameter("speech_end_threshold").value
                ),
                speech_end_silence_s=float(
                    self.get_parameter("speech_end_silence_s").value
                ),
                min_utterance_ms=float(self.get_parameter("min_utterance_ms").value),
                max_utterance_s=float(self.get_parameter("max_utterance_s").value),
            )

        self.create_subscription(UInt8MultiArray, "/audio/clean_pcm", self._on_audio, audio_qos)

        self.get_logger().info(
            "silero vad sidecar ready: enabled=%s sample_rate=%s threshold=%.2f"
            % (
                self._enabled,
                self.get_parameter("sample_rate").value,
                float(self.get_parameter("threshold").value),
            )
        )

    def _declare_parameters(self):
        defaults = {
            "enabled": True,
            "sample_rate": 16000,
            "frame_ms": 32,
            "threshold": 0.5,
            "speech_start_ms": 96.0,
            "speech_end_threshold": 0.35,
            "speech_end_silence_s": 0.4,
            "min_utterance_ms": 100.0,
            "max_utterance_s": 12.0,
            "use_onnx": True,
            "model_path": "",
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
        payload = {
            "name": event.name.value,
            "reason": event.reason,
            "probability": event.probability,
            "provider": "silero",
        }
        self._event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if event.name == VadEventName.SPEECH_STARTED:
            self._speech_started_pub.publish(Empty())
        elif event.name == VadEventName.SPEECH_ENDED:
            self._speech_ended_pub.publish(Empty())
            if bool(self.get_parameter("publish_legacy_silence_timeout").value):
                self._legacy_silence_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(SileroVadNode())
    except SileroVadUnavailableError as error:
        rclpy.logging.get_logger("silero_vad").fatal(str(error))
        raise SystemExit(1) from error
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
