import math
import struct
import threading
import wave
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from embodied_agent_interfaces.msg import (
    SpeakerEnrollRequest,
    SpeakerEnrollStatus,
    SpeakerIdentity,
)
from rclpy.node import Node
from std_msgs.msg import Empty, UInt8MultiArray

from embodied_agent_core.ros_qos import audio_qos, command_qos, event_qos, state_qos
from embodied_agent_core.speaker_transport import (
    enroll_status_to_message,
    identity_payload_to_message,
)


@dataclass
class AudioWindow:
    pcm: bytearray

    def append(self, data: bytes, max_bytes: int) -> None:
        self.pcm.extend(data)
        if len(self.pcm) > max_bytes:
            del self.pcm[: len(self.pcm) - max_bytes]

    def rms(self) -> float:
        if len(self.pcm) < 2:
            return 0.0
        sample_count = len(self.pcm) // 2
        samples = struct.unpack("<" + "h" * sample_count, self.pcm[: sample_count * 2])
        if not samples:
            return 0.0
        return math.sqrt(sum(sample * sample for sample in samples) / len(samples)) / 32768.0

    def clear(self) -> None:
        self.pcm.clear()


@dataclass
class EnrollmentSession:
    speaker_id: str
    display_name: str
    required: int
    collected: int = 0


@dataclass(frozen=True)
class SpeakerScoreDecision:
    speaker_id: str
    confidence: float
    second_best_score: float
    margin: float
    matched: bool
    reason: str


def classify_speaker_scores(
    scores: dict[str, float], *, threshold: float, min_margin: float
) -> SpeakerScoreDecision:
    """把实际余弦分数转换为可解释的身份决策。

    单看 top-1 超阈值仍可能把两个相似声纹混淆；margin 约束要求第一名明显
    高于第二名。歧义时宁可返回 unknown，也不把 A 用户的偏好写进 B 的画像。
    """

    ranked = sorted(
        (
            (speaker_id, float(score))
            for speaker_id, score in scores.items()
            if speaker_id and math.isfinite(float(score))
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    if not ranked:
        return SpeakerScoreDecision("unknown", 0.0, 0.0, 0.0, False, "no_scores")
    speaker_id, best_score = ranked[0]
    second_best = ranked[1][1] if len(ranked) > 1 else 0.0
    margin = best_score - second_best if len(ranked) > 1 else best_score
    if best_score < threshold:
        reason = "below_threshold"
    elif len(ranked) > 1 and margin < min_margin:
        reason = "ambiguous_match"
    else:
        return SpeakerScoreDecision(
            speaker_id, best_score, second_best, margin, True, "matched"
        )
    return SpeakerScoreDecision(
        "unknown", best_score, second_best, margin, False, reason
    )


class SpeakerIdentityNode(Node):
    """声纹识别 sidecar。

    第一版提供 mock 与 sherpa seam：
    - mock 用于自动验收和演示，不依赖模型文件；
    - sherpa 模式保留真实 sherpa-onnx 接入点，模型参数齐全后只替换 _identify_with_sherpa。
    """

    def __init__(self):
        super().__init__("speaker_identity")
        self.declare_parameter("mode", "mock")
        self.declare_parameter("speaker_id", "demo_user")
        self.declare_parameter("display_name", "演示用户")
        self.declare_parameter("confidence", 0.92)
        self.declare_parameter("min_audio_rms", 0.002)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("window_s", 8.0)
        self.declare_parameter("publish_on_start", True)
        self.declare_parameter("sherpa_model", "")
        self.declare_parameter("sherpa_speaker_file", "")
        self.declare_parameter("sherpa_threshold", 0.6)
        self.declare_parameter("sherpa_min_margin", 0.05)
        self.declare_parameter("sherpa_num_threads", 2)
        self.declare_parameter("sherpa_provider", "cpu")
        self.declare_parameter("enroll_dir", "~/.ros/embodied_agent/speaker_samples")
        self.declare_parameter("enroll_samples_required", 3)

        sample_rate = int(self.get_parameter("sample_rate").value)
        window_s = float(self.get_parameter("window_s").value)
        self._max_bytes = max(3200, int(sample_rate * window_s) * 2)
        self._audio = AudioWindow(bytearray())
        self._lock = threading.Lock()
        self._mode = str(self.get_parameter("mode").value)
        self._sherpa_extractor = None
        self._sherpa_manager = None
        self._enrollment: EnrollmentSession | None = None
        if self._mode == "sherpa":
            self._load_sherpa_backend()
        self._pub = self.create_publisher(
            SpeakerIdentity, "/agent/speaker_identity", state_qos()
        )
        self._enroll_status_pub = self.create_publisher(
            SpeakerEnrollStatus, "/agent/speaker_enroll_status", event_qos(depth=10)
        )
        self.create_subscription(
            UInt8MultiArray, "/audio/clean_pcm", self._on_audio, audio_qos(depth=20)
        )
        self.create_subscription(
            Empty, "/audio/speech_ended", self._on_speech_ended, event_qos(depth=10)
        )
        self.create_subscription(
            SpeakerEnrollRequest,
            "/agent/speaker_enroll_request",
            self._on_enroll_request,
            command_qos(depth=10),
        )
        if bool(self.get_parameter("publish_on_start").value):
            self.create_timer(0.5, self._publish_start_identity_once)
        self._published_start = False
        self.get_logger().info(f"speaker identity node ready: mode={self._mode}")

    def _publish_start_identity_once(self):
        if self._published_start:
            return
        self._published_start = True
        if self._mode == "mock":
            self._publish_identity(self._mock_identity(reason="startup"))
            return
        # sherpa 模式必须等真实语音 embedding；启动时发布 mock 用户会在第一句
        # 语音之前错误加载/写入 demo_user 的个人记忆。
        self._publish_identity(
            {
                "speaker_id": "unknown",
                "confidence": 0.0,
                "enrolled": False,
                "model": "sherpa-onnx",
                "reason": "awaiting_audio",
                "rms": 0.0,
            }
        )

    def _on_audio(self, message):
        with self._lock:
            self._audio.append(bytes(message.data), self._max_bytes)

    def _on_enroll_request(self, message):
        speaker_id = _safe_speaker_id(message.speaker_id or message.display_name or "user")
        display_name = message.display_name or speaker_id
        required = int(
            message.samples_required
            or self.get_parameter("enroll_samples_required").value
        )
        self._enrollment = EnrollmentSession(
            speaker_id=speaker_id,
            display_name=display_name,
            required=max(1, min(5, required)),
        )
        self._publish_enroll_status("started", reason="waiting_for_speech")

    def _on_speech_ended(self, _message):
        with self._lock:
            rms = self._audio.rms()
            pcm = bytes(self._audio.pcm)
            self._audio.clear()
        self._maybe_collect_enrollment_sample(pcm, rms)
        if rms < float(self.get_parameter("min_audio_rms").value):
            self._publish_identity(
                {
                    "speaker_id": "unknown",
                    "confidence": 0.0,
                    "enrolled": False,
                    "model": self._mode,
                    "reason": "low_audio_rms",
                    "rms": round(rms, 5),
                }
            )
            return
        if self._mode == "sherpa":
            self._publish_identity(self._identify_with_sherpa(rms, pcm))
            return
        self._publish_identity(self._mock_identity(reason="speech_ended", rms=rms))

    def _mock_identity(self, *, reason: str, rms: float = 0.0) -> dict:
        return {
            "speaker_id": str(self.get_parameter("speaker_id").value),
            "display_name": str(self.get_parameter("display_name").value),
            "confidence": float(self.get_parameter("confidence").value),
            "enrolled": True,
            "model": "mock-speaker",
            "reason": reason,
            "rms": round(rms, 5),
        }

    def _load_sherpa_backend(self):
        model = str(self.get_parameter("sherpa_model").value)
        speaker_file = self._speaker_file_path()
        if not model or not speaker_file:
            self.get_logger().warning(
                "sherpa speaker identity disabled: missing sherpa_model or sherpa_speaker_file"
            )
            return
        try:
            import numpy as np
            import sherpa_onnx  # noqa: F401
        except Exception as exc:
            self.get_logger().warning(f"sherpa speaker identity unavailable: {exc}")
            return
        config = sherpa_onnx.SpeakerEmbeddingExtractorConfig(
            model=model,
            num_threads=int(self.get_parameter("sherpa_num_threads").value),
            provider=str(self.get_parameter("sherpa_provider").value),
        )
        if not config.validate():
            self.get_logger().warning(f"invalid sherpa speaker config: {config}")
            return
        extractor = sherpa_onnx.SpeakerEmbeddingExtractor(config)
        manager = sherpa_onnx.SpeakerEmbeddingManager(extractor.dim)
        embeddings_by_speaker = {}
        for speaker_id, wav_path in self._read_speaker_file(speaker_file):
            try:
                samples, sample_rate = self._read_wav_mono(wav_path, np)
                stream = extractor.create_stream()
                stream.accept_waveform(sample_rate=sample_rate, waveform=samples)
                stream.input_finished()
                embedding = extractor.compute(stream)
                embeddings_by_speaker.setdefault(speaker_id, []).append(embedding)
            except Exception as exc:
                self.get_logger().warning(f"failed to enroll speaker sample {wav_path}: {exc}")
        registered_speakers = 0
        registered_samples = 0
        for speaker_id, embeddings in embeddings_by_speaker.items():
            # Sherpa manager 支持同一 speaker 的 embedding list；按人聚合后一次 add，
            # 才能真正利用注册流程采集的 3 段样本，而不是静默只保留第一段。
            if manager.add(speaker_id, embeddings):
                registered_speakers += 1
                registered_samples += len(embeddings)
        if registered_speakers <= 0:
            self.get_logger().warning("no speaker samples were enrolled for sherpa identity")
            return
        self._sherpa_extractor = extractor
        self._sherpa_manager = manager
        self.get_logger().info(
            "sherpa speaker identity loaded: "
            f"{registered_speakers} speaker(s), {registered_samples} sample(s)"
        )

    def _identify_with_sherpa(self, rms: float, pcm: bytes) -> dict:
        if self._sherpa_extractor is None or self._sherpa_manager is None:
            return {
                "speaker_id": "unknown",
                "confidence": 0.0,
                "enrolled": False,
                "model": "sherpa-onnx",
                "reason": "sherpa_backend_not_ready",
                "rms": round(rms, 5),
            }
        try:
            import numpy as np

            samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
            stream = self._sherpa_extractor.create_stream()
            stream.accept_waveform(
                sample_rate=int(self.get_parameter("sample_rate").value),
                waveform=np.ascontiguousarray(samples),
            )
            stream.input_finished()
            embedding = self._sherpa_extractor.compute(stream)
            threshold = float(self.get_parameter("sherpa_threshold").value)
            min_margin = float(self.get_parameter("sherpa_min_margin").value)
            scores = {
                speaker_id: self._sherpa_manager.score(speaker_id, embedding)
                for speaker_id in self._sherpa_manager.all_speakers
            }
            decision = classify_speaker_scores(
                scores, threshold=threshold, min_margin=min_margin
            )
            return {
                "speaker_id": decision.speaker_id,
                # confidence 必须是模型实际相似度，不能再用配置 threshold 冒充。
                "confidence": round(decision.confidence, 4),
                "second_best_score": round(decision.second_best_score, 4),
                "score_margin": round(decision.margin, 4),
                "threshold": threshold,
                "min_margin": min_margin,
                "enrolled": decision.matched,
                "model": "sherpa-onnx",
                "reason": decision.reason,
                "rms": round(rms, 5),
            }
        except Exception as exc:
            return {
                "speaker_id": "unknown",
                "confidence": 0.0,
                "enrolled": False,
                "model": "sherpa-onnx",
                "reason": f"sherpa_inference_failed:{exc}",
                "rms": round(rms, 5),
            }

    @staticmethod
    def _read_speaker_file(path: str):
        file_path = Path(path).expanduser()
        if not file_path.exists():
            return
        for line in file_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split(maxsplit=1)
            if len(fields) == 2:
                yield fields[0], fields[1]

    @staticmethod
    def _read_wav_mono(path: str, np):
        with wave.open(str(Path(path).expanduser()), "rb") as wav:
            sample_rate = wav.getframerate()
            channels = wav.getnchannels()
            sample_width = wav.getsampwidth()
            frames = wav.readframes(wav.getnframes())
        if sample_width != 2:
            raise ValueError("only int16 wav samples are supported for enrollment")
        data = np.frombuffer(frames, dtype=np.int16)
        if channels > 1:
            data = data.reshape(-1, channels)[:, 0]
        samples = data.astype(np.float32) / 32768.0
        return np.ascontiguousarray(samples), sample_rate

    def _maybe_collect_enrollment_sample(self, pcm: bytes, rms: float) -> None:
        session = self._enrollment
        if session is None:
            return
        if rms < float(self.get_parameter("min_audio_rms").value):
            self._publish_enroll_status("collecting", reason="low_audio_rms")
            return
        session.collected += 1
        sample_path = self._write_enrollment_wav(session, pcm)
        self._append_speaker_sample(session.speaker_id, sample_path)
        if session.collected >= session.required:
            self._publish_enroll_status(
                "completed",
                reason="samples_ready",
                sample_path=str(sample_path),
            )
            self._enrollment = None
            if self._mode == "sherpa":
                self._load_sherpa_backend()
            return
        self._publish_enroll_status(
            "collecting",
            reason="sample_saved",
            sample_path=str(sample_path),
        )

    def _write_enrollment_wav(self, session: EnrollmentSession, pcm: bytes) -> Path:
        root = Path(str(self.get_parameter("enroll_dir").value)).expanduser()
        speaker_dir = root / session.speaker_id
        speaker_dir.mkdir(parents=True, exist_ok=True)
        path = speaker_dir / f"sample_{session.collected}_{int(time.time())}.wav"
        with wave.open(str(path), "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(int(self.get_parameter("sample_rate").value))
            wav.writeframes(pcm)
        return path

    def _append_speaker_sample(self, speaker_id: str, sample_path: Path) -> None:
        speaker_file = Path(self._speaker_file_path()).expanduser()
        speaker_file.parent.mkdir(parents=True, exist_ok=True)
        line = f"{speaker_id} {sample_path}\n"
        existing = speaker_file.read_text(encoding="utf-8") if speaker_file.exists() else ""
        if line not in existing:
            with speaker_file.open("a", encoding="utf-8") as handle:
                handle.write(line)

    def _speaker_file_path(self) -> str:
        configured = str(self.get_parameter("sherpa_speaker_file").value)
        if configured:
            return configured
        return str(Path(str(self.get_parameter("enroll_dir").value)).expanduser() / "speakers.txt")

    def _publish_enroll_status(self, status: str, *, reason: str, sample_path: str = ""):
        session = self._enrollment
        payload = {
            "status": status,
            "reason": reason,
            "speaker_id": session.speaker_id if session else "",
            "display_name": session.display_name if session else "",
            "collected": session.collected if session else 0,
            "required": session.required if session else 0,
            "sample_path": sample_path,
        }
        self._enroll_status_pub.publish(
            enroll_status_to_message(payload, stamp=self.get_clock().now())
        )
        self.get_logger().info(
            "speaker enroll status: status=%s speaker_id=%s collected=%s/%s reason=%s"
            % (
                status,
                payload["speaker_id"],
                payload["collected"],
                payload["required"],
                reason,
            )
        )

    def _publish_identity(self, payload: dict):
        self._pub.publish(
            identity_payload_to_message(payload, stamp=self.get_clock().now())
        )
        self.get_logger().info(
            "speaker identity: speaker_id=%s confidence=%.4f enrolled=%s reason=%s"
            % (
                payload.get("speaker_id", "unknown"),
                float(payload.get("confidence") or 0.0),
                bool(payload.get("enrolled", False)),
                payload.get("reason", ""),
            )
        )


def main():
    rclpy.init()
    node = SpeakerIdentityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


def _safe_speaker_id(value: str) -> str:
    safe = "".join(ch for ch in value.strip() if ch.isalnum() or ch in "_-")
    return safe or "user"
