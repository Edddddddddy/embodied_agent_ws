#!/usr/bin/env python3
"""Calibrate and diagnose the ROS audio frontend for long-running voice control.

这个脚本订阅 /audio/frontend_metrics，收集几秒钟的 RMS、speech、丢帧等指标，
然后给出面向真实麦克风调参的建议。核心分析逻辑不依赖 ROS，方便单元测试。
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from dataclasses import asdict, dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class AudioMetricSample:
    """One JSON sample published by audio_frontend_node."""

    rms: float
    peak: float
    speech: bool
    dropped_input_frames: int = 0
    dropped_playback_chunks: int = 0
    vad_provider: str = ""
    audio_enhancer_requested: str = ""
    audio_enhancer_active: str = ""
    aec_active: bool = False
    noise_suppression_requested: bool = False
    noise_suppression_active: bool = False
    auto_gain_requested: bool = False
    auto_gain_active: bool = False


@dataclass(frozen=True)
class AudioHealthReport:
    """Human-facing health report for microphone/VAD calibration."""

    sample_count: int
    max_rms: float
    mean_rms: float
    speech_ratio: float
    dropped_input_delta: int
    dropped_playback_delta: int
    suggested_vad_threshold: float
    vad_provider: str = ""
    audio_enhancer_requested: str = ""
    audio_enhancer_active: str = ""
    aec_active: bool = False
    noise_suppression_active: bool = False
    auto_gain_active: bool = False
    warnings: tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.warnings


def parse_audio_metrics(serialized: str) -> AudioMetricSample | None:
    """Parse /audio/frontend_metrics JSON.

    容错地解析指标：脚本用于现场排障，单条坏消息不应该让整个校准退出。
    """

    try:
        payload = json.loads(serialized)
    except json.JSONDecodeError:
        return None
    if not isinstance(payload, dict):
        return None

    try:
        return AudioMetricSample(
            rms=float(payload.get("rms", 0.0)),
            peak=float(payload.get("peak", 0.0)),
            speech=bool(payload.get("speech", False)),
            dropped_input_frames=int(payload.get("dropped_input_frames", 0)),
            dropped_playback_chunks=int(payload.get("dropped_playback_chunks", 0)),
            vad_provider=str(payload.get("vad_provider", "")),
            audio_enhancer_requested=str(
                payload.get("audio_enhancer_requested", "")
            ),
            audio_enhancer_active=str(payload.get("audio_enhancer_active", "")),
            aec_active=bool(payload.get("aec_active", False)),
            noise_suppression_requested=bool(
                payload.get("noise_suppression_requested", False)
            ),
            noise_suppression_active=bool(
                payload.get("noise_suppression_active", False)
            ),
            auto_gain_requested=bool(payload.get("auto_gain_requested", False)),
            auto_gain_active=bool(payload.get("auto_gain_active", False)),
        )
    except (TypeError, ValueError):
        return None


def _delta(values: Iterable[int]) -> int:
    observed = list(values)
    if len(observed) < 2:
        return 0
    return max(0, observed[-1] - observed[0])


def _suggest_vad_threshold(max_rms: float, mean_rms: float) -> float:
    """Suggest a conservative energy-VAD threshold.

    经验规则：阈值低于清晰说话峰值的一半，同时略高于环境平均能量。
    这里给的是起点，不是绝对最优值；真实环境仍需要配合脚本复测。
    """

    if max_rms <= 0.0:
        return 0.018
    candidate = max(max_rms * 0.45, mean_rms * 1.8)
    return round(min(max(candidate, 0.006), 0.05), 4)


def analyze_audio_health(samples: Sequence[AudioMetricSample]) -> AudioHealthReport:
    """Analyze collected metrics and return actionable warnings."""

    if not samples:
        return AudioHealthReport(
            sample_count=0,
            max_rms=0.0,
            mean_rms=0.0,
            speech_ratio=0.0,
            dropped_input_delta=0,
            dropped_playback_delta=0,
            suggested_vad_threshold=0.018,
            vad_provider="",
            audio_enhancer_requested="",
            audio_enhancer_active="",
            aec_active=False,
            noise_suppression_active=False,
            auto_gain_active=False,
            warnings=("no_audio_metrics",),
        )

    rms_values = [sample.rms for sample in samples]
    max_rms = max(rms_values)
    mean_rms = statistics.fmean(rms_values)
    speech_ratio = sum(1 for sample in samples if sample.speech) / len(samples)
    dropped_input_delta = _delta(sample.dropped_input_frames for sample in samples)
    dropped_playback_delta = _delta(sample.dropped_playback_chunks for sample in samples)
    latest = samples[-1]
    warnings: list[str] = []

    if max_rms < 0.005:
        warnings.append("microphone_too_quiet_or_disconnected")
    if speech_ratio == 0.0 and max_rms > 0.02:
        warnings.append("vad_threshold_may_be_too_high")
    if speech_ratio > 0.95 and mean_rms > 0.01:
        warnings.append("vad_threshold_may_be_too_low_or_environment_noisy")
    if dropped_input_delta > 0:
        warnings.append("audio_input_overrun")
    if dropped_playback_delta > 0:
        warnings.append("tts_playback_overrun")
    if (
        latest.audio_enhancer_requested
        and latest.audio_enhancer_active
        and latest.audio_enhancer_requested != latest.audio_enhancer_active
    ):
        warnings.append("audio_enhancer_fallback")
    if latest.noise_suppression_requested and not latest.noise_suppression_active:
        warnings.append("noise_suppression_unavailable")
    if latest.auto_gain_requested and not latest.auto_gain_active:
        warnings.append("auto_gain_unavailable")

    return AudioHealthReport(
        sample_count=len(samples),
        max_rms=round(max_rms, 6),
        mean_rms=round(mean_rms, 6),
        speech_ratio=round(speech_ratio, 3),
        dropped_input_delta=dropped_input_delta,
        dropped_playback_delta=dropped_playback_delta,
        suggested_vad_threshold=_suggest_vad_threshold(max_rms, mean_rms),
        vad_provider=latest.vad_provider,
        audio_enhancer_requested=latest.audio_enhancer_requested,
        audio_enhancer_active=latest.audio_enhancer_active,
        aec_active=latest.aec_active,
        noise_suppression_active=latest.noise_suppression_active,
        auto_gain_active=latest.auto_gain_active,
        warnings=tuple(warnings),
    )


def format_report(report: AudioHealthReport) -> str:
    """Render a compact report for terminal users."""

    status = "PASS" if report.ok else "WARN"
    lines = [
        f"{status}: audio frontend calibration",
        f"  samples: {report.sample_count}",
        f"  rms: mean={report.mean_rms:.4f}, max={report.max_rms:.4f}",
        f"  speech_ratio: {report.speech_ratio:.2f}",
        f"  dropped_input_delta: {report.dropped_input_delta}",
        f"  dropped_playback_delta: {report.dropped_playback_delta}",
        f"  suggested energy vad threshold: {report.suggested_vad_threshold:.4f}",
        f"  vad_provider: {report.vad_provider or 'unknown'}",
        (
            "  audio_enhancer: "
            f"requested={report.audio_enhancer_requested or 'unknown'} "
            f"active={report.audio_enhancer_active or 'unknown'}"
        ),
        (
            "  audio_enhancement: "
            f"aec={report.aec_active} "
            f"ns={report.noise_suppression_active} "
            f"agc={report.auto_gain_active}"
        ),
    ]
    if report.warnings:
        lines.append("  warnings:")
        explanations = {
            "no_audio_metrics": "没有收到 /audio/frontend_metrics，请确认 audio_frontend 已启动。",
            "microphone_too_quiet_or_disconnected": "麦克风能量过低，检查输入设备、WSL 麦克风权限或靠近麦克风。",
            "vad_threshold_may_be_too_high": "检测到较大音量但 speech=false，可能是 VAD 阈值过高。",
            "vad_threshold_may_be_too_low_or_environment_noisy": "长时间 speech=true，可能是阈值过低或环境噪声过大。",
            "audio_input_overrun": "输入音频丢帧，可能是 CPU 忙、音频块处理过慢或队列太小。",
            "tts_playback_overrun": "TTS 回放队列丢块，长时间控制建议先关闭 speaker。",
            "audio_enhancer_fallback": "请求的音频增强器不可用，当前已回退到 active enhancer。",
            "noise_suppression_unavailable": "已请求降噪但当前增强器未实际启用 NS；如需真实降噪，后续接 WebRTC enhancer。",
            "auto_gain_unavailable": "已请求自动增益但当前增强器未实际启用 AGC；如需真实 AGC，后续接 WebRTC enhancer。",
        }
        for warning in report.warnings:
            lines.append(f"    - {warning}: {explanations.get(warning, warning)}")
    return "\n".join(lines)


class AudioCalibrationNode:
    """Small ROS2 subscriber wrapper kept separate from pure analysis logic."""

    def __init__(self, topic: str):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        class _Node(Node):
            def __init__(self, owner: AudioCalibrationNode):
                super().__init__("audio_frontend_calibration")
                self.create_subscription(String, topic, owner._on_metrics, 10)

        self.samples: list[AudioMetricSample] = []
        self._rclpy = rclpy
        self.node = _Node(self)

    def _on_metrics(self, message) -> None:
        sample = parse_audio_metrics(message.data)
        if sample is not None:
            self.samples.append(sample)

    def collect(self, duration_s: float) -> list[AudioMetricSample]:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        return list(self.samples)

    def close(self) -> None:
        self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=6.0, help="采样秒数")
    parser.add_argument(
        "--topic", default="/audio/frontend_metrics", help="音频指标 topic"
    )
    parser.add_argument("--json", action="store_true", help="输出机器可读 JSON")
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    collector = AudioCalibrationNode(args.topic)
    try:
        samples = collector.collect(max(0.1, args.duration))
    finally:
        collector.close()
        if rclpy.ok():
            rclpy.shutdown()

    report = analyze_audio_health(samples)
    if args.json:
        print(json.dumps(asdict(report), ensure_ascii=False, indent=2))
    else:
        print(format_report(report))


if __name__ == "__main__":
    main()
