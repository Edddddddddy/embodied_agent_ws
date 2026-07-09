#!/usr/bin/env python3
"""Unified readiness check for long-running voice control demos.

启动 continuous_voice_control.sh 后运行本脚本，可以同时检查：

- `/audio/frontend_metrics`：麦克风音量、VAD、丢帧；
- `/agent/kws_score`：openWakeWord/LiveKit 声学唤醒分数与阈值。

默认不强制要求 KWS 分数，因为项目仍支持 text wake。若本次演示使用
`KWS_PROVIDER=openwakeword|livekit`，请加 `--require-kws`。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from audio_frontend_calibration import (  # noqa: E402
    AudioHealthReport,
    AudioMetricSample,
    analyze_audio_health,
    parse_audio_metrics,
)
from kws_score_calibration import (  # noqa: E402
    KwsScoreReport,
    analyze_kws_scores,
    parse_kws_score,
)


@dataclass(frozen=True)
class VoiceReadinessReport:
    audio: AudioHealthReport
    kws: KwsScoreReport
    require_kws: bool
    blockers: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.blockers


def build_readiness_report(
    audio: AudioHealthReport,
    kws: KwsScoreReport,
    *,
    require_kws: bool,
) -> VoiceReadinessReport:
    """Merge audio and KWS calibration reports into one go/no-go decision."""

    blockers: list[str] = []
    warnings: list[str] = []

    audio_blockers = {
        "no_audio_metrics",
        "microphone_too_quiet_or_disconnected",
        "audio_input_overrun",
    }
    for warning in audio.warnings:
        target = blockers if warning in audio_blockers else warnings
        target.append(f"audio:{warning}")

    if kws.sample_count == 0 and not require_kws:
        warnings.append("kws:not_required_or_not_running")
    else:
        for warning in kws.warnings:
            target = blockers if require_kws and warning == "no_kws_scores" else warnings
            target.append(f"kws:{warning}")

    return VoiceReadinessReport(
        audio=audio,
        kws=kws,
        require_kws=require_kws,
        blockers=tuple(blockers),
        warnings=tuple(warnings),
    )


def format_readiness_report(report: VoiceReadinessReport) -> str:
    status = "PASS" if report.ok else "BLOCKED"
    lines = [
        f"{status}: voice control readiness",
        f"  audio_samples: {report.audio.sample_count}",
        f"  audio_rms: max={report.audio.max_rms:.4f}, mean={report.audio.mean_rms:.4f}",
        f"  suggested_vad_threshold: {report.audio.suggested_vad_threshold:.4f}",
        f"  recommended_voice_profile: {report.audio.recommended_voice_profile}",
        f"  profile_reason: {report.audio.profile_reason}",
        "  recommended_environment:",
        *[f"    - {item}" for item in report.audio.recommended_environment],
        f"  quick_apply: export VOICE_CONTROL_PROFILE={report.audio.recommended_voice_profile}",
        (
            "  quick_apply_threshold: "
            f"export SPEECH_START_THRESHOLD={report.audio.suggested_vad_threshold:.4f}"
        ),
        f"  next_command: {report.audio.next_command}",
        f"  vad_provider: {report.audio.vad_provider or 'unknown'}",
        (
            "  audio_enhancer: "
            f"requested={report.audio.audio_enhancer_requested or 'unknown'} "
            f"active={report.audio.audio_enhancer_active or 'unknown'} "
            f"aec={report.audio.aec_active} "
            f"ns={report.audio.noise_suppression_active} "
            f"agc={report.audio.auto_gain_active}"
        ),
        f"  kws_required: {report.require_kws}",
        f"  kws_samples: {report.kws.sample_count}",
        f"  kws_provider: {report.kws.provider}",
        f"  kws_top_score: {report.kws.max_top_score:.3f}",
        f"  suggested_kws_threshold: {report.kws.suggested_threshold:.3f}",
    ]
    if report.blockers:
        lines.append("  blockers:")
        for item in report.blockers:
            lines.append(f"    - {item}")
    if report.warnings:
        lines.append("  warnings:")
        for item in report.warnings:
            lines.append(f"    - {item}")
    if report.ok:
        lines.append("  next: 可以开始连续语音控制演示。")
    else:
        lines.append("  next: 先处理 blockers，再重新运行 readiness check。")
    return "\n".join(lines)


def readiness_exit_code(report: VoiceReadinessReport) -> int:
    """Shell contract: PASS returns 0, BLOCKED returns 1.

    continuous_voice_control.sh 用这个退出码决定是否提示“系统已就绪”。
    warnings 不阻断演示，只有 blockers 才返回非零。
    """

    return 0 if report.ok else 1


class VoiceReadinessNode:
    def __init__(self, audio_topic: str, kws_topic: str):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        class _Node(Node):
            def __init__(self, owner: VoiceReadinessNode):
                super().__init__("voice_control_readiness_check")
                self.create_subscription(String, audio_topic, owner._on_audio, 10)
                self.create_subscription(String, kws_topic, owner._on_kws, 10)

        self.audio_samples = []
        self.kws_samples = []
        self._rclpy = rclpy
        self.node = _Node(self)

    def _on_audio(self, message) -> None:
        sample = parse_audio_metrics(message.data)
        if sample is not None:
            self.audio_samples.append(sample)

    def _on_kws(self, message) -> None:
        sample = parse_kws_score(message.data)
        if sample is not None:
            self.kws_samples.append(sample)

    def collect(self, duration_s: float) -> tuple[list, list]:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        return list(self.audio_samples), list(self.kws_samples)

    def close(self) -> None:
        self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0, help="采样秒数")
    parser.add_argument(
        "--audio-topic", default="/audio/frontend_metrics", help="音频指标 topic"
    )
    parser.add_argument("--kws-topic", default="/agent/kws_score", help="KWS 分数 topic")
    parser.add_argument(
        "--require-kws",
        action="store_true",
        help="要求声学 KWS 分数存在；使用 openwakeword/livekit 时建议开启",
    )
    parser.add_argument("--json", action="store_true", help="输出机器可读 JSON")
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    collector = VoiceReadinessNode(args.audio_topic, args.kws_topic)
    try:
        audio_samples, kws_samples = collector.collect(max(0.1, args.duration))
    finally:
        collector.close()
        if rclpy.ok():
            rclpy.shutdown()

    report = build_readiness_report(
        analyze_audio_health(audio_samples),
        analyze_kws_scores(kws_samples),
        require_kws=args.require_kws,
    )
    if args.json:
        payload = asdict(report)
        payload["ok"] = report.ok
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(format_readiness_report(report))
    sys.exit(readiness_exit_code(report))


if __name__ == "__main__":
    main()
