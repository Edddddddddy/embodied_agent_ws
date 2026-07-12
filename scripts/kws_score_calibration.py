#!/usr/bin/env python3
"""Calibrate acoustic wake-word thresholds from /agent/kws_score.

真实 openWakeWord/LiveKit 模型接入后，低于阈值的候选也会发布到 `/agent/kws_score`。
本脚本收集这些分数并给出阈值建议，用于排查“没唤醒 / 太容易误唤醒”。
核心分析逻辑不依赖 ROS，方便单元测试。
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from dataclasses import asdict, dataclass
from typing import Sequence


@dataclass(frozen=True)
class KwsScoreSample:
    provider: str
    top_keyword: str
    top_score: float
    threshold: float
    above_threshold: bool
    scores: dict[str, float]


@dataclass(frozen=True)
class KwsScoreReport:
    sample_count: int
    provider: str
    top_keyword: str
    max_top_score: float
    mean_top_score: float
    trigger_ratio: float
    current_threshold: float
    suggested_threshold: float
    warnings: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.warnings


def parse_kws_score(serialized: str | dict) -> KwsScoreSample | None:
    """Parse a typed KWS dictionary or a saved legacy report sample."""

    if isinstance(serialized, dict):
        payload = serialized
    else:
        try:
            payload = json.loads(serialized)
        except json.JSONDecodeError:
            return None
    if not isinstance(payload, dict):
        return None

    raw_scores = payload.get("scores", {})
    if not isinstance(raw_scores, dict):
        raw_scores = {}
    scores: dict[str, float] = {}
    for keyword, score in raw_scores.items():
        try:
            scores[str(keyword)] = float(score)
        except (TypeError, ValueError):
            continue

    try:
        return KwsScoreSample(
            provider=str(payload.get("provider", "unknown")),
            top_keyword=str(payload.get("top_keyword", "unknown")),
            top_score=float(payload.get("top_score", 0.0)),
            threshold=float(payload.get("threshold", 0.5)),
            above_threshold=bool(payload.get("above_threshold", False)),
            scores=scores,
        )
    except (TypeError, ValueError):
        return None


def _suggest_threshold(max_score: float, mean_score: float) -> float:
    if max_score <= 0.0:
        return 0.5
    # 起点规则：低于清晰唤醒峰值，但高于背景均值。真实环境需结合静音/唤醒两段复测。
    candidate = max(max_score * 0.75, mean_score * 1.05, 0.3)
    return round(min(max(candidate, 0.2), 0.95), 3)


def analyze_kws_scores(samples: Sequence[KwsScoreSample]) -> KwsScoreReport:
    """Analyze score stream and return threshold advice."""

    if not samples:
        return KwsScoreReport(
            sample_count=0,
            provider="unknown",
            top_keyword="unknown",
            max_top_score=0.0,
            mean_top_score=0.0,
            trigger_ratio=0.0,
            current_threshold=0.5,
            suggested_threshold=0.5,
            warnings=("no_kws_scores",),
        )

    top_scores = [sample.top_score for sample in samples]
    max_index = max(range(len(samples)), key=lambda index: samples[index].top_score)
    best = samples[max_index]
    max_score = max(top_scores)
    mean_score = statistics.fmean(top_scores)
    trigger_ratio = sum(1 for sample in samples if sample.above_threshold) / len(samples)
    current_threshold = statistics.fmean(sample.threshold for sample in samples)
    suggested = _suggest_threshold(max_score, mean_score)
    warnings: list[str] = []

    if max_score < 0.35:
        warnings.append("no_strong_wake_candidate")
    if trigger_ratio == 0.0 and max_score >= 0.45:
        warnings.append("threshold_may_be_too_high")
    if trigger_ratio > 0.8 and mean_score >= current_threshold * 0.9:
        warnings.append("threshold_may_be_too_low_or_environment_noisy")

    return KwsScoreReport(
        sample_count=len(samples),
        provider=best.provider,
        top_keyword=best.top_keyword,
        max_top_score=round(max_score, 6),
        mean_top_score=round(mean_score, 6),
        trigger_ratio=round(trigger_ratio, 3),
        current_threshold=round(current_threshold, 6),
        suggested_threshold=suggested,
        warnings=tuple(warnings),
    )


def format_report(report: KwsScoreReport) -> str:
    status = "PASS" if report.ok else "WARN"
    lines = [
        f"{status}: KWS score calibration",
        f"  samples: {report.sample_count}",
        f"  provider: {report.provider}",
        f"  top_keyword: {report.top_keyword}",
        f"  top_score: max={report.max_top_score:.3f}, mean={report.mean_top_score:.3f}",
        f"  trigger_ratio: {report.trigger_ratio:.3f}",
        f"  current_threshold: {report.current_threshold:.3f}",
        f"  suggested_threshold: {report.suggested_threshold:.3f}",
    ]
    if report.warnings:
        lines.append("  warnings:")
        explanations = {
            "no_kws_scores": "没有收到 /agent/kws_score，请确认 KWS provider 为 openwakeword/livekit 且节点已启动。",
            "no_strong_wake_candidate": "采样期间没有明显唤醒候选，请靠近麦克风、检查模型或确认说了唤醒词。",
            "threshold_may_be_too_high": "存在较高候选分但没有触发，阈值可能过高。",
            "threshold_may_be_too_low_or_environment_noisy": "几乎每帧都触发，阈值可能过低或环境噪声/模型误报较高。",
        }
        for warning in report.warnings:
            lines.append(f"    - {warning}: {explanations.get(warning, warning)}")
    return "\n".join(lines)


class KwsScoreCalibrationNode:
    def __init__(self, topic: str):
        from embodied_agent_interfaces.msg import KwsScore
        from embodied_online_agent.runtime_status_transport import kws_score_to_dict
        import rclpy
        from rclpy.node import Node

        class _Node(Node):
            def __init__(self, owner: KwsScoreCalibrationNode):
                super().__init__("kws_score_calibration")
                self.create_subscription(
                    KwsScore,
                    topic,
                    lambda message: owner._on_score(kws_score_to_dict(message)),
                    10,
                )

        self.samples: list[KwsScoreSample] = []
        self._rclpy = rclpy
        self.node = _Node(self)

    def _on_score(self, message) -> None:
        sample = parse_kws_score(message)
        if sample is not None:
            self.samples.append(sample)

    def collect(self, duration_s: float) -> list[KwsScoreSample]:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        return list(self.samples)

    def close(self) -> None:
        self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0, help="采样秒数")
    parser.add_argument("--topic", default="/agent/kws_score", help="KWS 分数 topic")
    parser.add_argument("--json", action="store_true", help="输出机器可读 JSON")
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    collector = KwsScoreCalibrationNode(args.topic)
    try:
        samples = collector.collect(max(0.1, args.duration))
    finally:
        collector.close()
        if rclpy.ok():
            rclpy.shutdown()

    report = analyze_kws_scores(samples)
    if args.json:
        print(json.dumps(asdict(report), ensure_ascii=False, indent=2))
    else:
        print(format_report(report))


if __name__ == "__main__":
    main()
