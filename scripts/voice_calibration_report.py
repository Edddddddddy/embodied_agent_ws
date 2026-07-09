#!/usr/bin/env python3
"""Generate a live voice-control calibration bundle.

这个脚本把真实语音演示前最常见的几类诊断汇总到一份报告里：

- provider preflight：当前会选 Silero/WebRTC/energy 哪个 VAD，KWS 依赖是否可用；
- audio calibration：麦克风能量、speech ratio、推荐 profile 和阈值；
- KWS calibration：openWakeWord/LiveKit 分数与阈值建议；
- readiness decision：是否可以开始 continuous-offline/online 演示，下一条命令是什么。

真实现场用法：先启动连续语音或 audio frontend，再运行 `--collect` 订阅 topic。
自动测试用 `--synthetic-profile`，不依赖麦克风、ROS graph 或模型。
"""

from __future__ import annotations

import argparse
import json
import shlex
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from audio_frontend_calibration import (  # noqa: E402
    AudioMetricSample,
    analyze_audio_health,
)
from kws_score_calibration import KwsScoreSample, analyze_kws_scores  # noqa: E402
from voice_control_readiness_check import build_readiness_report  # noqa: E402
from voice_provider_preflight import check_voice_providers  # noqa: E402


def _synthetic_audio_samples(profile: str) -> list[AudioMetricSample]:
    """Return deterministic samples for smoke tests and documentation examples."""

    if profile == "ready":
        return [
            AudioMetricSample(rms=0.006, peak=120, speech=False, vad_provider="energy"),
            AudioMetricSample(rms=0.026, peak=1400, speech=True, vad_provider="energy"),
            AudioMetricSample(rms=0.030, peak=1600, speech=True, vad_provider="energy"),
            AudioMetricSample(rms=0.007, peak=130, speech=False, vad_provider="energy"),
        ]
    if profile == "low_gain":
        return [
            AudioMetricSample(rms=0.0003, peak=23, speech=False, vad_provider="energy"),
            AudioMetricSample(rms=0.0023, peak=180, speech=False, vad_provider="energy"),
        ]
    if profile == "noisy_room":
        return [
            AudioMetricSample(rms=0.019, peak=900, speech=True, vad_provider="energy"),
            AudioMetricSample(rms=0.021, peak=950, speech=True, vad_provider="energy"),
            AudioMetricSample(rms=0.018, peak=850, speech=True, vad_provider="energy"),
            AudioMetricSample(rms=0.020, peak=920, speech=True, vad_provider="energy"),
        ]
    return []


def _synthetic_kws_samples(profile: str) -> list[KwsScoreSample]:
    if profile == "ready":
        return [
            KwsScoreSample("openwakeword", "小智", 0.12, 0.5, False, {}),
            KwsScoreSample("openwakeword", "小智", 0.88, 0.5, True, {}),
            KwsScoreSample("openwakeword", "小智", 0.91, 0.5, True, {}),
        ]
    if profile == "noisy_room":
        return [
            KwsScoreSample("openwakeword", "noise", 0.55, 0.5, True, {}),
            KwsScoreSample("openwakeword", "noise", 0.58, 0.5, True, {}),
            KwsScoreSample("openwakeword", "noise", 0.60, 0.5, True, {}),
        ]
    return []


def _collect_ros_samples(
    *,
    duration_s: float,
    audio_topic: str,
    kws_topic: str,
) -> tuple[list[AudioMetricSample], list[KwsScoreSample]]:
    from voice_control_readiness_check import VoiceReadinessNode
    import rclpy

    rclpy.init()
    collector = VoiceReadinessNode(audio_topic, kws_topic)
    try:
        audio_samples, kws_samples = collector.collect(max(0.1, duration_s))
    finally:
        collector.close()
        if rclpy.ok():
            rclpy.shutdown()
    return audio_samples, kws_samples


def build_calibration_report(
    *,
    mode: str,
    audio_samples: list[AudioMetricSample],
    kws_samples: list[KwsScoreSample],
    require_kws: bool,
    vad_provider: str,
    kws_provider: str,
    config_path: Path | None = None,
) -> dict[str, Any]:
    provider = check_voice_providers(
        mode=mode,
        vad_provider=vad_provider,
        kws_provider=kws_provider,
        config_path=config_path,
    )
    audio = analyze_audio_health(audio_samples)
    kws = analyze_kws_scores(kws_samples)
    readiness = build_readiness_report(audio, kws, require_kws=require_kws)
    blockers = [f"provider:{item}" for item in provider.blockers]
    blockers.extend(readiness.blockers)
    warnings = [f"provider:{item}" for item in provider.warnings]
    warnings.extend(readiness.warnings)
    recommended_environment = list(audio.recommended_environment)
    recommended_environment.append(f"VAD_PROVIDER={provider.vad_provider}")
    if provider.kws_provider not in {"none", "disabled", "mock_text"}:
        recommended_environment.append(f"KWS_PROVIDER={provider.kws_provider}")
    if kws.sample_count > 0:
        if provider.kws_provider == "openwakeword":
            recommended_environment.append(
                f"OPENWAKEWORD_THRESHOLD={kws.suggested_threshold}"
            )
        elif provider.kws_provider == "livekit":
            recommended_environment.append(
                f"LIVEKIT_WAKEWORD_THRESHOLD={kws.suggested_threshold}"
            )
    next_command = (
        " ".join(recommended_environment)
        + f" bash scripts/acceptance_test.sh continuous-{mode}"
    )
    ok = not blockers
    decision = "ready" if ok and not warnings else "needs_tuning" if ok else "blocked"

    return {
        "schema_version": 1,
        "scenario": "voice_calibration_report",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "mode": mode,
        "ok": ok,
        "decision": decision,
        "provider": {**asdict(provider), "ok": provider.ok},
        "audio": {**asdict(audio), "ok": audio.ok},
        "kws": {**asdict(kws), "ok": kws.ok},
        "readiness": {**asdict(readiness), "ok": readiness.ok},
        "blockers": blockers,
        "warnings": warnings,
        "recommended_environment": recommended_environment,
        "next_command": next_command,
        "sample_counts": {
            "audio": len(audio_samples),
            "kws": len(kws_samples),
        },
    }


def render_markdown(report: dict[str, Any]) -> str:
    status = "PASS" if report["ok"] else "BLOCKED"
    lines = [
        "# 真实语音控制校准报告",
        "",
        f"- 状态：`{status}`",
        f"- 决策：`{report['decision']}`",
        f"- 模式：`{report['mode']}`",
        f"- 生成时间：`{report['generated_at']}`",
        f"- VAD provider：`{report['provider']['vad_provider']}`",
        f"- KWS provider：`{report['provider']['kws_provider']}`",
        f"- 音频样本数：`{report['sample_counts']['audio']}`",
        f"- KWS 样本数：`{report['sample_counts']['kws']}`",
        "",
        "## 推荐环境变量",
        "",
        "```bash",
    ]
    for item in report["recommended_environment"]:
        lines.append(f"export {item}")
    lines.extend(
        [
            "```",
            "",
            "也可以直接加载脚本生成的 env 文件：",
            "",
            "```bash",
            "source logs/voice_calibration.env",
            "bash scripts/acceptance_test.sh continuous-" + report["mode"],
            "```",
            "",
            "## 下一条建议命令",
            "",
            "```bash",
            report["next_command"],
            "```",
            "",
            "## Audio",
            "",
            f"- profile：`{report['audio']['recommended_voice_profile']}`",
            f"- reason：`{report['audio']['profile_reason']}`",
            f"- suggested threshold：`{report['audio']['suggested_vad_threshold']}`",
            f"- warnings：`{report['audio']['warnings']}`",
            "",
            "## KWS",
            "",
            f"- top keyword：`{report['kws']['top_keyword']}`",
            f"- top score：`{report['kws']['max_top_score']}`",
            f"- suggested threshold：`{report['kws']['suggested_threshold']}`",
            f"- warnings：`{report['kws']['warnings']}`",
            "",
            "## Blockers / Warnings",
            "",
            f"- blockers：`{report['blockers']}`",
            f"- warnings：`{report['warnings']}`",
            "",
        ]
    )
    return "\n".join(lines)


def render_env(report: dict[str, Any]) -> str:
    """Render shell-safe exports for the next live voice-control run."""

    lines = [
        "# Generated by scripts/voice_calibration_report.py",
        f"# generated_at={report['generated_at']}",
        f"# decision={report['decision']}",
        f"# next_command={report['next_command']}",
    ]
    for item in report["recommended_environment"]:
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        if not key.replace("_", "").isalnum():
            continue
        lines.append(f"export {key}={shlex.quote(value)}")
    lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("offline", "online"), default="offline")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--audio-topic", default="/audio/frontend_metrics")
    parser.add_argument("--kws-topic", default="/agent/kws_score")
    parser.add_argument("--collect", action="store_true", help="订阅 ROS topic 采集真实指标")
    parser.add_argument(
        "--synthetic-profile",
        choices=("none", "ready", "low_gain", "noisy_room"),
        default="none",
        help="测试/文档用合成样本；真实现场请用 --collect。",
    )
    parser.add_argument("--require-kws", action="store_true")
    parser.add_argument("--vad-provider", default="auto")
    parser.add_argument("--kws-provider", default="none")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--json-output", default="logs/voice_calibration_report.json")
    parser.add_argument("--md-output", default="logs/voice_calibration_report.md")
    parser.add_argument("--env-output", default="logs/voice_calibration.env")
    parser.add_argument("--print-json", action="store_true")
    args = parser.parse_args()
    if args.collect and args.synthetic_profile != "none":
        parser.error("--collect and --synthetic-profile are mutually exclusive")
    return args


def main() -> None:
    args = parse_args()
    if args.collect:
        audio_samples, kws_samples = _collect_ros_samples(
            duration_s=args.duration,
            audio_topic=args.audio_topic,
            kws_topic=args.kws_topic,
        )
    else:
        audio_samples = _synthetic_audio_samples(args.synthetic_profile)
        kws_samples = _synthetic_kws_samples(args.synthetic_profile)

    report = build_calibration_report(
        mode=args.mode,
        audio_samples=audio_samples,
        kws_samples=kws_samples,
        require_kws=args.require_kws,
        vad_provider=args.vad_provider,
        kws_provider=args.kws_provider,
        config_path=args.config,
    )

    json_output = Path(args.json_output).expanduser()
    md_output = Path(args.md_output).expanduser()
    env_output = Path(args.env_output).expanduser()
    if not json_output.is_absolute():
        json_output = ROOT / json_output
    if not md_output.is_absolute():
        md_output = ROOT / md_output
    if not env_output.is_absolute():
        env_output = ROOT / env_output
    json_output.parent.mkdir(parents=True, exist_ok=True)
    md_output.parent.mkdir(parents=True, exist_ok=True)
    env_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    md_output.write_text(render_markdown(report), encoding="utf-8")
    env_output.write_text(render_env(report), encoding="utf-8")

    summary = {
        "ok": report["ok"],
        "decision": report["decision"],
        "json_output": str(json_output),
        "md_output": str(md_output),
        "env_output": str(env_output),
        "recommended_environment": report["recommended_environment"],
        "next_command": report["next_command"],
        "blockers": report["blockers"],
        "warnings": report["warnings"],
    }
    print(json.dumps(report if args.print_json else summary, ensure_ascii=False, indent=2))
    if not report["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
