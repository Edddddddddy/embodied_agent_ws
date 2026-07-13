#!/usr/bin/env python3
"""把在线/离线长稳、指令准确率和离线延迟收敛成一份事实边界报告。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]


def _read(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else None


def _live_evidence(report: dict[str, Any] | None, mode: str) -> dict[str, Any]:
    if report is None:
        return {"status": "missing", "mode": mode}
    duration = float(report.get("source_report_duration_s", 0.0))
    actual_mode = str(report.get("agent_mode") or "unspecified")
    real_microphone = report.get("evidence_scope") == "operator_declared_real_microphone"
    proven = (
        report.get("passed") is True
        and duration >= 300.0
        and actual_mode == mode
        and real_microphone
    )
    return {
        "status": "proven" if proven else "failed",
        "mode": mode,
        "reported_mode": actual_mode,
        "duration_s": duration,
        "capture_source": report.get("capture_source", "unspecified"),
        "recognition_rate": report.get("recognition_rate"),
        "action_accuracy": report.get("action_accuracy"),
        "action_success_rate": report.get("action_success_rate"),
        "false_trigger_rate": report.get("false_trigger_rate"),
        "queue_rejected_count": int(report.get("queue_rejected_count", 0)),
        "queue_reject_rate": report.get("queue_reject_rate"),
        "ignored_transcript_count": int(report.get("ignored_transcript_count", 0)),
        "recognition_retry_count": int(report.get("recognition_retry_count", 0)),
        "final_cmd_vel_zero": bool(
            (report.get("checks") or {}).get("final_cmd_vel_zero", False)
        ),
        "latency": report.get("latency", {}),
    }


def _instruction_evidence(report: dict[str, Any] | None) -> dict[str, Any]:
    if report is None:
        return {"status": "missing"}
    raw = float(report.get("model_score", 0.0))
    effective = float(report.get("effective_score", 0.0))
    return {
        "status": "measured",
        "sample_count": int(report.get("total", 0)),
        # 两个分数永久分栏，禁止把 fallback 后的系统有效率写成模型原始能力。
        "raw_model_action_accuracy": raw,
        "fallback_and_safety_effective_accuracy": effective,
        "raw_model_target_70pct_met": raw >= 0.70,
        "policy": report.get("effective_score_policy", ""),
    }


def _offline_latency_evidence(report: dict[str, Any] | None) -> dict[str, Any]:
    if report is None:
        return {"status": "missing"}
    metrics = report.get("metrics") or {}
    turn_ms = metrics.get("turn_complete_ms")
    first_audio_ms = metrics.get("end_to_first_audio_ms")
    turn_value = float(turn_ms) if isinstance(turn_ms, (int, float)) else None
    return {
        "status": "measured" if turn_value is not None else "incomplete",
        "measurement_scope": report.get("measurement_scope", ""),
        "asr_text": report.get("asr_text", ""),
        "turn_complete_ms": turn_value,
        "end_to_first_audio_ms": (
            float(first_audio_ms)
            if isinstance(first_audio_ms, (int, float))
            else None
        ),
        "turn_target_3500ms_met": turn_value is not None and turn_value <= 3500.0,
    }


def build_summary(
    *,
    online: dict[str, Any] | None,
    offline: dict[str, Any] | None,
    instruction: dict[str, Any] | None,
    offline_latency: dict[str, Any] | None,
) -> dict[str, Any]:
    online_evidence = _live_evidence(online, "online")
    offline_evidence = _live_evidence(offline, "offline")
    instruction_evidence = _instruction_evidence(instruction)
    latency_evidence = _offline_latency_evidence(offline_latency)
    return {
        "schema_version": 1,
        "scenario": "runtime_evidence_closure",
        "continuous_online_5min": online_evidence,
        "continuous_offline_5min": offline_evidence,
        "offline_instruction_following": instruction_evidence,
        "offline_turn_latency": latency_evidence,
        "all_long_run_modes_proven": (
            online_evidence["status"] == "proven"
            and offline_evidence["status"] == "proven"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--online", type=Path, default=WORKSPACE / "logs/voice_benchmark_online_report.json"
    )
    parser.add_argument(
        "--offline", type=Path, default=WORKSPACE / "logs/voice_benchmark_offline_report.json"
    )
    parser.add_argument(
        "--instruction",
        type=Path,
        default=WORKSPACE / "logs/instruction_following_report.json",
    )
    parser.add_argument(
        "--offline-latency",
        type=Path,
        default=WORKSPACE / "logs/offline_voice_e2e_report.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE / "logs/runtime_evidence_summary.json",
    )
    args = parser.parse_args()
    summary = build_summary(
        online=_read(args.online),
        offline=_read(args.offline),
        instruction=_read(args.instruction),
        offline_latency=_read(args.offline_latency),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    # 汇总器是事实报告，不因缺少真人输入而阻止本地/CI；status 字段负责表达边界。
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
