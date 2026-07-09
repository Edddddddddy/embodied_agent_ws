#!/usr/bin/env python3
"""Audit offline showcase evidence and prevent over-claiming.

`generate_offline_showcase_report.py` 负责采集证据；本脚本负责回答另一个问题：
当前这份证据“足够支撑哪些说法，不足以支撑哪些说法”。这样汇报时可以把工程已完成、
真实 benchmark 已测、LoRA/训练未复现三类边界讲清楚。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]


def _status_from_executed_block(block: dict[str, Any]) -> str:
    if block.get("status") == "not_run":
        return "missing"
    if block.get("ok") is True:
        return "proven"
    if block.get("returncode") == 0 and (block.get("payload") or {}).get("ok") is True:
        return "proven"
    return "failed"


def _latency_payload_ok(block: dict[str, Any]) -> bool:
    if block.get("ok") is True:
        return True
    payload = block.get("payload")
    return isinstance(payload, dict) and payload.get("ok") is True


def _claim_statuses(report: dict[str, Any]) -> dict[str, str]:
    claim_evidence = report.get("claim_evidence")
    if not isinstance(claim_evidence, dict):
        return {}
    statuses: dict[str, str] = {}
    for item in claim_evidence.get("items", []):
        if isinstance(item, dict) and item.get("key"):
            statuses[str(item["key"])] = str(item.get("status", "unknown"))
    return statuses


def audit_report(
    report: dict[str, Any],
    *,
    parser_minimum: float = 0.95,
    require_latency: bool = False,
    require_asr_tts: bool = False,
) -> dict[str, Any]:
    blockers: list[str] = []
    warnings: list[str] = []

    models = report.get("model_inventory") or {}
    versions = report.get("runtime_versions") or {}
    parser = report.get("instruction_parser") or {}
    latency = report.get("latency") or {}
    asr_tts = report.get("asr_tts_benchmark") or {}
    claim_statuses = _claim_statuses(report)

    if not models.get("ok"):
        blockers.append("model_inventory:missing_required_assets")
    if not versions.get("ok"):
        blockers.append("runtime_versions:not_pinned_or_mismatch")

    parser_accuracy = float(parser.get("accuracy") or 0.0)
    if parser_accuracy < parser_minimum:
        blockers.append(f"instruction_parser:accuracy_below_{parser_minimum:.2f}")
    if parser.get("failed_cases"):
        blockers.append("instruction_parser:failed_cases_present")

    latency_status = _status_from_executed_block(latency)
    if latency_status == "missing":
        if require_latency:
            blockers.append("latency:required_but_not_measured")
        else:
            warnings.append("latency:not_measured")
    elif latency_status != "proven" or not _latency_payload_ok(latency):
        blockers.append("latency:measured_but_failed")

    asr_tts_status = _status_from_executed_block(asr_tts)
    if asr_tts_status == "missing":
        if require_asr_tts:
            blockers.append("asr_tts_benchmark:required_but_not_measured")
        else:
            warnings.append("asr_tts_benchmark:not_measured")
    elif asr_tts_status != "proven":
        blockers.append("asr_tts_benchmark:measured_but_failed")

    warnings.append("lora_training:not_reproduced_in_current_evidence")
    warnings.append("summertts:not_default_low_latency_provider")
    if not claim_statuses:
        warnings.append("claim_evidence:missing")
    else:
        # 将报告中的“不可宣称/未实测”状态透传到审计结果，方便发布前逐项检查。
        for key, status in sorted(claim_statuses.items()):
            if status in {"missing", "not_reproduced", "not_default", "failed"}:
                warnings.append(f"claim_evidence:{key}:{status}")

    evidence = {
        "model_inventory": {
            "status": "proven" if models.get("ok") else "failed",
            "total_size_mb": models.get("total_size_mb", 0),
        },
        "runtime_versions": {
            "status": "proven" if versions.get("ok") else "failed",
        },
        "instruction_parser": {
            "status": "proven" if parser_accuracy >= parser_minimum and not parser.get("failed_cases") else "failed",
            "passed": parser.get("passed", 0),
            "total": parser.get("total", 0),
            "accuracy": parser_accuracy,
            "dataset": parser.get("dataset", ""),
        },
        "latency": {
            "status": latency_status,
            "raw_status": latency.get("status", "executed"),
        },
        "asr_tts_benchmark": {
            "status": asr_tts_status,
            "raw_status": asr_tts.get("status", "executed"),
        },
        "lora_training": {
            "status": "not_proven",
        },
        "summertts_low_latency": {
            "status": "not_default",
        },
        "claim_evidence": {
            "status": "proven" if claim_statuses else "missing",
            "items": claim_statuses,
        },
    }

    claim_guidance = [
        "可以说：离线 Agent 的工程接口、模型资产清单、运行时版本和确定性指令解析评估可复查。",
        (
            "可以说：确定性 parser 在当前评估集上达到 "
            f"{parser.get('passed', 0)}/{parser.get('total', 0)}，准确率 {parser_accuracy:.4f}。"
        ),
    ]
    if latency_status == "proven":
        claim_guidance.append("可以说：本机离线延迟目标已有真实测量证据。")
    else:
        claim_guidance.append("不要说：本机 llama.cpp 首 token <1s 或 TTS 首音频 <300ms 已复现；除非先运行 offline-latency/--run-latency。")
    if asr_tts_status == "proven":
        claim_guidance.append("可以说：Sherpa ASR/TTS benchmark 已在当前环境真实测量。")
    else:
        claim_guidance.append("不要说：ASR/TTS realtime factor 已在当前环境复现；除非先运行 benchmark_offline 或 --run-asr-tts。")
    claim_guidance.append("不要说：LoRA 微调、Q8 指令遵循 85% 等训练指标已经复现；当前证据只支撑工程接口与评估闭环。")
    claim_guidance.append("不要说：SummerTTS 是默认 <300ms 低延迟 TTS；当前低延迟默认仍以 Sherpa-TTS 为准。")

    return {
        "schema_version": 1,
        "scenario": "offline_showcase_evidence_audit",
        "ok": not blockers,
        "blockers": blockers,
        "warnings": warnings,
        "evidence": evidence,
        "claim_guidance": claim_guidance,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="logs/offline_showcase_report.json")
    parser.add_argument("--output", default="logs/offline_evidence_audit.json")
    parser.add_argument("--parser-minimum", type=float, default=0.95)
    parser.add_argument("--require-latency", action="store_true")
    parser.add_argument("--require-asr-tts", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input).expanduser()
    output_path = Path(args.output).expanduser()
    if not input_path.is_absolute():
        input_path = WORKSPACE / input_path
    if not output_path.is_absolute():
        output_path = WORKSPACE / output_path
    report = json.loads(input_path.read_text(encoding="utf-8"))
    audit = audit_report(
        report,
        parser_minimum=args.parser_minimum,
        require_latency=args.require_latency,
        require_asr_tts=args.require_asr_tts,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {
        "status": "PASS" if audit["ok"] else "FAIL",
        "input": str(input_path),
        "output": str(output_path),
        "blockers": audit["blockers"],
        "warnings": audit["warnings"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not audit["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
