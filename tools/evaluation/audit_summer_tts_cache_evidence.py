#!/usr/bin/env python3
"""Audit SummerTTS service cache evidence without over-claiming default latency.

SummerTTS 已经服务化，但“服务化”不等于“整句 TTS 首音频 <300ms”。这个脚本只审计
短文本缓存路径：例如“好的”“收到”“正在执行”这类固定反馈，重复调用时是否命中
service-side cache，并且缓存命中后的 roundtrip 是否进入可演示的低延迟范围。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _load_report(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("SummerTTS probe report must be a JSON object")
    return payload


def audit_report(
    report: dict[str, Any],
    *,
    target_roundtrip_ms: float = 300.0,
    min_pcm_bytes: int = 1000,
    require_cache_hit: bool = True,
) -> dict[str, Any]:
    responses = report.get("responses")
    if not isinstance(responses, list):
        responses = []
    blockers: list[str] = []
    warnings: list[str] = []

    if report.get("ok") is not True:
        blockers.append("probe:not_ok")
    if len(responses) < 2:
        blockers.append("probe:repeat_lt_2")
    if int(report.get("pcm_bytes") or 0) < min_pcm_bytes:
        blockers.append("audio:pcm_bytes_below_minimum")

    cache_hit = bool(report.get("cache_hit"))
    cache_hits = int(report.get("cache_hits") or 0)
    if require_cache_hit and not cache_hit:
        blockers.append("cache:required_but_not_hit")
    if cache_hits <= 0:
        warnings.append("cache:no_hits_observed")

    first_roundtrip = float(report.get("first_roundtrip_ms") or 0.0)
    last_roundtrip = float(report.get("last_roundtrip_ms") or report.get("roundtrip_ms") or 0.0)
    if last_roundtrip <= 0:
        blockers.append("latency:last_roundtrip_missing")
    elif last_roundtrip > target_roundtrip_ms:
        blockers.append(f"latency:cache_roundtrip_above_{target_roundtrip_ms:.0f}ms")

    if first_roundtrip > 0 and last_roundtrip > first_roundtrip:
        warnings.append("latency:cache_roundtrip_not_faster_than_first_call")

    return {
        "schema_version": 1,
        "scenario": "summer_tts_cache_evidence_audit",
        "ok": not blockers,
        "blockers": blockers,
        "warnings": warnings,
        "evidence": {
            "cache_hit": cache_hit,
            "cache_hits": cache_hits,
            "response_count": len(responses),
            "first_roundtrip_ms": first_roundtrip,
            "last_roundtrip_ms": last_roundtrip,
            "target_roundtrip_ms": target_roundtrip_ms,
            "pcm_bytes": int(report.get("pcm_bytes") or 0),
            "service_synthesize_ms": float(report.get("service_synthesize_ms") or 0.0),
        },
        "claim_guidance": [
            "可以说：SummerTTS 已完成常驻 C++ ROS service 接入，并支持短文本缓存证据审计。",
            (
                "可以说：当本报告 PASS 时，固定短反馈语的缓存命中 roundtrip "
                f"≤ {target_roundtrip_ms:.0f}ms，可作为演示反馈路径。"
            ),
            "不要说：SummerTTS 整句生成或首音频默认 <300ms；该结论需要独立流式/分段 TTS benchmark。",
            "不要说：SummerTTS 已替代 Sherpa-TTS 成为默认低延迟链路；当前默认低延迟 gate 仍以 Sherpa-TTS 为主。",
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="logs/summer_tts_service_probe.json")
    parser.add_argument("--output", default="logs/summer_tts_cache_audit.json")
    parser.add_argument("--target-roundtrip-ms", type=float, default=300.0)
    parser.add_argument("--min-pcm-bytes", type=int, default=1000)
    parser.add_argument("--allow-cache-miss", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input).expanduser()
    output_path = Path(args.output).expanduser()
    report = _load_report(input_path)
    audit = audit_report(
        report,
        target_roundtrip_ms=args.target_roundtrip_ms,
        min_pcm_bytes=args.min_pcm_bytes,
        require_cache_hit=not args.allow_cache_miss,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "status": "PASS" if audit["ok"] else "FAIL",
                "input": str(input_path),
                "output": str(output_path),
                "blockers": audit["blockers"],
                "warnings": audit["warnings"],
                "evidence": audit["evidence"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    if not audit["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
