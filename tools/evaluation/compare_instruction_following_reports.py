#!/usr/bin/env python3
"""Compare baseline and tuned Q8 reports under a strict provenance contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[2]
COMPARABILITY_KEYS = (
    "dataset_sha256",
    "system_prompt_sha256",
    "temperature",
    "max_tokens",
    "seed",
)


def _load(path: Path) -> dict[str, Any]:
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("scenario") != "offline_llm_instruction_following_eval":
        raise ValueError(f"not an instruction-following report: {path}")
    required = ("model_score", "action_score", "protocol_score", "effective_score", "total")
    missing = [key for key in required if key not in report]
    if missing:
        raise ValueError(f"report needs schema-v2 split metrics ({', '.join(missing)}): {path}")
    return report


def _metric_block(report: dict[str, Any]) -> dict[str, Any]:
    provenance = report.get("provenance", {})
    return {
        "model": report.get("model", ""),
        "model_file_sha256": provenance.get("model_file_sha256"),
        "total": report["total"],
        "strict_protocol_and_action_score": report["model_score"],
        "raw_action_score": report["action_score"],
        "protocol_score": report["protocol_score"],
        "effective_fallback_score": report["effective_score"],
        "failure_counts": report.get("failure_counts", {}),
    }


def compare_reports(baseline: dict[str, Any], tuned: dict[str, Any]) -> dict[str, Any]:
    left = baseline.get("provenance", {})
    right = tuned.get("provenance", {})
    mismatches = {
        key: {"baseline": left.get(key), "tuned": right.get(key)}
        for key in COMPARABILITY_KEYS
        if left.get(key) != right.get(key)
    }
    if baseline.get("total") != tuned.get("total"):
        mismatches["total"] = {
            "baseline": baseline.get("total"),
            "tuned": tuned.get("total"),
        }
    if mismatches:
        raise ValueError(f"reports are not comparable: {json.dumps(mismatches, ensure_ascii=False)}")
    if left.get("model_file_sha256") == right.get("model_file_sha256"):
        raise ValueError("baseline and tuned reports point to the same model artifact")

    delta = {
        "strict_protocol_and_action_score": round(
            tuned["model_score"] - baseline["model_score"], 4
        ),
        "raw_action_score": round(tuned["action_score"] - baseline["action_score"], 4),
        "protocol_score": round(tuned["protocol_score"] - baseline["protocol_score"], 4),
        "effective_fallback_score": round(
            tuned["effective_score"] - baseline["effective_score"], 4
        ),
    }
    return {
        "schema_version": 1,
        "scenario": "baseline_vs_lora_q8_instruction_following",
        "comparable": True,
        "evaluation_contract": {
            key: left.get(key) for key in COMPARABILITY_KEYS
        }
        | {"total": baseline["total"]},
        "baseline": _metric_block(baseline),
        "tuned": _metric_block(tuned),
        "delta": delta,
        "interpretation": (
            "LoRA improved raw action semantics, while strict tagged-protocol and fallback scores "
            "must be reported separately. Synthetic holdout evidence is not production accuracy."
        ),
    }


def render_markdown(report: dict[str, Any]) -> str:
    baseline = report["baseline"]
    tuned = report["tuned"]
    delta = report["delta"]
    rows = (
        ("原始动作语义", "raw_action_score"),
        ("严格协议 + 动作", "strict_protocol_and_action_score"),
        ("标签协议完整", "protocol_score"),
        ("fallback 后工程出口", "effective_fallback_score"),
    )
    table = "\n".join(
        f"| {label} | {baseline[key]:.2%} | {tuned[key]:.2%} | {delta[key]:+.2%} |"
        for label, key in rows
    )
    contract = report["evaluation_contract"]
    return f"""# Qwen3-0.6B LoRA/Q8 指令遵循对照证据

| 指标 | 原始 Q8 | LoRA 合并 Q8 | 差值 |
|---|---:|---:|---:|
{table}

- 独立评估样本：{contract['total']} 条；dataset SHA256：`{contract['dataset_sha256']}`。
- 两侧使用相同 system prompt、temperature、max_tokens 和 seed；脚本发现不一致会拒绝比较。
- “原始动作语义”只比较模型 action 与期望动作；“严格协议 + 动作”还要求 `<speech>/<action>` 标签完整。
- fallback 分数是工程出口，不是模型准确率。本数据集为合成中文 holdout，不代表真实麦克风或生产分布。
- 原始模型 SHA256：`{baseline['model_file_sha256']}`。
- LoRA Q8 SHA256：`{tuned['model_file_sha256']}`。
"""


def _resolve(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else WORKSPACE / path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--tuned", required=True)
    parser.add_argument("--output", default="docs/evidence/lora_q8_instruction_comparison.json")
    parser.add_argument("--markdown", default="docs/evidence/lora_q8_instruction_comparison.md")
    args = parser.parse_args()
    try:
        report = compare_reports(_load(_resolve(args.baseline)), _load(_resolve(args.tuned)))
        output = _resolve(args.output)
        markdown = _resolve(args.markdown)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        markdown.parent.mkdir(parents=True, exist_ok=True)
        markdown.write_text(render_markdown(report), encoding="utf-8")
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}")
        return 1
    print(json.dumps({"status": "PASS", "output": str(output), "delta": report["delta"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
