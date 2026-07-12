#!/usr/bin/env python3
"""Evaluate offline LLM instruction following on robot dialogue samples.

这份评估和 deterministic parser 评估刻意分开：

- deterministic parser 证明规则/NLU 出口能稳定解析；
- 本脚本证明 llama.cpp 后端本身是否能按项目协议输出 `<speech>/<action>`。

默认会真实调用本机 llama-server；单测或复盘时可用 `--input-report` 复用已有报告，
避免 CI 启动大模型。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]


def _ensure_import_paths() -> None:
    for relative in ("src/embodied_offline_agent", "src/embodied_online_agent"):
        path = str(WORKSPACE / relative)
        if path not in sys.path:
            sys.path.insert(0, path)


def _canonical(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 3)
    if isinstance(value, list):
        return [_canonical(item) for item in value]
    if isinstance(value, dict):
        return {key: _canonical(value[key]) for key in sorted(value)}
    return value


def _dict_equal(left: dict, right: dict, *, tolerance: float) -> bool:
    if set(left) != set(right):
        return False
    for key in left:
        left_value = left[key]
        right_value = right[key]
        if isinstance(left_value, (int, float)) and isinstance(right_value, (int, float)):
            if math.fabs(float(left_value) - float(right_value)) > tolerance:
                return False
            continue
        if left_value != right_value:
            return False
    return True


def _actions_equal(actual: list[dict], expected: list[dict], *, tolerance: float = 1e-3) -> bool:
    if len(actual) != len(expected):
        return False
    for left, right in zip(actual, expected):
        if left.get("name") != right.get("name"):
            return False
        if not _dict_equal(left.get("arguments", {}), right.get("arguments", {}), tolerance=tolerance):
            return False
    return True


def parse_output(text: str) -> dict[str, Any]:
    _ensure_import_paths()
    from embodied_agent_core.protocol import TaggedStreamParser

    parser = TaggedStreamParser()
    events = parser.feed(text)
    final = parser.finish()
    return {
        "speech": "".join(events.speech + final.speech).strip(),
        "actions": [action.as_dict() for action in events.actions + final.actions],
        "errors": events.errors + final.errors,
    }


def _effective_actions(user_text: str, actual_actions: list[dict]) -> list[dict]:
    _ensure_import_paths()
    from embodied_agent_core.command_fallback import (
        parse_fallback_actions,
        should_block_model_actions,
    )

    if should_block_model_actions(user_text):
        return []
    fallback_actions = parse_fallback_actions(user_text)
    if fallback_actions:
        return [action.as_dict() for action in fallback_actions]
    return list(actual_actions)


def _failure_type(
    actual: dict[str, Any],
    expected_actions: list[dict],
    *,
    model_passed: bool,
    effective_passed: bool,
) -> str:
    if model_passed and effective_passed:
        return ""
    if actual.get("errors"):
        return "model_parse_error"
    if not actual.get("speech"):
        return "model_no_speech"
    if not model_passed:
        return "model_action_mismatch"
    return "effective_action_mismatch"


def _iter_dialogue_samples(dataset: Path) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for index, line in enumerate(dataset.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        payload = json.loads(line)
        conversations = payload["conversations"]
        user_text = conversations[0]["value"]
        expected = parse_output(conversations[1]["value"])
        samples.append(
            {
                "id": payload.get("id", f"case_{index:04d}"),
                "input": user_text,
                "expected_speech": expected["speech"],
                "expected_actions": expected["actions"],
                "tags": payload.get("tags", []),
            }
        )
    return samples


def _load_llm(args: argparse.Namespace):
    _ensure_import_paths()
    from embodied_offline_agent.providers.llama_cpp import LlamaCppLlm

    return LlamaCppLlm(
        args.base_url,
        args.model,
        args.temperature,
        args.max_tokens,
        seed=args.seed,
        timeout_s=args.timeout_s,
        max_retries=args.max_retries,
    )


def run_evaluation(args: argparse.Namespace) -> dict[str, Any]:
    root = Path(args.workspace).expanduser()
    dataset = Path(args.dataset).expanduser()
    if not dataset.is_absolute():
        dataset = root / dataset
    system_prompt = Path(args.system_prompt).expanduser()
    if not system_prompt.is_absolute():
        system_prompt = root / system_prompt

    system = system_prompt.read_text(encoding="utf-8")
    llm = _load_llm(args)
    cases: list[dict[str, Any]] = []
    for sample in _iter_dialogue_samples(dataset):
        output = "".join(
            llm.stream(
                [
                    {"role": "system", "content": system + "\n/no_think"},
                    {"role": "user", "content": sample["input"] + " /no_think"},
                ]
            )
        )
        actual = parse_output(output)
        expected_actions = sample["expected_actions"]
        model_passed = (
            bool(actual["speech"])
            and not actual["errors"]
            and _actions_equal(actual["actions"], expected_actions)
        )
        effective_actions = _effective_actions(sample["input"], actual["actions"])
        # effective_score 衡量的是“工程出口动作是否正确”，因此只看 fallback/安全层
        # 兜底后的动作序列，不把模型是否生成合法 speech 标签混进去。模型协议严格性由
        # model_score 单独承担，避免两个指标互相污染。
        effective_passed = _actions_equal(effective_actions, expected_actions)
        failure_type = _failure_type(
            actual,
            expected_actions,
            model_passed=model_passed,
            effective_passed=effective_passed,
        )
        cases.append(
            {
                "id": sample["id"],
                "input": sample["input"],
                "tags": sample["tags"],
                "passed": model_passed,
                "effective_passed": effective_passed,
                "failure_type": failure_type,
                "expected_actions": _canonical(expected_actions),
                "actual_actions": _canonical(actual["actions"]),
                "effective_actions": _canonical(effective_actions),
                "speech": actual["speech"],
                "errors": actual["errors"],
                "raw_output": output,
                "llama_metrics": llm.last_metrics,
            }
        )
    return build_report(
        cases,
        dataset=str(dataset.relative_to(root) if dataset.is_relative_to(root) else dataset),
        model=args.model,
        base_url=args.base_url,
    )


def build_report(
    cases: list[dict[str, Any]],
    *,
    dataset: str,
    model: str,
    base_url: str,
) -> dict[str, Any]:
    total = len(cases)
    model_passed = sum(1 for item in cases if item.get("passed"))
    effective_passed = sum(1 for item in cases if item.get("effective_passed"))
    failed_cases = [
        {
            "id": item["id"],
            "input": item["input"],
            "tags": item.get("tags", []),
            "failure_type": item.get("failure_type", ""),
            "expected_actions": item.get("expected_actions", []),
            "actual_actions": item.get("actual_actions", []),
            "effective_actions": item.get("effective_actions", []),
            "errors": item.get("errors", []),
            "raw_output": item.get("raw_output", ""),
        }
        for item in cases
        if not item.get("passed") or not item.get("effective_passed")
    ]
    failure_counts: dict[str, int] = {}
    for item in failed_cases:
        failure_type = item.get("failure_type") or "unknown"
        failure_counts[failure_type] = failure_counts.get(failure_type, 0) + 1
    return {
        "schema_version": 1,
        "scenario": "offline_llm_instruction_following_eval",
        "dataset": dataset,
        "model": model,
        "base_url": base_url,
        "model_passed": model_passed,
        "effective_passed": effective_passed,
        "total": total,
        "model_score": round(model_passed / total, 4) if total else 0.0,
        "effective_score": round(effective_passed / total, 4) if total else 0.0,
        "effective_score_policy": "action_only_after_fallback_and_safety",
        "failure_counts": dict(sorted(failure_counts.items())),
        "failed_cases": failed_cases,
        "cases": cases,
    }


def _load_report(path: Path) -> dict[str, Any]:
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("scenario") != "offline_llm_instruction_following_eval":
        raise ValueError("input report is not an offline LLM instruction-following report")
    return report


def _write_report(report: dict[str, Any], output: str) -> Path:
    output_path = Path(output).expanduser()
    if not output_path.is_absolute():
        output_path = WORKSPACE / output_path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return output_path


def _passes_thresholds(
    report: dict[str, Any],
    *,
    minimum: float | None,
    minimum_effective: float | None,
) -> bool:
    ok = True
    if minimum is not None:
        ok = ok and float(report.get("model_score") or 0.0) >= minimum
    if minimum_effective is not None:
        ok = ok and float(report.get("effective_score") or 0.0) >= minimum_effective
    return ok


def _print_summary(report: dict[str, Any], output_path: Path | None, ok: bool) -> None:
    summary = {
        "status": "PASS" if ok else "FAIL",
        "output": str(output_path) if output_path else "",
        "dataset": report.get("dataset", ""),
        "model": report.get("model", ""),
        "model_score": report.get("model_score", 0.0),
        "effective_score": report.get("effective_score", 0.0),
        "model_passed": report.get("model_passed", 0),
        "effective_passed": report.get("effective_passed", 0),
        "total": report.get("total", 0),
        "failed_cases": len(report.get("failed_cases", [])),
        "failure_counts": report.get("failure_counts", {}),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=str(WORKSPACE))
    parser.add_argument("--dataset", default="training/robot_dialogue_seed.jsonl")
    parser.add_argument(
        "--system-prompt",
        default="src/embodied_agent_core/prompts/system_prompt_zh.txt",
    )
    parser.add_argument("--base-url", default="http://127.0.0.1:8080/v1")
    parser.add_argument("--model", default="Qwen3-0.6B-Q8_0.gguf")
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--max-tokens", type=int, default=192)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--timeout-s", type=float, default=60.0)
    parser.add_argument("--max-retries", type=int, default=0)
    parser.add_argument("--input-report", help="reuse an existing instruction-following report")
    parser.add_argument("--output", default="logs/instruction_following_report.json")
    parser.add_argument("--minimum", type=float)
    parser.add_argument("--minimum-effective", type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.input_report:
            report = _load_report(Path(args.input_report).expanduser())
        else:
            report = run_evaluation(args)
        output_path = _write_report(report, args.output) if args.output else None
        ok = _passes_thresholds(
            report,
            minimum=args.minimum,
            minimum_effective=args.minimum_effective,
        )
    except Exception as exc:
        report = {
            "schema_version": 1,
            "scenario": "offline_llm_instruction_following_eval",
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        output_path = _write_report(report, args.output) if args.output else None
        ok = False
    _print_summary(report, output_path, ok)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
