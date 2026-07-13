#!/usr/bin/env python3
"""Evaluate parser accuracy on reviewable ASR/NLU eval candidates.

正式评估集 `training/robot_instruction_eval.jsonl` 需要人工审核后再合入；但真实麦克风现场
采到的 `logs/asr_nlu_eval_candidates.jsonl` 往往需要马上看 parser 是否还能识别。这个脚本
复用 `evaluate_instruction_parser.py` 的解析和比较逻辑，对候选集先做临时 accuracy 统计。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = SCRIPT_DIR.parent
for import_path in (
    SCRIPT_DIR,
    WORKSPACE / "src" / "embodied_online_agent",
):
    path_text = str(import_path)
    if path_text not in sys.path:
        sys.path.insert(0, path_text)

from evaluate_instruction_parser import _actions_equal, _canonical, _parse_actions  # noqa: E402


def _read_candidate_cases(path: Path) -> tuple[list[dict[str, Any]], int]:
    cases: list[dict[str, Any]] = []
    skipped = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            skipped += 1
            continue
        if not isinstance(payload, dict):
            skipped += 1
            continue
        # 转换脚本输出的候选包含 suggested_eval_case；手工整理时也允许直接写成 eval schema。
        case = payload.get("suggested_eval_case")
        if not isinstance(case, dict):
            case = payload
        if not isinstance(case.get("text"), str) or not isinstance(
            case.get("expected_actions"), list
        ):
            skipped += 1
            continue
        cases.append(
            {
                "id": str(case.get("id") or payload.get("id") or f"candidate_{len(cases)+1:03d}"),
                "text": case["text"],
                "expected_actions": case["expected_actions"],
                "tags": list(case.get("tags") or payload.get("tags") or []),
                "source_log": payload.get("source_log", ""),
                "review_state": "needs_review"
                if "needs_review" in set(case.get("tags") or payload.get("tags") or [])
                else "unknown",
            }
        )
    return cases, skipped


def _action_names(actions: list[dict[str, Any]]) -> list[str]:
    return [str(action.get("name") or "") for action in actions if action.get("name")]


def _classify_failure(
    *,
    expected_actions: list[dict[str, Any]],
    actual_actions: list[dict[str, Any]],
    tags: list[str],
    source: str,
    nlu_reason: str,
) -> str:
    if not actual_actions and expected_actions:
        return "no_action_produced"
    if actual_actions and not expected_actions:
        return "unexpected_action_produced"
    expected_names = _action_names(expected_actions)
    actual_names = _action_names(actual_actions)
    if len(expected_actions) > 1 and len(actual_actions) != len(expected_actions):
        return "multi_command_count_mismatch"
    if expected_names != actual_names:
        return "action_name_mismatch"
    if expected_actions != actual_actions:
        return "action_argument_mismatch"
    if "needs_review" in set(tags):
        return "review_label_inconsistent"
    if source == "none" or nlu_reason == "low_confidence":
        return "low_confidence_fallback_needed"
    return "unknown_mismatch"


def _improvement_plan(failure_counts: dict[str, int], tag_stats: dict[str, dict[str, int]]) -> list[dict[str, Any]]:
    plan: list[dict[str, Any]] = []
    if failure_counts.get("no_action_produced"):
        plan.append(
            {
                "area": "coverage",
                "reason": "no_action_produced",
                "next_action": "把对应 ASR final 加入 command_normalization 或 CommandNLU 训练/规则样本。",
            }
        )
    if failure_counts.get("multi_command_count_mismatch"):
        plan.append(
            {
                "area": "multi_command",
                "reason": "multi_command_count_mismatch",
                "next_action": "补充分隔词、顺序词和多动作 span 样本，优先覆盖“然后/再/先…最后…”。",
            }
        )
    if failure_counts.get("action_name_mismatch"):
        plan.append(
            {
                "area": "intent",
                "reason": "action_name_mismatch",
                "next_action": "检查意图词表和同义词归一化，必要时把失败样本加入正式 eval 集。",
            }
        )
    if failure_counts.get("action_argument_mismatch"):
        plan.append(
            {
                "area": "slots",
                "reason": "action_argument_mismatch",
                "next_action": "检查时长、角度、速度 slot 解析和短命令补全默认值。",
            }
        )
    weak_tags = [
        tag
        for tag, stats in sorted(tag_stats.items())
        if stats["total"] and stats["passed"] < stats["total"]
    ]
    if weak_tags:
        plan.append(
            {
                "area": "eval_dataset",
                "reason": "weak_tags",
                "tags": weak_tags,
                "next_action": "优先复核这些 tag 下的真实 ASR 样本，确认 expected_actions 后合入训练/回归集。",
            }
        )
    if not plan:
        plan.append(
            {
                "area": "maintenance",
                "reason": "no_current_failures",
                "next_action": "继续采集真实 ASR final，定期运行 asr-nlu-candidate-eval 防止回归。",
            }
        )
    return plan


def evaluate_candidates(path: str | Path, *, output: str | Path = "") -> dict[str, Any]:
    input_path = Path(path)
    samples, skipped = _read_candidate_cases(input_path)
    cases: list[dict[str, Any]] = []
    tag_stats: dict[str, dict[str, int]] = {}
    review_state_counts: dict[str, int] = {}
    for sample in samples:
        actual_actions, source, debug = _parse_actions(sample["text"])
        expected_actions = sample["expected_actions"]
        passed = _actions_equal(actual_actions, expected_actions)
        review_state_counts[sample["review_state"]] = (
            review_state_counts.get(sample["review_state"], 0) + 1
        )
        for tag in sample.get("tags", []):
            stats = tag_stats.setdefault(tag, {"passed": 0, "total": 0})
            stats["total"] += 1
            stats["passed"] += int(passed)
        cases.append(
            {
                "id": sample["id"],
                "text": sample["text"],
                "tags": sample.get("tags", []),
                "review_state": sample["review_state"],
                "source": source,
                "passed": passed,
                "expected_actions": _canonical(expected_actions),
                "actual_actions": _canonical(actual_actions),
                **debug,
            }
        )
    passed_count = sum(1 for item in cases if item["passed"])
    total = len(cases)
    failed_cases = [
        {
            "id": item["id"],
            "text": item["text"],
            "tags": item["tags"],
            "source": item["source"],
            "expected_actions": item["expected_actions"],
            "actual_actions": item["actual_actions"],
            "nlu_reason": item.get("nlu_reason", ""),
            "failure_type": _classify_failure(
                expected_actions=item["expected_actions"],
                actual_actions=item["actual_actions"],
                tags=item["tags"],
                source=item["source"],
                nlu_reason=item.get("nlu_reason", ""),
            ),
        }
        for item in cases
        if not item["passed"]
    ]
    failure_counts: dict[str, int] = {}
    for item in failed_cases:
        failure_type = item["failure_type"]
        failure_counts[failure_type] = failure_counts.get(failure_type, 0) + 1
    tag_accuracy = {
        tag: {
            "passed": stats["passed"],
            "total": stats["total"],
            "accuracy": round(stats["passed"] / stats["total"], 4),
        }
        for tag, stats in sorted(tag_stats.items())
    }
    report = {
        "schema_version": 1,
        "input": str(input_path),
        "passed": passed_count,
        "total": total,
        "accuracy": round(passed_count / total, 4) if total else 0.0,
        "skipped_lines": skipped,
        "review_state_counts": dict(sorted(review_state_counts.items())),
        "tag_accuracy": tag_accuracy,
        "failure_analysis": {
            "failure_counts": dict(sorted(failure_counts.items())),
            "improvement_plan": _improvement_plan(failure_counts, tag_stats),
        },
        "failed_cases": failed_cases,
        "cases": cases,
    }
    if output:
        output_path = Path(output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        default="logs/asr_nlu_eval_candidates.jsonl",
        help="由 asr_nlu_samples_to_eval_candidates.py 生成的候选集 JSONL。",
    )
    parser.add_argument(
        "--output",
        default="logs/asr_nlu_candidate_eval_report.json",
        help="输出 parser accuracy 报告 JSON。",
    )
    parser.add_argument("--minimum", type=float, default=0.8)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    report = evaluate_candidates(args.input, output=args.output)
    summary = {
        "status": "PASS" if report["accuracy"] >= args.minimum else "FAIL",
        "input": report["input"],
        "output": args.output,
        "passed": report["passed"],
        "total": report["total"],
        "accuracy": report["accuracy"],
        "skipped_lines": report["skipped_lines"],
        "failed_cases": report["failed_cases"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if report["accuracy"] < args.minimum:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
