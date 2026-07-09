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
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

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
        }
        for item in cases
        if not item["passed"]
    ]
    report = {
        "schema_version": 1,
        "input": str(input_path),
        "passed": passed_count,
        "total": total,
        "accuracy": round(passed_count / total, 4) if total else 0.0,
        "skipped_lines": skipped,
        "review_state_counts": dict(sorted(review_state_counts.items())),
        "tag_accuracy": {
            tag: {
                "passed": stats["passed"],
                "total": stats["total"],
                "accuracy": round(stats["passed"] / stats["total"], 4),
            }
            for tag, stats in sorted(tag_stats.items())
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
