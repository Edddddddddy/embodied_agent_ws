#!/usr/bin/env python3
"""Evaluate deterministic robot command parsing on the instruction eval set."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from embodied_agent_core.command_completion import CommandCompleter
from embodied_agent_core.command_fallback import parse_fallback_actions, should_block_model_actions
from embodied_agent_core.command_nlu import CommandNLU
from embodied_agent_core.command_normalizer import CommandNormalizer


def _canonical(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 3)
    if isinstance(value, list):
        return [_canonical(item) for item in value]
    if isinstance(value, dict):
        return {key: _canonical(value[key]) for key in sorted(value)}
    return value


def _actions_equal(actual: list[dict], expected: list[dict], *, tolerance: float = 1e-3) -> bool:
    if len(actual) != len(expected):
        return False
    for left, right in zip(actual, expected):
        if left.get("name") != right.get("name"):
            return False
        if not _dict_equal(left.get("arguments", {}), right.get("arguments", {}), tolerance=tolerance):
            return False
    return True


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


def _parse_actions(text: str) -> tuple[list[dict], str, dict]:
    normalizer = CommandNormalizer()
    completer = CommandCompleter()
    nlu = CommandNLU()

    normalized = normalizer.normalize(text)
    completed = completer.complete(normalized.text)
    working_text = completed.text
    if should_block_model_actions(working_text):
        return [], "blocked", {
            "normalized": normalized.text,
            "completed": working_text,
            "nlu_reason": "skipped_by_safety_policy",
        }

    nlu_result = nlu.parse(working_text)
    if nlu_result.accepted:
        actions = [
            action.as_dict()
            for parsed in nlu_result.commands
            for action in parsed.actions
        ]
        return actions, "nlu", {
            "normalized": normalized.text,
            "completed": working_text,
            "nlu_reason": nlu_result.reason,
            "nlu_intents": [parsed.intent for parsed in nlu_result.commands],
            "nlu_slots": [parsed.slots for parsed in nlu_result.commands],
        }

    fallback_actions = parse_fallback_actions(working_text)
    return [action.as_dict() for action in fallback_actions], "fallback", {
        "normalized": normalized.text,
        "completed": working_text,
        "nlu_reason": nlu_result.reason,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset",
        default="/home/ubuntu/embodied_agent_ws/training/robot_instruction_eval.jsonl",
    )
    parser.add_argument("--output", default="")
    parser.add_argument("--minimum", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    dataset_path = Path(args.dataset)
    cases = []
    tag_stats: dict[str, dict[str, int]] = {}
    for line in dataset_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        sample = json.loads(line)
        actual_actions, source, debug = _parse_actions(sample["text"])
        expected_actions = sample["expected_actions"]
        passed = _actions_equal(actual_actions, expected_actions)
        for tag in sample.get("tags", []):
            stats = tag_stats.setdefault(tag, {"passed": 0, "total": 0})
            stats["total"] += 1
            stats["passed"] += int(passed)
        cases.append(
            {
                "id": sample["id"],
                "text": sample["text"],
                "tags": sample.get("tags", []),
                "source": source,
                "passed": passed,
                "expected_actions": _canonical(expected_actions),
                "actual_actions": _canonical(actual_actions),
                **debug,
            }
        )
    passed_count = sum(1 for item in cases if item["passed"])
    total = len(cases)
    source_counts: dict[str, int] = {}
    for item in cases:
        source_counts[item["source"]] = source_counts.get(item["source"], 0) + 1
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
        "dataset": str(dataset_path),
        "passed": passed_count,
        "total": total,
        "accuracy": round(passed_count / total, 4) if total else 0.0,
        "source_counts": dict(sorted(source_counts.items())),
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
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report["accuracy"] < args.minimum:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
