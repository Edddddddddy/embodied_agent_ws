#!/usr/bin/env python3
"""Export offline LLM instruction-following failures as LoRA review candidates.

脚本只生成“候选集”，不会自动覆盖正式 `robot_dialogue_seed.jsonl`。原因很简单：
失败样例可能包含 ASR 噪声、prompt 问题或模型随机性，训练前必须人工审核。
输出格式沿用 LLaMA-Factory 的 ShareGPT JSONL，便于审核后合并进训练集。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[2]


def _resolve(path: str) -> Path:
    candidate = Path(path).expanduser()
    return candidate if candidate.is_absolute() else WORKSPACE / candidate


def _load_report(path: Path) -> dict[str, Any]:
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("scenario") != "offline_llm_instruction_following_eval":
        raise ValueError("report is not an offline_llm_instruction_following_eval report")
    return report


def _load_dataset_answers(path: Path) -> dict[str, dict[str, Any]]:
    answers: dict[str, dict[str, Any]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        conversations = row.get("conversations") or []
        if len(conversations) < 2:
            continue
        user = conversations[0].get("value", "")
        assistant = conversations[1].get("value", "")
        if user and assistant:
            answers[user] = row
    return answers


def _candidate_from_case(case: dict[str, Any], seed_row: dict[str, Any]) -> dict[str, Any]:
    conversations = seed_row["conversations"]
    return {
        "conversations": [
            {"from": "human", "value": conversations[0]["value"]},
            {"from": "gpt", "value": conversations[1]["value"]},
        ],
        "metadata": {
            "source": "instruction_following_failure",
            "source_case_id": case.get("id", ""),
            "failure_type": case.get("failure_type", ""),
            "expected_actions": case.get("expected_actions", []),
            "actual_actions": case.get("actual_actions", []),
            "effective_actions": case.get("effective_actions", []),
            "raw_output": case.get("raw_output", ""),
        },
    }


def export_candidates(report: dict[str, Any], dataset_rows: dict[str, dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    candidates: list[dict[str, Any]] = []
    skipped: list[dict[str, Any]] = []
    seen_inputs: set[str] = set()
    for case in report.get("failed_cases", []):
        user_text = case.get("input", "")
        if not user_text:
            skipped.append({"case": case, "reason": "missing_input"})
            continue
        if user_text in seen_inputs:
            skipped.append({"case": case, "reason": "duplicate_input"})
            continue
        seed_row = dataset_rows.get(user_text)
        if seed_row is None:
            skipped.append({"case": case, "reason": "not_found_in_dataset"})
            continue
        seen_inputs.add(user_text)
        candidates.append(_candidate_from_case(case, seed_row))
    return candidates, skipped


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def _failure_type_counts(cases: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for case in cases:
        failure_type = case.get("failure_type", "unknown") or "unknown"
        counts[failure_type] = counts.get(failure_type, 0) + 1
    return dict(sorted(counts.items()))


def _display_path(path: Path) -> str:
    try:
        return str(path.relative_to(WORKSPACE))
    except ValueError:
        return str(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", default="logs/instruction_following_report.json")
    parser.add_argument("--dataset", default="training/robot_dialogue_seed.jsonl")
    parser.add_argument("--output", default="training/robot_dialogue_lora_candidates.jsonl")
    parser.add_argument(
        "--metadata-output",
        default="training/robot_dialogue_lora_candidates.meta.json",
    )
    parser.add_argument("--fail-if-empty", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report_path = _resolve(args.report)
    dataset_path = _resolve(args.dataset)
    output_path = _resolve(args.output)
    metadata_path = _resolve(args.metadata_output)

    report = _load_report(report_path)
    dataset_rows = _load_dataset_answers(dataset_path)
    candidates, skipped = export_candidates(report, dataset_rows)

    _write_jsonl(output_path, candidates)
    metadata = {
        "schema_version": 1,
        "scenario": "instruction_following_lora_candidate_export",
        "source_report": _display_path(report_path),
        "source_dataset": _display_path(dataset_path),
        "output": _display_path(output_path),
        "exported": len(candidates),
        "skipped": len(skipped),
        "source_model_score": report.get("model_score"),
        "source_effective_score": report.get("effective_score"),
        "failure_type_counts": _failure_type_counts(report.get("failed_cases", [])),
        "skipped_cases": [
            {
                "id": item["case"].get("id", ""),
                "input": item["case"].get("input", ""),
                "reason": item["reason"],
            }
            for item in skipped
        ],
        "review_required": True,
        "review_note": "人工审核后再合并进正式 LoRA 训练集；不要直接把候选集当作已训练证据。",
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    ok = bool(candidates) or not args.fail_if_empty
    summary = {
        "status": "PASS" if ok else "FAIL",
        "output": str(output_path),
        "metadata_output": str(metadata_path),
        "exported": len(candidates),
        "skipped": len(skipped),
        "source_model_score": report.get("model_score"),
        "source_effective_score": report.get("effective_score"),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
