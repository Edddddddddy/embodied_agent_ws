#!/usr/bin/env python3
"""Review LoRA candidates and export an approved training JSONL.

这个脚本是“离线模型训练证据链”的保护层：上一阶段会把 LLM 失败样例导出为
`robot_dialogue_lora_candidates.jsonl`，但候选样例不能直接进入训练集。原因是失败可能
来自 ASR 噪声、评测 prompt 波动或 parser 兜底行为；直接训练会把脏样本固化到模型里。

因此这里分两步：

1. `--init-review` 生成一个人工审核清单，每条候选默认 `needs_review`。
2. 审核人把可信样例标记为 `approved` 后，再导出 `robot_dialogue_lora_approved.jsonl`。

脚本只做数据治理，不宣称 LoRA 已经训练完成。
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[2]
VALID_DECISIONS = {"approved", "rejected", "needs_edit", "needs_review"}


def _resolve(path: str) -> Path:
    candidate = Path(path).expanduser()
    return candidate if candidate.is_absolute() else WORKSPACE / candidate


def _display_path(path: Path) -> str:
    try:
        return str(path.relative_to(WORKSPACE))
    except ValueError:
        return str(path)


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        row = json.loads(line)
        if not isinstance(row, dict):
            raise ValueError(f"{path}:{line_no} is not a JSON object")
        rows.append(row)
    return rows


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def _conversation_value(row: dict[str, Any], index: int) -> str:
    conversations = row.get("conversations") or []
    if not isinstance(conversations, list) or len(conversations) <= index:
        return ""
    value = conversations[index].get("value", "")
    return value if isinstance(value, str) else ""


def _source_case_id(row: dict[str, Any], fallback_index: int) -> str:
    metadata = row.get("metadata") or {}
    if isinstance(metadata, dict):
        value = metadata.get("source_case_id")
        if isinstance(value, str) and value:
            return value
    return f"candidate_{fallback_index:04d}"


def _build_review_template(candidates: list[dict[str, Any]], candidates_path: Path) -> dict[str, Any]:
    reviews: list[dict[str, Any]] = []
    for index, row in enumerate(candidates, start=1):
        metadata = row.get("metadata") if isinstance(row.get("metadata"), dict) else {}
        reviews.append(
            {
                "source_case_id": _source_case_id(row, index),
                "input": _conversation_value(row, 0),
                "assistant": _conversation_value(row, 1),
                "failure_type": metadata.get("failure_type", "") if isinstance(metadata, dict) else "",
                "raw_output": metadata.get("raw_output", "") if isinstance(metadata, dict) else "",
                # 默认不自动纳入训练，必须人工把 decision 改为 approved。
                "decision": "needs_review",
                "notes": "",
            }
        )
    return {
        "schema_version": 1,
        "scenario": "lora_candidate_review",
        "source_candidates": _display_path(candidates_path),
        "reviews": reviews,
    }


def _load_review(path: Path) -> dict[str, dict[str, str]]:
    doc = json.loads(path.read_text(encoding="utf-8"))
    if doc.get("scenario") != "lora_candidate_review":
        raise ValueError("review file scenario must be lora_candidate_review")
    reviews = doc.get("reviews")
    if not isinstance(reviews, list):
        raise ValueError("review file must contain a reviews list")

    by_case_id: dict[str, dict[str, str]] = {}
    for item in reviews:
        if not isinstance(item, dict):
            raise ValueError("each review item must be an object")
        case_id = item.get("source_case_id")
        decision = item.get("decision")
        if not isinstance(case_id, str) or not case_id:
            raise ValueError("each review item requires source_case_id")
        if decision not in VALID_DECISIONS:
            raise ValueError(
                f"invalid decision for {case_id}: {decision!r}; "
                f"expected one of {sorted(VALID_DECISIONS)}"
            )
        by_case_id[case_id] = {
            "decision": decision,
            "notes": item.get("notes", "") if isinstance(item.get("notes"), str) else "",
        }
    return by_case_id


def _export_approved(
    candidates: list[dict[str, Any]],
    review_by_case_id: dict[str, dict[str, str]],
) -> tuple[list[dict[str, Any]], Counter[str], list[str]]:
    approved: list[dict[str, Any]] = []
    decision_counts: Counter[str] = Counter()
    missing_review: list[str] = []

    for index, row in enumerate(candidates, start=1):
        case_id = _source_case_id(row, index)
        review = review_by_case_id.get(case_id)
        if review is None:
            missing_review.append(case_id)
            decision_counts["missing_review"] += 1
            continue

        decision = review["decision"]
        decision_counts[decision] += 1
        if decision != "approved":
            continue

        exported = dict(row)
        # 只浅拷贝顶层和 metadata：conversations 保持原训练样例内容不变，metadata 记录审核来源。
        metadata = dict(exported.get("metadata") or {})
        metadata["review_decision"] = decision
        metadata["review_notes"] = review["notes"]
        exported["metadata"] = metadata
        approved.append(exported)

    return approved, decision_counts, missing_review


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", default="training/robot_dialogue_lora_candidates.jsonl")
    parser.add_argument("--review", default="training/robot_dialogue_lora_review.json")
    parser.add_argument("--output", default="training/robot_dialogue_lora_approved.jsonl")
    parser.add_argument("--metadata-output", default="training/robot_dialogue_lora_approved.meta.json")
    parser.add_argument("--init-review", action="store_true")
    parser.add_argument("--fail-if-empty", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidates_path = _resolve(args.candidates)
    review_path = _resolve(args.review)
    output_path = _resolve(args.output)
    metadata_path = _resolve(args.metadata_output)

    candidates = _load_jsonl(candidates_path)

    if args.init_review:
        review_doc = _build_review_template(candidates, candidates_path)
        review_path.parent.mkdir(parents=True, exist_ok=True)
        review_path.write_text(
            json.dumps(review_doc, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(
            json.dumps(
                {
                    "status": "PASS",
                    "review": str(review_path),
                    "review_items": len(review_doc["reviews"]),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    review_by_case_id = _load_review(review_path)
    approved, decision_counts, missing_review = _export_approved(candidates, review_by_case_id)

    _write_jsonl(output_path, approved)
    metadata = {
        "schema_version": 1,
        "scenario": "lora_approved_dataset_export",
        "source_candidates": _display_path(candidates_path),
        "source_review": _display_path(review_path),
        "output": _display_path(output_path),
        "approved": len(approved),
        "decision_counts": dict(sorted(decision_counts.items())),
        "missing_review": missing_review,
        "review_required": False,
        "training_note": "该文件只是审核后的训练数据输入；LoRA 训练和量化仍需单独执行并记录 benchmark。",
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    ok = bool(approved) or not args.fail_if_empty
    print(
        json.dumps(
            {
                "status": "PASS" if ok else "FAIL",
                "output": str(output_path),
                "metadata_output": str(metadata_path),
                "approved": len(approved),
                "missing_review": missing_review,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
