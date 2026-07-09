#!/usr/bin/env python3
"""Convert live ASR/NLU monitor samples into reviewable eval-set candidates.

`continuous_voice_monitor.py --sample-output` 会把真实麦克风演示中的 ASR final、
归一化/补全/NLU feedback、动作候选和执行结果写成 JSONL。这个脚本把这些运行时事件
整理成接近 `training/robot_instruction_eval.jsonl` schema 的候选样本，方便人工审核后
沉淀到回归评估集。
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable


VOLATILE_ACTION_KEYS = {
    "request_id",
    "command_id",
    "batch_id",
    "sequence",
    "schema_version",
    "topic",
    "ts",
    "kind",
}


SYNTHETIC_EVENTS: list[dict[str, Any]] = [
    {
        "schema_version": 1,
        "sequence": 1,
        "kind": "asr_final",
        "topic": "/agent/asr_final",
        "text": "向右转然后向前走一秒",
    },
    {
        "schema_version": 1,
        "sequence": 2,
        "kind": "recognition_feedback",
        "topic": "/agent/recognition_feedback",
        "status": "nlu_parsed",
        "batch_id": "synthetic-demo",
        "commands": [
            {"intent": "turn_right", "span_text": "向右转"},
            {"intent": "move_forward", "span_text": "向前走一秒"},
        ],
    },
    {
        "schema_version": 1,
        "sequence": 3,
        "kind": "action_candidate",
        "topic": "/agent/action_candidate",
        "name": "turn",
        "arguments": {"angular_z": -0.8, "duration_s": 1.4},
    },
    {
        "schema_version": 1,
        "sequence": 4,
        "kind": "action_candidate",
        "topic": "/agent/action_candidate",
        "name": "move",
        "arguments": {"linear_x": 0.2, "duration_s": 1.0},
    },
    {
        "schema_version": 1,
        "sequence": 5,
        "kind": "action_result",
        "topic": "/robot/action_result",
        "success": True,
        "message": "succeeded",
    },
]


@dataclass
class SampleGroup:
    text: str
    source_log: str
    sample_index: int
    source_sequences: list[int] = field(default_factory=list)
    feedback: list[dict[str, Any]] = field(default_factory=list)
    queue_events: list[dict[str, Any]] = field(default_factory=list)
    action_candidates: list[dict[str, Any]] = field(default_factory=list)
    results: list[dict[str, Any]] = field(default_factory=list)

    def append_event(self, event: dict[str, Any]) -> None:
        sequence = event.get("sequence")
        if isinstance(sequence, int):
            self.source_sequences.append(sequence)
        kind = event.get("kind")
        if kind == "recognition_feedback":
            self.feedback.append(_strip_runtime_fields(event))
        elif kind == "command_queue":
            self.queue_events.append(_strip_runtime_fields(event))
        elif kind == "action_candidate":
            self.action_candidates.append(_sanitize_action(event))
        elif kind in {"action_result", "action_ack"}:
            self.results.append(_strip_runtime_fields(event))

    def to_candidate(self) -> dict[str, Any]:
        expected_actions = [
            action for action in self.action_candidates if action.get("name")
        ]
        tags = _candidate_tags(self.feedback, expected_actions, self.results)
        candidate_id = _candidate_id(self.text, expected_actions, self.sample_index)
        suggested_eval_case = {
            "id": candidate_id,
            "text": self.text,
            "expected_actions": expected_actions,
            "tags": tags,
        }
        return {
            "schema_version": 1,
            "id": candidate_id,
            "source_log": self.source_log,
            "sample_index": self.sample_index,
            "source_sequences": self.source_sequences,
            "text": self.text,
            "expected_actions": expected_actions,
            "tags": tags,
            "recognition_feedback": self.feedback,
            "queue_events": self.queue_events,
            "results": self.results,
            "suggested_eval_case": suggested_eval_case,
        }


def _strip_runtime_fields(event: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in event.items()
        if key not in {"schema_version", "sequence", "ts", "kind", "topic"}
    }


def _sanitize_action(event: dict[str, Any]) -> dict[str, Any]:
    """只保留可进入评估集的动作语义，过滤 request_id 等运行时关联字段。"""

    action: dict[str, Any] = {
        key: value
        for key, value in event.items()
        if key not in VOLATILE_ACTION_KEYS
    }
    arguments = action.get("arguments", {})
    action["arguments"] = arguments if isinstance(arguments, dict) else {}
    return action


def _candidate_tags(
    feedback: list[dict[str, Any]],
    expected_actions: list[dict[str, Any]],
    results: list[dict[str, Any]],
) -> list[str]:
    tags = {"real_asr", "needs_review"}
    if len(expected_actions) > 1:
        tags.add("multi_command")
    if not expected_actions:
        tags.add("no_action_candidate")
    for action in expected_actions:
        name = action.get("name")
        if isinstance(name, str) and name:
            tags.add(name)
    for item in feedback:
        status = item.get("status")
        reason = item.get("reason")
        if status in {"nlu_parsed", "normalized", "completed", "ignored"}:
            tags.add(status)
        if reason == "completed_missing_slot":
            tags.add("completed_missing_slot")
    if any(result.get("success") is False for result in results):
        tags.add("failed_execution")
    return sorted(tags)


def _candidate_id(text: str, expected_actions: list[dict[str, Any]], sample_index: int) -> str:
    payload = json.dumps(
        {"text": text, "expected_actions": expected_actions},
        ensure_ascii=False,
        sort_keys=True,
    )
    digest = hashlib.sha1(payload.encode("utf-8")).hexdigest()[:8]
    return f"real_asr_{sample_index:03d}_{digest}"


def _read_jsonl(path: Path) -> tuple[list[dict[str, Any]], int]:
    events: list[dict[str, Any]] = []
    skipped = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            skipped += 1
            continue
        if isinstance(payload, dict):
            events.append(payload)
        else:
            skipped += 1
    return events, skipped


def _group_events(events: Iterable[dict[str, Any]], *, source_log: str) -> list[SampleGroup]:
    groups: list[SampleGroup] = []
    current: SampleGroup | None = None
    sample_index = 0
    for event in events:
        kind = event.get("kind")
        if kind == "asr_final":
            if current is not None:
                groups.append(current)
            text = str(event.get("text", "")).strip()
            sample_index += 1
            current = SampleGroup(text=text, source_log=source_log, sample_index=sample_index)
            current.append_event(event)
            continue
        if current is None:
            continue
        current.append_event(event)
    if current is not None:
        groups.append(current)
    return [group for group in groups if group.text]


def convert_events(
    events: Iterable[dict[str, Any]],
    output_path: Path,
    *,
    source_log: str,
    skipped_lines: int = 0,
) -> dict[str, Any]:
    groups = _group_events(events, source_log=source_log)
    candidates = [group.to_candidate() for group in groups]
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        "".join(
            json.dumps(candidate, ensure_ascii=False, sort_keys=True) + "\n"
            for candidate in candidates
        ),
        encoding="utf-8",
    )
    tag_counts: dict[str, int] = {}
    for candidate in candidates:
        for tag in candidate["tags"]:
            tag_counts[tag] = tag_counts.get(tag, 0) + 1
    return {
        "status": "PASS",
        "source_log": source_log,
        "output": str(output_path),
        "candidates": len(candidates),
        "skipped_lines": skipped_lines,
        "tag_counts": dict(sorted(tag_counts.items())),
    }


def convert_file(input_path: str | Path, output_path: str | Path) -> dict[str, Any]:
    path = Path(input_path)
    events, skipped = _read_jsonl(path)
    return convert_events(
        events,
        Path(output_path),
        source_log=str(path),
        skipped_lines=skipped,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        default="logs/asr_nlu_samples.jsonl",
        help="continuous monitor 采集的 JSONL；配合 --synthetic-demo 时可省略。",
    )
    parser.add_argument(
        "--output",
        default="logs/asr_nlu_eval_candidates.jsonl",
        help="输出待人工审核的 eval candidate JSONL。",
    )
    parser.add_argument(
        "--synthetic-demo",
        action="store_true",
        help="使用内置多命令样本生成候选集，便于 CI/验收不依赖真实麦克风日志。",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = Path(args.output)
    if args.synthetic_demo:
        summary = convert_events(
            SYNTHETIC_EVENTS,
            output_path,
            source_log="synthetic-demo",
        )
    else:
        summary = convert_file(Path(args.input), output_path)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
