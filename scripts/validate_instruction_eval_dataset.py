#!/usr/bin/env python3
"""Validate the lightweight robot instruction evaluation dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


ALLOWED_ACTIONS = {
    "move",
    "turn",
    "stop",
    "arc",
    "wave",
    "set_led",
    "set_mode",
    "navigate_to",
    "follow_waypoints",
    "cancel_navigation",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset",
        default="/home/ubuntu/embodied_agent_ws/training/robot_instruction_eval.jsonl",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    path = Path(args.dataset)
    seen_ids: set[str] = set()
    tag_counts: dict[str, int] = {}
    records = 0
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        payload = json.loads(line)
        records += 1
        sample_id = payload.get("id")
        assert isinstance(sample_id, str) and sample_id, f"line {line_no}: missing id"
        assert sample_id not in seen_ids, f"line {line_no}: duplicate id {sample_id}"
        seen_ids.add(sample_id)
        assert isinstance(payload.get("text"), str) and payload["text"], f"line {line_no}: missing text"
        actions = payload.get("expected_actions")
        assert isinstance(actions, list), f"line {line_no}: expected_actions must be list"
        tags = payload.get("tags")
        assert isinstance(tags, list) and tags, f"line {line_no}: tags must be non-empty"
        for tag in tags:
            assert isinstance(tag, str) and tag, f"line {line_no}: invalid tag"
            tag_counts[tag] = tag_counts.get(tag, 0) + 1
        for action in actions:
            assert isinstance(action, dict), f"line {line_no}: action must be object"
            name = action.get("name")
            assert name in ALLOWED_ACTIONS, f"line {line_no}: unsupported action {name!r}"
            assert isinstance(action.get("arguments", {}), dict), (
                f"line {line_no}: arguments must be object"
            )
    assert records >= 6, "dataset should include at least six seed eval cases"
    print(
        json.dumps(
            {"status": "PASS", "records": records, "tag_counts": tag_counts},
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
