#!/usr/bin/env python3
"""ASR/NLU 现场采样日志到评估候选集的转换验收。"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "evaluation" / "asr_nlu_samples_to_eval_candidates.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("asr_nlu_samples_to_eval_candidates", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules["asr_nlu_samples_to_eval_candidates"] = module
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(
        "\n".join(json.dumps(row, ensure_ascii=False) for row in rows) + "\n",
        encoding="utf-8",
    )


def test_real_asr_sample_log_becomes_reviewable_eval_candidate(tmp_path):
    """一条真实 ASR final 里的多命令，应沉淀成带 actions 的待审核样本。"""

    module = _load_module()
    sample_log = tmp_path / "asr_nlu_samples.jsonl"
    output = tmp_path / "eval_candidates.jsonl"
    _write_jsonl(
        sample_log,
        [
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
                "batch_id": "batch-1",
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
                "request_id": "volatile-runtime-id",
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
        ],
    )

    summary = module.convert_file(sample_log, output)

    rows = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    assert summary["candidates"] == 1
    assert rows[0]["text"] == "向右转然后向前走一秒"
    assert rows[0]["expected_actions"] == [
        {"name": "turn", "arguments": {"angular_z": -0.8, "duration_s": 1.4}},
        {"name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}},
    ]
    assert rows[0]["suggested_eval_case"]["id"].startswith("real_asr_")
    assert rows[0]["suggested_eval_case"]["expected_actions"] == rows[0]["expected_actions"]
    assert {"real_asr", "needs_review", "multi_command", "nlu_parsed"} <= set(rows[0]["tags"])
    assert rows[0]["results"][0]["success"] is True


def test_cli_can_generate_synthetic_demo_candidates(tmp_path):
    """验收入口可以无真实麦克风日志运行，便于 CI/本地快速验证工具可用。"""

    output = tmp_path / "synthetic_candidates.jsonl"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--synthetic-demo",
            "--output",
            str(output),
        ],
        text=True,
        capture_output=True,
        check=True,
    )

    summary = json.loads(result.stdout)
    rows = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    assert summary["status"] == "PASS"
    assert summary["candidates"] >= 1
    assert rows
    assert "needs_review" in rows[0]["tags"]
