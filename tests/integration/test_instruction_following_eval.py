import json
import importlib.util
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "evaluate_instruction_following.py"
SPEC = importlib.util.spec_from_file_location("evaluate_instruction_following", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write_instruction_following_report(path: Path, *, model_score: float = 0.5) -> None:
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "scenario": "offline_llm_instruction_following_eval",
                "dataset": "training/robot_dialogue_seed.jsonl",
                "model": "Qwen3-0.6B-Q8_0.gguf",
                "base_url": "http://127.0.0.1:8080/v1",
                "model_passed": int(model_score * 4),
                "effective_passed": 3,
                "total": 4,
                "model_score": model_score,
                "effective_score": 0.75,
                "effective_score_policy": "action_only_after_fallback_and_safety",
                "failed_cases": [
                    {
                        "id": "case_0002",
                        "input": "左转九十度",
                        "failure_type": "model_action_mismatch",
                        "expected_actions": [
                            {"name": "turn", "arguments": {"angular_z": 0.6, "duration_s": 2.6}}
                        ],
                        "actual_actions": [],
                        "effective_actions": [
                            {"name": "turn", "arguments": {"angular_z": 0.6, "duration_s": 2.6}}
                        ],
                    }
                ],
                "cases": [],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def test_instruction_following_eval_reuses_existing_report(tmp_path):
    source = tmp_path / "source_report.json"
    output = tmp_path / "instruction_following_report.json"
    _write_instruction_following_report(source, model_score=0.5)

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input-report",
            str(source),
            "--output",
            str(output),
            "--minimum",
            "0.4",
            "--minimum-effective",
            "0.7",
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    summary = json.loads(completed.stdout)
    report = json.loads(output.read_text(encoding="utf-8"))
    assert summary["status"] == "PASS"
    assert summary["model_score"] == 0.5
    assert summary["effective_score"] == 0.75
    assert report["scenario"] == "offline_llm_instruction_following_eval"
    assert report["effective_score_policy"] == "action_only_after_fallback_and_safety"
    assert report["failed_cases"][0]["failure_type"] == "model_action_mismatch"


def test_instruction_following_eval_fails_when_model_score_below_threshold(tmp_path):
    source = tmp_path / "source_report.json"
    output = tmp_path / "instruction_following_report.json"
    _write_instruction_following_report(source, model_score=0.25)

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input-report",
            str(source),
            "--output",
            str(output),
            "--minimum",
            "0.7",
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    summary = json.loads(completed.stdout)
    assert completed.returncode == 1
    assert summary["status"] == "FAIL"
    assert summary["model_score"] == 0.25


def test_eval_loader_accepts_independent_action_dataset(tmp_path):
    dataset = tmp_path / "eval.jsonl"
    dataset.write_text(
        json.dumps(
            {
                "id": "holdout_move",
                "text": "请往前移动一秒钟",
                "expected_actions": [
                    {"name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}}
                ],
                "tags": ["move", "holdout"],
            },
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )
    samples = MODULE._iter_dialogue_samples(dataset)
    assert samples == [
        {
            "id": "holdout_move",
            "input": "请往前移动一秒钟",
            "expected_speech": "",
            "expected_actions": [
                {"name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}}
            ],
            "tags": ["move", "holdout"],
        }
    ]


def test_report_has_tag_level_model_and_effective_scores():
    cases = [
        {
            "id": "a",
            "input": "a",
            "tags": ["move"],
            "passed": True,
            "action_passed": True,
            "protocol_passed": True,
            "effective_passed": True,
            "failure_type": "",
        },
        {
            "id": "b",
            "input": "b",
            "tags": ["move", "noise"],
            "passed": False,
            "action_passed": True,
            "protocol_passed": False,
            "effective_passed": True,
            "failure_type": "model_action_mismatch",
        },
    ]
    report = MODULE.build_report(cases, dataset="eval.jsonl", model="model", base_url="url")
    assert report["schema_version"] == 2
    assert report["tag_metrics"]["move"]["model_score"] == 0.5
    assert report["tag_metrics"]["move"]["action_score"] == 1.0
    assert report["tag_metrics"]["move"]["protocol_score"] == 0.5
    assert report["tag_metrics"]["move"]["effective_score"] == 1.0
    assert report["tag_metrics"]["noise"]["model_score"] == 0.0


def test_thresholds_can_gate_action_and_protocol_separately():
    report = {"model_score": 0.2, "action_score": 0.8, "protocol_score": 0.5}
    assert MODULE._passes_thresholds(
        report,
        minimum=None,
        minimum_action=0.8,
        minimum_protocol=0.5,
        minimum_effective=None,
    )
    assert not MODULE._passes_thresholds(
        report,
        minimum=None,
        minimum_action=0.9,
        minimum_protocol=None,
        minimum_effective=None,
    )


def test_old_case_report_is_upgraded_without_new_model_call(tmp_path):
    path = tmp_path / "old.json"
    path.write_text(
        json.dumps(
            {
                "scenario": "offline_llm_instruction_following_eval",
                "dataset": "holdout.jsonl",
                "model": "baseline.gguf",
                "base_url": "http://localhost",
                "provenance": {"seed": 42},
                "cases": [
                    {
                        "id": "turn",
                        "input": "左转",
                        "tags": ["turn"],
                        "expected_actions": [{"name": "turn", "arguments": {}}],
                        "actual_actions": [{"name": "turn", "arguments": {}}],
                        "effective_actions": [{"name": "turn", "arguments": {}}],
                        "effective_passed": True,
                        "speech": "",
                        "errors": [],
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    report = MODULE._load_report(path)
    assert report["action_score"] == 1.0
    assert report["protocol_score"] == 0.0
    assert report["model_score"] == 0.0
