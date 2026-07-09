import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "evaluate_instruction_following.py"


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
