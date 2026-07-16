import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "evaluation" / "export_instruction_following_lora_candidates.py"


def _write_dataset(path: Path) -> None:
    rows = [
        {
            "conversations": [
                {"from": "human", "value": "左转九十度"},
                {
                    "from": "gpt",
                    "value": '<speech>好的，左转九十度。</speech><action>{"name":"turn","arguments":{"angular_z":0.6,"duration_s":2.6}}</action>',
                },
            ]
        },
        {
            "conversations": [
                {"from": "human", "value": "今天天气怎么样"},
                {"from": "gpt", "value": "<speech>当前是离线模式，我无法查询实时天气。</speech>"},
            ]
        },
    ]
    path.write_text(
        "\n".join(json.dumps(row, ensure_ascii=False) for row in rows) + "\n",
        encoding="utf-8",
    )


def _write_report(path: Path, dataset: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "scenario": "offline_llm_instruction_following_eval",
                "dataset": str(dataset),
                "model_score": 0.25,
                "effective_score": 1.0,
                "failed_cases": [
                    {
                        "id": "case_0001",
                        "input": "左转九十度",
                        "failure_type": "model_parse_error",
                        "expected_actions": [
                            {"name": "turn", "arguments": {"angular_z": 0.6, "duration_s": 2.6}}
                        ],
                        "actual_actions": [],
                        "raw_output": "好的，左转九十度。",
                    },
                    {
                        "id": "case_0002",
                        "input": "今天天气怎么样",
                        "failure_type": "model_parse_error",
                        "expected_actions": [],
                        "actual_actions": [],
                        "raw_output": "今天天气不错。",
                    },
                    {
                        "id": "case_missing",
                        "input": "数据集中没有这条",
                        "failure_type": "model_parse_error",
                        "expected_actions": [],
                        "actual_actions": [],
                    },
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def test_export_instruction_following_failures_to_lora_candidates(tmp_path):
    dataset = tmp_path / "robot_dialogue_seed.jsonl"
    report = tmp_path / "instruction_following_report.json"
    output = tmp_path / "robot_dialogue_lora_candidates.jsonl"
    metadata = tmp_path / "robot_dialogue_lora_candidates.meta.json"
    _write_dataset(dataset)
    _write_report(report, dataset)

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--report",
            str(report),
            "--dataset",
            str(dataset),
            "--output",
            str(output),
            "--metadata-output",
            str(metadata),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    summary = json.loads(completed.stdout)
    rows = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    meta = json.loads(metadata.read_text(encoding="utf-8"))

    assert summary["status"] == "PASS"
    assert summary["exported"] == 2
    assert summary["skipped"] == 1
    assert meta["source_model_score"] == 0.25
    assert meta["failure_type_counts"] == {"model_parse_error": 3}
    assert rows[0]["conversations"][0]["value"] == "左转九十度"
    assert rows[0]["conversations"][1]["value"].startswith("<speech>")
    assert rows[0]["metadata"]["source_case_id"] == "case_0001"
    assert rows[0]["metadata"]["failure_type"] == "model_parse_error"
