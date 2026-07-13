import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "review_lora_candidates.py"


def _candidate(case_id: str, text: str) -> dict:
    return {
        "conversations": [
            {"from": "human", "value": text},
            {"from": "gpt", "value": f"<speech>{text}</speech>"},
        ],
        "metadata": {
            "source_case_id": case_id,
            "failure_type": "model_parse_error",
            "raw_output": text,
        },
    }


def _write_candidates(path: Path) -> None:
    rows = [
        _candidate("case_0001", "马上停下"),
        _candidate("case_0002", "左转九十度"),
        _candidate("case_0003", "今天天气怎么样"),
    ]
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def test_review_lora_candidates_initializes_review_template(tmp_path):
    candidates = tmp_path / "candidates.jsonl"
    review = tmp_path / "review.json"
    _write_candidates(candidates)

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--candidates",
            str(candidates),
            "--review",
            str(review),
            "--init-review",
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    summary = json.loads(completed.stdout)
    review_doc = json.loads(review.read_text(encoding="utf-8"))
    assert summary["status"] == "PASS"
    assert summary["review_items"] == 3
    assert review_doc["schema_version"] == 1
    assert {item["decision"] for item in review_doc["reviews"]} == {"needs_review"}
    assert review_doc["reviews"][0]["source_case_id"] == "case_0001"


def test_review_lora_candidates_exports_only_approved_rows(tmp_path):
    candidates = tmp_path / "candidates.jsonl"
    review = tmp_path / "review.json"
    output = tmp_path / "approved.jsonl"
    metadata = tmp_path / "approved.meta.json"
    _write_candidates(candidates)
    review.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "scenario": "lora_candidate_review",
                "reviews": [
                    {"source_case_id": "case_0001", "decision": "approved", "notes": "ok"},
                    {"source_case_id": "case_0002", "decision": "rejected", "notes": "duplicate"},
                    {"source_case_id": "case_0003", "decision": "needs_edit", "notes": "rewrite"},
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--candidates",
            str(candidates),
            "--review",
            str(review),
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
    assert summary["approved"] == 1
    assert rows[0]["metadata"]["source_case_id"] == "case_0001"
    assert rows[0]["metadata"]["review_decision"] == "approved"
    assert rows[0]["metadata"]["review_notes"] == "ok"
    assert meta["decision_counts"] == {"approved": 1, "needs_edit": 1, "rejected": 1}
    assert meta["review_required"] is False
