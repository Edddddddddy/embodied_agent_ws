import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_offline_showcase_report_generates_json_and_markdown(tmp_path):
    json_output = tmp_path / "offline_showcase_report.json"
    md_output = tmp_path / "offline_showcase_report.md"
    completed = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts" / "generate_offline_showcase_report.py"),
            "--json-output",
            str(json_output),
            "--md-output",
            str(md_output),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout
    report = json.loads(json_output.read_text(encoding="utf-8"))
    markdown = md_output.read_text(encoding="utf-8")

    assert report["scenario"] == "offline_deployment_showcase"
    assert "model_inventory" in report
    assert "runtime_versions" in report
    assert "instruction_parser" in report
    assert report["instruction_parser"]["total"] >= 24
    assert report["instruction_parser"]["accuracy"] >= 0.95
    assert "离线端侧部署展示报告" in markdown
    assert "模型资产" in markdown
    assert "指令解析评估" in markdown


def test_offline_showcase_report_classifies_claim_evidence(tmp_path):
    json_output = tmp_path / "offline_showcase_report.json"
    md_output = tmp_path / "offline_showcase_report.md"
    completed = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts" / "generate_offline_showcase_report.py"),
            "--json-output",
            str(json_output),
            "--md-output",
            str(md_output),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout
    report = json.loads(json_output.read_text(encoding="utf-8"))
    markdown = md_output.read_text(encoding="utf-8")

    claims = report["claim_evidence"]
    by_key = {item["key"]: item for item in claims["items"]}

    assert claims["summary"]["proven"] >= 1
    assert by_key["q8_gguf_model"]["status"] == "proven"
    assert by_key["lora_training"]["status"] == "not_reproduced"
    assert by_key["llama_decode_speed"]["status"] == "missing"
    assert by_key["llm_first_token_latency"]["status"] == "missing"
    assert by_key["sherpa_tts_first_audio"]["status"] == "missing"
    assert by_key["deterministic_parser_accuracy"]["status"] == "proven"
    assert any("LoRA" in item for item in claims["restricted_claims"])
    assert "指标证据矩阵" in markdown
    assert "`lora_training`" in markdown
    assert "not_reproduced" in markdown
