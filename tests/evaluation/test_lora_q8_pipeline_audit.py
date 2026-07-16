from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "audit_lora_q8_pipeline", ROOT / "tools" / "evaluation" / "audit_lora_q8_pipeline.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_pipeline_audit_distinguishes_ready_from_reproduced(tmp_path) -> None:
    missing_f16 = tmp_path / "missing-f16.gguf"
    missing_q8 = tmp_path / "missing-q8.gguf"
    report = MODULE.build_report(
        missing_f16,
        missing_q8,
        tmp_path / "missing-baseline.gguf",
        tmp_path / "missing-comparison.json",
    )
    assert report["static_pipeline_ready"] is True
    assert report["status"] == "pipeline_ready_not_executed"
    assert report["reproduced_training"] is False


def test_static_pipeline_does_not_require_local_third_party_builds(tmp_path, monkeypatch) -> None:
    """干净检出应能验证仓库能力，外部工具是否安装必须单独报告。"""

    workspace = tmp_path / "workspace"
    tracked_inputs = (
        "training/qwen3_0_6b_lora.yaml",
        "training/qwen3_0_6b_lora_merge.yaml",
        "training/dataset_info.json",
        "training/robot_dialogue_train.jsonl",
        "training/robot_dialogue_train.meta.json",
        "training/robot_instruction_eval.jsonl",
        "training/system_prompt_sft_zh.txt",
        "src/embodied_agent_core/prompts/system_prompt_zh.txt",
        "scripts/build_robot_lora_dataset.py",
        "scripts/build_qwen_lora_q8.sh",
        "scripts/quantize_qwen_q8.sh",
        "tools/evaluation/compare_instruction_following_reports.py",
    )
    for relative_path in tracked_inputs:
        path = workspace / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}\n" if path.suffix == ".json" else "\n", encoding="utf-8")

    monkeypatch.setattr(MODULE, "WORKSPACE", workspace)
    report = MODULE.build_report(
        tmp_path / "missing-f16.gguf",
        tmp_path / "missing-q8.gguf",
        tmp_path / "missing-baseline.gguf",
        tmp_path / "missing-comparison.json",
    )

    assert report["static_pipeline_ready"] is True
    assert report["external_tools_ready"] is False
    assert report["missing_runtime_dependencies"] == ["llama_converter", "llama_quantize"]


def test_pipeline_audit_uses_actual_artifact_sizes(tmp_path) -> None:
    f16 = tmp_path / "model-f16.gguf"
    q8 = tmp_path / "model-q8.gguf"
    baseline = tmp_path / "baseline-q8.gguf"
    f16.write_bytes(b"GGUF" + bytes(996))
    q8.write_bytes(b"GGUF" + bytes(496))
    baseline.write_bytes(b"GGUF" + bytes(496))
    report = MODULE.build_report(f16, q8, baseline, tmp_path / "comparison.json")
    assert report["q8_to_f16_ratio"] == 0.5
    assert report["artifacts_verified"] is True
    assert report["reproduced_training"] is False
