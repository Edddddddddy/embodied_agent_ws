#!/usr/bin/env python3
"""Audit LoRA training, merge, GGUF/Q8 conversion and holdout evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _file(path: Path, *, gguf: bool = False) -> dict[str, Any]:
    exists = path.is_file()
    size = path.stat().st_size if exists else 0
    result: dict[str, Any] = {
        "path": str(path),
        "exists": exists,
        "size_bytes": size,
        "sha256": _sha256(path) if exists else None,
    }
    if gguf:
        if exists and size >= 4:
            with path.open("rb") as stream:
                result["gguf_magic"] = stream.read(4) == b"GGUF"
        else:
            result["gguf_magic"] = False
    return result


def _json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def _manifest_checks(manifest: dict[str, Any], files: dict[str, Path]) -> dict[str, bool]:
    return {
        "training_dataset_hash": manifest.get("training_sha256") == _sha256(files["training_dataset"]),
        "evaluation_dataset_hash": manifest.get("evaluation_sha256") == _sha256(files["evaluation_dataset"]),
        "training_prompt_hash": manifest.get("training_system_prompt_sha256")
        == _sha256(files["training_prompt"]),
        "runtime_prompt_hash": manifest.get("runtime_system_prompt_sha256")
        == _sha256(files["runtime_prompt"]),
        "no_exact_utterance_overlap": manifest.get("exact_utterance_overlap") == [],
    }


def _comparison_checks(
    comparison: dict[str, Any],
    *,
    baseline: dict[str, Any],
    tuned: dict[str, Any],
    evaluation_dataset: Path,
    runtime_prompt: Path,
) -> dict[str, bool]:
    contract = comparison.get("evaluation_contract", {})
    return {
        "scenario": comparison.get("scenario") == "baseline_vs_lora_q8_instruction_following",
        "comparable": comparison.get("comparable") is True,
        "baseline_model_hash": comparison.get("baseline", {}).get("model_file_sha256")
        == baseline.get("sha256"),
        "tuned_model_hash": comparison.get("tuned", {}).get("model_file_sha256")
        == tuned.get("sha256"),
        "evaluation_dataset_hash": contract.get("dataset_sha256") == _sha256(evaluation_dataset),
        "runtime_prompt_hash": contract.get("system_prompt_sha256") == _sha256(runtime_prompt),
        "non_empty_holdout": int(contract.get("total") or 0) > 0,
    }


def build_report(
    f16_path: Path,
    q8_path: Path,
    baseline_q8_path: Path,
    comparison_path: Path,
) -> dict[str, Any]:
    static_required = {
        "train_config": WORKSPACE / "training" / "qwen3_0_6b_lora.yaml",
        "merge_config": WORKSPACE / "training" / "qwen3_0_6b_lora_merge.yaml",
        "dataset_info": WORKSPACE / "training" / "dataset_info.json",
        "dataset_builder": WORKSPACE / "scripts" / "build_robot_lora_dataset.py",
        "training_dataset": WORKSPACE / "training" / "robot_dialogue_train.jsonl",
        "dataset_manifest": WORKSPACE / "training" / "robot_dialogue_train.meta.json",
        "evaluation_dataset": WORKSPACE / "training" / "robot_instruction_eval.jsonl",
        "training_prompt": WORKSPACE / "training" / "system_prompt_sft_zh.txt",
        "runtime_prompt": WORKSPACE / "src" / "embodied_agent_core" / "prompts" / "system_prompt_zh.txt",
        "pipeline": WORKSPACE / "scripts" / "build_qwen_lora_q8.sh",
        "quantizer_wrapper": WORKSPACE / "scripts" / "quantize_qwen_q8.sh",
        "comparison_script": WORKSPACE / "scripts" / "compare_instruction_following_reports.py",
    }
    runtime_required = {
        "llama_converter": WORKSPACE / "third_party" / "llama.cpp" / "convert_hf_to_gguf.py",
        "llama_quantize": WORKSPACE / "third_party" / "llama.cpp" / "build" / "bin" / "llama-quantize",
    }
    training_files = {
        "adapter_config": WORKSPACE / "outputs" / "qwen3-0.6b-robot-lora" / "adapter_config.json",
        "adapter_model": WORKSPACE / "outputs" / "qwen3-0.6b-robot-lora" / "adapter_model.safetensors",
        "trainer_state": WORKSPACE / "outputs" / "qwen3-0.6b-robot-lora" / "trainer_state.json",
        "train_results": WORKSPACE / "outputs" / "qwen3-0.6b-robot-lora" / "train_results.json",
        "eval_results": WORKSPACE / "outputs" / "qwen3-0.6b-robot-lora" / "eval_results.json",
        "merged_config": WORKSPACE / "outputs" / "qwen3-0.6b-robot-merged" / "config.json",
        "merged_model": WORKSPACE / "outputs" / "qwen3-0.6b-robot-merged" / "model.safetensors",
    }

    missing_static = [name for name, path in static_required.items() if not path.is_file()]
    missing_runtime = [name for name, path in runtime_required.items() if not path.exists()]
    missing_training = [name for name, path in training_files.items() if not path.is_file()]
    manifest = _json(static_required["dataset_manifest"])
    manifest_checks = (
        _manifest_checks(manifest, static_required) if not missing_static else {}
    )

    f16 = _file(f16_path, gguf=True)
    q8 = _file(q8_path, gguf=True)
    baseline = _file(baseline_q8_path, gguf=True)
    artifacts_verified = all(
        item.get("exists") and item.get("gguf_magic") for item in (f16, q8, baseline)
    )
    ratio = q8["size_bytes"] / f16["size_bytes"] if artifacts_verified else None

    comparison = _json(comparison_path)
    comparison_checks = (
        _comparison_checks(
            comparison,
            baseline=baseline,
            tuned=q8,
            evaluation_dataset=static_required["evaluation_dataset"],
            runtime_prompt=static_required["runtime_prompt"],
        )
        if comparison and artifacts_verified and not missing_static
        else {}
    )
    training_verified = not missing_training and bool(manifest_checks) and all(manifest_checks.values())
    comparison_verified = bool(comparison_checks) and all(comparison_checks.values())
    reproduced = artifacts_verified and training_verified and comparison_verified
    if reproduced:
        status = "training_quantization_and_holdout_verified"
    elif artifacts_verified and training_verified:
        status = "training_and_artifacts_verified_evaluation_unproven"
    elif artifacts_verified:
        status = "artifacts_verified_training_unproven"
    else:
        status = "pipeline_ready_not_executed"

    train_results = _json(training_files["train_results"])
    eval_results = _json(training_files["eval_results"])
    return {
        "schema_version": 2,
        "scenario": "lora_gguf_q8_reproducibility_audit",
        "status": status,
        "static_pipeline_ready": not missing_static,
        "missing_static_dependencies": missing_static,
        "external_tools_ready": not missing_runtime,
        "missing_runtime_dependencies": missing_runtime,
        "dataset": {
            "samples": manifest.get("sample_count", 0),
            "manifest": str(static_required["dataset_manifest"]),
            "checks": manifest_checks,
            "claim_boundary": manifest.get("claim_boundary", ""),
        },
        "training": {
            "evidence": {name: _file(path) for name, path in training_files.items()},
            "missing": missing_training,
            "metrics": {
                "epochs": train_results.get("epoch"),
                "train_loss": train_results.get("train_loss"),
                "eval_loss": eval_results.get("eval_loss"),
                "train_runtime_s": train_results.get("train_runtime"),
            },
            "verified": training_verified,
        },
        "artifacts": {"baseline_q8": baseline, "tuned_f16": f16, "tuned_q8_0": q8},
        "artifacts_verified": artifacts_verified,
        "q8_to_f16_ratio": None if ratio is None else round(ratio, 4),
        "q8_size_reduction": None if ratio is None else round(1.0 - ratio, 4),
        "holdout_comparison": {
            "path": str(comparison_path),
            "checks": comparison_checks,
            "verified": comparison_verified,
            "delta": comparison.get("delta", {}),
        },
        "reproduced_training": reproduced,
        "claim_guidance": (
            "可声明：LoRA 训练、合并、GGUF/Q8 量化与同口径独立 holdout 对照均有哈希绑定证据；"
            "合成评估不代表真实语音生产准确率。"
            if reproduced
            else "只能声明已通过的子阶段；缺失项和失败哈希检查必须先修复。"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--f16", type=Path, default=WORKSPACE / "outputs" / "qwen3-0.6b-robot-f16.gguf")
    parser.add_argument("--q8", type=Path, default=WORKSPACE / "models" / "Qwen3-0.6B-robot-Q8_0.gguf")
    parser.add_argument(
        "--baseline-q8", type=Path, default=WORKSPACE / "models" / "Qwen3-0.6B-Q8_0.gguf"
    )
    parser.add_argument(
        "--comparison",
        type=Path,
        default=WORKSPACE / "docs" / "evidence" / "lora_q8_instruction_comparison.json",
    )
    parser.add_argument("--output", type=Path, default=WORKSPACE / "logs" / "lora_q8_pipeline_report.json")
    parser.add_argument("--strict-artifacts", action="store_true")
    parser.add_argument("--strict-reproduced", action="store_true")
    args = parser.parse_args()
    report = build_report(args.f16, args.q8, args.baseline_q8, args.comparison)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["static_pipeline_ready"]:
        return 1
    if args.strict_artifacts and not report["artifacts_verified"]:
        return 2
    if args.strict_reproduced and not report["reproduced_training"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
