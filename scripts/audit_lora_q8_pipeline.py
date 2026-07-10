#!/usr/bin/env python3
"""Audit the reproducibility and artifacts of the LoRA -> GGUF -> Q8 pipeline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]


def _gguf_file(path: Path) -> dict[str, Any]:
    exists = path.is_file()
    size = path.stat().st_size if exists else 0
    magic = path.read_bytes()[:4] if exists and size >= 4 else b""
    return {"path": str(path), "exists": exists, "size_bytes": size, "gguf_magic": magic == b"GGUF"}


def build_report(f16_path: Path, q8_path: Path) -> dict[str, Any]:
    required = {
        "train_config": WORKSPACE / "training" / "qwen3_0_6b_lora.yaml",
        "merge_config": WORKSPACE / "training" / "qwen3_0_6b_lora_merge.yaml",
        "dataset_info": WORKSPACE / "training" / "dataset_info.json",
        "seed_dataset": WORKSPACE / "training" / "robot_dialogue_seed.jsonl",
        "pipeline": WORKSPACE / "scripts" / "build_qwen_lora_q8.sh",
        "quantizer_wrapper": WORKSPACE / "scripts" / "quantize_qwen_q8.sh",
        "llama_converter": WORKSPACE / "third_party" / "llama.cpp" / "convert_hf_to_gguf.py",
        "llama_quantize": WORKSPACE / "third_party" / "llama.cpp" / "build" / "bin" / "llama-quantize",
    }
    missing = [name for name, path in required.items() if not path.exists()]
    seed_count = 0
    if required["seed_dataset"].is_file():
        seed_count = sum(
            1 for line in required["seed_dataset"].read_text(encoding="utf-8").splitlines() if line.strip()
        )
    f16 = _gguf_file(f16_path)
    q8 = _gguf_file(q8_path)
    artifacts_verified = all(
        (f16["exists"], f16["gguf_magic"], q8["exists"], q8["gguf_magic"])
    )
    training_evidence_files = {
        "adapter_config": WORKSPACE / "outputs" / "qwen3-0.6b-robot-lora" / "adapter_config.json",
        "trainer_state": WORKSPACE / "outputs" / "qwen3-0.6b-robot-lora" / "trainer_state.json",
    }
    training_evidence = all(path.is_file() for path in training_evidence_files.values())
    reproduced_training = artifacts_verified and training_evidence
    ratio = q8["size_bytes"] / f16["size_bytes"] if artifacts_verified else None
    if reproduced_training:
        status = "training_and_artifacts_verified"
    elif artifacts_verified:
        status = "artifacts_verified_training_unproven"
    else:
        status = "pipeline_ready_not_executed"
    return {
        "schema_version": 1,
        "status": status,
        "static_pipeline_ready": not missing,
        "missing_static_dependencies": missing,
        "dataset": {
            "seed_samples": seed_count,
            "approved_dataset_exists": (WORKSPACE / "training" / "robot_dialogue_lora_approved.jsonl").is_file(),
            "quality_warning": "seed data validates format only; it does not prove instruction accuracy",
        },
        "artifacts": {"f16": f16, "q8_0": q8},
        "training_evidence": {
            name: {"path": str(path), "exists": path.is_file()}
            for name, path in training_evidence_files.items()
        },
        "artifacts_verified": artifacts_verified,
        "q8_to_f16_ratio": None if ratio is None else round(ratio, 4),
        "reproduced_training": reproduced_training,
        "claim_guidance": (
            "可以说：LoRA 合并、GGUF 转换和 Q8 量化产物已验证。"
            if reproduced_training
            else "只能说：流水线与配置已就绪；当前没有已验证的 LoRA/Q8 训练产物。"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--f16", type=Path, default=WORKSPACE / "outputs" / "qwen3-0.6b-robot-f16.gguf")
    parser.add_argument("--q8", type=Path, default=WORKSPACE / "models" / "Qwen3-0.6B-robot-Q8_0.gguf")
    parser.add_argument("--output", type=Path, default=WORKSPACE / "logs" / "lora_q8_pipeline_report.json")
    parser.add_argument("--strict-artifacts", action="store_true")
    parser.add_argument("--strict-reproduced", action="store_true")
    args = parser.parse_args()
    report = build_report(args.f16, args.q8)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["static_pipeline_ready"]:
        return 1
    if args.strict_artifacts and not report["reproduced_training"]:
        if not report["artifacts_verified"]:
            return 2
    if args.strict_reproduced and not report["reproduced_training"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
