#!/usr/bin/env python3
"""Generate a reproducible offline-deployment showcase report.

这份报告把“端侧离线部署能力”从 README 叙述落到可复查证据：

1. 模型资产是否存在、体积是多少；
2. llama.cpp / SummerTTS / sherpa-onnx 版本是否符合固定版本；
3. 确定性指令解析评估集的准确率、失败样例和来源统计；
4. 可选地运行真实 llama.cpp 首 token、TTS 首音频、Sherpa ASR/TTS benchmark。

默认模式刻意不启动大模型服务，适合 CI 和日常快速复盘；真实演示前再打开重型测量开关。
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]
MODEL_FILES = (
    (
        "llm_qwen3_0_6b_q8",
        WORKSPACE / "models" / "Qwen3-0.6B-Q8_0.gguf",
        "Qwen3-0.6B Q8 GGUF",
    ),
    (
        "asr_zipformer_encoder",
        WORKSPACE
        / "models"
        / "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
        / "encoder-epoch-99-avg-1.int8.onnx",
        "Sherpa-ONNX ZipFormer encoder int8",
    ),
    (
        "asr_zipformer_decoder",
        WORKSPACE
        / "models"
        / "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
        / "decoder-epoch-99-avg-1.int8.onnx",
        "Sherpa-ONNX ZipFormer decoder int8",
    ),
    (
        "asr_zipformer_joiner",
        WORKSPACE
        / "models"
        / "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
        / "joiner-epoch-99-avg-1.int8.onnx",
        "Sherpa-ONNX ZipFormer joiner int8",
    ),
    (
        "tts_sherpa_vits",
        WORKSPACE / "models" / "vits-melo-tts-zh_en" / "model.onnx",
        "Sherpa VITS TTS ONNX",
    ),
    (
        "tts_summer_fast",
        WORKSPACE
        / "third_party"
        / "SummerTTS"
        / "models"
        / "single_speaker_fast.bin",
        "SummerTTS single speaker fast model",
    ),
)


def _ensure_import_paths() -> None:
    for relative in (
        "scripts",
        "src/embodied_online_agent",
        "src/embodied_offline_agent",
    ):
        path = str(WORKSPACE / relative)
        if path not in sys.path:
            sys.path.insert(0, path)


def _file_item(key: str, path: Path, description: str) -> dict[str, Any]:
    exists = path.is_file()
    size_bytes = path.stat().st_size if exists else 0
    return {
        "key": key,
        "description": description,
        "path": str(path.relative_to(WORKSPACE)),
        "exists": exists,
        "size_bytes": size_bytes,
        "size_mb": round(size_bytes / 1024 / 1024, 2),
    }


def collect_model_inventory() -> dict[str, Any]:
    items = [_file_item(key, path, description) for key, path, description in MODEL_FILES]
    return {
        "ok": all(item["exists"] for item in items),
        "total_size_mb": round(sum(item["size_bytes"] for item in items) / 1024 / 1024, 2),
        "items": items,
    }


def collect_runtime_versions() -> dict[str, Any]:
    _ensure_import_paths()
    from offline_runtime_versions import collect

    return collect()


def evaluate_instruction_parser(dataset: Path) -> dict[str, Any]:
    _ensure_import_paths()
    from evaluate_instruction_parser import _actions_equal, _canonical, _parse_actions

    cases = []
    source_counts: dict[str, int] = {}
    tag_stats: dict[str, dict[str, int]] = {}
    for line in dataset.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        sample = json.loads(line)
        actual_actions, source, debug = _parse_actions(sample["text"])
        expected_actions = sample["expected_actions"]
        passed = _actions_equal(actual_actions, expected_actions)
        source_counts[source] = source_counts.get(source, 0) + 1
        for tag in sample.get("tags", []):
            stats = tag_stats.setdefault(tag, {"passed": 0, "total": 0})
            stats["total"] += 1
            stats["passed"] += int(passed)
        cases.append(
            {
                "id": sample["id"],
                "text": sample["text"],
                "tags": sample.get("tags", []),
                "source": source,
                "passed": passed,
                "expected_actions": _canonical(expected_actions),
                "actual_actions": _canonical(actual_actions),
                "normalized": debug.get("normalized", ""),
                "completed": debug.get("completed", ""),
                "nlu_reason": debug.get("nlu_reason", ""),
            }
        )

    passed_count = sum(1 for item in cases if item["passed"])
    total = len(cases)
    return {
        "dataset": str(dataset.relative_to(WORKSPACE)),
        "passed": passed_count,
        "total": total,
        "accuracy": round(passed_count / total, 4) if total else 0.0,
        "source_counts": dict(sorted(source_counts.items())),
        "tag_accuracy": {
            tag: {
                "passed": stats["passed"],
                "total": stats["total"],
                "accuracy": round(stats["passed"] / stats["total"], 4),
            }
            for tag, stats in sorted(tag_stats.items())
        },
        "failed_cases": [item for item in cases if not item["passed"]],
    }


def _run_json_command(command: list[str], *, timeout_s: float) -> dict[str, Any]:
    env = os.environ.copy()
    python_path = os.pathsep.join(
        [
            str(WORKSPACE / "src" / "embodied_offline_agent"),
            str(WORKSPACE / "src" / "embodied_online_agent"),
            env.get("PYTHONPATH", ""),
        ]
    )
    env["PYTHONPATH"] = python_path
    completed = subprocess.run(
        command,
        cwd=WORKSPACE,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
        check=False,
    )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError:
        payload = {"raw_output_tail": completed.stdout.splitlines()[-80:]}
    return {
        "command": command,
        "returncode": completed.returncode,
        "ok": completed.returncode == 0,
        "payload": payload,
    }


def collect_latency(run_latency: bool, timeout_s: float) -> dict[str, Any]:
    if not run_latency:
        return {
            "status": "not_run",
            "reason": "pass --run-latency to start/check llama.cpp and measure first token/TTS",
        }
    return _run_json_command(
        ["bash", "scripts/smoke_test_offline_latency.sh"],
        timeout_s=timeout_s,
    )


def collect_asr_tts_benchmark(run_asr_tts: bool, timeout_s: float) -> dict[str, Any]:
    if not run_asr_tts:
        return {
            "status": "not_run",
            "reason": "pass --run-asr-tts to run Sherpa ASR/TTS benchmark",
        }
    return _run_json_command([sys.executable, "scripts/benchmark_offline.py"], timeout_s=timeout_s)


def collect_voice_e2e(
    run_voice_e2e: bool,
    timeout_s: float,
    input_report: str | None,
    output_report: str,
) -> dict[str, Any]:
    """Collect metrics from the actual ASR -> LLM -> pseudo-streaming TTS pipeline."""
    if not run_voice_e2e:
        return {
            "status": "not_run",
            "reason": "pass --run-voice-e2e to measure the real offline Agent pipeline",
        }

    if input_report:
        input_path = Path(input_report).expanduser()
        if not input_path.is_absolute():
            input_path = WORKSPACE / input_path
        payload = json.loads(input_path.read_text(encoding="utf-8"))
        return {
            "command": ["reuse", str(input_path)],
            "returncode": 0,
            "ok": payload.get("ok") is True,
            "payload": payload,
        }

    output_path = Path(output_report).expanduser()
    if not output_path.is_absolute():
        output_path = WORKSPACE / output_path
    # 避免本轮执行失败时误读上一次成功报告，导致证据看起来仍然有效。
    output_path.unlink(missing_ok=True)
    env = os.environ.copy()
    env["OFFLINE_VOICE_E2E_REPORT"] = str(output_path)
    command = ["bash", "scripts/smoke_test_offline_voice_real.sh"]
    completed = subprocess.run(
        command,
        cwd=WORKSPACE,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
        check=False,
    )
    if output_path.is_file():
        payload = json.loads(output_path.read_text(encoding="utf-8"))
    else:
        payload = {"raw_output_tail": completed.stdout.splitlines()[-80:]}
    return {
        "command": command,
        "returncode": completed.returncode,
        "ok": completed.returncode == 0 and payload.get("ok") is True,
        "payload": payload,
    }


def collect_llama_decode_benchmark(
    run_llama_bench: bool,
    timeout_s: float,
    llama_bench_input: str | None,
    llama_bench_output: str,
) -> dict[str, Any]:
    if not run_llama_bench:
        return {
            "status": "not_run",
            "reason": "pass --run-llama-bench to measure llama.cpp decode tokens/s",
        }
    command = [sys.executable, "scripts/benchmark_llama_decode_speed.py"]
    if llama_bench_input:
        command.extend(["--input-json", llama_bench_input])
    if llama_bench_output:
        command.extend(["--output", llama_bench_output])
    return _run_json_command(command, timeout_s=timeout_s)


def collect_instruction_following(
    run_instruction_following: bool,
    timeout_s: float,
    input_report: str | None,
    output_report: str,
) -> dict[str, Any]:
    if not run_instruction_following:
        return {
            "status": "not_run",
            "reason": "pass --run-instruction-following to evaluate offline LLM instruction following",
        }
    command = [sys.executable, "scripts/evaluate_instruction_following.py"]
    if input_report:
        command.extend(["--input-report", input_report])
    if output_report:
        command.extend(["--output", output_report])
    return _run_json_command(command, timeout_s=timeout_s)


def _inventory_item(models: dict[str, Any], key: str) -> dict[str, Any] | None:
    for item in models.get("items", []):
        if item.get("key") == key:
            return item
    return None


def _command_payload(block: dict[str, Any]) -> dict[str, Any]:
    payload = block.get("payload")
    return payload if isinstance(payload, dict) else {}


def _executed_ok(block: dict[str, Any]) -> bool:
    if block.get("status") == "not_run":
        return False
    if block.get("ok") is True:
        return True
    payload = _command_payload(block)
    return block.get("returncode") == 0 and payload.get("ok") is True


def _claim(
    key: str,
    title: str,
    status: str,
    evidence: str,
    *,
    metric: Any = None,
    caveat: str = "",
    next_step: str = "",
) -> dict[str, Any]:
    item = {
        "key": key,
        "title": title,
        "status": status,
        "evidence": evidence,
    }
    if metric is not None:
        item["metric"] = metric
    if caveat:
        item["caveat"] = caveat
    if next_step:
        item["next_step"] = next_step
    return item


def build_claim_evidence(report: dict[str, Any]) -> dict[str, Any]:
    """把“可宣称能力”按证据强度分级，避免汇报时把接口接入误说成实测指标。

    离线端侧项目最容易被追问的是：模型是否真的在本机部署、速度是否实测、
    LoRA/量化/准确率是否复现。这里不改变任何功能链路，只把证据边界结构化输出。
    """

    models = report["model_inventory"]
    parser = report["instruction_parser"]
    latency = report["latency"]
    asr_tts = report["asr_tts_benchmark"]
    llama_bench = report["llama_decode_benchmark"]
    instruction_following = report["instruction_following"]
    voice_e2e = report["voice_e2e"]
    q8_model = _inventory_item(models, "llm_qwen3_0_6b_q8")
    tts_model = _inventory_item(models, "tts_sherpa_vits")

    latency_payload = _command_payload(latency)
    latency_ok = _executed_ok(latency)
    llm_latency = latency_payload.get("llm") if isinstance(latency_payload.get("llm"), dict) else {}
    tts_latency = latency_payload.get("tts") if isinstance(latency_payload.get("tts"), dict) else {}

    asr_tts_payload = _command_payload(asr_tts)
    asr_tts_ok = _executed_ok(asr_tts)
    llama_bench_payload = _command_payload(llama_bench)
    llama_bench_ok = _executed_ok(llama_bench)
    instruction_payload = _command_payload(instruction_following)
    instruction_ok = _executed_ok(instruction_following)
    voice_e2e_payload = _command_payload(voice_e2e)
    voice_e2e_ok = _executed_ok(voice_e2e) and voice_e2e_payload.get("ok") is True
    voice_metrics = voice_e2e_payload.get("metrics")
    if not isinstance(voice_metrics, dict):
        voice_metrics = {}
    voice_tts_metrics = voice_metrics.get("tts_pipeline")
    if not isinstance(voice_tts_metrics, dict):
        voice_tts_metrics = {}
    instruction_failed_cases = instruction_payload.get("failed_cases")
    if isinstance(instruction_failed_cases, list):
        instruction_failed_count = len(instruction_failed_cases)
    elif isinstance(instruction_failed_cases, int):
        instruction_failed_count = instruction_failed_cases
    else:
        instruction_failed_count = 0

    parser_ok = parser.get("accuracy", 0.0) >= 0.95 and not parser.get("failed_cases")

    items = [
        _claim(
            "q8_gguf_model",
            "Qwen3-0.6B Q8 GGUF 模型资产",
            "proven" if q8_model and q8_model.get("exists") else "missing",
            "model_inventory",
            metric={
                "path": (q8_model or {}).get("path", "models/Qwen3-0.6B-Q8_0.gguf"),
                "size_mb": (q8_model or {}).get("size_mb", 0),
            },
            caveat="证明当前仓库/机器具备 Q8 GGUF 离线推理资产，不等同于 LoRA 训练效果已复现。",
        ),
        _claim(
            "lora_training",
            "LLaMA-Factory LoRA 微调复现",
            "not_reproduced",
            "no_training_run_attached",
            caveat="当前报告没有训练日志、checkpoint、eval result，因此不能宣称 LoRA 训练指标已完成复现。",
            next_step="运行 LLaMA-Factory 训练并归档 config、checkpoint、eval JSON 和训练日志。",
        ),
        _claim(
            "llama_decode_speed",
            "llama.cpp CPU decode tokens/s",
            "proven"
            if llama_bench_ok and llama_bench_payload.get("decode_tokens_per_s") is not None
            else "missing",
            "llama-bench" if llama_bench_ok else "not_measured",
            metric={
                "decode_tokens_per_s": llama_bench_payload.get("decode_tokens_per_s"),
                "prompt_tokens_per_s": llama_bench_payload.get("prompt_tokens_per_s"),
                "minimum_decode_tokens_per_s": llama_bench_payload.get(
                    "minimum_decode_tokens_per_s"
                ),
                "n_threads": llama_bench_payload.get("n_threads"),
                "n_gpu_layers": llama_bench_payload.get("n_gpu_layers"),
            }
            if llama_bench_payload
            else None,
            caveat="该指标来自 llama.cpp 自带 llama-bench，和真实 Agent 长上下文、多轮对话吞吐仍有差异。",
            next_step="运行 bash scripts/acceptance_test.sh llama-decode-benchmark 或 --run-llama-bench。",
        ),
        _claim(
            "llm_first_token_latency",
            "LLM 首 token 延迟",
            "proven" if latency_ok and llm_latency.get("first_token_ms") is not None else "missing",
            "offline_latency_targets" if latency_ok else "not_measured",
            metric={
                "first_token_ms": llm_latency.get("first_token_ms"),
                "target_ms": llm_latency.get("target_ms"),
            }
            if llm_latency
            else None,
            next_step="运行 python3 scripts/generate_offline_showcase_report.py --run-latency。",
        ),
        _claim(
            "sherpa_tts_first_audio",
            "Sherpa-TTS 伪流式首音频延迟",
            "proven"
            if voice_e2e_ok
            and voice_tts_metrics.get("first_text_to_first_audio_ms") is not None
            else "missing",
            "offline_voice_e2e" if voice_e2e_ok else "not_measured",
            metric={
                "provider": "sherpa",
                "first_text_to_first_audio_ms": voice_tts_metrics.get(
                    "first_text_to_first_audio_ms"
                ),
                "measurement": "first_sentence_enqueued_to_first_pcm_chunk_published",
            }
            if voice_tts_metrics
            else None,
            caveat="这是实际伪流式管线的首块 PCM，不是整句 synthesize() 完成耗时。",
            next_step="运行 --run-voice-e2e，并保持默认 tts-provider=sherpa。",
        ),
        _claim(
            "sherpa_tts_full_synthesis",
            "Sherpa-TTS 短句整句合成耗时",
            "proven"
            if latency_ok
            and tts_latency.get("provider") == "sherpa"
            and tts_latency.get("synthesis_ms") is not None
            else "missing",
            "offline_latency_targets" if latency_ok else "not_measured",
            metric={
                "provider": tts_latency.get("provider"),
                "synthesis_ms": tts_latency.get("synthesis_ms"),
                "target_ms": tts_latency.get("target_ms"),
                "measurement_kind": tts_latency.get("measurement_kind"),
            }
            if tts_latency
            else None,
            caveat="provider 返回整句 PCM，因此这里只能证明短句整句合成耗时，不是首音频。",
            next_step="运行 offline-latency，并保持默认 tts-provider=sherpa。",
        ),
        _claim(
            "offline_voice_e2e",
            "离线语音 Agent 端到首音频延迟",
            "proven"
            if voice_e2e_ok
            and voice_metrics.get("end_to_first_audio_ms") is not None
            else "missing",
            "offline_voice_e2e" if voice_e2e_ok else "not_measured",
            metric={
                "asr_finalize_ms": voice_metrics.get("asr_finalize_ms"),
                "llm_first_token_ms": voice_metrics.get("llm_first_token_ms"),
                "end_to_first_audio_ms": voice_metrics.get("end_to_first_audio_ms"),
                "turn_complete_ms": voice_metrics.get("turn_complete_ms"),
                "e2e_target_met": voice_metrics.get("e2e_target_met"),
                "message_buffer_dropped": voice_metrics.get("message_buffer_dropped"),
                "audio_buffer_dropped": voice_metrics.get("audio_buffer_dropped"),
            }
            if voice_metrics
            else None,
            caveat="从 speech endpoint 到第一块 TTS PCM 发布，来自真实 Agent 并行管线。",
            next_step="运行 bash scripts/acceptance_test.sh offline-voice-e2e-report。",
        ),
        _claim(
            "asr_tts_realtime_factor",
            "Sherpa ASR/TTS realtime factor",
            "proven" if asr_tts_ok else "missing",
            "benchmark_offline" if asr_tts_ok else "not_measured",
            metric={
                "asr_realtime_factor": asr_tts_payload.get("asr_realtime_factor"),
                "tts_realtime_factor": asr_tts_payload.get("tts_realtime_factor"),
            }
            if asr_tts_payload
            else None,
            next_step="运行 python3 scripts/generate_offline_showcase_report.py --run-asr-tts。",
        ),
        _claim(
            "deterministic_parser_accuracy",
            "确定性动作解析评估准确率",
            "proven" if parser_ok else "failed",
            "training/robot_instruction_eval.jsonl",
            metric={
                "passed": parser.get("passed", 0),
                "total": parser.get("total", 0),
                "accuracy": parser.get("accuracy", 0.0),
            },
            caveat="证明当前确定性 parser/轻量 NLU 在代表集上的能力，不等同于离线 LLM 指令遵循准确率。",
        ),
        _claim(
            "offline_llm_instruction_following",
            "离线 LLM 指令遵循准确率",
            "proven"
            if instruction_ok and instruction_payload.get("model_score") is not None
            else "missing",
            "evaluate_instruction_following"
            if instruction_ok
            else "not_measured",
            metric={
                "model_score": instruction_payload.get("model_score"),
                "effective_score": instruction_payload.get("effective_score"),
                "model_passed": instruction_payload.get("model_passed"),
                "effective_passed": instruction_payload.get("effective_passed"),
                "total": instruction_payload.get("total"),
                "failed_cases": instruction_failed_count,
                "effective_score_policy": instruction_payload.get("effective_score_policy"),
            }
            if instruction_payload
            else None,
            caveat="model_score 只看离线 LLM 原始协议输出；effective_score 只看 fallback/安全层兜底后的动作序列。",
            next_step="运行 bash scripts/acceptance_test.sh instruction-following-eval 或 --run-instruction-following。",
        ),
        _claim(
            "summertts_low_latency",
            "SummerTTS 默认低延迟能力",
            "not_default",
            "summertts_service_smoke",
            metric={
                "model_present": bool(_inventory_item(models, "tts_summer_fast") or {}),
                "sherpa_tts_model_present": bool(tts_model and tts_model.get("exists")),
            },
            caveat="SummerTTS 已完成服务化接入，但当前低延迟默认链路仍以 Sherpa-TTS 为主。",
        ),
    ]

    summary: dict[str, int] = {}
    for item in items:
        status = item["status"]
        summary[status] = summary.get(status, 0) + 1

    allowed_claims = [
        "可以说：当前离线链路具备模型资产清单、运行时版本和确定性指令解析评估证据。",
        (
            "可以说：确定性 parser 在当前代表集上达到 "
            f"{parser.get('passed', 0)}/{parser.get('total', 0)}，准确率 {parser.get('accuracy', 0.0):.4f}。"
        ),
    ]
    if q8_model and q8_model.get("exists"):
        allowed_claims.append(
            f"可以说：Q8 GGUF 模型资产已就绪，大小约 {q8_model.get('size_mb', 0)} MB。"
        )
    if latency_ok:
        allowed_claims.append("可以说：LLM/TTS 延迟已有本机真实测量证据。")
    if llama_bench_ok and llama_bench_payload.get("decode_tokens_per_s") is not None:
        allowed_claims.append(
            "可以说：llama.cpp CPU decode speed 已通过 llama-bench 本机测量，"
            f"decode≈{llama_bench_payload.get('decode_tokens_per_s')} tokens/s。"
        )
    if asr_tts_ok:
        allowed_claims.append("可以说：Sherpa ASR/TTS realtime factor 已在本机测量。")
    if instruction_ok and instruction_payload.get("model_score") is not None:
        allowed_claims.append(
            "可以说：离线 LLM 指令遵循已有本机评估报告，"
            f"model_score={instruction_payload.get('model_score')}，"
            f"effective_score={instruction_payload.get('effective_score')}。"
        )
    if voice_e2e_ok:
        allowed_claims.append(
            "可以说：离线 ASR→LLM→伪流式 TTS 的端到首音频延迟已有真实 Agent 指标，"
            f"end_to_first_audio≈{voice_metrics.get('end_to_first_audio_ms')} ms。"
        )

    restricted_claims = [
        "不要说：LoRA 微调训练、checkpoint 和训练后准确率已经复现；当前报告没有这类证据。",
        "不要说：Q8 指令遵循精度约 85% 已复现；除非补充离线 LLM 指令评估报告。",
        "不要说：SummerTTS 是默认 <300ms TTS；当前低延迟默认仍以 Sherpa-TTS 为主。",
    ]
    if not llama_bench_ok:
        restricted_claims.insert(
            2,
            "不要说：llama.cpp CPU decode 已达到某个 tokens/s；除非报告中出现真实 llama-bench benchmark。",
        )
    if not instruction_ok:
        restricted_claims.insert(
            2,
            "不要说：离线 LLM 指令遵循准确率已经复现；除非报告中出现 instruction-following eval。",
        )

    return {
        "schema_version": 1,
        "summary": summary,
        "items": items,
        "allowed_claims": allowed_claims,
        "restricted_claims": restricted_claims,
    }


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    dataset = Path(args.dataset).expanduser()
    if not dataset.is_absolute():
        dataset = WORKSPACE / dataset
    report = {
        "schema_version": 1,
        "scenario": "offline_deployment_showcase",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "workspace": str(WORKSPACE),
        "model_inventory": collect_model_inventory(),
        "runtime_versions": collect_runtime_versions(),
        "instruction_parser": evaluate_instruction_parser(dataset),
        "latency": collect_latency(args.run_latency, args.timeout_s),
        "asr_tts_benchmark": collect_asr_tts_benchmark(args.run_asr_tts, args.timeout_s),
        "llama_decode_benchmark": collect_llama_decode_benchmark(
            args.run_llama_bench,
            args.timeout_s,
            args.llama_bench_input,
            args.llama_bench_output,
        ),
        "instruction_following": collect_instruction_following(
            args.run_instruction_following,
            args.timeout_s,
            args.instruction_following_input,
            args.instruction_following_output,
        ),
        "voice_e2e": collect_voice_e2e(
            args.run_voice_e2e,
            args.timeout_s,
            args.voice_e2e_input,
            args.voice_e2e_output,
        ),
    }
    report["claim_evidence"] = build_claim_evidence(report)
    return report


def _ok_badge(ok: bool) -> str:
    return "PASS" if ok else "CHECK"


def render_markdown(report: dict[str, Any]) -> str:
    models = report["model_inventory"]
    versions = report["runtime_versions"]
    parser = report["instruction_parser"]
    lines = [
        "# 离线端侧部署展示报告",
        "",
        f"- 生成时间：`{report['generated_at']}`",
        f"- 模型资产：{_ok_badge(bool(models['ok']))}，总大小 `{models['total_size_mb']} MB`",
        f"- 运行时版本：{_ok_badge(bool(versions['ok']))}",
        (
            "- 指令解析："
            f"`{parser['passed']}/{parser['total']}`，准确率 `{parser['accuracy']}`"
        ),
        "",
        "## 1. 模型资产",
        "",
        "| Key | 说明 | 存在 | 大小(MB) | 路径 |",
        "| --- | --- | --- | ---: | --- |",
    ]
    for item in models["items"]:
        lines.append(
            f"| `{item['key']}` | {item['description']} | {item['exists']} | "
            f"{item['size_mb']} | `{item['path']}` |"
        )

    lines.extend(
        [
            "",
            "## 2. 运行时版本",
            "",
            "| 组件 | 期望 | 实际 | 状态 |",
            "| --- | --- | --- | --- |",
        ]
    )
    for item in versions["items"]:
        lines.append(
            f"| {item['name']} | `{item['expected']}` | `{item['actual']}` | "
            f"{_ok_badge(bool(item['ok']))} |"
        )

    lines.extend(
        [
            "",
            "## 3. 指令解析评估",
            "",
            f"- 数据集：`{parser['dataset']}`",
            f"- 总数：`{parser['total']}`",
            f"- 通过：`{parser['passed']}`",
            f"- 准确率：`{parser['accuracy']}`",
            f"- 来源统计：`{json.dumps(parser['source_counts'], ensure_ascii=False)}`",
            f"- 失败样例数：`{len(parser['failed_cases'])}`",
            "",
            "## 4. 延迟与 ASR/TTS benchmark",
            "",
            f"- 延迟测量：`{report['latency'].get('status', 'executed')}`",
            f"- llama.cpp decode benchmark：`{report['llama_decode_benchmark'].get('status', 'executed')}`",
            f"- 离线 LLM 指令遵循评估：`{report['instruction_following'].get('status', 'executed')}`",
            f"- ASR/TTS benchmark：`{report['asr_tts_benchmark'].get('status', 'executed')}`",
            f"- 真实离线语音 E2E：`{report['voice_e2e'].get('status', 'executed')}`",
            "",
            "说明：默认报告不启动 llama.cpp 或真实 ASR/TTS benchmark。演示前可运行：",
            "",
            "```bash",
            "python3 scripts/generate_offline_showcase_report.py --run-latency --run-llama-bench --run-instruction-following --run-asr-tts --run-voice-e2e",
            "```",
        ]
    )
    claims = report["claim_evidence"]
    lines.extend(
        [
            "",
            "## 5. 指标证据矩阵",
            "",
            "这部分用于区分“已经有证据支撑的说法”和“暂时不能过度宣称的指标”。",
            "",
            "| Key | 指标/能力 | 状态 | 证据来源 | 备注 |",
            "| --- | --- | --- | --- | --- |",
        ]
    )
    for item in claims["items"]:
        metric = item.get("metric")
        if metric is None:
            note = item.get("caveat", "")
        else:
            note = json.dumps(metric, ensure_ascii=False)
            if item.get("caveat"):
                note = f"{note}；{item['caveat']}"
        lines.append(
            f"| `{item['key']}` | {item['title']} | `{item['status']}` | "
            f"`{item['evidence']}` | {note} |"
        )

    lines.extend(["", "### 可宣称", ""])
    for claim in claims["allowed_claims"]:
        lines.append(f"- {claim}")
    lines.extend(["", "### 不应过度宣称", ""])
    for claim in claims["restricted_claims"]:
        lines.append(f"- {claim}")
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="training/robot_instruction_eval.jsonl")
    parser.add_argument("--json-output", default="logs/offline_showcase_report.json")
    parser.add_argument("--md-output", default="logs/offline_showcase_report.md")
    parser.add_argument("--timeout-s", type=float, default=900.0)
    parser.add_argument("--run-latency", action="store_true")
    parser.add_argument("--run-llama-bench", action="store_true")
    parser.add_argument("--llama-bench-input", help="parse saved llama-bench JSON instead of running it")
    parser.add_argument("--llama-bench-output", default="logs/llama_decode_benchmark.json")
    parser.add_argument("--run-instruction-following", action="store_true")
    parser.add_argument("--instruction-following-input", help="reuse saved instruction-following report")
    parser.add_argument(
        "--instruction-following-output",
        default="logs/instruction_following_report.json",
    )
    parser.add_argument("--run-asr-tts", action="store_true")
    parser.add_argument("--run-voice-e2e", action="store_true")
    parser.add_argument("--voice-e2e-input", help="reuse a saved offline voice E2E report")
    parser.add_argument(
        "--voice-e2e-output",
        default="logs/offline_voice_e2e_report.json",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    report = build_report(args)
    json_output = Path(args.json_output).expanduser()
    md_output = Path(args.md_output).expanduser()
    if not json_output.is_absolute():
        json_output = WORKSPACE / json_output
    if not md_output.is_absolute():
        md_output = WORKSPACE / md_output
    json_output.parent.mkdir(parents=True, exist_ok=True)
    md_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    md_output.write_text(render_markdown(report), encoding="utf-8")
    print(
        json.dumps(
            {
                "ok": bool(report["model_inventory"]["ok"])
                and bool(report["runtime_versions"]["ok"])
                and not report["instruction_parser"]["failed_cases"],
                "json_output": str(json_output),
                "md_output": str(md_output),
                "parser_accuracy": report["instruction_parser"]["accuracy"],
                "model_total_size_mb": report["model_inventory"]["total_size_mb"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
