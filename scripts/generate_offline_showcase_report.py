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
        [sys.executable, "scripts/offline_latency_targets.py", "--check"],
        timeout_s=timeout_s,
    )


def collect_asr_tts_benchmark(run_asr_tts: bool, timeout_s: float) -> dict[str, Any]:
    if not run_asr_tts:
        return {
            "status": "not_run",
            "reason": "pass --run-asr-tts to run Sherpa ASR/TTS benchmark",
        }
    return _run_json_command([sys.executable, "scripts/benchmark_offline.py"], timeout_s=timeout_s)


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    dataset = Path(args.dataset).expanduser()
    if not dataset.is_absolute():
        dataset = WORKSPACE / dataset
    return {
        "schema_version": 1,
        "scenario": "offline_deployment_showcase",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "workspace": str(WORKSPACE),
        "model_inventory": collect_model_inventory(),
        "runtime_versions": collect_runtime_versions(),
        "instruction_parser": evaluate_instruction_parser(dataset),
        "latency": collect_latency(args.run_latency, args.timeout_s),
        "asr_tts_benchmark": collect_asr_tts_benchmark(args.run_asr_tts, args.timeout_s),
    }


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
            f"- ASR/TTS benchmark：`{report['asr_tts_benchmark'].get('status', 'executed')}`",
            "",
            "说明：默认报告不启动 llama.cpp 或真实 ASR/TTS benchmark。演示前可运行：",
            "",
            "```bash",
            "python3 scripts/generate_offline_showcase_report.py --run-latency --run-asr-tts",
            "```",
        ]
    )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="training/robot_instruction_eval.jsonl")
    parser.add_argument("--json-output", default="logs/offline_showcase_report.json")
    parser.add_argument("--md-output", default="logs/offline_showcase_report.md")
    parser.add_argument("--timeout-s", type=float, default=900.0)
    parser.add_argument("--run-latency", action="store_true")
    parser.add_argument("--run-asr-tts", action="store_true")
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
