#!/usr/bin/env python3
"""Measure llama.cpp prompt/decode throughput with llama-bench.

这个脚本专门解决一个展示风险：不能只在简历里写 “CPU Decode xx tokens/s”，
却没有可复查的本机 benchmark。默认运行 llama.cpp 自带的 `llama-bench -o json`；
测试或复盘时也可以用 `--input-json` 解析已经保存的原始 llama-bench 输出。
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]


def _load_bench_rows(path: Path) -> list[dict[str, Any]]:
    raw = path.read_text(encoding="utf-8")
    payload = json.loads(raw)
    if not isinstance(payload, list):
        raise ValueError("llama-bench JSON output must be a list")
    rows = [item for item in payload if isinstance(item, dict)]
    if not rows:
        raise ValueError("llama-bench JSON output contains no rows")
    return rows


def _load_processed_report(path: Path) -> dict[str, Any] | None:
    raw = path.read_text(encoding="utf-8")
    payload = json.loads(raw)
    if not isinstance(payload, dict):
        return None
    if payload.get("scenario") != "llama_decode_speed_benchmark":
        return None
    if payload.get("decode_tokens_per_s") is None:
        raise ValueError("processed llama decode benchmark report lacks decode_tokens_per_s")
    return payload


def _run_llama_bench(args: argparse.Namespace) -> tuple[list[dict[str, Any]], list[str]]:
    binary = Path(args.llama_bench).expanduser()
    model = Path(args.model).expanduser()
    if not binary.is_absolute():
        binary = WORKSPACE / binary
    if not model.is_absolute():
        model = WORKSPACE / model
    if not binary.is_file():
        raise FileNotFoundError(f"missing llama-bench binary: {binary}")
    if not model.is_file() or model.stat().st_size == 0:
        raise FileNotFoundError(f"missing GGUF model: {model}")

    command = [
        str(binary),
        "-m",
        str(model),
        "-p",
        str(args.prompt_tokens),
        "-n",
        str(args.generation_tokens),
        "-r",
        str(args.repetitions),
        "-t",
        str(args.threads),
        "-ngl",
        str(args.gpu_layers),
        "-o",
        "json",
    ]
    if args.no_warmup:
        command.append("--no-warmup")
    completed = subprocess.run(
        command,
        cwd=WORKSPACE,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=args.timeout_s,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "llama-bench failed with exit "
            f"{completed.returncode}: {completed.stderr[-2000:] or completed.stdout[-2000:]}"
        )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise ValueError(f"llama-bench did not emit JSON: {completed.stdout[-2000:]}") from exc
    if not isinstance(payload, list):
        raise ValueError("llama-bench JSON output must be a list")
    return [item for item in payload if isinstance(item, dict)], command


def _best_row(rows: list[dict[str, Any]], *, prompt: bool) -> dict[str, Any] | None:
    candidates = []
    for row in rows:
        n_prompt = int(row.get("n_prompt") or 0)
        n_gen = int(row.get("n_gen") or 0)
        if prompt and n_prompt > 0 and n_gen == 0:
            candidates.append(row)
        if not prompt and n_gen > 0:
            candidates.append(row)
    if not candidates:
        return None
    return max(candidates, key=lambda item: float(item.get("avg_ts") or 0.0))


def summarize_rows(
    rows: list[dict[str, Any]],
    *,
    minimum_decode_tokens_per_s: float,
    source: str,
    command: list[str] | None = None,
) -> dict[str, Any]:
    prompt_row = _best_row(rows, prompt=True)
    decode_row = _best_row(rows, prompt=False)
    if decode_row is None:
        raise ValueError("llama-bench output has no decode row with n_gen > 0")

    decode_tps = round(float(decode_row.get("avg_ts") or 0.0), 4)
    prompt_tps = (
        round(float(prompt_row.get("avg_ts") or 0.0), 4) if prompt_row is not None else None
    )
    ok = decode_tps >= float(minimum_decode_tokens_per_s)
    report: dict[str, Any] = {
        "schema_version": 1,
        "scenario": "llama_decode_speed_benchmark",
        "backend": "llama-bench",
        "source": source,
        "ok": ok,
        "minimum_decode_tokens_per_s": minimum_decode_tokens_per_s,
        "decode_tokens_per_s": decode_tps,
        "prompt_tokens_per_s": prompt_tps,
        "decode": {
            "n_gen": int(decode_row.get("n_gen") or 0),
            "avg_ns": int(decode_row.get("avg_ns") or 0),
            "samples_ts": decode_row.get("samples_ts", []),
        },
        "prompt": {
            "n_prompt": int((prompt_row or {}).get("n_prompt") or 0),
            "avg_ns": int((prompt_row or {}).get("avg_ns") or 0),
            "samples_ts": (prompt_row or {}).get("samples_ts", []),
        },
        "model_type": decode_row.get("model_type", ""),
        "model_filename": decode_row.get("model_filename", ""),
        "model_size": decode_row.get("model_size", 0),
        "model_n_params": decode_row.get("model_n_params", 0),
        "n_threads": decode_row.get("n_threads"),
        "n_gpu_layers": decode_row.get("n_gpu_layers"),
        "cpu_info": decode_row.get("cpu_info", ""),
        "backends": decode_row.get("backends", ""),
    }
    if command:
        report["command"] = command
    return report


def summarize_processed_report(
    report: dict[str, Any],
    *,
    minimum_decode_tokens_per_s: float,
    source: str,
) -> dict[str, Any]:
    decode_tps = float(report.get("decode_tokens_per_s") or 0.0)
    reused = dict(report)
    reused["source"] = source
    reused["minimum_decode_tokens_per_s"] = minimum_decode_tokens_per_s
    reused["ok"] = decode_tps >= minimum_decode_tokens_per_s
    return reused


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=str(WORKSPACE))
    parser.add_argument("--llama-bench", default="third_party/llama.cpp/build/bin/llama-bench")
    parser.add_argument("--model", default="models/Qwen3-0.6B-Q8_0.gguf")
    parser.add_argument("--input-json", help="parse an existing llama-bench -o json output")
    parser.add_argument("--output", default="logs/llama_decode_benchmark.json")
    parser.add_argument("--prompt-tokens", type=int, default=64)
    parser.add_argument("--generation-tokens", type=int, default=32)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--gpu-layers", type=int, default=0)
    parser.add_argument("--timeout-s", type=float, default=300.0)
    parser.add_argument("--minimum-decode-tokens-per-s", type=float, default=0.0)
    parser.add_argument(
        "--no-warmup",
        action="store_true",
        help="skip llama-bench warmup; useful for a quick local evidence run",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = Path(args.output).expanduser()
    if not output.is_absolute():
        output = WORKSPACE / output

    try:
        if args.input_json:
            input_path = Path(args.input_json).expanduser()
            processed = _load_processed_report(input_path)
            if processed is not None:
                report = summarize_processed_report(
                    processed,
                    minimum_decode_tokens_per_s=args.minimum_decode_tokens_per_s,
                    source=str(input_path),
                )
            else:
                rows = _load_bench_rows(input_path)
                report = summarize_rows(
                    rows,
                    minimum_decode_tokens_per_s=args.minimum_decode_tokens_per_s,
                    source=str(input_path),
                )
        else:
            rows, command = _run_llama_bench(args)
            report = summarize_rows(
                rows,
                minimum_decode_tokens_per_s=args.minimum_decode_tokens_per_s,
                source="subprocess",
                command=command,
            )
    except Exception as exc:
        report = {
            "schema_version": 1,
            "scenario": "llama_decode_speed_benchmark",
            "backend": "llama-bench",
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {
        "status": "PASS" if report.get("ok") else "FAIL",
        "output": str(output),
        "decode_tokens_per_s": report.get("decode_tokens_per_s"),
        "prompt_tokens_per_s": report.get("prompt_tokens_per_s"),
        "minimum_decode_tokens_per_s": report.get("minimum_decode_tokens_per_s"),
        "model_type": report.get("model_type"),
        "n_threads": report.get("n_threads"),
        "n_gpu_layers": report.get("n_gpu_layers"),
        "error": report.get("error"),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
