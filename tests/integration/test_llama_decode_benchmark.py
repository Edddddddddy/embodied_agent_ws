import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "benchmark_llama_decode_speed.py"


def _write_llama_bench_json(path: Path) -> None:
    path.write_text(
        json.dumps(
            [
                {
                    "model_type": "qwen3 0.6B Q8_0",
                    "model_filename": "models/Qwen3-0.6B-Q8_0.gguf",
                    "n_threads": 8,
                    "n_prompt": 64,
                    "n_gen": 0,
                    "avg_ts": 128.5,
                    "avg_ns": 498000000,
                },
                {
                    "model_type": "qwen3 0.6B Q8_0",
                    "model_filename": "models/Qwen3-0.6B-Q8_0.gguf",
                    "n_threads": 8,
                    "n_prompt": 0,
                    "n_gen": 32,
                    "avg_ts": 9.25,
                    "avg_ns": 3459000000,
                },
            ],
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def test_llama_decode_benchmark_parses_llama_bench_json(tmp_path):
    raw = tmp_path / "llama_bench.json"
    _write_llama_bench_json(raw)
    output = tmp_path / "llama_decode_benchmark.json"

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input-json",
            str(raw),
            "--output",
            str(output),
            "--minimum-decode-tokens-per-s",
            "8.0",
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
    assert report["ok"] is True
    assert report["decode_tokens_per_s"] == 9.25
    assert report["prompt_tokens_per_s"] == 128.5
    assert report["model_type"] == "qwen3 0.6B Q8_0"
    assert report["backend"] == "llama-bench"


def test_llama_decode_benchmark_fails_when_decode_speed_below_threshold(tmp_path):
    raw = tmp_path / "llama_bench.json"
    output = tmp_path / "llama_decode_benchmark.json"
    _write_llama_bench_json(raw)

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input-json",
            str(raw),
            "--output",
            str(output),
            "--minimum-decode-tokens-per-s",
            "10.0",
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
    assert summary["decode_tokens_per_s"] == 9.25


def test_llama_decode_benchmark_reuses_processed_report(tmp_path):
    processed = tmp_path / "llama_decode_benchmark.json"
    processed.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "scenario": "llama_decode_speed_benchmark",
                "backend": "llama-bench",
                "ok": True,
                "decode_tokens_per_s": 12.5,
                "prompt_tokens_per_s": 140.0,
                "minimum_decode_tokens_per_s": 0.0,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    output = tmp_path / "reused.json"

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input-json",
            str(processed),
            "--output",
            str(output),
            "--minimum-decode-tokens-per-s",
            "10.0",
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
    assert report["source"] == str(processed)
    assert report["decode_tokens_per_s"] == 12.5
    assert report["ok"] is True
