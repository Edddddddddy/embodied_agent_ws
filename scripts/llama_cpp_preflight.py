#!/usr/bin/env python3
import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


def get_json(url: str, timeout_s: float) -> dict:
    with urllib.request.urlopen(url, timeout=timeout_s) as response:
        raw = response.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw else {}


def post_stream(url: str, payload: dict, timeout_s: float) -> tuple[str, float, int]:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    started = time.perf_counter()
    first_token_ms = None
    parts: list[str] = []
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        for raw_line in response:
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line or line.startswith(":") or not line.startswith("data:"):
                continue
            data = line.removeprefix("data:").strip()
            if data == "[DONE]":
                break
            event = json.loads(data)
            choices = event.get("choices") or []
            if not choices:
                continue
            delta = choices[0].get("delta") or {}
            token = delta.get("content")
            if token:
                if first_token_ms is None:
                    first_token_ms = (time.perf_counter() - started) * 1000.0
                parts.append(token)
    if first_token_ms is None:
        raise RuntimeError("streaming chat returned no text token")
    return "".join(parts), first_token_ms, len(parts)


def require_file(path: Path, label: str) -> None:
    if not path.exists() or path.stat().st_size == 0:
        raise FileNotFoundError(f"missing {label}: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Preflight llama.cpp llama-server OpenAI-compatible API."
    )
    parser.add_argument("--workspace", default="/home/ubuntu/embodied_agent_ws")
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")
    parser.add_argument("--model-name", default="Qwen3-0.6B-Q8_0.gguf")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--infer", action="store_true", help="also run a tiny streaming chat")
    args = parser.parse_args()

    workspace = Path(args.workspace)
    binary = workspace / "third_party/llama.cpp/build/bin/llama-server"
    model = workspace / "models/Qwen3-0.6B-Q8_0.gguf"
    report: dict = {
        "binary": str(binary),
        "model": str(model),
        "base_url": args.base_url,
        "infer": args.infer,
    }
    try:
        require_file(binary, "llama-server binary")
        require_file(model, "GGUF model")
        health = get_json(f"{args.base_url}/health", args.timeout_s)
        models = get_json(f"{args.base_url}/v1/models", args.timeout_s)
        report["health"] = health
        report["models"] = models
        if args.infer:
            # 低 token smoke：只验证流式接口可用，不消耗大量本地推理时间。
            text, first_token_ms, token_count = post_stream(
                f"{args.base_url}/v1/chat/completions",
                {
                    "model": args.model_name,
                    "messages": [
                        {"role": "system", "content": "你是机器人动作解析助手。/no_think"},
                        {"role": "user", "content": "只回答：好的 /no_think"},
                    ],
                    "temperature": 0.0,
                    "max_tokens": 8,
                    "stream": True,
                    "chat_template_kwargs": {"enable_thinking": False},
                    "seed": 42,
                },
                args.timeout_s,
            )
            report["chat"] = {
                "text": text,
                "first_token_ms": round(first_token_ms, 2),
                "token_count": token_count,
            }
        report["status"] = "ready"
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except (OSError, RuntimeError, urllib.error.URLError, urllib.error.HTTPError, json.JSONDecodeError) as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        report["hint"] = (
            "先运行 bash scripts/setup_offline_runtime.sh 准备 llama.cpp，"
            "确认 models/Qwen3-0.6B-Q8_0.gguf 存在，再用 bash scripts/start_llama_server.sh 启动。"
        )
        print(json.dumps(report, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
