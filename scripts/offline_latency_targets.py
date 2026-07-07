#!/usr/bin/env python3
"""Offline latency gate for llama.cpp first token and local TTS first audio."""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
LLM_FIRST_TOKEN_TARGET_MS = 1000.0
TTS_FIRST_AUDIO_TARGET_MS = 300.0


def _ensure_repo_import_path() -> None:
    for relative in ("src/embodied_offline_agent", "src/embodied_online_agent"):
        path = str(WORKSPACE / relative)
        if path not in sys.path:
            sys.path.insert(0, path)


def post_llama_stream(base_url: str, timeout_s: float) -> dict[str, object]:
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}/v1/chat/completions",
        data=json.dumps(
            {
                "model": "Qwen3-0.6B-Q8_0.gguf",
                "messages": [
                    {"role": "system", "content": "你是机器人动作解析助手。/no_think"},
                    {"role": "user", "content": "只回答：好的 /no_think"},
                ],
                "temperature": 0.0,
                "max_tokens": 8,
                "stream": True,
                "chat_template_kwargs": {"enable_thinking": False},
                "seed": 42,
            }
        ).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    started = time.perf_counter()
    first_token_ms = None
    parts: list[str] = []
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        for raw_line in response:
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line.startswith("data:"):
                continue
            data = line.removeprefix("data:").strip()
            if data == "[DONE]":
                break
            event = json.loads(data)
            for choice in event.get("choices") or []:
                token = (choice.get("delta") or {}).get("content")
                if not token:
                    continue
                if first_token_ms is None:
                    first_token_ms = (time.perf_counter() - started) * 1000.0
                parts.append(token)
    if first_token_ms is None:
        raise RuntimeError("llama.cpp stream returned no text token")
    return {
        "text": "".join(parts),
        "first_token_ms": round(first_token_ms, 2),
        "target_ms": LLM_FIRST_TOKEN_TARGET_MS,
        "ok": first_token_ms <= LLM_FIRST_TOKEN_TARGET_MS,
    }


def create_tts(provider: str):
    _ensure_repo_import_path()
    if provider == "sherpa":
        from embodied_offline_agent.providers.sherpa_tts import SherpaVitsTts

        return SherpaVitsTts(
            str(WORKSPACE / "models" / "vits-melo-tts-zh_en"),
            num_threads=2,
            speaker_id=0,
            speed=1.0,
        )
    if provider == "summer":
        from embodied_offline_agent.providers.summer_tts import SummerTts

        return SummerTts(
            str(WORKSPACE / "third_party" / "SummerTTS" / "build" / "tts_test"),
            str(
                WORKSPACE
                / "third_party"
                / "SummerTTS"
                / "models"
                / "single_speaker_fast.bin"
            ),
            timeout_s=30.0,
        )
    raise ValueError(f"unsupported tts provider: {provider}")


def measure_tts(provider: str, text: str, warmup: bool) -> dict[str, object]:
    started = time.perf_counter()
    tts = create_tts(provider)
    load_ms = (time.perf_counter() - started) * 1000.0
    if warmup:
        # 模型加载后的首句才是常驻 provider 的在线服务延迟；预热句不计入目标。
        tts.synthesize("好。")
    synth_started = time.perf_counter()
    pcm = tts.synthesize(text)
    first_audio_ms = (time.perf_counter() - synth_started) * 1000.0
    return {
        "provider": provider,
        "text": text,
        "load_ms": round(load_ms, 2),
        "first_audio_ms": round(first_audio_ms, 2),
        "target_ms": TTS_FIRST_AUDIO_TARGET_MS,
        "pcm_bytes": len(pcm),
        "sample_rate": getattr(tts, "sample_rate", None),
        "warmup": warmup,
        "ok": first_audio_ms <= TTS_FIRST_AUDIO_TARGET_MS and len(pcm) > 0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--tts-provider", default="sherpa", choices=["sherpa", "summer"])
    parser.add_argument("--tts-text", default="好的。")
    parser.add_argument("--no-llm", action="store_true")
    parser.add_argument("--no-tts", action="store_true")
    parser.add_argument("--no-tts-warmup", action="store_true")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    report: dict[str, object] = {
        "targets": {
            "llm_first_token_ms": LLM_FIRST_TOKEN_TARGET_MS,
            "tts_first_audio_ms": TTS_FIRST_AUDIO_TARGET_MS,
        }
    }
    ok = True
    if not args.no_llm:
        report["llm"] = post_llama_stream(args.base_url, args.timeout_s)
        ok = ok and bool(report["llm"]["ok"])  # type: ignore[index]
    if not args.no_tts:
        report["tts"] = measure_tts(
            args.tts_provider, args.tts_text, warmup=not args.no_tts_warmup
        )
        ok = ok and bool(report["tts"]["ok"])  # type: ignore[index]
    report["ok"] = ok
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.check and not ok:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
