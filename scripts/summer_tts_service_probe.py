#!/usr/bin/env python3
"""Probe the resident C++ SummerTTS ROS service."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from embodied_agent_interfaces.srv import SynthesizeSpeech


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default="/tts/synthesize")
    parser.add_argument("--text", default="好的。")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--min-pcm-bytes", type=int, default=1000)
    parser.add_argument(
        "--repeat",
        type=int,
        default=2,
        help="call the same request repeatedly; repeat>=2 verifies short-text cache hits",
    )
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("summer_tts_service_probe")
    try:
        client = node.create_client(SynthesizeSpeech, args.service)
        if not client.wait_for_service(timeout_sec=args.timeout_s):
            raise SystemExit(f"service not available: {args.service}")
        responses = []
        for index in range(max(args.repeat, 1)):
            request = SynthesizeSpeech.Request()
            request.text = args.text
            request.speaker_id = -1
            request.length_scale = 0.0
            started = time.perf_counter()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout_s)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            response = future.result()
            if response is None:
                raise SystemExit("service returned no response")
            responses.append(
                {
                    "index": index + 1,
                    "ok": bool(response.ok),
                    "error": response.error,
                    "sample_rate": int(response.sample_rate),
                    "pcm_bytes": len(response.pcm),
                    "cache_hit": bool(getattr(response, "cache_hit", False)),
                    "service_synthesize_ms": round(float(response.synthesize_ms), 2),
                    "roundtrip_ms": round(elapsed_ms, 2),
                }
            )

        first = responses[0]
        last = responses[-1]
        report = {
            # 保留旧字段，避免调用方只看顶层 ok/pcm_bytes 时需要同步大改。
            "ok": bool(all(item["ok"] for item in responses)),
            "error": next((item["error"] for item in responses if item["error"]), ""),
            "sample_rate": int(last["sample_rate"]),
            "pcm_bytes": int(last["pcm_bytes"]),
            "service_synthesize_ms": float(last["service_synthesize_ms"]),
            "roundtrip_ms": float(last["roundtrip_ms"]),
            "cache_hit": bool(last["cache_hit"]),
            "cache_hits": sum(1 for item in responses if item["cache_hit"]),
            "first_roundtrip_ms": float(first["roundtrip_ms"]),
            "last_roundtrip_ms": float(last["roundtrip_ms"]),
            "responses": responses,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if not report["ok"] or any(item["pcm_bytes"] < args.min_pcm_bytes for item in responses):
            return 1
        if args.repeat >= 2 and not report["cache_hit"]:
            print("WARN: repeated short text did not hit cache; check cache parameters or text length")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
