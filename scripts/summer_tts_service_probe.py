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
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("summer_tts_service_probe")
    try:
        client = node.create_client(SynthesizeSpeech, args.service)
        if not client.wait_for_service(timeout_sec=args.timeout_s):
            raise SystemExit(f"service not available: {args.service}")
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
        report = {
            "ok": bool(response.ok),
            "error": response.error,
            "sample_rate": int(response.sample_rate),
            "pcm_bytes": len(response.pcm),
            "service_synthesize_ms": round(float(response.synthesize_ms), 2),
            "roundtrip_ms": round(elapsed_ms, 2),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if not response.ok or len(response.pcm) < args.min_pcm_bytes:
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
