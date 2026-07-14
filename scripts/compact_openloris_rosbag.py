#!/usr/bin/env python3
"""Create a provenance-bound SLAM-only ROS 1 bag from an OpenLORIS bag.

The public OpenLORIS bag contains camera and IMU streams that are irrelevant to
the 2D laser SLAM experiment. Reading those compressed chunks dominates each
parameter sweep. This tool copies only the three replay-contract topics while
preserving message payloads and timestamps, then writes a derived ``source.json``
that points back to the verified public bag.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


SELECTED_TOPICS = ("/odom", "/scan", "/tf_static")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_commit(workspace: Path) -> str | None:
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=workspace,
        check=False,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip() or None


def _git_dirty(workspace: Path) -> bool:
    completed = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=workspace,
        check=False,
        capture_output=True,
        text=True,
    )
    return bool(completed.stdout.strip())


def validate_source(source: dict[str, object], raw_bag: Path) -> None:
    """Reject ad-hoc bags before deriving evidence from them."""

    declared = Path(str(source.get("bag", "")))
    if source.get("dataset") != "OpenLORIS-Scene":
        raise ValueError("source.json is not an OpenLORIS-Scene provenance record")
    if not declared.is_absolute() or declared.resolve() != raw_bag.resolve():
        raise ValueError("source.json bag path does not match --input")
    if len(str(source.get("bag_sha256", ""))) != 64:
        raise ValueError("source.json does not contain a verified bag SHA256")
    if source.get("archive_verification") not in {
        "full_size_and_sha256",
        "pinned_https_range_not_full_hash",
    }:
        raise ValueError("source.json does not declare a supported verification method")


def build_derived_source(
    *,
    source: dict[str, object],
    raw_bag: Path,
    output_bag: Path,
    counts: dict[str, int],
    first_timestamp_ns: int,
    last_timestamp_ns: int,
    workspace: Path,
) -> dict[str, object]:
    """Build a source record compatible with the real-data experiment manifest."""

    payload = dict(source)
    payload["bag"] = str(output_bag.resolve())
    payload["bag_size_bytes"] = output_bag.stat().st_size
    payload["bag_sha256"] = sha256(output_bag)
    # 使用当前实际执行文件的哈希；workspace 可能是测试仓库或外部调用目录。
    tool_path = Path(__file__).resolve()
    payload["derived"] = {
        "kind": "lossless_topic_subset",
        "source_bag": str(raw_bag.resolve()),
        "source_bag_size_bytes": raw_bag.stat().st_size,
        "source_bag_sha256": source["bag_sha256"],
        "selected_topics": list(SELECTED_TOPICS),
        "message_counts": counts,
        "first_timestamp_ns": first_timestamp_ns,
        "last_timestamp_ns": last_timestamp_ns,
        "duration_s": max(0.0, (last_timestamp_ns - first_timestamp_ns) * 1e-9),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "tool": "scripts/compact_openloris_rosbag.py",
        "tool_git_commit": _git_commit(workspace),
        "tool_git_dirty": _git_dirty(workspace),
        "tool_sha256": sha256(tool_path),
        "transformation": "timestamps and serialized ROS messages preserved",
    }
    return payload


def compact(
    *,
    raw_bag: Path,
    raw_source: Path,
    output_bag: Path,
    output_source: Path,
    workspace: Path,
    verify_source_hash: bool = False,
) -> dict[str, object]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "missing rosbags; run: pip install -r requirements-slam-eval.txt"
        ) from exc

    if output_bag.exists() or output_source.exists():
        raise FileExistsError("refusing to overwrite an existing derived artifact")
    source = json.loads(raw_source.read_text(encoding="utf-8"))
    validate_source(source, raw_bag)
    if verify_source_hash and sha256(raw_bag) != source["bag_sha256"]:
        raise ValueError("input bag SHA256 does not match source.json")

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    counts = {topic: 0 for topic in SELECTED_TOPICS}
    first_timestamp_ns: int | None = None
    last_timestamp_ns: int | None = None
    output_bag.parent.mkdir(parents=True, exist_ok=True)
    try:
        with AnyReader([raw_bag], default_typestore=typestore) as reader:
            selected = [
                connection
                for connection in reader.connections
                if connection.topic in SELECTED_TOPICS
            ]
            available = {connection.topic for connection in selected}
            missing = set(SELECTED_TOPICS) - available
            if missing:
                raise ValueError(f"required topics missing: {sorted(missing)}")
            with Writer(output_bag) as writer:
                outputs = {
                    connection.id: writer.add_connection(
                        connection.topic,
                        connection.msgtype,
                        typestore=typestore,
                        callerid="/embodied_openloris_compactor",
                        latching=1 if connection.topic == "/tf_static" else 0,
                    )
                    for connection in selected
                }
                for connection, timestamp_ns, rawdata in reader.messages(
                    connections=selected
                ):
                    # ROS1 -> dataclass -> ROS1 会归一化旧消息定义，但测量值和 bag 时间戳不变。
                    message = reader.deserialize(rawdata, connection.msgtype)
                    writer.write(
                        outputs[connection.id],
                        timestamp_ns,
                        typestore.serialize_ros1(message, connection.msgtype),
                    )
                    counts[connection.topic] += 1
                    first_timestamp_ns = (
                        timestamp_ns
                        if first_timestamp_ns is None
                        else min(first_timestamp_ns, timestamp_ns)
                    )
                    last_timestamp_ns = (
                        timestamp_ns
                        if last_timestamp_ns is None
                        else max(last_timestamp_ns, timestamp_ns)
                    )
        if first_timestamp_ns is None or last_timestamp_ns is None:
            raise ValueError("no selected messages were copied")
        if any(value == 0 for value in counts.values()):
            raise ValueError(f"derived bag has an empty required stream: {counts}")
        derived = build_derived_source(
            source=source,
            raw_bag=raw_bag,
            output_bag=output_bag,
            counts=counts,
            first_timestamp_ns=first_timestamp_ns,
            last_timestamp_ns=last_timestamp_ns,
            workspace=workspace,
        )
        output_source.parent.mkdir(parents=True, exist_ok=True)
        output_source.write_text(
            json.dumps(derived, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        return derived
    except Exception:
        # 只清理本次尚未交付的目标文件，不触碰原始公开数据。
        output_bag.unlink(missing_ok=True)
        output_source.unlink(missing_ok=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--output-source", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--verify-source-hash", action="store_true")
    args = parser.parse_args()
    result = compact(
        raw_bag=args.input,
        raw_source=args.source,
        output_bag=args.output,
        output_source=args.output_source,
        workspace=args.workspace,
        verify_source_hash=args.verify_source_hash,
    )
    print(json.dumps(result["derived"], ensure_ascii=False, indent=2))
    print(f"PASS: compact OpenLORIS bag -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
