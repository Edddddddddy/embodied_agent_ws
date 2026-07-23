"""Optional rosbags source and OpenLORIS topic-contract inspection."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterator, NamedTuple


REQUIRED_TOPICS = {
    "/odom": "nav_msgs/msg/Odometry",
    "/scan": "sensor_msgs/msg/LaserScan",
}


class BagEvent(NamedTuple):
    topic: str
    msgtype: str
    timestamp_ns: int
    message: object


def _reader(path: Path):
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "missing optional rosbags runtime; run: pip install -r requirements-slam-eval.txt"
        ) from exc
    # AnyReader 在消息迭代时才反序列化，可直接处理大型 ROS 1 bag，避免整包载入内存。
    # OpenLORIS 是 ROS 1 数据，因此显式提供 typestore 以解析未携带完整定义的旧消息连接。
    return AnyReader(
        [path], default_typestore=get_typestore(Stores.ROS2_HUMBLE)
    )


def inspect_bag(path: Path) -> dict[str, object]:
    with _reader(path) as reader:
        topics: dict[str, dict[str, object]] = {}
        for connection in reader.connections:
            item = topics.setdefault(
                connection.topic, {"msgtype": connection.msgtype, "connections": 0}
            )
            item["connections"] = int(item["connections"]) + 1
        # SLAM 输入契约必须在启动重型求解器前失败；缺 topic 时不能靠 frame 猜测兜底。
        checks = {
            f"topic:{topic}": bool(
                topic in topics and topics[topic]["msgtype"] == expected_type
            )
            for topic, expected_type in REQUIRED_TOPICS.items()
        }
        checks["static_extrinsics"] = "/tf_static" in topics
        duration_s = max(0.0, (reader.end_time - reader.start_time) * 1e-9)
        return {
            "passed": all(checks.values()),
            "checks": checks,
            "path": str(path),
            "start_time_ns": reader.start_time,
            "end_time_ns": reader.end_time,
            "duration_s": duration_s,
            "topics": dict(sorted(topics.items())),
            "known_dataset_issue": (
                "old office bags may contain duplicate /odom stamps; replay deduplicates them"
            ),
        }


def iter_events(path: Path, topics: set[str]) -> Iterator[BagEvent]:
    """Yield selected messages in bag-time order without loading the bag into memory."""

    with _reader(path) as reader:
        connections = [item for item in reader.connections if item.topic in topics]
        available = {item.topic for item in connections}
        missing = (topics - {"/tf_static"}) - available
        if missing:
            raise ValueError(f"required bag topics missing: {sorted(missing)}")
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            yield BagEvent(
                connection.topic,
                connection.msgtype,
                timestamp,
                reader.deserialize(rawdata, connection.msgtype),
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = inspect_bag(args.bag)
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    print(f"{'PASS' if report['passed'] else 'FAIL'}: OpenLORIS bag contract")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
