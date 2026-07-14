#!/usr/bin/env python3
"""Rank OpenLORIS sequences by ground-truth revisit evidence before bag download."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import tempfile
import zipfile
from pathlib import Path, PurePosixPath


def _load_revisit_analyzer():
    path = Path(__file__).with_name("analyze_openloris_revisits.py")
    spec = importlib.util.spec_from_file_location("openloris_revisit_analyzer", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load revisit analyzer: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REVISITS = _load_revisit_analyzer()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sequence_members(archive: zipfile.ZipFile) -> list[tuple[str, str]]:
    members: list[tuple[str, str]] = []
    for name in archive.namelist():
        path = PurePosixPath(name)
        if (
            len(path.parts) == 3
            and path.parts[0] == "per-sequence"
            and path.parts[2] == "groundtruth.txt"
            and ".." not in path.parts
        ):
            members.append((path.parts[1], name))
    return sorted(members)


def rank_sequences(
    archive_path: Path,
    *,
    loop_radius_m: float = 1.0,
    directional_yaw_tolerance_deg: float = 30.0,
    loop_min_separation_s: float = 60.0,
    loop_sample_interval_s: float = 1.0,
    loop_event_gap_s: float = 2.0,
    path_sample_interval_s: float = 0.1,
    minimum_long_path_m: float = 100.0,
    minimum_long_duration_s: float = 120.0,
    sensor_contracts: dict[str, object] | None = None,
    sensor_profile: str | None = None,
) -> dict[str, object]:
    """Compare directional-view and position-only revisit definitions for every sequence."""

    rows: list[dict[str, object]] = []
    with zipfile.ZipFile(archive_path) as archive, tempfile.TemporaryDirectory(
        prefix="openloris-revisit-rank-"
    ) as temporary:
        for sequence, member in _sequence_members(archive):
            trajectory_path = Path(temporary) / f"{sequence}.txt"
            trajectory_path.write_bytes(archive.read(member))
            reports = {}
            for policy, yaw_tolerance in (
                ("directional_view", directional_yaw_tolerance_deg),
                # 2D 360° LiDAR 回环关注“回到同一地点”；机器人反向通过时仍有完整扫描重叠。
                ("position_only_360_lidar", 180.0),
            ):
                config = REVISITS.EVALUATOR.EvaluationConfig(
                    loop_radius_m=loop_radius_m,
                    loop_yaw_tolerance_deg=yaw_tolerance,
                    loop_min_separation_s=loop_min_separation_s,
                    loop_sample_interval_s=loop_sample_interval_s,
                    loop_event_gap_s=loop_event_gap_s,
                )
                reports[policy] = REVISITS.analyze(
                    trajectory_path,
                    config,
                    min_events=0,
                    path_sample_interval_s=path_sample_interval_s,
                )
            directional = reports["directional_view"]
            positional = reports["position_only_360_lidar"]
            trajectory = positional["trajectory"]
            directional_catalog = directional["revisit_catalog"]
            positional_catalog = positional["revisit_catalog"]
            position_long_candidate = (
                int(positional_catalog["event_count"]) > 0
                and float(trajectory["path_length_m"]) >= minimum_long_path_m
                and float(trajectory["duration_s"]) >= minimum_long_duration_s
            )
            directional_long_candidate = (
                int(directional_catalog["event_count"]) > 0
                and float(trajectory["path_length_m"]) >= minimum_long_path_m
                and float(trajectory["duration_s"]) >= minimum_long_duration_s
            )
            rows.append(
                {
                    "sequence": sequence,
                    "trajectory": trajectory,
                    "directional_view": directional_catalog,
                    "position_only_360_lidar": positional_catalog,
                    # 正式长序列优先要求同向重访，避免只靠 180° 放宽口径制造机会。
                    "long_loop_candidate": directional_long_candidate,
                    "directional_long_loop_candidate": directional_long_candidate,
                    "position_long_loop_candidate": position_long_candidate,
                    "groundtruth_sha256": positional["source"]["sha256"],
                }
            )

    profile = None
    if sensor_contracts is not None:
        if not sensor_profile:
            raise ValueError("sensor_profile is required with sensor_contracts")
        profiles = sensor_contracts.get("profiles", {})
        if not isinstance(profiles, dict) or sensor_profile not in profiles:
            raise ValueError(f"unknown sensor profile: {sensor_profile}")
        profile = profiles[sensor_profile]
        if not isinstance(profile, dict):
            raise ValueError(f"invalid sensor profile: {sensor_profile}")
        policy = str(profile.get("heading_policy"))
        if policy not in {"directional_view", "position_only_360_lidar"}:
            raise ValueError(f"unsupported heading policy: {policy}")
        sequence_contracts = sensor_contracts.get("sequences", {})
        if not isinstance(sequence_contracts, dict):
            raise ValueError("sensor contracts sequences must be an object")
        for row in rows:
            contract = sequence_contracts.get(str(row["sequence"]), {})
            if not isinstance(contract, dict):
                contract = {}
            status = str(contract.get("status", "missing"))
            long_candidate = bool(
                row[
                    "position_long_loop_candidate"
                    if policy == "position_only_360_lidar"
                    else "directional_long_loop_candidate"
                ]
            )
            row["sensor_contract"] = {
                "profile": sensor_profile,
                "status": status,
                "verified_compatible": status == "verified_compatible",
                "evidence": contract.get("evidence"),
            }
            row["sensor_profile_long_loop_candidate"] = long_candidate
            row["eligible_for_sensor_profile"] = (
                status == "verified_compatible" and long_candidate
            )

    rows.sort(
        key=lambda row: (
            -int(bool(row.get("eligible_for_sensor_profile", False))),
            -int(bool(row["long_loop_candidate"])),
            -int(row["directional_view"]["event_count"]),  # type: ignore[index]
            -float(row["trajectory"]["path_length_m"]),  # type: ignore[index]
            -int(row["position_only_360_lidar"]["event_count"]),  # type: ignore[index]
            str(row["sequence"]),
        )
    )
    long_loop_candidates = [row for row in rows if row["long_loop_candidate"]]
    sensor_candidates = [
        row for row in rows if row.get("eligible_for_sensor_profile", False)
    ]
    position_only_candidates = [
        row
        for row in rows
        if int(row["position_only_360_lidar"]["event_count"]) > 0  # type: ignore[index]
    ]
    checks = {
        "sequences_present": bool(rows),
        "position_revisit_candidate_present": bool(position_only_candidates),
    }
    if sensor_contracts is not None:
        checks["verified_sensor_profile_long_loop_candidate_present"] = bool(
            sensor_candidates
        )
    else:
        # 没有声明传感器模型时，保守地只推荐同向重访；有 profile 时则遵循其 heading policy。
        checks["directional_long_loop_candidate_present"] = bool(long_loop_candidates)
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "source": {
            "archive": str(archive_path.resolve()),
            "sha256": _sha256(archive_path),
        },
        "configuration": {
            "loop_radius_m": loop_radius_m,
            "directional_yaw_tolerance_deg": directional_yaw_tolerance_deg,
            "position_only_yaw_tolerance_deg": 180.0,
            "loop_min_separation_s": loop_min_separation_s,
            "loop_sample_interval_s": loop_sample_interval_s,
            "loop_event_gap_s": loop_event_gap_s,
            "path_sample_interval_s": path_sample_interval_s,
            "minimum_long_path_m": minimum_long_path_m,
            "minimum_long_duration_s": minimum_long_duration_s,
        },
        "recommendation": (
            sensor_candidates[0]["sequence"]
            if sensor_contracts is not None and sensor_candidates
            else long_loop_candidates[0]["sequence"]
            if sensor_contracts is None and long_loop_candidates
            else None
        ),
        "trajectory_only_recommendation": (
            long_loop_candidates[0]["sequence"] if long_loop_candidates else None
        ),
        "sensor_profile": sensor_profile,
        "position_only_fallback": (
            position_only_candidates[0]["sequence"]
            if position_only_candidates
            else None
        ),
        "ranked_sequences": rows,
        "claim_boundary": (
            "ranking uses ground-truth revisit opportunities only; raw sensor availability, "
            "front-end detection and accepted loop constraints require separate evidence"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--loop-radius", type=float, default=1.0)
    parser.add_argument("--directional-yaw-tolerance-deg", type=float, default=30.0)
    parser.add_argument("--loop-min-separation", type=float, default=60.0)
    parser.add_argument("--loop-sample-interval", type=float, default=1.0)
    parser.add_argument("--loop-event-gap", type=float, default=2.0)
    parser.add_argument("--path-sample-interval", type=float, default=0.1)
    parser.add_argument("--minimum-long-path", type=float, default=100.0)
    parser.add_argument("--minimum-long-duration", type=float, default=120.0)
    parser.add_argument("--sensor-contracts", type=Path)
    parser.add_argument("--sensor-profile", default="slam_toolbox_2d")
    parser.add_argument(
        "--verbose", action="store_true", help="Print every ranked trajectory"
    )
    args = parser.parse_args()
    sensor_contracts = (
        json.loads(args.sensor_contracts.read_text(encoding="utf-8"))
        if args.sensor_contracts
        else None
    )
    report = rank_sequences(
        args.archive,
        loop_radius_m=args.loop_radius,
        directional_yaw_tolerance_deg=args.directional_yaw_tolerance_deg,
        loop_min_separation_s=args.loop_min_separation,
        loop_sample_interval_s=args.loop_sample_interval,
        loop_event_gap_s=args.loop_event_gap,
        path_sample_interval_s=args.path_sample_interval,
        minimum_long_path_m=args.minimum_long_path,
        minimum_long_duration_s=args.minimum_long_duration,
        sensor_contracts=sensor_contracts,
        sensor_profile=args.sensor_profile if sensor_contracts is not None else None,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    selected = next(
        (
            row
            for row in report["ranked_sequences"]
            if row["sequence"] == report["recommendation"]
        ),
        None,
    )
    console_report = report if args.verbose else {
        "passed": report["passed"],
        "checks": report["checks"],
        "recommendation": report["recommendation"],
        "trajectory_only_recommendation": report["trajectory_only_recommendation"],
        "sensor_profile": report["sensor_profile"],
        "selected": (
            {
                "sequence": selected["sequence"],
                "trajectory": selected["trajectory"],
                "directional_events": selected["directional_view"]["event_count"],
                "position_only_events": selected["position_only_360_lidar"]["event_count"],
                "sensor_contract": selected.get("sensor_contract"),
            }
            if selected is not None
            else None
        ),
        "full_report": str(args.output.resolve()),
    }
    print(json.dumps(console_report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: OpenLORIS revisit ranking")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
