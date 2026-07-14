from __future__ import annotations

import importlib.util
import math
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "rank_openloris_revisit_sequences",
    ROOT / "scripts" / "rank_openloris_revisit_sequences.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _trajectory(*, return_with_opposite_heading: bool) -> str:
    rows = []
    positions = [0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0]
    for stamp, x in enumerate(positions):
        yaw = math.pi if return_with_opposite_heading and stamp >= 6 else 0.0
        rows.append(
            f"{stamp} {x} 0 0 0 0 {math.sin(yaw / 2)} {math.cos(yaw / 2)}"
        )
    return "\n".join(rows) + "\n"


def test_ranking_distinguishes_directional_and_position_only_revisits(tmp_path):
    archive = tmp_path / "groundtruth.zip"
    with zipfile.ZipFile(archive, "w") as zipped:
        zipped.writestr(
            "per-sequence/opposite/groundtruth.txt",
            _trajectory(return_with_opposite_heading=True),
        )
        zipped.writestr(
            "per-sequence/same_heading/groundtruth.txt",
            _trajectory(return_with_opposite_heading=False),
        )
        zipped.writestr(
            "per-sequence/no_loop/groundtruth.txt",
            "\n".join(
                f"{stamp} {stamp} 0 0 0 0 0 1" for stamp in range(11)
            )
            + "\n",
        )
        zipped.writestr("../../escape/groundtruth.txt", "not a sequence")

    report = MODULE.rank_sequences(
        archive,
        loop_radius_m=0.1,
        loop_min_separation_s=5.0,
        loop_sample_interval_s=1.0,
        loop_event_gap_s=1.5,
        path_sample_interval_s=1.0,
        minimum_long_path_m=5.0,
        minimum_long_duration_s=5.0,
    )

    assert report["passed"] is True
    assert report["recommendation"] == "same_heading"
    assert report["position_only_fallback"] == "same_heading"
    opposite = next(
        row for row in report["ranked_sequences"] if row["sequence"] == "opposite"
    )
    assert opposite["directional_view"]["event_count"] == 0
    assert opposite["position_only_360_lidar"]["event_count"] == 1
    assert {row["sequence"] for row in report["ranked_sequences"]} == {
        "opposite",
        "same_heading",
        "no_loop",
    }


def test_sensor_profile_requires_verified_compatible_long_loop(tmp_path):
    archive = tmp_path / "groundtruth.zip"
    with zipfile.ZipFile(archive, "w") as zipped:
        zipped.writestr(
            "per-sequence/directional/groundtruth.txt",
            _trajectory(return_with_opposite_heading=False),
        )
        zipped.writestr(
            "per-sequence/opposite/groundtruth.txt",
            _trajectory(return_with_opposite_heading=True),
        )
    contracts = {
        "profiles": {
            "slam_toolbox_2d": {"heading_policy": "position_only_360_lidar"}
        },
        "sequences": {
            "directional": {"status": "verified_incompatible"},
            "opposite": {"status": "verified_compatible"},
        },
    }

    report = MODULE.rank_sequences(
        archive,
        loop_radius_m=0.1,
        loop_min_separation_s=5.0,
        loop_sample_interval_s=1.0,
        loop_event_gap_s=1.5,
        path_sample_interval_s=1.0,
        minimum_long_path_m=5.0,
        minimum_long_duration_s=5.0,
        sensor_contracts=contracts,
        sensor_profile="slam_toolbox_2d",
    )

    assert report["passed"] is True
    assert report["trajectory_only_recommendation"] == "directional"
    assert report["recommendation"] == "opposite"
    assert report["ranked_sequences"][0]["eligible_for_sensor_profile"] is True


def test_position_only_sensor_profile_does_not_require_an_unrelated_directional_loop(
    tmp_path,
):
    archive = tmp_path / "groundtruth.zip"
    with zipfile.ZipFile(archive, "w") as zipped:
        zipped.writestr(
            "per-sequence/opposite/groundtruth.txt",
            _trajectory(return_with_opposite_heading=True),
        )
    contracts = {
        "profiles": {
            "slam_toolbox_2d": {"heading_policy": "position_only_360_lidar"}
        },
        "sequences": {"opposite": {"status": "verified_compatible"}},
    }

    report = MODULE.rank_sequences(
        archive,
        loop_radius_m=0.1,
        loop_min_separation_s=5.0,
        loop_sample_interval_s=1.0,
        loop_event_gap_s=1.5,
        path_sample_interval_s=1.0,
        minimum_long_path_m=5.0,
        minimum_long_duration_s=5.0,
        sensor_contracts=contracts,
        sensor_profile="slam_toolbox_2d",
    )

    assert report["passed"] is True
    assert report["trajectory_only_recommendation"] is None
    assert report["recommendation"] == "opposite"
