"""Unknown-world 定位质量验收的公共接口契约。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


MAXIMUM_POSITION_ERROR_P95_M = 0.25
MAXIMUM_ALIGNMENT_GAP_S = 0.10
MINIMUM_ALIGNED_SAMPLES = 3


def _localization_api():
    """延迟导入，使生产接口尚未实现时测试以明确的 RED 状态失败。"""

    from tools.acceptance.unknown_world_evidence import (  # noqa: PLC0415
        LocalizationSample,
        evaluate_localization_quality,
    )

    return LocalizationSample, evaluate_localization_quality


def _sample_series(
    sample_type,
    *,
    timestamps_s: tuple[float, ...],
    positions_x_m: tuple[float, ...],
):
    assert len(timestamps_s) == len(positions_x_m)
    return tuple(
        sample_type(timestamp_s=timestamp_s, x_m=x_m, y_m=0.0)
        for timestamp_s, x_m in zip(timestamps_s, positions_x_m)
    )


def _evaluate(
    *,
    amcl_timestamps_s: tuple[float, ...],
    amcl_positions_x_m: tuple[float, ...],
    gazebo_timestamps_s: tuple[float, ...],
    gazebo_positions_x_m: tuple[float, ...],
):
    LocalizationSample, evaluate_localization_quality = _localization_api()
    return evaluate_localization_quality(
        amcl_samples=_sample_series(
            LocalizationSample,
            timestamps_s=amcl_timestamps_s,
            positions_x_m=amcl_positions_x_m,
        ),
        gazebo_samples=_sample_series(
            LocalizationSample,
            timestamps_s=gazebo_timestamps_s,
            positions_x_m=gazebo_positions_x_m,
        ),
        maximum_position_error_p95_m=MAXIMUM_POSITION_ERROR_P95_M,
        maximum_alignment_gap_s=MAXIMUM_ALIGNMENT_GAP_S,
        minimum_aligned_samples=MINIMUM_ALIGNED_SAMPLES,
    )


def test_time_aligned_amcl_and_gazebo_position_error_p95_passes_at_025m():
    # 这里假设采集适配器已把 AMCL(map) 与 Gazebo(world) 位姿转换到同一坐标系；
    # 纯证据层若直接混用两个原点，会把坐标系偏移误报成定位漂移。
    report = _evaluate(
        amcl_timestamps_s=(0.04, 1.04, 2.04, 3.04),
        amcl_positions_x_m=(0.25, 1.25, 2.25, 3.25),
        gazebo_timestamps_s=(0.00, 1.00, 2.00, 3.00),
        gazebo_positions_x_m=(0.00, 1.00, 2.00, 3.00),
    )

    # P95 比均值更难被大量小误差稀释，又不会像最大值那样被单个毛刺主导。
    assert report["passed"] is True
    assert report["checks"]["time_ranges_overlap"] is True
    assert report["checks"]["minimum_aligned_samples"] is True
    assert report["checks"]["position_error_p95"] is True
    assert report["metrics"]["aligned_sample_count"] == 4
    assert report["metrics"]["position_error_p95_m"] == pytest.approx(0.25)


def test_position_error_p95_above_025m_fails_localization_quality():
    report = _evaluate(
        amcl_timestamps_s=(0.02, 1.02, 2.02, 3.02),
        amcl_positions_x_m=(0.30, 1.30, 2.30, 3.30),
        gazebo_timestamps_s=(0.00, 1.00, 2.00, 3.00),
        gazebo_positions_x_m=(0.00, 1.00, 2.00, 3.00),
    )

    assert report["passed"] is False
    assert report["checks"]["position_error_p95"] is False
    assert report["metrics"]["position_error_p95_m"] == pytest.approx(0.30)


def test_too_few_aligned_samples_cannot_claim_localization_quality():
    report = _evaluate(
        amcl_timestamps_s=(0.02, 1.02),
        amcl_positions_x_m=(0.10, 1.10),
        gazebo_timestamps_s=(0.00, 1.00, 2.00),
        gazebo_positions_x_m=(0.00, 1.00, 2.00),
    )

    assert report["passed"] is False
    assert report["checks"]["time_ranges_overlap"] is True
    assert report["checks"]["minimum_aligned_samples"] is False
    assert report["metrics"]["aligned_sample_count"] == 2


def test_non_overlapping_time_ranges_cannot_claim_localization_quality():
    report = _evaluate(
        amcl_timestamps_s=(10.00, 11.00, 12.00),
        amcl_positions_x_m=(0.00, 1.00, 2.00),
        gazebo_timestamps_s=(0.00, 1.00, 2.00),
        gazebo_positions_x_m=(0.00, 1.00, 2.00),
    )

    assert report["passed"] is False
    assert report["checks"]["time_ranges_overlap"] is False
    assert report["checks"]["minimum_aligned_samples"] is False
    assert report["metrics"]["aligned_sample_count"] == 0
    assert report["metrics"]["position_error_p95_m"] is None


def test_truth_is_interpolated_for_a_moving_robot():
    report = _evaluate(
        amcl_timestamps_s=(0.05, 0.15, 0.20),
        amcl_positions_x_m=(0.50, 1.50, 2.00),
        gazebo_timestamps_s=(0.00, 0.10, 0.20),
        gazebo_positions_x_m=(0.00, 1.00, 2.00),
    )

    # 机器人运动时不能把异步 AMCL 位姿直接与上一帧真值比较，否则会把
    # 正常位移当成定位误差；两侧真值足够近时应按时间线性插值。
    assert report["passed"] is True
    assert report["metrics"]["position_error_p95_m"] == pytest.approx(0.0)


def test_one_truth_frame_cannot_be_reused_to_meet_minimum_sample_count():
    report = _evaluate(
        amcl_timestamps_s=(0.01, 0.02, 0.03),
        amcl_positions_x_m=(0.00, 0.00, 0.00),
        gazebo_timestamps_s=(0.00, 1.00),
        gazebo_positions_x_m=(0.00, 0.00),
    )

    assert report["passed"] is False
    assert report["metrics"]["aligned_sample_count"] == 1
    assert report["checks"]["minimum_aligned_samples"] is False


def test_duplicate_timestamps_are_reported_and_fail_evidence_integrity():
    LocalizationSample, evaluate_localization_quality = _localization_api()
    report = evaluate_localization_quality(
        amcl_samples=(
            LocalizationSample(0.00, 0.00, 0.00),
            LocalizationSample(0.00, 0.00, 0.00),
            LocalizationSample(0.10, 0.10, 0.00),
            LocalizationSample(0.20, 0.20, 0.00),
        ),
        gazebo_samples=(
            LocalizationSample(0.00, 0.00, 0.00),
            LocalizationSample(0.10, 0.10, 0.00),
            LocalizationSample(0.20, 0.20, 0.00),
        ),
        maximum_position_error_p95_m=MAXIMUM_POSITION_ERROR_P95_M,
        maximum_alignment_gap_s=MAXIMUM_ALIGNMENT_GAP_S,
        minimum_aligned_samples=MINIMUM_ALIGNED_SAMPLES,
    )

    assert report["passed"] is False
    assert report["checks"]["unique_sample_timestamps"] is False
    assert report["metrics"]["duplicate_amcl_timestamp_count"] == 1
    assert report["metrics"]["unique_amcl_sample_count"] == 3


def test_map_world_transform_uses_spawn_truth_without_amcl_fitting():
    from tools.acceptance.unknown_world_evidence import (  # noqa: PLC0415
        MapWorldTransform,
    )

    transform = MapWorldTransform(
        spawn_world_x_m=-4.0,
        spawn_world_y_m=-3.0,
        spawn_world_yaw_rad=0.5,
    )

    world = transform.map_to_world(2.0, -1.0)
    recovered = transform.world_to_map(*world)

    assert recovered == pytest.approx((2.0, -1.0))
