"""Unknown-world 地图质量验收的契约测试。"""

from __future__ import annotations

import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
LEGACY_FIXTURE = (
    ROOT / "tests" / "fixtures" / "unknown_world" / "legacy_20260719_partial"
)


def _quality_api():
    """延迟导入，让每条契约在生产模块尚未实现时分别给出红测。"""

    from tools.acceptance.unknown_world_evidence import (  # noqa: PLC0415
        MapQualityThresholds,
        MapRegion,
        evaluate_unknown_world_map,
    )

    thresholds = MapQualityThresholds(
        minimum_reachable_coverage_ratio=0.90,
        minimum_region_coverage_ratio=0.85,
        maximum_reachable_unknown_ratio=0.10,
    )
    return MapRegion, evaluate_unknown_world_map, thresholds


def _write_map(
    directory: Path,
    name: str,
    pixels: list[list[int]],
    *,
    resolution: float = 1.0,
    origin_xy: tuple[float, float] = (0.0, 0.0),
) -> Path:
    """写入小型 P2 PGM，避免测试依赖图像处理库。"""

    directory.mkdir(parents=True, exist_ok=True)
    height = len(pixels)
    width = len(pixels[0])
    assert height > 0 and width > 0
    assert all(len(row) == width for row in pixels)

    pgm_path = directory / f"{name}.pgm"
    pgm_rows = "\n".join(" ".join(str(value) for value in row) for row in pixels)
    pgm_path.write_text(f"P2\n{width} {height}\n255\n{pgm_rows}\n", encoding="ascii")

    yaml_path = directory / f"{name}.yaml"
    yaml_path.write_text(
        "\n".join(
            (
                f"image: {pgm_path.name}",
                "mode: trinary",
                f"resolution: {resolution}",
                f"origin: [{origin_xy[0]}, {origin_xy[1]}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            )
        ),
        encoding="utf-8",
    )
    return yaml_path


def _evaluate(
    built_map_yaml: Path,
    truth_map_yaml: Path,
    *,
    robot_start_xy: tuple[float, float],
    region_bounds: dict[str, tuple[float, float, float, float]],
):
    MapRegion, evaluate_unknown_world_map, thresholds = _quality_api()
    regions = tuple(
        MapRegion(
            name=name,
            min_x_m=bounds[0],
            min_y_m=bounds[1],
            max_x_m=bounds[2],
            max_y_m=bounds[3],
        )
        for name, bounds in region_bounds.items()
    )
    return evaluate_unknown_world_map(
        built_map_yaml=built_map_yaml,
        truth_map_yaml=truth_map_yaml,
        robot_start_xy=robot_start_xy,
        regions=regions,
        thresholds=thresholds,
    )


def test_legacy_20260719_map_is_incomplete_under_unknown_world_contract():
    legacy_report = json.loads(
        (LEGACY_FIXTURE / "legacy_report.json").read_text(encoding="utf-8")
    )

    report = _evaluate(
        LEGACY_FIXTURE / "built_map.yaml",
        LEGACY_FIXTURE / "truth_map.yaml",
        robot_start_xy=(0.0, 0.0),
        region_bounds={
            # floor 边界由 evaluator-only 场景清单减去出生点得到，不能传给机器人。
            "living_room": (-0.79, -0.79, 4.09, 4.09),
            "kitchen": (-0.79, 4.21, 4.09, 7.09),
            "meeting_room": (4.21, -0.79, 9.09, 4.09),
            "office": (4.21, 4.21, 9.09, 7.09),
        },
    )

    # 旧门槛只证明链路跑通；新门槛必须揭示厨房、办公室等大片漏建区域。
    assert legacy_report["passed"] is True
    assert report["passed"] is False
    assert 0.66 <= report["metrics"]["reachable_free_coverage_ratio"] <= 0.69
    assert report["metrics"]["reachable_unknown_ratio"] > 0.10
    region_coverage = report["metrics"]["region_coverage_ratios"]
    assert region_coverage["kitchen"] < 0.35
    assert region_coverage["office"] < 0.20
    assert min(region_coverage.values()) < 0.85


def test_complete_synthetic_map_passes_90_85_10_thresholds(tmp_path: Path):
    free_map = [[254] * 6 for _ in range(6)]
    truth_yaml = _write_map(tmp_path, "truth", free_map)
    built_yaml = _write_map(tmp_path, "built", free_map)

    report = _evaluate(
        built_yaml,
        truth_yaml,
        robot_start_xy=(0.5, 0.5),
        region_bounds={
            "west": (0.0, 0.0, 3.0, 6.0),
            "east": (3.0, 0.0, 6.0, 6.0),
        },
    )

    assert report["passed"] is True
    assert report["metrics"]["reachable_free_coverage_ratio"] == 1.0
    assert report["metrics"]["reachable_unknown_ratio"] == 0.0
    assert report["metrics"]["region_coverage_ratios"] == {
        "east": 1.0,
        "west": 1.0,
    }


def test_all_white_map_cannot_fake_coverage_when_truth_has_walls(tmp_path: Path):
    truth_pixels = [[254, 254, 254, 0, 254, 254] for _ in range(6)]
    truth_yaml = _write_map(tmp_path, "truth_with_wall", truth_pixels)
    built_yaml = _write_map(
        tmp_path,
        "unsafe_all_free",
        [[254] * 6 for _ in range(6)],
    )

    report = _evaluate(
        built_yaml,
        truth_yaml,
        robot_start_xy=(0.5, 0.5),
        region_bounds={"reachable_room": (0.0, 0.0, 3.0, 6.0)},
    )

    # free/unknown 三项都会很好看，但墙被涂成 free 必须独立失败。
    assert report["metrics"]["reachable_free_coverage_ratio"] == 1.0
    assert report["metrics"]["reachable_unknown_ratio"] == 0.0
    assert report["checks"]["obstacle_boundary_recall"] is False
    assert report["checks"]["obstacle_false_free_ratio"] is False
    assert report["passed"] is False


def test_matching_reachable_wall_passes_obstacle_fidelity(tmp_path: Path):
    pixels = [[254, 254, 254, 0, 254, 254] for _ in range(6)]
    truth_yaml = _write_map(tmp_path, "truth_wall", pixels)
    built_yaml = _write_map(tmp_path, "built_wall", pixels)

    report = _evaluate(
        built_yaml,
        truth_yaml,
        robot_start_xy=(0.5, 0.5),
        region_bounds={"reachable_room": (0.0, 0.0, 3.0, 6.0)},
    )

    assert report["passed"] is True
    assert report["metrics"]["obstacle_boundary_recall_ratio"] == 1.0
    assert report["metrics"]["obstacle_false_free_ratio"] == 0.0


def test_world_coordinates_align_maps_with_different_origins(tmp_path: Path):
    truth_yaml = _write_map(tmp_path, "truth", [[254] * 4 for _ in range(4)])
    padded = [[205] * 6 for _ in range(6)]
    for row in range(1, 5):
        for column in range(1, 5):
            padded[row][column] = 254
    built_yaml = _write_map(tmp_path, "built", padded, origin_xy=(-1.0, -1.0))

    # 地图原点会随 SLAM 扩图变化，按像素下标直接比较会产生虚假漏建。
    report = _evaluate(
        built_yaml,
        truth_yaml,
        robot_start_xy=(0.5, 0.5),
        region_bounds={"all": (0.0, 0.0, 4.0, 4.0)},
    )

    assert report["passed"] is True
    assert report["metrics"]["reachable_free_coverage_ratio"] == 1.0
    assert report["metrics"]["reachable_unknown_ratio"] == 0.0
    assert report["metrics"]["region_coverage_ratios"] == {"all": 1.0}
