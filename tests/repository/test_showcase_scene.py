from __future__ import annotations

from collections import deque
import subprocess
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SIMULATION = ROOT / "src/embodied_simulation"
SPEC = SIMULATION / "config/showcase_apartment.yaml"
MAP = SIMULATION / "maps/showcase_apartment.pgm"
SLAM_FRAME_MAP_YAML = SIMULATION / "maps/showcase_apartment_slam_frame.yaml"
STATIC_PLACES = SIMULATION / "config/showcase_places.yaml"
MAPPING_PLACES = SIMULATION / "config/showcase_mapping_places.yaml"


def _read_compact_places(path: Path) -> dict[str, dict[str, float]]:
    # 生产侧 C++ 读取器使用轻量 flow-map 契约；测试侧用 YAML 验证生成值。
    return yaml.safe_load(path.read_text(encoding="utf-8"))["places"]


def _read_p2(path: Path) -> tuple[int, int, list[list[int]]]:
    tokens = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            tokens.extend(line.split())
    assert tokens[0] == "P2"
    width, height, maximum = map(int, tokens[1:4])
    assert maximum == 255
    values = list(map(int, tokens[4:]))
    assert len(values) == width * height
    return width, height, [values[row * width : (row + 1) * width] for row in range(height)]


def _grid_cell(spec: dict, x: float, y: float) -> tuple[int, int]:
    bounds = spec["world"]["bounds"]
    resolution = float(spec["world"]["resolution"])
    column = int((x - bounds["min_x"]) / resolution)
    row = int((bounds["max_y"] - y) / resolution)
    return row, column


def _inflated_free_grid(grid: list[list[int]], radius_cells: int) -> list[list[bool]]:
    height, width = len(grid), len(grid[0])
    occupied = [(row, col) for row in range(height) for col in range(width) if grid[row][col] < 100]
    free = [[True] * width for _ in range(height)]
    for row, col in occupied:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                nr, nc = row + dy, col + dx
                if 0 <= nr < height and 0 <= nc < width:
                    free[nr][nc] = False
    return free


def test_showcase_assets_are_generated_from_one_manifest():
    result = subprocess.run(
        ["python3", str(ROOT / "scripts/generate_showcase_scene.py"), "--check"],
        cwd=ROOT,
        check=False,
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr
    spec = yaml.safe_load(SPEC.read_text(encoding="utf-8"))
    assert spec["schema_version"] == 1
    assert len(spec["objects"]) >= 30
    assert {item["category"] for item in spec["objects"]} >= {
        "floor", "wall", "furniture", "landmark"
    }
    assert len(spec["places"]) >= 8


def test_showcase_semantic_places_share_one_inflated_connected_free_space():
    spec = yaml.safe_load(SPEC.read_text(encoding="utf-8"))
    width, height, grid = _read_p2(MAP)
    assert (width, height) == (200, 160)
    resolution = float(spec["world"]["resolution"])
    free = _inflated_free_grid(grid, round(0.22 / resolution))
    start = _grid_cell(spec, spec["world"]["spawn"]["x"], spec["world"]["spawn"]["y"])
    assert free[start[0]][start[1]]

    reached = {start}
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nxt = row + dr, col + dc
            if (
                0 <= nxt[0] < height
                and 0 <= nxt[1] < width
                and free[nxt[0]][nxt[1]]
                and nxt not in reached
            ):
                reached.add(nxt)
                queue.append(nxt)

    for name, place in spec["places"].items():
        cell = _grid_cell(spec, place["x"], place["y"])
        assert cell in reached, f"semantic place {name} is not Nav2-reachable"


def test_mapping_places_are_relative_to_slam_start_pose():
    spec = yaml.safe_load(SPEC.read_text(encoding="utf-8"))
    static_places = _read_compact_places(STATIC_PLACES)
    mapping_places = _read_compact_places(MAPPING_PLACES)
    spawn = spec["world"]["spawn"]

    assert mapping_places["home"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    for name, place in static_places.items():
        assert mapping_places[name]["x"] == round(place["x"] - spawn["x"], 3)
        assert mapping_places[name]["y"] == round(place["y"] - spawn["y"], 3)
        assert mapping_places[name]["yaw"] == place["yaw"]

    slam_map = yaml.safe_load(SLAM_FRAME_MAP_YAML.read_text(encoding="utf-8"))
    bounds = spec["world"]["bounds"]
    assert slam_map["origin"] == [
        bounds["min_x"] - spawn["x"],
        bounds["min_y"] - spawn["y"],
        0.0,
    ]


def test_showcase_shell_exposes_mapping_save_and_navigation_stages():
    script = (ROOT / "scripts/voice_slam_nav_showcase.sh").read_text(encoding="utf-8")
    continuous = (ROOT / "scripts/continuous_nav2_voice_control.sh").read_text(encoding="utf-8")
    for stage in ("mapping)", "save)", "navigation)", "navigation-static)", "audit)"):
        assert stage in script
    assert "map_saver_cli" in script
    assert 'NAV2_SLAM="${NAV2_SLAM:-false}"' in continuous
    assert 'EMBODIED_NAV2_PLACES_FILE' in continuous
    assert 'NAV2_EXECUTOR_PLUGIN="${NAV2_EXECUTOR_PLUGIN:-embodied_simulation/Nav2RobotExecutor}"' in continuous
    assert 'embodied_simulation/GazeboRobotExecutor' in script
    assert '"$MAPPING_PLACES" 0.0 0.0 0.0' in script
    assert '"$STATIC_PLACES" -4.15 -3.15 0.0' in script
    assert 'NAV2_SPAWN_X="${NAV2_SPAWN_X:--4.15}"' in script
    assert 'add_launch_arg x_pose "$SPAWN_X"' in continuous
    assert '--x "$INITIAL_X" --y "$INITIAL_Y"' in continuous
