from __future__ import annotations

from collections import deque
import json
import subprocess
from pathlib import Path

import yaml

from tools.acceptance.catalog import MODE_BY_NAME


ROOT = Path(__file__).resolve().parents[2]
SIMULATION = ROOT / "src/embodied_simulation"
SPEC = SIMULATION / "config/showcase_apartment.yaml"
MAP = SIMULATION / "maps/showcase_apartment.pgm"
SLAM_FRAME_MAP_YAML = SIMULATION / "maps/showcase_apartment_slam_frame.yaml"
STATIC_PLACES = SIMULATION / "config/showcase_places.yaml"
MAPPING_PLACES = SIMULATION / "config/showcase_mapping_places.yaml"
WORKPLACE_MISSION = SIMULATION / "config/showcase_workplace_mission.yaml"
DYNAMIC_SCENARIO = (
    ROOT
    / "src/embodied_navigation/config/showcase_dynamic_obstacle_scenario.json"
)


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
    assert spec["dynamic_obstacles"][0]["name"] == "crossing_cart"
    world = (
        SIMULATION / "worlds/showcase_apartment.sdf.xacro"
    ).read_text(encoding="utf-8")
    assert '<model name="crossing_cart"><static>true</static>' in world


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


def test_showcase_shell_exposes_fresh_map_navigation_and_internal_diagnostic():
    script = (ROOT / "scripts/voice_slam_nav_showcase.sh").read_text(encoding="utf-8")
    continuous = (ROOT / "scripts/continuous_nav2_voice_control.sh").read_text(encoding="utf-8")
    for stage in (
        "auto)", "mapping)", "save)", "navigation)", "audit)"
    ):
        assert stage in script
    assert "map_saver_cli" in script
    assert 'NAV2_SLAM="${NAV2_SLAM:-false}"' in continuous
    assert 'EMBODIED_NAV2_PLACES_FILE' in continuous
    assert 'NAV2_EXECUTOR_PLUGIN="${NAV2_EXECUTOR_PLUGIN:-embodied_simulation/Nav2RobotExecutor}"' in continuous
    assert 'embodied_simulation/GazeboRobotExecutor' in script
    assert '"$MAPPING_PLACES" 0.0 0.0 0.0' in script
    assert 'NAV2_SPAWN_X="${NAV2_SPAWN_X:--4.15}"' in script
    assert 'add_launch_arg x_pose "$SPAWN_X"' in continuous
    assert '--x "$INITIAL_X" --y "$INITIAL_Y"' in continuous
    assert "voice_slam_session_orchestrator" in script
    help_text = script.split("EOF", 1)[0]
    assert "navigation-static" not in script
    assert MODE_BY_NAME["slam-nav-e2e"].public is False
    assert MODE_BY_NAME["unknown-world-slam-e2e"].public is False
    assert MODE_BY_NAME["verify"].public is True


def test_showcase_dynamic_obstacle_scenario_binds_visible_actor_and_new_map_frame():
    scenario = json.loads(DYNAMIC_SCENARIO.read_text(encoding="utf-8"))
    spec = yaml.safe_load(SPEC.read_text(encoding="utf-8"))
    actor_names = {item["name"] for item in spec["dynamic_obstacles"]}
    assert scenario["entity_name"] in actor_names
    assert scenario["world_name"] == spec["world"]["name"]
    assert scenario["map_to_world_translation"] == {
        "x": spec["world"]["spawn"]["x"],
        "y": spec["world"]["spawn"]["y"],
    }
    assert scenario["path_relative_motion"]["path_fraction"] > 0.0
    assert scenario["path_relative_motion"]["path_fraction"] < 1.0
    assert len(scenario["fallback_goals"]) >= 2
    assert all(
        {"name", "x", "y"}.issubset(goal)
        for goal in [scenario["goal"], *scenario["fallback_goals"]]
    )
    assert (
        scenario["path_relative_motion"]["minimum_anchor_lateral_clearance_m"]
        >= 0.8
    )
    assert scenario["warmup"]["sample_count"] >= 4
    assert scenario["navigation"]["sample_count"] >= 4
    assert scenario["route_selection"]["dynamic_path_retry_attempts"] >= 2
    assert scenario["route_selection"]["reset_wait_s"] > 1.0
    assert scenario["thresholds"]["minimum_unique_navigation_plans"] >= 2


def test_session_orchestrator_uses_canonical_readiness_topic():
    source = (
        ROOT
        / "src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py"
    ).read_text(encoding="utf-8")
    assert '"readiness_topic", "/system/readiness"' in source
    assert '"/agent/system_readiness"' not in source


def test_workplace_mission_covers_mapping_and_multiple_semantic_targets():
    mission = yaml.safe_load(WORKPLACE_MISSION.read_text(encoding="utf-8"))
    places = _read_compact_places(MAPPING_PLACES)
    route = mission["mapping_route"]
    bootstrap_route = mission["automatic_exploration"]["bootstrap_route"]
    navigation = mission["navigation_mission"]
    acceptance = mission["acceptance"]

    assert mission["schema_version"] == 1
    assert len(route) >= 12
    assert {step["action"] for step in route} == {"move", "turn"}
    assert all(step["label"] and step["text"] for step in route)
    labels = " ".join(step["label"] for step in route)
    assert all(room in labels for room in ("客厅", "厨房", "中央走廊", "办公室"))
    assert len(bootstrap_route) >= 5
    assert {step["action"] for step in bootstrap_route} == {"move", "turn"}
    assert all(step["label"] and step["text"] for step in bootstrap_route)
    assert "充电" in bootstrap_route[0]["label"]
    assert "中央" in " ".join(step["label"] for step in bootstrap_route)
    assert len(navigation["expected_targets"]) >= 3
    assert set(navigation["expected_targets"]).issubset(places)
    assert acceptance["min_mapping_path_m"] >= 10.0
    assert acceptance["min_navigation_targets"] >= 3
