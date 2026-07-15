#!/usr/bin/env python3
"""从单一 YAML 清单生成 Gazebo world、Nav2 静态地图和语义地点配置。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SPEC = ROOT / "src/embodied_simulation/config/showcase_apartment.yaml"
DEFAULT_WORLD = ROOT / "src/embodied_simulation/worlds/showcase_apartment.sdf.xacro"
DEFAULT_MAP = ROOT / "src/embodied_simulation/maps/showcase_apartment.pgm"
DEFAULT_MAP_YAML = ROOT / "src/embodied_simulation/maps/showcase_apartment.yaml"
DEFAULT_SLAM_FRAME_MAP_YAML = (
    ROOT / "src/embodied_simulation/maps/showcase_apartment_slam_frame.yaml"
)
DEFAULT_PLACES = ROOT / "src/embodied_simulation/config/showcase_places.yaml"
DEFAULT_MAPPING_PLACES = (
    ROOT / "src/embodied_simulation/config/showcase_mapping_places.yaml"
)


def _fmt(values) -> str:
    return " ".join(f"{float(value):.4f}" for value in values)


def _geometry_xml(item: dict) -> str:
    if item["shape"] == "box":
        return f"<box><size>{_fmt(item['size'])}</size></box>"
    if item["shape"] == "cylinder":
        return (
            f"<cylinder><radius>{float(item['radius']):.4f}</radius>"
            f"<length>{float(item['height']):.4f}</length></cylinder>"
        )
    raise ValueError(f"unsupported shape: {item['shape']}")


def _height(item: dict) -> float:
    return float(item["size"][2] if item["shape"] == "box" else item["height"])


def render_world(spec: dict) -> str:
    world = spec["world"]
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.9" xmlns:xacro="http://www.ros.org/wiki/xacro">',
        '  <xacro:arg name="headless" default="true"/>',
        f'  <world name="{world["name"]}">',
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
        '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
        '    <xacro:unless value="$(arg headless)">',
        '      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        '    </xacro:unless>',
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>',
        '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>',
        '    <scene><ambient>0.62 0.62 0.60 1</ambient><background>0.16 0.20 0.26 1</background><shadows>true</shadows></scene>',
        '    <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows><diffuse>0.86 0.84 0.78 1</diffuse><direction>-0.45 0.25 -0.86</direction></light>',
        '    <light name="living_light" type="point"><pose>-2.5 -1.5 2.8 0 0 0</pose><diffuse>1.0 0.82 0.65 1</diffuse><attenuation><range>8</range><constant>0.7</constant><linear>0.03</linear><quadratic>0.01</quadratic></attenuation></light>',
        '    <light name="office_light" type="point"><pose>2.5 2.2 2.8 0 0 0</pose><diffuse>0.76 0.86 1.0 1</diffuse><attenuation><range>8</range><constant>0.7</constant><linear>0.03</linear><quadratic>0.01</quadratic></attenuation></light>',
        '    <model name="ground_plane"><static>true</static><link name="link">',
        '      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>',
        f'      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><material><ambient>{_fmt(world["background"])}</ambient><diffuse>{_fmt(world["background"])}</diffuse></material></visual>',
        '    </link></model>',
        '    <model name="showcase_interior"><static>true</static><link name="interior">',
    ]
    for item in spec["objects"]:
        geometry = _geometry_xml(item)
        z = _height(item) / 2.0
        name = item["name"]
        if item.get("collision", True):
            lines.append(
                f'      <collision name="{name}_collision"><pose>{item["x"]:.4f} {item["y"]:.4f} {z:.4f} 0 0 0</pose><geometry>{geometry}</geometry></collision>'
            )
        lines.append(
            f'      <visual name="{name}_visual"><pose>{item["x"]:.4f} {item["y"]:.4f} {z:.4f} 0 0 0</pose><geometry>{geometry}</geometry><material><ambient>{_fmt(item["color"])}</ambient><diffuse>{_fmt(item["color"])}</diffuse><specular>0.18 0.18 0.18 1</specular></material></visual>'
        )
    lines.extend(
        [
            '    </link></model>',
            '    <physics name="1ms" type="ode"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>',
            '  </world>',
            '</sdf>',
            '',
        ]
    )
    return "\n".join(lines)


def _occupied(item: dict, x: float, y: float) -> bool:
    if not item.get("collision", True):
        return False
    if item["shape"] == "box":
        return (
            abs(x - float(item["x"])) <= float(item["size"][0]) / 2.0
            and abs(y - float(item["y"])) <= float(item["size"][1]) / 2.0
        )
    return math.hypot(x - float(item["x"]), y - float(item["y"])) <= float(item["radius"])


def render_pgm(spec: dict) -> str:
    bounds = spec["world"]["bounds"]
    resolution = float(spec["world"]["resolution"])
    width = round((bounds["max_x"] - bounds["min_x"]) / resolution)
    height = round((bounds["max_y"] - bounds["min_y"]) / resolution)
    rows = []
    for image_row in range(height):
        y = bounds["max_y"] - (image_row + 0.5) * resolution
        values = []
        for column in range(width):
            x = bounds["min_x"] + (column + 0.5) * resolution
            values.append("0" if any(_occupied(item, x, y) for item in spec["objects"]) else "254")
        rows.append(" ".join(values))
    return f"P2\n# generated by scripts/generate_showcase_scene.py\n{width} {height}\n255\n" + "\n".join(rows) + "\n"


def render_map_yaml(
    spec: dict, image_name: str, *, relative_to_spawn: bool = False
) -> str:
    world = spec["world"]
    bounds = world["bounds"]
    spawn = world["spawn"]
    offset_x = float(spawn["x"]) if relative_to_spawn else 0.0
    offset_y = float(spawn["y"]) if relative_to_spawn else 0.0
    return yaml.safe_dump(
        {
            "image": image_name,
            "mode": "trinary",
            "resolution": float(world["resolution"]),
            "origin": [
                float(bounds["min_x"]) - offset_x,
                float(bounds["min_y"]) - offset_y,
                0.0,
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        },
        sort_keys=False,
        allow_unicode=True,
    )


def render_places(spec: dict, *, relative_to_spawn: bool = False) -> str:
    # Nav2Places 是一个刻意保持轻量的 C++ 配置读取器，公开格式为单行 flow map。
    # 这里直接生成其稳定契约，而不是依赖 PyYAML 对 block/flow 风格的隐式选择。
    spawn = spec["world"]["spawn"]
    offset_x = float(spawn["x"]) if relative_to_spawn else 0.0
    offset_y = float(spawn["y"]) if relative_to_spawn else 0.0
    coordinate_note = (
        "SLAM 建图起点坐标系" if relative_to_spawn else "Gazebo 世界/静态地图坐标系"
    )
    lines = [
        "# 由 showcase_apartment.yaml 生成，请勿手工修改。",
        f"# 坐标语义：{coordinate_note}。",
        "frame_id: map",
        "places:",
    ]
    for name, data in spec["places"].items():
        lines.append(
            f"  {name}: {{x: {float(data['x']) - offset_x:.3f}, "
            f"y: {float(data['y']) - offset_y:.3f}, "
            f"yaw: {float(data['yaw']):.3f}}}"
        )
    return "\n".join(lines) + "\n"


def _validate(spec: dict) -> None:
    names = [item["name"] for item in spec["objects"]]
    if len(names) != len(set(names)):
        raise ValueError("scene object names must be unique")
    for name, place in spec["places"].items():
        # 语义目标点至少留出 0.26m Burger 半径；否则 Nav2 到点时会落入膨胀层。
        for angle_index in range(24):
            angle = angle_index * math.tau / 24.0
            x = float(place["x"]) + math.cos(angle) * 0.26
            y = float(place["y"]) + math.sin(angle) * 0.26
            if any(_occupied(item, x, y) for item in spec["objects"]):
                raise ValueError(f"place {name} lacks 0.26m collision clearance")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--check", action="store_true", help="只校验已提交产物是否可复现")
    args = parser.parse_args()
    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    _validate(spec)
    outputs = {
        DEFAULT_WORLD: render_world(spec),
        DEFAULT_MAP: render_pgm(spec),
        DEFAULT_MAP_YAML: render_map_yaml(spec, DEFAULT_MAP.name),
        # 自动回归使用完整确定性栅格模拟“已充分探索并保存”的 SLAM map frame，
        # 从而验证 spawn/world 与 AMCL/map 坐标拆分，而不依赖人工探索覆盖率。
        DEFAULT_SLAM_FRAME_MAP_YAML: render_map_yaml(
            spec, DEFAULT_MAP.name, relative_to_spawn=True
        ),
        DEFAULT_PLACES: render_places(spec),
        # slam_toolbox 默认把建图开始时的底盘作为 map 原点；第二套地点表把世界
        # 坐标平移到该原点，避免重启 AMCL 后所有语义目标整体偏移一个 spawn。
        DEFAULT_MAPPING_PLACES: render_places(spec, relative_to_spawn=True),
    }
    if args.check:
        stale = [str(path.relative_to(ROOT)) for path, content in outputs.items() if not path.exists() or path.read_text(encoding="utf-8") != content]
        if stale:
            print("stale showcase assets: " + ", ".join(stale), file=sys.stderr)
            return 1
        print("PASS: showcase scene assets are reproducible")
        return 0
    for path, content in outputs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        print(path.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
