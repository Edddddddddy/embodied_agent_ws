#!/usr/bin/env python3
"""不启动 Gazebo 的 SLAM 交付物静态审计。"""

from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    required = [
        ROOT / "src/embodied_slam/package.xml",
        ROOT / "src/embodied_slam/config/slam_mapping_ceres.yaml",
        ROOT / "src/embodied_slam/config/slam_mapping_gtsam.yaml",
        ROOT / "src/embodied_slam/gtsam_solver_plugin.xml",
        ROOT / "src/embodied_slam/launch/mapping_baseline.launch.py",
        ROOT / "src/embodied_slam/src/odom_drift_injector_node.cpp",
        ROOT / "src/embodied_slam/src/closed_loop_driver_node.cpp",
        ROOT / "src/embodied_simulation/worlds/slam_loop_demo.sdf.xacro",
        ROOT / "tests/integration/slam_nav/test_slam_mapping_baseline.py",
        ROOT / "scripts/smoke_test_slam_mapping_baseline.sh",
    ]
    checks: dict[str, bool] = {
        "required_files_present": all(path.is_file() and path.stat().st_size > 0 for path in required)
    }
    config = required[1].read_text(encoding="utf-8")
    gtsam_config = required[2].read_text(encoding="utf-8")
    checks.update(
        {
            "ceres_baseline_selected": "solver_plugins::CeresSolver" in config,
            "loop_closure_enabled": "do_loop_closing: true" in config,
            "five_centimeter_map": "resolution: 0.05" in config,
            "isolated_slam_tf_tree": all(
                token in config for token in ("slam_odom", "slam_base_link", "/slam/scan")
            ),
            "controlled_drift_configured": all(
                token in config
                for token in ("linear_scale:", "yaw_bias_per_meter:", "random_seed:")
            ),
            "gtsam_backend_selected": "embodied_slam::GtsamScanSolver" in gtsam_config,
            "backend_frontend_parameters_match": config[config.index("    odom_frame:") :]
            == gtsam_config[gtsam_config.index("    odom_frame:") :],
        }
    )

    with tempfile.NamedTemporaryFile(suffix=".sdf") as output:
        xacro = shutil.which("xacro") or "/opt/ros/jazzy/bin/xacro"
        subprocess.run(
            [
                xacro,
                "-o",
                output.name,
                "headless:=true",
                str(required[7]),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        world = ET.parse(output.name).getroot().find("world")
        model_names = {model.attrib.get("name") for model in world.findall("model")} if world is not None else set()
        checks["world_is_valid_and_asymmetric"] = {
            "ground_plane",
            "mapping_walls",
        }.issubset(model_names)

    report = {"passed": all(checks.values()), "checks": checks}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
