#!/usr/bin/env python3
"""同一 Gazebo/RViz/机器人底座上的建图→导航持久演示门禁。"""

from tools.acceptance.scenarios.unknown_world_run_profile import (
    UnknownWorldRunProfile,
)
from tools.acceptance.scenarios.unknown_world_slam_e2e import run


def main() -> int:
    """复用 strict unknown-world 全证据，只额外启用运行时连续性门禁。"""

    return run(
        UnknownWorldRunProfile.synthetic(),
        persistent_runtime_enabled=True,
    )


if __name__ == "__main__":
    raise SystemExit(main())
