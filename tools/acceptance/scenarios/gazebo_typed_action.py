#!/usr/bin/env python3
"""在受控 ROS overlay 中运行 typed Action -> Gazebo 物理运动验收。"""

from __future__ import annotations

from pathlib import Path

from tools.acceptance.session import (
    AcceptanceSession,
    AcceptanceSessionConfig,
    RosEnvironmentIsolation,
)


ROOT = Path(__file__).resolve().parents[3]
SESSION_TIMEOUT_S = 110.0
SMOKE_TIMEOUT_S = 95.0


def run(*, workspace: Path = ROOT) -> int:
    """隔离宿主源码工作区后复用既有 Gazebo smoke。

    用户可能在同一终端 source 过自行编译的 Nav2。若该 overlay 与当前 Jazzy
    二进制 ABI 不一致，Lifecycle Manager 会在 configure 前退出。这里复用重型
    SLAM 门禁同一套来源策略，只保留当前 install、系统 ROS 和白名单 Frontier。
    """

    resolved_workspace = workspace.resolve()
    default_policy = RosEnvironmentIsolation()
    isolation_policy = RosEnvironmentIsolation(
        required_packages=(
            *default_policy.required_packages,
            "embodied_simulation",
            "embodied_agent_interfaces",
        ),
        audited_packages=(
            *default_policy.audited_packages,
            "embodied_simulation",
            "embodied_agent_interfaces",
        ),
    )
    config = AcceptanceSessionConfig(
        name="gazebo-typed-action",
        workspace=resolved_workspace,
        artifact_root=resolved_workspace / "logs/acceptance/gazebo_typed_action",
        # 内层最坏预算约 81 s（readiness + discovery + move/stop）。
        # 额外预算留给低性能 WSL 调度和 AcceptanceSession 的有界清理。
        timeout_s=SESSION_TIMEOUT_S,
        inherit_ros_domain_id=False,
        environment={
            # 隔离后的环境已经验证当前 install；子 smoke 再次 source activate.sh
            # 时允许幂等短路，避免 setup.bash 把已剔除的宿主 underlay 加回来。
            "EMBODIED_ACTIVE_WORKSPACE": str(resolved_workspace),
        },
        ros_environment_isolation=isolation_policy,
    )
    with AcceptanceSession(config) as session:
        session.run(
            ["bash", "scripts/smoke_test_gazebo_typed_action.sh"],
            timeout_s=SMOKE_TIMEOUT_S,
        )
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
