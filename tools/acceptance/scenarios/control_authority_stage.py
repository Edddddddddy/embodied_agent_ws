#!/usr/bin/env python3
"""启动最小 ROS graph，验证多控制源权限与速度安全门。"""

from __future__ import annotations

from pathlib import Path

from tools.acceptance.paths import repository_root
from tools.acceptance.session import AcceptanceSession, AcceptanceSessionConfig


def main() -> int:
    workspace = repository_root(Path(__file__))
    config = AcceptanceSessionConfig(
        name="control-authority-stage",
        workspace=workspace,
        artifact_root=workspace / "logs" / "acceptance" / "control_authority",
        timeout_s=60.0,
        inherit_ros_domain_id=False,
    )
    with AcceptanceSession(config) as session:
        runtime_log = session.log_path("runtime")
        session.spawn(
            "twist_mux",
            [
                "ros2",
                "run",
                "twist_mux",
                "twist_mux",
                "--ros-args",
                "--params-file",
                str(
                    workspace
                    / "src/embodied_simulation/config/twist_mux.yaml"
                ),
                "-r",
                "/cmd_vel_out:=/control/autonomy/cmd_vel",
            ],
            log_path=runtime_log,
        )
        authority_handle = session.spawn(
            "control_authority",
            [
                "ros2",
                "run",
                "embodied_agent_cpp",
                "control_authority",
            ],
            log_path=runtime_log,
        )
        # 探针需要暂停/重启真实 manager，才能验证 state lease，而不是伪造一条状态消息。
        session.environment["CONTROL_AUTHORITY_PROCESS_GROUP_ID"] = str(
            authority_handle.process_group_id
        )
        session.spawn(
            "velocity_authority_gate",
            [
                "ros2",
                "run",
                "embodied_agent_cpp",
                "velocity_authority_gate",
            ],
            log_path=runtime_log,
        )
        session.run(
            [
                "bash",
                str(workspace / "tools/acceptance/run_probe.sh"),
                str(
                    workspace
                    / "tools/acceptance/probes/control/"
                    "control_authority_pipeline.py"
                ),
            ],
            timeout_s=35.0,
        )
        print(f"Evidence log: {runtime_log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
