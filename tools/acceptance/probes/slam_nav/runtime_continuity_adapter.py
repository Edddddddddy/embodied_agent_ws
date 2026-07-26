"""将进程连续性深模块接入 SLAM/Nav2 探针。"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import MutableMapping

from tools.acceptance.runtime_continuity import (
    build_runtime_continuity_evidence,
    capture_persistent_runtime_checkpoint,
)
from tools.acceptance.runtime_identity import RuntimeCheckpoint


@dataclass
class RuntimeContinuityProbe:
    """拥有两个阶段快照，避免主探针重复进程审计细节。"""

    enabled: bool
    require_rviz: bool
    agent_mode: str
    mapping_ready: RuntimeCheckpoint | None = None
    navigation_ready: RuntimeCheckpoint | None = None

    @classmethod
    def from_flags(
        cls,
        *,
        enabled: bool,
        require_rviz: bool,
        agent_mode: str,
    ) -> "RuntimeContinuityProbe":
        if require_rviz and not enabled:
            raise ValueError(
                "--require-rviz-continuity requires "
                "--require-runtime-continuity"
            )
        if agent_mode not in {"offline", "online"}:
            raise ValueError("persistent Agent mode must be offline or online")
        return cls(
            enabled=enabled,
            require_rviz=require_rviz,
            agent_mode=agent_mode,
        )

    def progress_label(self, *, unknown_world: bool) -> str:
        if self.enabled:
            return "showcase-gazebo-e2e"
        return "unknown-world-slam-e2e" if unknown_world else "slam-nav-e2e"

    def _capture(self, label: str) -> RuntimeCheckpoint:
        return capture_persistent_runtime_checkpoint(
            label,
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
            gz_partition=os.environ.get("GZ_PARTITION", ""),
            require_rviz=self.require_rviz,
            agent_mode=self.agent_mode,
        )

    def capture_mapping_ready(self) -> None:
        if self.enabled:
            # 此时 launch 已越过 fork→exec，身份采集仍会要求两次样本一致。
            self.mapping_ready = self._capture("mapping_ready")

    def capture_navigation_ready(self) -> None:
        if self.enabled:
            self.navigation_ready = self._capture("navigation_ready")

    def attach_to(self, core_report: MutableMapping[str, object]) -> None:
        if not self.enabled:
            return
        if self.mapping_ready is None or self.navigation_ready is None:
            raise RuntimeError(
                "persistent runtime continuity evidence is missing"
            )
        # Agent 保存唤醒、会话和命令队列状态，必须与 Gazebo 一样跨阶段常驻；
        # 只证明仿真底座未重启，不能证明“同一语音会话”。
        roles = [
            "gazebo_server",
            f"{self.agent_mode}_agent",
            "robot_state_publisher",
        ]
        if self.require_rviz:
            roles.append("rviz")
        evidence = build_runtime_continuity_evidence(
            self.mapping_ready,
            self.navigation_ready,
            required_roles=roles,
        )
        # 只添加一项附加 gate；原 schema v4 的地图/定位/导航判定保持原样。
        core_report["runtime_continuity"] = evidence
        checks = core_report.get("checks")
        if not isinstance(checks, MutableMapping):
            raise RuntimeError("strict report checks are missing")
        checks["runtime_continuity"] = bool(evidence["passed"])
        core_report["passed"] = bool(
            core_report.get("passed") and evidence["passed"]
        )
