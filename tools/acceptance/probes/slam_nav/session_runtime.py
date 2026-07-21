"""SLAM/Nav2 验收探针的启动等待与进程收口策略。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from embodied_agent_interfaces.msg import SlamSessionState
from rclpy.executors import ExternalShutdownException

from tools.acceptance.probes.slam_nav.session_actions import wait_until


class MappingStartupObserver(Protocol):
    current_phase: int
    current_detail: str

    def has_phase(self, phase: int) -> bool: ...


class SpinExecutor(Protocol):
    def spin(self) -> None: ...


def wait_for_mapping_startup(
    node: MappingStartupObserver, timeout_s: float
) -> None:
    """等待建图就绪，并把启动失败原因为即时错误返回给操作者。"""

    # FAILED 与 MAPPING 同为启动等待终态；否则底层进程已退出后仍会空等整段门禁超时。
    wait_until(
        lambda: (
            node.has_phase(SlamSessionState.MAPPING)
            or node.current_phase == SlamSessionState.FAILED
        ),
        timeout_s,
        "orchestrator did not enter MAPPING",
    )
    if node.current_phase == SlamSessionState.FAILED:
        detail = str(node.current_detail).strip() or "no failure detail"
        raise RuntimeError(
            f"orchestrator entered FAILED before MAPPING: {detail}"
        )


def spin_executor_until_shutdown(executor: SpinExecutor) -> None:
    """运行 ROS executor；外部 context 关闭属于正常收口而不是线程故障。"""

    try:
        executor.spin()
    except ExternalShutdownException:
        # Ctrl-C 会先关闭 rclpy context；后台线程不应再打印误导性 traceback。
        return


def run_cli(entrypoint: Callable[[], None]) -> None:
    """运行探针 CLI，并把用户/外部中断转换成无 traceback 的标准退出。"""

    try:
        entrypoint()
    except (KeyboardInterrupt, ExternalShutdownException):
        raise SystemExit(130) from None
