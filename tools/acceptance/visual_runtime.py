"""长时 Gazebo/RViz 验收的图形与 WSL 资源策略。

严格 SLAM 门禁需要持续十几分钟。WSLg 使用 ``llvmpipe`` 时同时打开
Gazebo GUI 与 RViz，会把渲染压力放到 CPU 和统一内存上；这里把“用户想看
什么”和“本机安全地能开什么”分开，避免场景脚本散落资源判断。
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
import subprocess
from typing import Mapping


_KIB_PER_GIB = 1024 * 1024


@dataclass(frozen=True, slots=True)
class LinuxMemorySnapshot:
    """从 ``/proc/meminfo`` 提取的稳定资源快照，单位统一为 KiB。"""

    total_kib: int
    available_kib: int
    swap_total_kib: int
    swap_free_kib: int

    @property
    def swap_used_ratio(self) -> float:
        if self.swap_total_kib <= 0:
            return 0.0
        used = max(0, self.swap_total_kib - self.swap_free_kib)
        return min(1.0, used / self.swap_total_kib)


@dataclass(frozen=True, slots=True)
class GraphicsProbe:
    """一次 OpenGL 探针；``None`` 表示环境无法可靠判断加速状态。"""

    renderer: str
    accelerated: bool | None
    driver_override: str | None = None
    detail: str = ""


@dataclass(frozen=True, slots=True)
class VisualRuntimeDecision:
    """请求配置到有效配置的可审计决策。"""

    requested_headless: bool
    use_rviz: bool
    effective_headless: bool
    mode: str
    reason: str
    renderer: str
    accelerated: bool | None
    driver_override: str | None = None
    memory_total_kib: int = 0
    memory_available_kib: int = 0

    @property
    def downgraded(self) -> bool:
        return (
            not self.requested_headless
            and self.effective_headless
            and self.use_rviz
        )

    def environment(self) -> dict[str, str]:
        """生成 launch 与 manifest 共用的唯一环境映射。"""

        values = {
            "HEADLESS": str(self.effective_headless).lower(),
            "USE_RVIZ": str(self.use_rviz).lower(),
            "ACCEPTANCE_GUI_REQUESTED_HEADLESS": str(
                self.requested_headless
            ).lower(),
            "ACCEPTANCE_GUI_EFFECTIVE_MODE": self.mode,
            "ACCEPTANCE_GUI_POLICY_REASON": self.reason,
            "ACCEPTANCE_GL_RENDERER": self.renderer,
            "ACCEPTANCE_GL_ACCELERATED": (
                "unknown"
                if self.accelerated is None
                else str(self.accelerated).lower()
            ),
            "ACCEPTANCE_MEMORY_TOTAL_MIB": (
                f"{self.memory_total_kib / 1024:.1f}"
            ),
            "ACCEPTANCE_MEMORY_AVAILABLE_MIB": (
                f"{self.memory_available_kib / 1024:.1f}"
            ),
        }
        if self.driver_override:
            values["GALLIUM_DRIVER"] = self.driver_override
        return values


def read_linux_memory(
    path: Path = Path("/proc/meminfo"),
) -> LinuxMemorySnapshot:
    """读取 Linux/WSL 总内存，不依赖 psutil。"""

    values: dict[str, int] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, remainder = line.partition(":")
        if not separator:
            continue
        match = re.match(r"\s*(\d+)", remainder)
        if match:
            values[key] = int(match.group(1))
    required = ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree")
    missing = [key for key in required if key not in values]
    if missing:
        raise ValueError(
            f"memory snapshot is missing fields: {', '.join(missing)}"
        )
    return LinuxMemorySnapshot(
        total_kib=values["MemTotal"],
        available_kib=values["MemAvailable"],
        swap_total_kib=values["SwapTotal"],
        swap_free_kib=values["SwapFree"],
    )


def _parse_glxinfo(output: str, *, driver_override: str | None) -> GraphicsProbe:
    renderer = "unknown"
    accelerated: bool | None = None
    for line in output.splitlines():
        label, separator, value = line.partition(":")
        if not separator:
            continue
        normalized = label.strip().lower()
        if normalized == "opengl renderer string":
            renderer = value.strip() or "unknown"
        elif normalized == "accelerated":
            rendered = value.strip().lower()
            if rendered in {"yes", "true"}:
                accelerated = True
            elif rendered in {"no", "false"}:
                accelerated = False
    if accelerated is None and "llvmpipe" in renderer.lower():
        accelerated = False
    return GraphicsProbe(
        renderer=renderer,
        accelerated=accelerated,
        driver_override=driver_override,
    )


def _run_glxinfo(
    environment: Mapping[str, str],
    *,
    driver_override: str | None,
) -> GraphicsProbe:
    probe_environment = dict(environment)
    if driver_override:
        probe_environment["GALLIUM_DRIVER"] = driver_override
    try:
        completed = subprocess.run(
            ["glxinfo", "-B"],
            env=probe_environment,
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
    except (FileNotFoundError, subprocess.SubprocessError) as error:
        return GraphicsProbe(
            renderer="unavailable",
            accelerated=None,
            driver_override=driver_override,
            detail=str(error),
        )
    output = f"{completed.stdout}\n{completed.stderr}"
    parsed = _parse_glxinfo(output, driver_override=driver_override)
    if completed.returncode == 0:
        return parsed
    return GraphicsProbe(
        renderer=parsed.renderer,
        accelerated=parsed.accelerated,
        driver_override=driver_override,
        detail=f"glxinfo exited with {completed.returncode}",
    )


def probe_graphics(
    environment: Mapping[str, str] | None = None,
) -> GraphicsProbe:
    """优先复用默认驱动；WSLg 可用 D3D12 时自动恢复硬件加速。

    ``GALLIUM_DRIVER=d3d12`` 只注入本次验收子树，不修改用户 shell 或系统配置。
    即使后续因内存预算降级到 RViz-only，这也能降低单 GUI 的 CPU 渲染压力。
    """

    base = dict(os.environ if environment is None else environment)
    current_override = base.get("GALLIUM_DRIVER")
    default = _run_glxinfo(base, driver_override=current_override)
    if default.accelerated is True:
        return default
    if Path("/dev/dxg").exists() and current_override != "d3d12":
        d3d12 = _run_glxinfo(base, driver_override="d3d12")
        if d3d12.accelerated is True:
            return d3d12
    return default


def decide_visual_runtime(
    *,
    requested_headless: bool,
    use_rviz: bool,
    graphics: GraphicsProbe,
    memory: LinuxMemorySnapshot,
    allow_dual_gui: bool,
) -> VisualRuntimeDecision:
    """为长时验收选择安全的可视化模式。

    双 GUI 只在硬件加速、内存余量充足时默认开放；显式 override 仍可用于
    高配机器调试，但会在 manifest 中留下审计原因。
    """

    dual_gui_requested = not requested_headless and use_rviz
    if not dual_gui_requested:
        mode = (
            "rviz_only"
            if requested_headless and use_rviz
            else "headless"
            if requested_headless
            else "gazebo_only"
        )
        return VisualRuntimeDecision(
            requested_headless=requested_headless,
            use_rviz=use_rviz,
            effective_headless=requested_headless,
            mode=mode,
            reason="requested_profile",
            renderer=graphics.renderer,
            accelerated=graphics.accelerated,
            driver_override=graphics.driver_override,
            memory_total_kib=memory.total_kib,
            memory_available_kib=memory.available_kib,
        )

    if allow_dual_gui:
        return VisualRuntimeDecision(
            requested_headless=False,
            use_rviz=True,
            effective_headless=False,
            mode="dual_gui",
            reason="explicit_dual_gui_override",
            renderer=graphics.renderer,
            accelerated=graphics.accelerated,
            driver_override=graphics.driver_override,
            memory_total_kib=memory.total_kib,
            memory_available_kib=memory.available_kib,
        )

    blockers: list[str] = []
    if graphics.accelerated is not True:
        blockers.append("software_renderer")
    if memory.total_kib < 12 * _KIB_PER_GIB:
        blockers.append("wsl_memory_below_12gib")
    if memory.available_kib < 6 * _KIB_PER_GIB:
        blockers.append("available_memory_below_6gib")
    if blockers:
        return VisualRuntimeDecision(
            requested_headless=False,
            use_rviz=True,
            effective_headless=True,
            mode="rviz_only",
            reason=",".join(blockers),
            renderer=graphics.renderer,
            accelerated=graphics.accelerated,
            driver_override=graphics.driver_override,
            memory_total_kib=memory.total_kib,
            memory_available_kib=memory.available_kib,
        )
    return VisualRuntimeDecision(
        requested_headless=False,
        use_rviz=True,
        effective_headless=False,
        mode="dual_gui",
        reason="accelerated_with_resource_headroom",
        renderer=graphics.renderer,
        accelerated=graphics.accelerated,
        driver_override=graphics.driver_override,
        memory_total_kib=memory.total_kib,
        memory_available_kib=memory.available_kib,
    )


def resolve_visual_runtime(
    *,
    requested_headless: bool,
    use_rviz: bool,
    environment: Mapping[str, str] | None = None,
    allow_dual_gui: bool = False,
) -> VisualRuntimeDecision:
    """执行只读探针并返回最终策略。"""

    memory = read_linux_memory()
    graphics = (
        GraphicsProbe(renderer="not_requested", accelerated=None)
        if requested_headless and not use_rviz
        else probe_graphics(environment)
    )
    return decide_visual_runtime(
        requested_headless=requested_headless,
        use_rviz=use_rviz,
        graphics=graphics,
        memory=memory,
        allow_dual_gui=allow_dual_gui,
    )
