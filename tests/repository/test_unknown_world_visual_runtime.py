"""未知世界长时 GUI 验收的资源策略契约。"""

from tools.acceptance.visual_runtime import (
    GraphicsProbe,
    LinuxMemorySnapshot,
    decide_visual_runtime,
)


def _memory(*, total_gib: float, available_gib: float) -> LinuxMemorySnapshot:
    gib = 1024 * 1024
    return LinuxMemorySnapshot(
        total_kib=int(total_gib * gib),
        available_kib=int(available_gib * gib),
        swap_total_kib=2 * gib,
        swap_free_kib=2 * gib,
    )


def test_dual_gui_is_downgraded_on_small_wsl_even_with_acceleration():
    decision = decide_visual_runtime(
        requested_headless=False,
        use_rviz=True,
        graphics=GraphicsProbe(
            renderer="D3D12 (NVIDIA GeForce RTX 3060 Ti)",
            accelerated=True,
            driver_override="d3d12",
        ),
        memory=_memory(total_gib=8.0, available_gib=5.0),
        allow_dual_gui=False,
    )

    assert decision.effective_headless is True
    assert decision.mode == "rviz_only"
    assert decision.driver_override == "d3d12"
    assert decision.downgraded is True
    assert decision.environment()["HEADLESS"] == "true"
    assert decision.environment()["USE_RVIZ"] == "true"
    assert decision.environment()["GALLIUM_DRIVER"] == "d3d12"


def test_dual_gui_is_downgraded_when_only_software_rendering_is_available():
    decision = decide_visual_runtime(
        requested_headless=False,
        use_rviz=True,
        graphics=GraphicsProbe(
            renderer="llvmpipe (LLVM 19.1.7, 256 bits)",
            accelerated=False,
        ),
        memory=_memory(total_gib=16.0, available_gib=10.0),
        allow_dual_gui=False,
    )

    assert decision.effective_headless is True
    assert decision.mode == "rviz_only"
    assert "software_renderer" in decision.reason


def test_explicit_dual_gui_override_is_auditable():
    decision = decide_visual_runtime(
        requested_headless=False,
        use_rviz=True,
        graphics=GraphicsProbe(renderer="llvmpipe", accelerated=False),
        memory=_memory(total_gib=8.0, available_gib=5.0),
        allow_dual_gui=True,
    )

    assert decision.effective_headless is False
    assert decision.mode == "dual_gui"
    assert decision.reason == "explicit_dual_gui_override"


def test_existing_rviz_only_request_is_not_reported_as_downgrade():
    decision = decide_visual_runtime(
        requested_headless=True,
        use_rviz=True,
        graphics=GraphicsProbe(renderer="llvmpipe", accelerated=False),
        memory=_memory(total_gib=8.0, available_gib=5.0),
        allow_dual_gui=False,
    )

    assert decision.effective_headless is True
    assert decision.mode == "rviz_only"
    assert decision.downgraded is False
