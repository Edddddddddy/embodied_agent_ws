import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "system_readiness_check.py"
SPEC = importlib.util.spec_from_file_location("system_readiness_check", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_system_readiness_payload_preserves_actionable_component_lists():
    payload = MODULE.readiness_to_dict(
        SimpleNamespace(
            profile="voice_simulation",
            ready=False,
            required_components=["agent", "simulation_control"],
            ready_components=["agent"],
            missing_components=["simulation_control"],
            degraded_components=[],
            detail="required_component_missing_or_stale",
        )
    )

    assert payload["profile"] == "voice_simulation"
    assert payload["missing_components"] == ["simulation_control"]
    assert "BLOCKED" in MODULE.format_readiness(payload)
    assert "simulation_control" in MODULE.format_readiness(payload)


def test_system_readiness_format_reports_success_without_missing_items():
    rendered = MODULE.format_readiness(
        {
            "profile": "demo",
            "ready": True,
            "ready_components": ["agent", "action_guard"],
            "missing_components": [],
            "degraded_components": [],
            "detail": "all_required_components_ready",
        }
    )

    assert rendered.startswith("PASS: system readiness profile=demo")
    assert "missing: -" in rendered


def test_system_readiness_progress_explains_which_gate_is_blocking():
    rendered = MODULE.format_readiness_progress(
        {
            "profile": "voice_nav2",
            "ready": False,
            "ready_components": ["simulation_control", "typed_action_bridge"],
            "missing_components": ["agent", "action_guard"],
            "degraded_components": [],
            "detail": "required_component_missing_or_stale",
        },
        elapsed_s=15.2,
    )

    assert rendered.startswith("WAIT: system readiness profile=voice_nav2")
    assert "elapsed=15.2s" in rendered
    assert "missing=agent, action_guard" in rendered
    assert "Lifecycle" in rendered
