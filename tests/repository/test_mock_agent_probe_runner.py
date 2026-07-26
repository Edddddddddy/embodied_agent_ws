from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_mock_agent_scenarios_share_one_process_runner():
    runner = ROOT / "tools/acceptance/run_mock_agent_probe.sh"
    source = runner.read_text(encoding="utf-8")

    subprocess.run(["bash", "-n", str(runner)], check=True)
    for scenario in (
        "continuous)",
        "soak)",
        "endpoint)",
        "multi-command)",
        "queue-full)",
        "ttl)",
        "timeout)",
        "kws)",
        "navigation)",
        "navigation-natural)",
    ):
        assert scenario in source
    assert "activate_lifecycle_node action_guard" in source
    assert "wait_for_topic_subscribers /robot/action_command_typed" in source
    assert 'kill -TERM -- "-$pid"' in source


def test_replaced_continuous_smoke_wrappers_do_not_return():
    removed = (
        "smoke_test_continuous_voice.sh",
        "smoke_test_continuous_voice_soak.sh",
        "smoke_test_continuous_endpoint_asr.sh",
        "smoke_test_continuous_multi_command.sh",
        "smoke_test_continuous_queue_full.sh",
        "smoke_test_continuous_command_ttl.sh",
        "smoke_test_continuous_session_timeout.sh",
        "smoke_test_continuous_kws_sidecar.sh",
        "smoke_test_continuous_navigation_queue.sh",
        "smoke_test_continuous_navigation_natural.sh",
    )

    assert all(not (ROOT / "scripts" / name).exists() for name in removed)
    searchable_roots = (
        ROOT / ".github",
        ROOT / "docs",
        ROOT / "scripts",
        ROOT / "tools",
    )
    current_sources = "\n".join(
        path.read_text(encoding="utf-8", errors="replace")
        for source_root in searchable_roots
        if source_root.exists()
        for path in source_root.rglob("*")
        if path.is_file() and ".git" not in path.parts
    )
    assert all(name not in current_sources for name in removed)


def test_handlers_route_continuous_probes_through_shared_runner():
    handlers = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            ROOT / "tools/acceptance/handlers/common.sh",
            ROOT / "tools/acceptance/handlers/voice.sh",
            ROOT / "tools/acceptance/handlers/slam_nav.sh",
        )
    )

    assert "tools/acceptance/run_mock_agent_probe.sh" in handlers
    assert "scripts/smoke_test_continuous_" not in handlers
