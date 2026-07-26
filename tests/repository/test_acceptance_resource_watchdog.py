"""重型验收资源看门狗的纯 Python 契约。"""

import json

from tools.acceptance.resource_watchdog import (
    ResourcePressurePolicy,
    ResourcePressureTracker,
    ResourceWatchdog,
    ResourceWatchdogConfig,
)
from tools.acceptance.visual_runtime import LinuxMemorySnapshot


def _sample(available_mib: int, *, swap_free_mib: int = 2048):
    return LinuxMemorySnapshot(
        total_kib=8 * 1024 * 1024,
        available_kib=available_mib * 1024,
        swap_total_kib=2048 * 1024,
        swap_free_kib=swap_free_mib * 1024,
    )


def test_pressure_tracker_requires_consecutive_bad_samples():
    tracker = ResourcePressureTracker(
        ResourcePressurePolicy(
            minimum_available_kib=768 * 1024,
            maximum_swap_used_ratio=0.90,
            consecutive_samples=3,
        )
    )

    assert tracker.observe(_sample(600)) is None
    assert tracker.observe(_sample(900)) is None
    assert tracker.observe(_sample(600)) is None
    assert tracker.observe(_sample(500, swap_free_mib=50)) is None
    failure = tracker.observe(_sample(400, swap_free_mib=0))

    assert failure is not None
    assert "resource_exhaustion" in failure
    assert "available_mib=400.0" in failure


def test_watchdog_streams_samples_and_keeps_only_summary_in_memory(tmp_path):
    samples = iter(
        [
            _sample(1200),
            _sample(700, swap_free_mib=100),
            _sample(600, swap_free_mib=50),
        ]
    )
    failures = []
    output = tmp_path / "resource_samples.jsonl"
    watchdog = ResourceWatchdog(
        config=ResourceWatchdogConfig(
            interval_s=5.0,
            policy=ResourcePressurePolicy(
                minimum_available_kib=768 * 1024,
                maximum_swap_used_ratio=0.90,
                consecutive_samples=2,
            ),
        ),
        session_id="session-1",
        output_path=output,
        on_failure=failures.append,
        memory_sampler=lambda: next(samples),
        process_sampler=lambda _session_id: (3, 512 * 1024),
    )

    watchdog.sample_once()
    watchdog.sample_once()
    watchdog.sample_once()

    records = [
        json.loads(line)
        for line in output.read_text(encoding="utf-8").splitlines()
    ]
    assert len(records) == 3
    assert records[-1]["session_process_count"] == 3
    assert records[-1]["session_rss_mib"] == 512.0
    assert failures and "resource_exhaustion" in failures[0]
    assert watchdog.summary.sample_count == 3
    assert watchdog.summary.peak_session_rss_kib == 512 * 1024
    assert watchdog.summary.latest_memory.available_kib == 600 * 1024
