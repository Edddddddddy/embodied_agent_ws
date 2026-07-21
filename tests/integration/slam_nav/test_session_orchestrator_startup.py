"""SLAM/Nav2 会话探针启动阶段的失败快速反馈契约。"""

from __future__ import annotations

import time

import pytest
from embodied_agent_interfaces.msg import (
    SlamMappingCompletionEvidence,
    SlamSessionState,
)
from rclpy.executors import ExternalShutdownException

from tools.acceptance.probes.slam_nav.session_runtime import (
    run_cli,
    spin_executor_until_shutdown,
    wait_for_mapping_startup,
)
from tools.acceptance.probes.slam_nav.mapping_completion_adapter import (
    mapping_completion_message_to_dict,
)


class _FailedObserver:
    current_phase = SlamSessionState.FAILED
    current_detail = "mapping process exited code=1"

    @staticmethod
    def has_phase(_phase: int) -> bool:
        return False


def test_mapping_startup_raises_failed_detail_without_waiting_for_timeout():
    started_at = time.monotonic()

    with pytest.raises(RuntimeError) as raised:
        wait_for_mapping_startup(_FailedObserver(), timeout_s=5.0)

    assert "mapping process exited code=1" in str(raised.value)
    assert time.monotonic() - started_at < 1.0


def test_executor_external_shutdown_is_treated_as_normal_teardown():
    class _ExternallyStoppedExecutor:
        @staticmethod
        def spin() -> None:
            raise ExternalShutdownException()

    spin_executor_until_shutdown(_ExternallyStoppedExecutor())


def test_cli_keyboard_interrupt_exits_without_a_traceback():
    def _interrupt() -> None:
        raise KeyboardInterrupt

    with pytest.raises(SystemExit) as raised:
        run_cli(_interrupt)

    assert raised.value.code == 130
    assert raised.value.__suppress_context__ is True


def test_typed_mapping_completion_adapter_preserves_return_checks():
    message = SlamMappingCompletionEvidence()
    message.mode = SlamMappingCompletionEvidence.MODE_BOUNDED_SATURATION
    message.valid = True
    message.trigger_reason = "time_budget_exhausted"
    message.low_yield_epoch_count = 2
    message.required_low_yield_epoch_count = 2
    message.residual_available_frontiers = 4
    message.ledger_drained = True
    message.final_probe_gain_cells = 8
    message.final_probe_gain_ratio = 0.0003
    message.typed_stop_succeeded = True
    message.return_home_valid = True
    for field in (
        "return_typed_action_command",
        "return_typed_action_succeeded",
        "return_before_map_save",
        "return_same_pose_frame",
        "return_final_pose_in_window",
        "return_xy_within_tolerance",
        "return_yaw_within_tolerance",
        "return_cmd_vel_after_return",
        "return_final_cmd_vel_fresh",
        "return_final_cmd_vel_zero",
    ):
        setattr(message, field, True)
    message.return_xy_error_m = 0.12
    message.return_yaw_error_rad = 0.08
    message.return_max_xy_error_m = 0.35
    message.return_max_yaw_error_rad = 0.35
    message.return_zero_velocity_tolerance = 0.001
    message.return_max_cmd_vel_age_s = 1.0

    adapted = mapping_completion_message_to_dict(message)

    assert adapted is not None
    assert adapted["mode"] == "bounded_saturation"
    assert adapted["valid"] is True
    assert adapted["return_home"]["passed"] is True
    assert adapted["return_home"]["failed_checks"] == []
