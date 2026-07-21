"""强类型建图收口消息到 evaluator 领域结构的单一 Adapter。"""

from __future__ import annotations


def mapping_completion_message_to_dict(message) -> dict[str, object] | None:
    """保留逐项返航/饱和证据；不在 ROS 观察器里复制判定规则。"""

    mode = int(getattr(message, "mode", 0))
    if mode == 0:
        return None
    return_checks = {
        "typed_action_command": bool(message.return_typed_action_command),
        "typed_action_succeeded": bool(message.return_typed_action_succeeded),
        "return_before_map_save": bool(message.return_before_map_save),
        "same_pose_frame": bool(message.return_same_pose_frame),
        "final_pose_in_return_window": bool(
            message.return_final_pose_in_window
        ),
        "xy_within_tolerance": bool(message.return_xy_within_tolerance),
        "yaw_within_tolerance": bool(message.return_yaw_within_tolerance),
        "cmd_vel_after_return": bool(message.return_cmd_vel_after_return),
        "final_cmd_vel_fresh": bool(message.return_final_cmd_vel_fresh),
        "final_cmd_vel_zero": bool(message.return_final_cmd_vel_zero),
    }
    failed = [name for name, passed in return_checks.items() if not passed]
    return_home = {
        "schema_version": 1,
        "passed": bool(message.return_home_valid),
        "checks": return_checks,
        "failed_checks": failed,
        "thresholds": {
            "max_xy_error_m": float(message.return_max_xy_error_m),
            "max_yaw_error_rad": float(message.return_max_yaw_error_rad),
            "zero_velocity_tolerance": float(
                message.return_zero_velocity_tolerance
            ),
            "max_cmd_vel_age_s": float(message.return_max_cmd_vel_age_s),
        },
        "errors": {
            "xy_error_m": float(message.return_xy_error_m),
            "yaw_error_rad": float(message.return_yaw_error_rad),
        },
    }
    return {
        "mode": (
            "bounded_saturation"
            if mode == getattr(message, "MODE_BOUNDED_SATURATION", 2)
            else "strict_frontier"
        ),
        "valid": bool(message.valid),
        "trigger_reason": str(message.trigger_reason),
        "low_yield_epoch_count": int(message.low_yield_epoch_count),
        "required_low_yield_epoch_count": int(
            message.required_low_yield_epoch_count
        ),
        "residual_available_frontiers": int(
            message.residual_available_frontiers
        ),
        "ledger_drained": bool(message.ledger_drained),
        "final_probe_gain_cells": int(message.final_probe_gain_cells),
        "final_probe_gain_ratio": float(message.final_probe_gain_ratio),
        "typed_stop_succeeded": bool(message.typed_stop_succeeded),
        "return_home": return_home,
        "detail": str(message.detail),
    }
