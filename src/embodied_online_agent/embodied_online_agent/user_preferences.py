"""Apply speaker-specific preferences to deterministic robot actions.

用户记忆不能只停留在 prompt 文本里。这个模块把已经确认的用户偏好转换成
ActionCommand 参数变化，让在线/离线、fallback、NLU 多命令和模型输出都走同一套
策略层。真正的硬安全限幅仍然留给后面的 C++ ActionGuard。
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Mapping

from .types import ActionCommand


_SPEED_FACTORS = {
    "slow": 0.75,
    "fast": 1.15,
}
_DEFAULT_MOVE_DURATION_S = 1.0
_DEFAULT_TURN_DURATION_S = 2.6
_DEFAULT_TURN_DEGREES = 90.0
_SPIN_DURATION_S = 7.85


def apply_user_preferences(
    actions: Iterable[ActionCommand],
    preferences: Mapping[str, Any] | None,
) -> list[ActionCommand]:
    """Return actions adjusted by safe, deterministic user preferences.

    这里不直接修改输入对象，避免连续队列里的 preparsed action 被不同用户/不同轮次复用时
    发生“隐形串味”。所有调整都只作用于低层运动参数，不改变动作名称和 request_id。
    """

    preference_map = dict(preferences or {})
    return [_apply_to_action(action, preference_map) for action in actions]


def _apply_to_action(action: ActionCommand, preferences: Mapping[str, Any]) -> ActionCommand:
    if action.name in {"stop", "cancel_navigation"}:
        return action
    args = dict(action.arguments or {})

    if action.name == "move":
        _apply_default_move_duration(args, preferences)
        _scale_numeric_arg(args, "linear_x", _movement_speed_factor(preferences))
    elif action.name == "turn":
        default_turn_applied = _apply_default_turn_degrees(args, preferences)
        _scale_turn_speed_preserving_default_angle(
            args,
            _movement_speed_factor(preferences),
            force_preserve_angle=default_turn_applied,
        )
    elif action.name == "arc":
        _scale_arc_speed_preserving_shape(args, _movement_speed_factor(preferences))

    return ActionCommand(action.name, args, request_id=action.request_id)


def _apply_default_move_duration(args: dict[str, Any], preferences: Mapping[str, Any]) -> None:
    duration = _safe_float(preferences.get("default_move_duration_s"))
    if duration is None:
        return
    if "duration_s" not in args or _close_to(args.get("duration_s"), _DEFAULT_MOVE_DURATION_S):
        args["duration_s"] = _clamp(duration, 0.2, 10.0)


def _apply_default_turn_degrees(args: dict[str, Any], preferences: Mapping[str, Any]) -> bool:
    degrees = _safe_float(preferences.get("default_turn_degrees"))
    if degrees is None:
        return False
    if "duration_s" in args and not _close_to(args.get("duration_s"), _DEFAULT_TURN_DURATION_S):
        return False
    degrees = _clamp(degrees, 15.0, 360.0)
    args["duration_s"] = round(_DEFAULT_TURN_DURATION_S * degrees / _DEFAULT_TURN_DEGREES, 3)
    return True


def _scale_turn_speed_preserving_default_angle(
    args: dict[str, Any],
    factor: float,
    *,
    force_preserve_angle: bool = False,
) -> None:
    if _close_to(factor, 1.0):
        return
    original_duration = _safe_float(args.get("duration_s"))
    # “左转/右转/原地转一圈”这类默认角度动作，降速时同步拉长时间，避免偏好把 90 度变成 60 度。
    preserve_angle = (
        force_preserve_angle
        or _close_to(original_duration, _DEFAULT_TURN_DURATION_S)
        or _close_to(original_duration, _SPIN_DURATION_S)
    )
    _scale_numeric_arg(args, "angular_z", factor)
    if preserve_angle and original_duration is not None:
        args["duration_s"] = round(_clamp(original_duration / factor, 0.2, 12.0), 3)


def _scale_arc_speed_preserving_shape(args: dict[str, Any], factor: float) -> None:
    if _close_to(factor, 1.0):
        return
    original_duration = _safe_float(args.get("duration_s"))
    _scale_numeric_arg(args, "linear_x", factor)
    _scale_numeric_arg(args, "angular_z", factor)
    if original_duration is not None:
        # arc/绕圈是展示轨迹，慢一点时应尽量保留圆弧形状，只是花更久执行。
        args["duration_s"] = round(_clamp(original_duration / factor, 0.2, 12.0), 3)


def _movement_speed_factor(preferences: Mapping[str, Any]) -> float:
    value = str(preferences.get("movement_speed") or "").strip().lower()
    return _SPEED_FACTORS.get(value, 1.0)


def _scale_numeric_arg(args: dict[str, Any], key: str, factor: float) -> None:
    if _close_to(factor, 1.0):
        return
    value = _safe_float(args.get(key))
    if value is None:
        return
    args[key] = round(value * factor, 4)


def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _close_to(value: Any, expected: float, *, tolerance: float = 0.05) -> bool:
    numeric = _safe_float(value)
    return numeric is not None and abs(numeric - expected) <= tolerance


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))
