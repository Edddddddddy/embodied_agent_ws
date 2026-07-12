"""运行时状态领域数据与 ROS 2 typed message 的转换边界。"""

from __future__ import annotations

from builtin_interfaces.msg import Time
from embodied_agent_interfaces.msg import (
    AudioFrontendStatus,
    BehaviorTreeStatus,
    KwsEvent,
    KwsScore,
    RobotActionAck,
    SimulationState,
    VadEvent,
)


_ACK_STATUS = {
    RobotActionAck.STATUS_ACCEPTED: "accepted",
    RobotActionAck.STATUS_REJECTED: "rejected",
    RobotActionAck.STATUS_SUCCEEDED: "succeeded",
    RobotActionAck.STATUS_CANCELED: "canceled",
    RobotActionAck.STATUS_TIMED_OUT: "timed_out",
    RobotActionAck.STATUS_BLOCKED: "blocked",
}
_BT_OUTCOME = {
    BehaviorTreeStatus.OUTCOME_RUNNING: "running",
    BehaviorTreeStatus.OUTCOME_SUCCEEDED: "succeeded",
    BehaviorTreeStatus.OUTCOME_REJECTED: "rejected",
    BehaviorTreeStatus.OUTCOME_CANCELED: "canceled",
    BehaviorTreeStatus.OUTCOME_TIMED_OUT: "timed_out",
    BehaviorTreeStatus.OUTCOME_BLOCKED: "blocked",
    BehaviorTreeStatus.OUTCOME_FAILED: "failed",
}
_VAD_EVENT = {
    VadEvent.EVENT_SPEECH_STARTED: "speech_started",
    VadEvent.EVENT_SPEECH_ENDED: "speech_ended",
}


def _assign_stamp(message, stamp) -> None:
    """同时接受 builtin_interfaces/Time 与 rclpy.time.Time，收口节点侧时间转换。"""
    if stamp is None:
        return
    message.stamp = stamp.to_msg() if hasattr(stamp, "to_msg") else stamp


def audio_frontend_status_to_message(
    payload: dict, *, stamp: Time | None = None
) -> AudioFrontendStatus:
    message = AudioFrontendStatus()
    _assign_stamp(message, stamp)
    message.rms = float(payload.get("rms") or 0.0)
    message.peak = max(0, int(payload.get("peak") or 0))
    message.speech = bool(payload.get("speech", False))
    message.vad_provider = str(payload.get("vad_provider") or "")
    message.endpoint_events_enabled = bool(
        payload.get("endpoint_events_enabled", True)
    )
    message.audio_enhancer_requested = str(
        payload.get("audio_enhancer_requested") or ""
    )
    message.audio_enhancer_active = str(
        payload.get("audio_enhancer_active") or ""
    )
    message.aec_active = bool(payload.get("aec_active", False))
    message.noise_suppression_requested = bool(
        payload.get("noise_suppression_requested", False)
    )
    message.noise_suppression_active = bool(
        payload.get("noise_suppression_active", False)
    )
    message.auto_gain_requested = bool(
        payload.get("auto_gain_requested", False)
    )
    message.auto_gain_active = bool(payload.get("auto_gain_active", False))
    message.dropped_input_frames = max(
        0, int(payload.get("dropped_input_frames") or 0)
    )
    message.dropped_playback_chunks = max(
        0, int(payload.get("dropped_playback_chunks") or 0)
    )
    return message


def audio_frontend_status_to_dict(message: AudioFrontendStatus) -> dict:
    return {
        "rms": float(message.rms),
        "peak": int(message.peak),
        "speech": bool(message.speech),
        "vad_provider": message.vad_provider,
        "endpoint_events_enabled": bool(message.endpoint_events_enabled),
        "audio_enhancer_requested": message.audio_enhancer_requested,
        "audio_enhancer_active": message.audio_enhancer_active,
        "aec_active": bool(message.aec_active),
        "noise_suppression_requested": bool(
            message.noise_suppression_requested
        ),
        "noise_suppression_active": bool(message.noise_suppression_active),
        "auto_gain_requested": bool(message.auto_gain_requested),
        "auto_gain_active": bool(message.auto_gain_active),
        "dropped_input_frames": int(message.dropped_input_frames),
        "dropped_playback_chunks": int(message.dropped_playback_chunks),
    }


def vad_event_to_message(event, *, provider: str, stamp=None) -> VadEvent:
    message = VadEvent()
    _assign_stamp(message, stamp)
    name = getattr(getattr(event, "name", None), "value", "")
    if name == "speech_started":
        message.event = VadEvent.EVENT_SPEECH_STARTED
    elif name == "speech_ended":
        message.event = VadEvent.EVENT_SPEECH_ENDED
    message.provider = provider
    message.reason = str(getattr(event, "reason", ""))
    message.probability = float(getattr(event, "probability", 0.0))
    return message


def vad_event_to_dict(message: VadEvent) -> dict:
    return {
        "name": _VAD_EVENT.get(message.event, "unknown"),
        "reason": message.reason,
        "probability": float(message.probability),
        "provider": message.provider,
    }


def kws_event_to_message(match, *, stamp=None) -> KwsEvent:
    message = KwsEvent()
    _assign_stamp(message, stamp)
    message.provider = str(match.provider)
    message.keyword = str(match.keyword)
    message.score = float(match.score)
    message.detected = True
    return message


def kws_event_to_dict(message: KwsEvent) -> dict:
    return {
        "kind": "wake",
        "provider": message.provider,
        "transcript": message.keyword,
        "score": float(message.score),
        "status": "detected" if message.detected else "candidate",
    }


def kws_score_to_message(
    *, provider: str, scores: dict[str, float], threshold: float, stamp=None
) -> KwsScore | None:
    if not scores:
        return None
    normalized = {str(key): float(value) for key, value in scores.items()}
    top_keyword, top_score = max(normalized.items(), key=lambda item: item[1])
    message = KwsScore()
    _assign_stamp(message, stamp)
    message.provider = provider
    message.top_keyword = top_keyword
    message.top_score = top_score
    message.threshold = float(threshold)
    message.above_threshold = top_score >= float(threshold)
    message.keywords = list(normalized)
    message.scores = [normalized[key] for key in message.keywords]
    return message


def kws_score_to_dict(message: KwsScore) -> dict:
    return {
        "provider": message.provider,
        "top_keyword": message.top_keyword,
        "top_score": float(message.top_score),
        "threshold": float(message.threshold),
        "above_threshold": bool(message.above_threshold),
        "scores": dict(zip(message.keywords, map(float, message.scores))),
    }


def simulation_state_to_dict(message: SimulationState) -> dict:
    return {
        "mode": message.mode,
        "sensor_stale": bool(message.sensor_stale),
        "safety_stopped": bool(message.safety_stopped),
        "front_distance": (
            float(message.front_distance) if message.front_distance_valid else None
        ),
        "right_distance": (
            float(message.right_distance) if message.right_distance_valid else None
        ),
        "reason": message.reason,
        "linear_x": float(message.linear_x),
        "angular_z": float(message.angular_z),
    }


def action_ack_to_dict(message: RobotActionAck) -> dict:
    payload = {
        "action": message.action,
        "backend": message.backend,
        "sequence": int(message.sequence),
        "status": _ACK_STATUS.get(message.status, "unknown"),
    }
    if message.detail:
        payload["detail"] = message.detail
    return payload


def behavior_tree_status_to_dict(message: BehaviorTreeStatus) -> dict:
    return {
        "command_id": message.command_id,
        "stage": message.stage,
        "outcome": _BT_OUTCOME.get(message.outcome, "unknown"),
        "detail": message.detail,
    }
