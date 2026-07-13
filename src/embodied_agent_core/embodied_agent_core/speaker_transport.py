"""声纹领域对象与 ROS 2 typed message 的唯一转换 seam。"""

from __future__ import annotations

from embodied_agent_interfaces.msg import (
    SpeakerEnrollRequest as SpeakerEnrollRequestMessage,
    SpeakerEnrollStatus as SpeakerEnrollStatusMessage,
    SpeakerIdentity as SpeakerIdentityMessage,
)

from .user_memory import SpeakerIdentity


def _assign_stamp(message, stamp) -> None:
    if stamp is not None:
        message.stamp = stamp.to_msg() if hasattr(stamp, "to_msg") else stamp


def identity_payload_to_message(payload: dict, *, stamp=None) -> SpeakerIdentityMessage:
    message = SpeakerIdentityMessage()
    _assign_stamp(message, stamp)
    message.speaker_id = str(payload.get("speaker_id") or "unknown")
    message.confidence = float(payload.get("confidence") or 0.0)
    message.enrolled = bool(payload.get("enrolled", False))
    message.model = str(payload.get("model") or "unknown")
    message.display_name = str(payload.get("display_name") or "")
    message.reason = str(payload.get("reason") or "")
    message.rms = float(payload.get("rms") or 0.0)
    message.second_best_score = float(payload.get("second_best_score") or 0.0)
    message.score_margin = float(payload.get("score_margin") or 0.0)
    message.threshold = float(payload.get("threshold") or 0.0)
    message.min_margin = float(payload.get("min_margin") or 0.0)
    return message


def identity_message_to_domain(
    message: SpeakerIdentityMessage, *, min_confidence: float = 0.55
) -> SpeakerIdentity:
    confidence = float(message.confidence)
    enrolled = bool(message.enrolled) and confidence >= float(min_confidence)
    return SpeakerIdentity(
        speaker_id=message.speaker_id if enrolled else "unknown",
        confidence=confidence,
        enrolled=enrolled,
        model=message.model or "unknown",
        display_name=message.display_name,
        updated_at=(
            float(message.stamp.sec) + float(message.stamp.nanosec) / 1_000_000_000.0
        ),
    )


def identity_message_to_dict(message: SpeakerIdentityMessage) -> dict:
    return {
        "speaker_id": message.speaker_id,
        "display_name": message.display_name,
        "confidence": float(message.confidence),
        "enrolled": bool(message.enrolled),
        "model": message.model,
        "reason": message.reason,
        "rms": float(message.rms),
        "second_best_score": float(message.second_best_score),
        "score_margin": float(message.score_margin),
        "threshold": float(message.threshold),
        "min_margin": float(message.min_margin),
    }


def enroll_request_to_message(request, *, stamp=None) -> SpeakerEnrollRequestMessage:
    message = SpeakerEnrollRequestMessage()
    _assign_stamp(message, stamp)
    message.speaker_id = request.speaker_id
    message.display_name = request.display_name or request.speaker_id
    message.samples_required = max(1, int(request.samples_required))
    return message


def enroll_status_to_message(payload: dict, *, stamp=None) -> SpeakerEnrollStatusMessage:
    message = SpeakerEnrollStatusMessage()
    _assign_stamp(message, stamp)
    status_map = {
        "started": message.STATUS_STARTED,
        "collecting": message.STATUS_COLLECTING,
        "completed": message.STATUS_COMPLETED,
        "failed": message.STATUS_FAILED,
    }
    message.status = status_map.get(str(payload.get("status") or ""), message.STATUS_UNKNOWN)
    message.reason = str(payload.get("reason") or "")
    message.speaker_id = str(payload.get("speaker_id") or "")
    message.display_name = str(payload.get("display_name") or "")
    message.collected = max(0, int(payload.get("collected") or 0))
    message.required = max(0, int(payload.get("required") or 0))
    message.sample_path = str(payload.get("sample_path") or "")
    return message


def enroll_status_to_dict(message: SpeakerEnrollStatusMessage) -> dict:
    status_map = {
        message.STATUS_STARTED: "started",
        message.STATUS_COLLECTING: "collecting",
        message.STATUS_COMPLETED: "completed",
        message.STATUS_FAILED: "failed",
    }
    return {
        "status": status_map.get(message.status, "unknown"),
        "reason": message.reason,
        "speaker_id": message.speaker_id,
        "display_name": message.display_name,
        "collected": int(message.collected),
        "required": int(message.required),
        "sample_path": message.sample_path,
    }
