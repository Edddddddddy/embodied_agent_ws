"""自动任务的合成/真人触发策略与联合报告适配。"""

from __future__ import annotations

from std_msgs.msg import String

from tools.acceptance.probes.slam_nav.session_observer import (
    SessionObserver,
    wait_until,
)


def await_automatic_mission_trigger(
    node: SessionObserver,
    *,
    source: str,
    timeout_s: float,
    agent_mode: str,
) -> None:
    """等待或注入一次自动任务意图；触发来源由 CLI 显式选择。"""

    if source == "synthetic":
        if node.asr_pub is None:
            raise RuntimeError("synthetic ASR publisher is unavailable")
        wait_until(
            lambda: node.asr_pub.get_subscription_count() > 0,
            5.0,
            "ASR final subscriber unavailable",
        )
        node.asr_pub.publish(String(data="开始自动巡检建图"))
        return
    if source != "live_voice":
        raise ValueError(f"unsupported automatic trigger source: {source}")

    node.begin_live_voice_trigger_window()
    wait_until(
        node.live_voice_sources_ready,
        min(10.0, timeout_s),
        "live audio/endpoint/wake/ASR publishers are unavailable",
    )
    print(
        f"真人语音触发窗口已就绪（{agent_mode}）。请清晰说：\n"
        "    小智，开始自动巡检建图",
        flush=True,
    )
    if not node.wait_for_live_voice_trigger(timeout_s):
        raise TimeoutError(
            f"no valid live automatic-mission utterance within {timeout_s:.1f}s"
        )


def finalize_trigger_report(
    node: SessionObserver,
    core_report: dict[str, object],
    *,
    live_voice_trigger: bool,
    agent_mode: str,
) -> dict[str, object]:
    """真人模式包装语音事实；合成模式保持原 schema v4 兼容。"""

    if not live_voice_trigger:
        return core_report
    # 只有 observer 持有同一 ROS 会话内的事件窗口。这里统一落盘可避免
    # “语音单独 PASS + SLAM 单独 PASS”被误写成从未发生过的联合链路。
    try:
        return node.build_live_voice_evidence(
            core_report, agent_mode=agent_mode
        )
    except RuntimeError as error:
        # 若系统连 MAPPING 都没进入，语音窗口还不存在；仍写出统一 schema v1
        # 的 fail-closed 诊断，而不是让同一公开入口在失败时突然换回另一 schema。
        return {
            "schema_version": 1,
            "evidence_kind": "voice_unknown_world_slam_nav_e2e",
            "session_id": str(core_report.get("session_id", "")),
            "agent_mode": agent_mode,
            "trigger_source": "live_voice",
            "passed": False,
            "checks": {
                "real_audio_observed": False,
                "endpoint_observed": False,
                "wake_accepted": False,
                "automatic_mission_asr_final": False,
                "strict_core_schema_v4_passed": False,
            },
            "voice_window": None,
            "core_report": core_report,
            "error": str(error),
        }
