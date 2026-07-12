"""在线/离线 Agent 共享的 ROS 2 Topic 契约。"""

from dataclasses import dataclass


@dataclass(frozen=True)
class AgentTopicContract:
    """集中保存跨节点接口名，避免在线/离线节点各自复制字符串。

    默认值保持现有公开接口兼容；特殊部署继续使用标准 ROS remap 覆盖接口，
    不在业务节点内制造第二套 topic 名称。
    """

    text_input: str = "/agent/text_input"
    wake_event_input: str = "/agent/wake_event_input"
    speaker_identity: str = "/agent/speaker_identity"
    clear_memory: str = "/agent/clear_memory"
    action_result: str = "/robot/action_result"
    clean_audio: str = "/audio/clean_pcm"
    silence_timeout: str = "/audio/silence_timeout"
    speech_started: str = "/audio/speech_started"
    speech_ended: str = "/audio/speech_ended"

    asr_partial: str = "/agent/asr_partial"
    asr_final: str = "/agent/asr_final"
    response_text: str = "/agent/response_text"
    response_delta: str = "/agent/response_delta"
    action_candidate: str = "/agent/action_candidate"
    speaker_enroll_request: str = "/agent/speaker_enroll_request"
    tts_pcm: str = "/audio/tts_pcm"
    metrics: str = "/agent/metrics"

    state: str = "/agent/state"
    wake_event: str = "/agent/wake_event"
    session_state: str = "/agent/session_state"
    command_queue: str = "/agent/command_queue"
    command_execution: str = "/agent/command_execution"
    recognition_feedback: str = "/agent/recognition_feedback"
    nlu_parse: str = "/agent/nlu_parse"
    component_health: str = "system/component_health"
