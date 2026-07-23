"""在线/离线 launch 共用的 Agent 控制面参数映射。

这里仅暴露部署时经常覆盖的参数。模型路径、模型名等 provider 配置继续由各自 YAML
profile 管理，防止一个 launch 接口膨胀成所有模块的参数转发器。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from embodied_agent_core.agent_parameters import default_parameter_values


@dataclass(frozen=True)
class LaunchArgumentSpec:
    name: str
    value_type: type
    description: str


AGENT_CONTROL_ARGUMENTS = (
    LaunchArgumentSpec("mode", str, "Agent provider mode."),
    LaunchArgumentSpec("microphone_enabled", bool, "Subscribe to clean microphone PCM."),
    LaunchArgumentSpec("wake_word_enabled", bool, "Require wake word before commands."),
    LaunchArgumentSpec(
        "continuous_control_enabled", bool, "Enable long-running command session."
    ),
    LaunchArgumentSpec("voice_session_timeout_s", float, "Voice session idle timeout."),
    LaunchArgumentSpec(
        "continuous_command_queue_size", int, "Pending command queue capacity."
    ),
    LaunchArgumentSpec(
        "continuous_command_max_age_s", float, "Maximum queued command age."
    ),
    LaunchArgumentSpec(
        "continuous_duplicate_window_s", float, "Duplicate ASR final filter window."
    ),
    LaunchArgumentSpec("user_memory_retention_days", float, "Long-term user-memory retention."),
    LaunchArgumentSpec("memory_path", str, "Short-term conversation memory file."),
    LaunchArgumentSpec("user_memory_dir", str, "Long-term user memory directory."),
    LaunchArgumentSpec(
        "command_normalization_enabled", bool, "Normalize common ASR substitutions."
    ),
    LaunchArgumentSpec(
        "command_normalization_feedback_enabled",
        bool,
        "Publish normalization feedback.",
    ),
    LaunchArgumentSpec(
        "command_normalization_fuzzy_threshold",
        float,
        "Fuzzy normalization threshold.",
    ),
    LaunchArgumentSpec("command_normalization_path", str, "Optional normalization rule file."),
    LaunchArgumentSpec("command_completion_enabled", bool, "Complete safe missing action slots."),
    LaunchArgumentSpec("asr_commit_delay_ms", int, "Delay final commit after speech endpoint."),
    LaunchArgumentSpec("asr_partial_merge_enabled", bool, "Recover missing slots from recent partial."),
    LaunchArgumentSpec("asr_partial_max_age_s", float, "Maximum partial age used during merge."),
    LaunchArgumentSpec(
        "long_action_result_timeout_s",
        float,
        "Timeout for Nav2 navigation and multi-waypoint action results.",
    ),
)

# 上层 Gazebo/Nav2 launch 只透传会影响现场控制体验的参数；记忆路径等持久化配置
# 继续由 online/offline YAML 分别持有，避免动态 agent_type 下默认路径含义不明确。
FORWARDED_AGENT_ARGUMENT_NAMES = (
    "microphone_enabled",
    "wake_word_enabled",
    "continuous_control_enabled",
    "voice_session_timeout_s",
    "continuous_command_queue_size",
    "continuous_command_max_age_s",
    "continuous_duplicate_window_s",
    "command_normalization_enabled",
    "command_normalization_feedback_enabled",
    "command_normalization_fuzzy_threshold",
    "command_normalization_path",
    "command_completion_enabled",
    "asr_commit_delay_ms",
    "asr_partial_merge_enabled",
    "asr_partial_max_age_s",
    "long_action_result_timeout_s",
)


def launch_default(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def declare_agent_control_arguments(profile: str):
    """从节点 schema 生成 launch 默认值，消除第三份手写默认参数表。"""

    defaults = default_parameter_values(profile)
    return [
        DeclareLaunchArgument(
            spec.name,
            default_value=launch_default(defaults[spec.name]),
            description=spec.description,
        )
        for spec in AGENT_CONTROL_ARGUMENTS
    ]


def declare_forwarded_agent_arguments(
    *, default_overrides: dict[str, Any] | None = None
):
    """生成仿真组合 launch 的公共参数，并允许场景覆盖少量体验参数。"""

    defaults = default_parameter_values("online")
    defaults.update(default_overrides or {})
    specs = {spec.name: spec for spec in AGENT_CONTROL_ARGUMENTS}
    return [
        DeclareLaunchArgument(
            name,
            default_value=launch_default(defaults[name]),
            description=specs[name].description,
        )
        for name in FORWARDED_AGENT_ARGUMENT_NAMES
    ]


def forwarded_agent_configurations() -> dict[str, LaunchConfiguration]:
    return {name: LaunchConfiguration(name) for name in FORWARDED_AGENT_ARGUMENT_NAMES}


def forwarded_agent_launch_arguments(provider_mode: Any) -> dict[str, Any]:
    """构造 include online/offline launch 时完全相同的控制面转发表。"""

    return {
        "mode": provider_mode,
        **forwarded_agent_configurations(),
    }


def agent_control_configurations() -> dict[str, LaunchConfiguration]:
    return {spec.name: LaunchConfiguration(spec.name) for spec in AGENT_CONTROL_ARGUMENTS}


def agent_control_parameter_overrides() -> dict[str, Any]:
    """为 ROS 参数附加明确类型，避免 launch 字符串被误解析为错误类型。"""

    result: dict[str, Any] = {}
    for spec in AGENT_CONTROL_ARGUMENTS:
        configuration = LaunchConfiguration(spec.name)
        result[spec.name] = (
            configuration
            if spec.value_type is str
            else ParameterValue(configuration, value_type=spec.value_type)
        )
    return result
