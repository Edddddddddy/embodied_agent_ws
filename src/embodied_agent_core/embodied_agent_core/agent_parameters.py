"""在线/离线 Agent 的启动参数契约。

参数过去分散在两个节点、YAML 和 launch 文件中，修改一处很容易让另一条链路漂移。
本模块把节点参数的默认值、ROS 描述元数据与启动校验收口为单一权威来源；模型
供应商专属参数仍按 online/offline 分组，避免公共控制面依赖具体模型实现。
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Callable, Mapping

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor


Profile = str
Validator = Callable[[Any], bool]


class AgentParameterError(ValueError):
    """参数不满足启动契约时抛出，阻止节点带着危险配置进入 ready。"""


@dataclass(frozen=True)
class ParameterSpec:
    name: str
    default: Any | Mapping[Profile, Any]
    description: str
    minimum: float | int | None = None
    maximum: float | int | None = None
    choices: tuple[Any, ...] = ()
    validator: Validator | None = None

    def default_for(self, profile: Profile) -> Any:
        if isinstance(self.default, Mapping):
            return self.default[profile]
        return self.default

    def descriptor(self, profile: Profile) -> ParameterDescriptor:
        default = self.default_for(profile)
        # Agent provider/队列对象均在构造期绑定，运行时热改会产生“ROS 参数已变、业务对象未变”
        # 的假象，因此契约明确设为只读；调整配置后应由 Lifecycle/launch 重启节点。
        descriptor = ParameterDescriptor(description=self.description, read_only=True)
        if isinstance(default, bool) or self.minimum is None or self.maximum is None:
            return descriptor
        if isinstance(default, int):
            descriptor.integer_range = [
                IntegerRange(
                    from_value=int(self.minimum),
                    to_value=int(self.maximum),
                    step=1,
                )
            ]
        elif isinstance(default, float):
            descriptor.floating_point_range = [
                FloatingPointRange(
                    from_value=float(self.minimum),
                    to_value=float(self.maximum),
                    step=0.0,
                )
            ]
        return descriptor

    def validate(self, value: Any) -> str | None:
        if self.choices and value not in self.choices:
            return f"必须是 {list(self.choices)} 之一，实际为 {value!r}"
        if self.minimum is not None and value < self.minimum:
            return f"必须 >= {self.minimum}，实际为 {value!r}"
        if self.maximum is not None and value > self.maximum:
            return f"必须 <= {self.maximum}，实际为 {value!r}"
        if self.validator is not None and not self.validator(value):
            return f"不满足约束：{self.description}，实际为 {value!r}"
        return None


@dataclass(frozen=True)
class AgentParameters:
    """节点启动时的不可变参数快照，避免运行中半更新造成控制面状态不一致。"""

    profile: Profile
    values: Mapping[str, Any]

    def get(self, name: str) -> Any:
        return self.values[name]


def _positive_text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


COMMON_PARAMETER_SPECS = (
    ParameterSpec(
        "agent_lifecycle_autostart",
        True,
        "独立 ros2 run 时是否自行 configure/activate；launch manager 场景应关闭。",
    ),
    ParameterSpec(
        "agent_deactivate_timeout_s",
        5.0,
        "Lifecycle deactivate 等待 LLM/TTS/队列线程静默的超时。",
        0.1,
        120.0,
    ),
    ParameterSpec("microphone_enabled", False, "是否订阅实时麦克风音频。"),
    ParameterSpec("audio_sample_rate", 16000, "ASR 输入采样率。", 8000, 192000),
    ParameterSpec("wake_word_enabled", True, "是否要求唤醒词后才接受命令。"),
    ParameterSpec(
        "wake_words",
        ["小智", "你好小智"],
        "标准唤醒词列表。",
        validator=lambda v: bool(v),
    ),
    ParameterSpec(
        "wake_word_aliases",
        ["小志", "小治", "晓智", "晓志"],
        "ASR 常见唤醒词别名。",
    ),
    ParameterSpec("wake_active_timeout_s", 10.0, "单次唤醒有效时长。", 0.1, 3600.0),
    ParameterSpec("recognition_max_retries", 3, "识别失败后的最大重试次数。", 0, 20),
    ParameterSpec("command_normalization_enabled", True, "是否启用命令错词归一化。"),
    ParameterSpec("command_normalization_feedback_enabled", True, "是否发布归一化反馈。"),
    ParameterSpec(
        "command_normalization_fuzzy_threshold",
        0.82,
        "模糊匹配置信度阈值。",
        0.0,
        1.0,
    ),
    ParameterSpec("command_normalization_path", "", "自定义命令归一化规则路径。"),
    ParameterSpec("command_completion_enabled", True, "是否补齐缺少时长或角度的短命令。"),
    ParameterSpec("command_nlu_enabled", True, "是否启用本地轻量多命令 NLU。"),
    ParameterSpec("command_nlu_min_confidence", 0.18, "本地 NLU 最低置信度。", 0.0, 1.0),
    ParameterSpec(
        "memory_path",
        {
            "online": "~/.ros/embodied_agent/memory.json",
            "offline": "~/.ros/embodied_agent/offline_memory.json",
        },
        "短期会话记忆文件。",
        validator=_positive_text,
    ),
    ParameterSpec(
        "memory_max_turns",
        {"online": 10, "offline": 3},
        "送入 LLM 的最近对话轮数。",
        1,
        100,
    ),
    ParameterSpec(
        "user_memory_dir",
        "~/.ros/embodied_agent/users",
        "用户长期记忆目录。",
        validator=_positive_text,
    ),
    ParameterSpec("user_memory_max_recent", 8, "每个用户保留的近期行为条数。", 1, 1000),
    ParameterSpec("user_memory_retention_days", 90.0, "用户记忆保留天数。", 0.0, 3650.0),
    ParameterSpec(
        "speaker_identity_min_confidence",
        0.55,
        "应用用户偏好所需的最低声纹置信度。",
        0.0,
        1.0,
    ),
    ParameterSpec("system_prompt_path", "", "自定义系统提示词路径；留空使用包内默认值。"),
    ParameterSpec(
        "llm_context_window_tokens",
        {"online": 32768, "offline": 2048},
        "模型上下文窗口；Prompt 会扣除生成上限与安全余量后再装入历史/RAG。",
        256,
        1048576,
    ),
    ParameterSpec(
        "prompt_safety_reserve_tokens",
        {"online": 512, "offline": 256},
        "为 ChatML 模板差异和 token 估算误差保留的上下文安全余量。",
        0,
        65536,
    ),
    ParameterSpec(
        "rag_enabled",
        True,
        "是否为知识问答启用本地 RAG；本地控制命令始终绕过检索。",
    ),
    ParameterSpec(
        "rag_cloud_context_policy",
        {"online": "builtin_only", "offline": "allow_custom"},
        "知识证据发送边界：在线默认只允许包内公开手册，自定义文档需显式授权。",
        choices=("builtin_only", "allow_custom", "off"),
    ),
    ParameterSpec(
        "rag_knowledge_paths",
        [""],
        "UTF-8 Markdown/文本知识库路径；首项留空时使用包内机器人运行手册。",
    ),
    ParameterSpec(
        "rag_query_policy",
        "adaptive",
        "RAG 路由策略。",
        choices=("adaptive", "always", "off"),
    ),
    ParameterSpec("rag_top_k", 3, "单次检索最大片段数。", 0, 20),
    ParameterSpec(
        "rag_max_context_chars",
        {"online": 1800, "offline": 1200},
        "单轮检索上下文字符预算。",
        0,
        20000,
    ),
    ParameterSpec("rag_min_score", 0.05, "稀疏检索最低 BM25 分数。", 0.0, 1000.0),
    ParameterSpec("rag_chunk_chars", 600, "知识文档切片字符数。", 120, 5000),
    ParameterSpec("rag_chunk_overlap_chars", 80, "相邻知识片段重叠字符数。", 0, 1000),
    ParameterSpec("mock_token_delay_s", 0.0, "mock 模型逐 token 延迟。", 0.0, 60.0),
    ParameterSpec("mock_asr_finals", "", "mock ASR final 脚本，使用竖线分隔。"),
    ParameterSpec("mock_asr_partials", "", "mock ASR partial 脚本，使用竖线分隔。"),
    ParameterSpec(
        "action_sequence_wait_timeout_s",
        12.0,
        "普通短动作等待单步结果的超时。",
        0.1,
        600.0,
    ),
    ParameterSpec(
        "long_action_result_timeout_s",
        330.0,
        "Nav2 导航与多航点等长任务等待结果的超时。",
        1.0,
        3600.0,
    ),
    ParameterSpec("continuous_control_enabled", False, "是否启用长会话命令队列。"),
    ParameterSpec("voice_session_timeout_s", 60.0, "连续语音会话空闲超时。", 0.1, 3600.0),
    ParameterSpec("continuous_command_queue_size", 8, "连续命令队列容量。", 1, 256),
    ParameterSpec("continuous_command_max_age_s", 30.0, "排队命令允许的最大年龄。", 0.1, 3600.0),
    ParameterSpec(
        "continuous_duplicate_window_s",
        1.2,
        "重复 ASR final 去重窗口。",
        0.0,
        60.0,
    ),
    ParameterSpec(
        "speech_endpoint_events_enabled",
        True,
        "是否使用 speech_started/ended 端点事件。",
    ),
    ParameterSpec("asr_commit_delay_ms", 0, "speech_ended 后等待 partial 稳定的时间。", 0, 5000),
    ParameterSpec("asr_partial_merge_enabled", True, "final 缺槽位时是否合并最近 partial。"),
    ParameterSpec(
        "asr_partial_max_age_s",
        2.0,
        "允许参与 final 合并的 partial 最大年龄。",
        0.1,
        30.0,
    ),
    ParameterSpec(
        "external_wake_event_enabled",
        True,
        "是否接受独立 KWS 节点的 typed wake event。",
    ),
)


ONLINE_PARAMETER_SPECS = (
    ParameterSpec("mode", "mock", "online Agent 运行模式。", choices=("mock", "online")),
    ParameterSpec("tts_sample_rate", 24000, "云 TTS 输出采样率。", 8000, 192000),
    ParameterSpec("llm_model", "qwen-plus", "在线 LLM 模型名。", validator=_positive_text),
    ParameterSpec(
        "llm_base_url",
        "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "OpenAI 兼容 LLM endpoint。",
        validator=_positive_text,
    ),
    ParameterSpec("llm_temperature", 0.0, "动作生成温度。", 0.0, 2.0),
    ParameterSpec("llm_max_tokens", 512, "单轮最大生成 token。", 1, 8192),
    ParameterSpec(
        "asr_model",
        "qwen3-asr-flash-realtime",
        "在线流式 ASR 模型名。",
        validator=_positive_text,
    ),
    ParameterSpec(
        "asr_url",
        "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
        "在线 ASR WebSocket endpoint。",
        validator=_positive_text,
    ),
    ParameterSpec("asr_language", "zh", "ASR 语言。", validator=_positive_text),
    ParameterSpec(
        "tts_model",
        "qwen3-tts-flash-realtime",
        "在线流式 TTS 模型名。",
        validator=_positive_text,
    ),
    ParameterSpec("tts_voice", "Cherry", "TTS 音色。", validator=_positive_text),
    ParameterSpec(
        "tts_url",
        "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
        "在线 TTS WebSocket endpoint。",
        validator=_positive_text,
    ),
    ParameterSpec("tts_language", "Chinese", "TTS 语言。", validator=_positive_text),
    ParameterSpec("tts_chunk_max_chars", 32, "每个 TTS 文本块的最大字符数。", 1, 1000),
    ParameterSpec("llm_first_token_target_ms", 1000.0, "LLM 首 token 性能目标。", 1.0, 60000.0),
    ParameterSpec("tts_first_audio_target_ms", 300.0, "TTS 首音频性能目标。", 1.0, 60000.0),
    ParameterSpec("online_warmup_enabled", True, "ready 前是否预连接在线模型。"),
)


OFFLINE_PARAMETER_SPECS = (
    ParameterSpec("mode", "mock", "offline Agent 运行模式。", choices=("mock", "offline")),
    ParameterSpec(
        "asr_model_dir",
        "/home/ubuntu/embodied_agent_ws/models/"
        "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16",
        "Sherpa ZipFormer 模型目录。",
        validator=_positive_text,
    ),
    ParameterSpec("asr_num_threads", 2, "Sherpa ASR 线程数。", 1, 128),
    ParameterSpec(
        "asr_tail_padding_s",
        0.66,
        "提交流式 Sherpa ASR 前注入的尾部静音时长。",
        0.0,
        2.0,
    ),
    ParameterSpec(
        "asr_decoding_method",
        "modified_beam_search",
        "Sherpa 解码方法。",
        choices=("greedy_search", "modified_beam_search"),
    ),
    ParameterSpec("asr_hotwords_file", "", "离线 ASR 热词文件。"),
    ParameterSpec("asr_hotwords_score", 3.0, "离线 ASR 热词偏置分数。", 0.0, 100.0),
    ParameterSpec("asr_max_active_paths", 16, "beam search 最大活跃路径数。", 1, 128),
    ParameterSpec(
        "asr_modeling_unit",
        "cjkchar",
        "热词建模单元。",
        choices=("cjkchar", "bpe", "char"),
    ),
    ParameterSpec(
        "llm_base_url",
        "http://127.0.0.1:8080/v1",
        "llama.cpp OpenAI 兼容 endpoint。",
        validator=_positive_text,
    ),
    ParameterSpec("llm_model", "Qwen3-0.6B-Q8_0.gguf", "离线模型标识。", validator=_positive_text),
    ParameterSpec("llm_temperature", 0.7, "离线 LLM 温度。", 0.0, 2.0),
    ParameterSpec("llm_max_tokens", 192, "单轮最大生成 token。", 1, 8192),
    ParameterSpec("llm_seed", 42, "离线推理随机种子。"),
    ParameterSpec("llm_timeout_s", 30.0, "llama.cpp 请求超时。", 0.1, 600.0),
    ParameterSpec("llm_max_retries", 1, "llama.cpp 请求重试次数。", 0, 20),
    ParameterSpec(
        "llm_first_token_warn_ms",
        1000.0,
        "首 token 慢请求告警阈值。",
        1.0,
        60000.0,
    ),
    ParameterSpec("runtime_warmup_enabled", True, "ready 前是否预热 llama.cpp/TTS。"),
    ParameterSpec(
        "tts_model_dir",
        "/home/ubuntu/embodied_agent_ws/models/vits-melo-tts-zh_en",
        "Sherpa-TTS 模型目录。",
        validator=_positive_text,
    ),
    ParameterSpec(
        "tts_provider",
        "sherpa",
        "离线 TTS 实现。",
        choices=("sherpa", "summer", "summer_ros"),
    ),
    ParameterSpec("tts_num_threads", 2, "离线 TTS 线程数。", 1, 128),
    ParameterSpec("tts_speaker_id", 0, "TTS speaker id。", 0, 100000),
    ParameterSpec("tts_speed", 1.0, "TTS 语速倍率。", 0.1, 5.0),
    ParameterSpec("tts_sample_rate", 44100, "离线 TTS 输出采样率。", 8000, 192000),
    ParameterSpec("tts_chunk_max_chars", 24, "伪流式 TTS 文本块最大字符数。", 1, 1000),
    ParameterSpec("tts_pcm_chunk_ms", 80, "PCM 发布块时长。", 5, 2000),
    ParameterSpec(
        "summer_tts_binary",
        "/home/ubuntu/embodied_agent_ws/third_party/SummerTTS/build/tts_test",
        "SummerTTS 可执行文件。",
        validator=_positive_text,
    ),
    ParameterSpec(
        "summer_tts_model",
        "/home/ubuntu/embodied_agent_ws/third_party/SummerTTS/"
        "models/single_speaker_fast.bin",
        "SummerTTS 模型文件。",
        validator=_positive_text,
    ),
    ParameterSpec("summer_tts_timeout_s", 30.0, "SummerTTS 进程超时。", 0.1, 600.0),
    ParameterSpec(
        "summer_tts_service_name",
        "/tts/synthesize",
        "SummerTTS ROS 服务名。",
        validator=_positive_text,
    ),
    ParameterSpec(
        "summer_tts_service_timeout_s",
        10.0,
        "SummerTTS ROS 服务超时。",
        0.1,
        600.0,
    ),
    ParameterSpec(
        "summer_tts_service_speaker_id",
        -1,
        "SummerTTS 服务 speaker id；-1 使用模型默认值。",
        -1,
        100000,
    ),
    ParameterSpec(
        "summer_tts_service_length_scale",
        0.0,
        "SummerTTS length scale；0 使用默认值。",
        0.0,
        10.0,
    ),
)


def parameter_specs(profile: Profile) -> tuple[ParameterSpec, ...]:
    if profile == "online":
        return COMMON_PARAMETER_SPECS + ONLINE_PARAMETER_SPECS
    if profile == "offline":
        return COMMON_PARAMETER_SPECS + OFFLINE_PARAMETER_SPECS
    raise AgentParameterError(f"未知 Agent 参数 profile: {profile!r}")


def default_parameter_values(profile: Profile) -> dict[str, Any]:
    return {spec.name: spec.default_for(profile) for spec in parameter_specs(profile)}


def validate_parameter_values(profile: Profile, values: Mapping[str, Any]) -> None:
    errors: list[str] = []
    for spec in parameter_specs(profile):
        if spec.name not in values:
            errors.append(f"{spec.name}: 缺少必需参数")
            continue
        try:
            error = spec.validate(values[spec.name])
        except TypeError:
            error = f"类型错误，实际为 {values[spec.name]!r}"
        if error:
            errors.append(f"{spec.name}: {error}")

    # partial 的有效期必须覆盖 commit 延迟，否则延迟提交反而拿不到尾部 partial。
    if not errors and values["asr_partial_merge_enabled"]:
        commit_delay_s = values["asr_commit_delay_ms"] / 1000.0
        if values["asr_partial_max_age_s"] < commit_delay_s:
            errors.append(
                "asr_partial_max_age_s: 必须 >= asr_commit_delay_ms / 1000，"
                "否则 endpoint 延迟期间 partial 会先过期"
            )
    if not errors and values["rag_chunk_overlap_chars"] >= values["rag_chunk_chars"]:
        errors.append(
            "rag_chunk_overlap_chars: 必须小于 rag_chunk_chars，"
            "否则切片窗口无法向前推进"
        )
    if (
        not errors
        and values["llm_context_window_tokens"]
        <= values["llm_max_tokens"] + values["prompt_safety_reserve_tokens"]
    ):
        errors.append(
            "llm_context_window_tokens: 必须大于 llm_max_tokens + "
            "prompt_safety_reserve_tokens，确保系统约束和当前问题仍有输入空间"
        )
    if errors:
        raise AgentParameterError(
            f"{profile} Agent 参数校验失败:\n- " + "\n- ".join(errors)
        )


def declare_agent_parameters(node: Any, profile: Profile) -> AgentParameters:
    """声明、读取并一次性校验参数；任何错误都发生在创建 provider 之前。"""

    specs = parameter_specs(profile)
    for spec in specs:
        node.declare_parameter(
            spec.name,
            spec.default_for(profile),
            spec.descriptor(profile),
        )
    values = {spec.name: node.get_parameter(spec.name).value for spec in specs}
    validate_parameter_values(profile, values)
    return AgentParameters(profile, MappingProxyType(values))
