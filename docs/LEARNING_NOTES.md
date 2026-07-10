# 学习笔记：关键技术点与设计取舍

这份笔记面向复盘和面试讲解。每个技术点都按四个问题组织：

- 关键代码在哪里？
- 这一层怎么设计？
- 为什么这样设计？
- 和其他方案相比有什么区别？

如果只想先看主链路图，优先打开
[FINAL_ARCHITECTURE_DIAGRAMS.md](FINAL_ARCHITECTURE_DIAGRAMS.md)。
如果要准备面试代码走读，优先打开
[VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md](VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md)，
它按“语音输入 → Agent → ActionGuard → ROS 2 Action → Gazebo/Nav2 执行”列出了关键文件、
关键函数和上下游接口。

## 1. ROS 2 通信模型：topic、msg、action 的分工

关键代码：

- `src/embodied_agent_interfaces/msg/RobotCommand.msg`
- `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action`
- `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp`
- `src/embodied_agent_cpp/src/typed_action_demo_client.cpp`
- `src/embodied_simulation/src/simulation_control_node.cpp`

设计方式：

- Agent 先发布 `/agent/action_candidate`，内容是 LLM 或 fallback parser 生成的结构化动作候选。
- C++ ActionGuard 将动作候选转为强类型 `RobotCommand`。
- typed action bridge 将 `RobotCommand` 发送为 `ExecuteRobotCommand` goal。
- `typed_action_demo_client` 是面试/调试用最小 C++ action client：从命令行构造
  `RobotCommand`，直接发送 action goal，打印 feedback/result 并用结果决定进程退出码。
- simulation executor 返回 feedback/result，并驱动 `/cmd_vel`。

为什么这样设计：

- topic 适合广播状态和瞬时事件，例如 ASR final、动作候选、监控日志。
- ROS 2 Action 适合“移动一秒”“转九十度”这种有持续时间、可取消、需要反馈的动作。
- 自定义 msg/action 让动作接口可测试、可限幅、可扩展，比纯字符串事件载荷更工程化。

方案对比：

- 只用 `/cmd_vel`：简单，但 LLM 直接控制速度风险高，也难以表达执行结果。
- 只用 service：适合短请求，不适合持续动作和取消。
- 只用字符串事件 topic：开发快，但类型不安全，后期维护和测试成本高。
- 保留一个独立 demo client：比 bridge 更适合讲解 rclcpp_action 的 goal/feedback/result
  生命周期，也能在没有 Agent 的情况下单独验证 action server。

## 2. ActionGuard：LLM 输出和机器人执行之间的安全边界

关键代码：

- `src/embodied_agent_cpp/src/action_guard_node.cpp`
- `src/embodied_agent_cpp/include/embodied_agent_cpp/robot_command_adapter.hpp`
- `src/embodied_agent_cpp/src/robot_command_adapter.cpp`
- `src/embodied_agent_cpp/include/embodied_agent_cpp/action_validator.hpp`
- `src/embodied_agent_cpp/src/action_validator.cpp`

设计方式：

- 订阅 `/agent/action_candidate`。
- 解析动作候选。
- 校验动作类型、速度、时长、颜色、模式等字段。
- 通过后发布 `/robot/action_command_typed` 强类型 ROS 2 msg。
- 拒绝时发布 `/robot/action_rejected`。

为什么这样设计：

- 大模型输出不可完全信任，必须在进入机器人执行层前做白名单和限幅。
- 删除旧字符串动作命令入口，避免仿真/硬件执行层出现双入口。
- 使用 typed message，方便 C++、Action、仿真执行器稳定对接。

方案对比：

- 在 prompt 里约束模型：必要但不够，模型仍可能输出非法字段。
- 在执行器里校验：太晚，安全边界分散。
- 单独 ActionGuard：边界清晰，便于单测和面试讲解。

## 3. 连续语音会话：一次唤醒，多轮控制

关键代码：

- `src/embodied_online_agent/embodied_online_agent/continuous_voice.py`
- `src/embodied_online_agent/embodied_online_agent/wakeword.py`
- `src/embodied_online_agent/embodied_online_agent/wake_provider.py`
- `scripts/continuous_voice_monitor.py`

设计方式：

- `ContinuousVoiceSession` 负责判断一句 ASR final 是唤醒、命令、拒绝还是休眠。
- 支持唤醒词别名，例如“小志”“晓智”。
- 支持 filler 过滤，例如“嗯”“啊”。
- 支持 duplicate window，过滤短时间重复 ASR final。
- 支持 session timeout，超时后必须重新唤醒。

为什么这样设计：

- 真实麦克风会持续产生 ASR final，如果每句话都直接进 LLM，会出现误触发和卡顿。
- 会话层把“听到了什么”和“是否应该执行”分开，便于监控和调参。
- 文本唤醒先跑通，不强依赖声学 KWS 模型，部署更稳。

方案对比：

- 每条命令都要求带“小智”：安全但体验差。
- 完全不用唤醒：误触发多，不适合现场演示。
- 声学 KWS 优先：体验更好，但依赖模型和音频环境；当前项目保留 seam，默认用文本唤醒保证可部署。

## 4. 连续命令队列与急停抢占

关键代码：

- `src/embodied_online_agent/embodied_online_agent/continuous_voice.py`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`

设计方式：

- 普通命令进入 `ContinuousCommandQueue`，按 FIFO 顺序执行。
- worker 线程逐条调用 `_run_turn()`。
- 命令执行前发布 started，执行后发布 finished。
- `停下/急停` 是 priority stop：清空队列、取消当前 sequence、立即发布 stop。
- 非优先命令设置 TTL，太旧会过期丢弃并上报。

为什么这样设计：

- 用户会连续说多条命令，不能因为上一条动作 busy 就静默丢弃下一条。
- 急停不能排队等待，必须抢占。
- TTL 防止机器人执行用户很久之前说过、已经过时的命令。

方案对比：

- busy 时直接丢弃：实现简单，但体验像“卡住”。
- 并行执行所有命令：机器人动作冲突，安全性差。
- FIFO + priority stop：兼顾连续体验和安全边界。

## 5. VAD、endpoint 与 ASR commit delay

关键代码：

- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`
- `src/embodied_agent_cpp/src/audio_processing.cpp`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `scripts/audio_frontend_calibration.py`
- `scripts/voice_calibration_report.py`

设计方式：

- C++ audio frontend 发布 `/audio/clean_pcm`、`/audio/speech_started`、`/audio/speech_ended`、`/audio/silence_timeout`。
- `VAD_PROVIDER=auto` 会在启动脚本里先跑 `voice_provider_preflight.py`：Silero VAD 依赖可用时，
  AudioFrontend 只发布 clean PCM，`silero_vad` sidecar 负责 endpoint；Silero 不可用但
  `webrtcvad` 可用时，`webrtc_vad` sidecar 接管 endpoint；都不可用时降级 energy VAD。
- `scripts/setup_voice_vad_runtime.sh` 提供 WebRTC/Silero 可选依赖安装入口，支持 dry-run；
  它会安装 `embodied_online_agent[webrtc-vad]`、`embodied_online_agent[silero-vad]`
  对应 extra，并在安装后跑 provider preflight。
- `voice_provider_preflight.py` 不只判断 PASS/BLOCKED，还会在 auto 降级或显式 provider
  缺依赖时输出 `recommendations`。这样真实麦克风演示前可以从“缺什么包”直接走到
  “运行哪个 setup 脚本”，减少现场排障成本。
- `webrtc-vad-sidecar` 是安装 WebRTC runtime 后的显式验收入口：它启动 C++ audio frontend
  和 `webrtc_vad` sidecar，确认端点事件由成熟 VAD 接管，而不只是检查 Python 包是否存在。
- `scripts/setup_voice_kws_runtime.sh` 提供 openWakeWord、sherpa-onnx KWS、LiveKit WakeWord
  的可选运行时入口；sherpa profile 会复用 ZipFormer ASR 模型路径，生成默认关键词文件，
  并写出 `logs/sherpa_kws.env`，方便后续 `source` 后直接跑 `provider-preflight`。
- `sherpa-kws-sidecar` 会实际启动 `sherpa_onnx.KeywordSpotter`，证明声学 KWS 不只是
  参数 seam；默认关键词文件使用 `小 智` / `你 好 小 智` 这种 tokenized 写法，
  避免 sherpa 无法从 tokens.txt 编码整句中文。
- Agent 收到 endpoint 后调用 ASR commit。
- `asr_commit_delay_ms` 允许在 endpoint 后等待少量时间，再提交 final。
- `VOICE_CONTROL_PROFILE` 提供 normal、quiet、low_gain、noisy_room 四种参数预设。
- `voice_calibration_report.py` 把 provider preflight、audio calibration、KWS score calibration
  汇总成 `logs/voice_calibration_report.json/.md`，并额外生成可 `source` 的
  `logs/voice_calibration.env`，用于真实麦克风演示前保存和复用调参证据。对
  openWakeWord/LiveKit，采到 KWS 分数后会把建议阈值写成
  `OPENWAKEWORD_THRESHOLD` / `LIVEKIT_WAKEWORD_THRESHOLD`。
- `continuous_voice_control.sh` 默认 `APPLY_VOICE_CALIBRATION=auto`：如果
  `logs/voice_calibration.env` 存在，会在 profile 默认值计算前加载；如果用户显式传入
  `VOICE_CONTROL_PROFILE/SPEECH_START_THRESHOLD/VAD_PROVIDER` 等关键变量，则显式值优先。

为什么这样设计：

- 真实 ASR 容易漏掉尾部数字和量词，例如“左转90度”只 final 成“左转”。
- 适当延迟 300～500ms 可以换取更完整的识别结果。
- 校准文件默认自动复用，可以减少演示前忘记 `source logs/voice_calibration.env` 的概率；
  但显式环境变量优先，避免旧校准文件覆盖现场临时调参。
- 把 VAD 依赖安装封装成项目脚本，是为了让“成熟 VAD sidecar”不只是代码 seam；
  演示环境可以通过 dry-run、install、preflight 三步确认真的没有降级到 energy VAD。
- KWS 单独做 runtime setup，是因为“唤醒词检测”比文本触发更接近真实机器人交互；openWakeWord
  适合快速准备 Python runtime，sherpa KWS 则能复用离线 ASR 运行时资产。
- VAD 和 commit 分离，便于定位“音频没听到”和“ASR final 太早”两类问题。
- 成熟 VAD 做成 sidecar，而不是塞进 PortAudio 回调线程，是为了避免模型推理阻塞音频采集。

方案对比：

- 极短静音阈值：响应快，但尾部漏识别多。
- 很长静音阈值：完整但交互迟钝。
- profile + commit delay：保留可调空间，适合不同环境。
- auto Silero/WebRTC sidecar：Silero 判断更稳但依赖较重，WebRTC VAD 更轻、更易部署但只有二分类；
  降级 energy VAD 保证基础演示不被可选依赖卡死。

## 6. 短命令补全与模糊归一化

关键代码：

- `src/embodied_online_agent/embodied_online_agent/command_normalizer.py`
- `src/embodied_online_agent/config/command_normalization_zh.yaml`
- `src/embodied_online_agent/embodied_online_agent/command_completion.py`
- `src/embodied_online_agent/test/test_command_completion.py`

设计方式：

- command normalizer 处理错别字、同音词、常见 ASR 误识别。
- command completer 处理缺槽短命令：
  - `前进` → `前进一秒`
  - `后退` → `后退一秒`
  - `左转` → `左转九十度`
  - `右转` → `右转九十度`
- 补全只作用于普通控制命令，不处理 `停下/急停`。

为什么这样设计：

- 演示中“短 ASR final”不应该让链路中断。
- 用规则补全比再调一次 LLM 更快、更稳定、可测试。
- 安全命令保持原样，避免误改写。

方案对比：

- 全交给 LLM：泛化强，但慢且不可预测。
- 全靠 ASR 热词：能改善识别，但不能解决所有尾部漏识别。
- 规则补全 + ASR 调参：更适合当前演示目标。

## 7. 在线 Agent：流式 ASR/LLM/TTS 与动作回调

关键代码：

- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py`
- `src/embodied_online_agent/embodied_online_agent/providers/openai_compatible_llm.py`
- `src/embodied_online_agent/embodied_online_agent/providers/qwen_tts.py`
- `src/embodied_online_agent/prompts/system_prompt_zh.txt`
- `src/embodied_online_agent/embodied_online_agent/protocol.py`

设计方式：

- ASR partial/final 分开发布。
- LLM 输出通过 tagged stream parser 解析文本和动作。
- TTS 按句子 chunk 合成，减少首包等待。
- 动作候选通过 `/agent/action_candidate` 进入 ActionGuard。
- metrics 记录 LLM 首 token、TTS 首音频等延迟。

为什么这样设计：

- 流式交互要尽早给用户反馈，不能等完整回复生成完。
- 动作回调必须结构化，不能从自然语言里临时猜。
- TTS 与 LLM 分块并行，降低体感延迟。

方案对比：

- 非流式 LLM/TTS：实现简单，但等待时间长。
- LLM 直接发控制 topic：快但安全边界差。
- Agent 只产动作候选，C++ guard 再执行：更符合机器人系统分层。

## 8. 轻量 NLU：一句话多个命令识别

关键代码：

- `src/embodied_online_agent/embodied_online_agent/command_nlu.py`
- `src/embodied_online_agent/config/command_nlu_zh.json`
- `scripts/train_command_nlu.py`
- `tests/integration/test_continuous_multi_command.py`

设计方式：

- 使用字符 n-gram 原型模型识别控制意图，不依赖 torch/transformers。
- 输入一条 ASR final，输出多个动作片段和置信度。
- 例如“向右转，向前走一秒”会输出 `turn -> move`。
- 每个队列项带 `batch_id / batch_index / batch_size`，便于 monitor 解释顺序。
- 每个动作候选带 `request_id`，ActionGuard 映射成 `RobotCommand.command_id`，用于 result 关联。
- monitor 支持 `CONTINUOUS_SAMPLE_LOG=logs/asr_nlu_samples.jsonl`，把真实 ASR final、
  NLU/补全/归一化 feedback、动作候选和 result 写成 JSONL，方便把现场错词沉淀成回归集。
- `scripts/asr_nlu_samples_to_eval_candidates.py` 会把这些运行时事件按 `asr_final`
  分组，生成 `logs/asr_nlu_eval_candidates.jsonl`；候选样本带 `suggested_eval_case`，
  人工确认后即可补进 `training/robot_instruction_eval.jsonl`。
- `scripts/evaluate_asr_nlu_eval_candidates.py` 复用正式 parser 评估逻辑，对候选集先跑
  临时 accuracy；这让现场采到的错词即使还没合入正式数据集，也能马上用于回归观察。

为什么这样设计：

- 纯字符串 split 对无标点语音不稳，例如“向右转向前走一秒”。
- 大模型理解更强，但慢、不可预测、在线成本高。
- 轻量 NLU 覆盖固定机器人动作域，速度快、可测试、可解释。
- 真实 ASR 的错词分布很依赖麦克风和环境，靠人工凭记忆补测试很容易漏；采样日志转候选集
  可以把现场失败直接变成可回归的数据资产。
- 候选集和正式评估集分开，是为了避免“观察到的动作候选”未经人工确认就污染 ground truth；
  但候选集临时评估又能让工程迭代保持速度。

方案对比：

- 规则拆分：部署最简单，但表达能力弱。
- 大模型 function calling：泛化强，但响应和稳定性受模型影响。
- 本地轻量 NLU + ActionGuard：在固定动作域内更适合端侧演示。

## 9. 声纹识别与用户行为记忆

关键代码：

- `src/embodied_online_agent/embodied_online_agent/speaker_identity_node.py`
- `src/embodied_online_agent/embodied_online_agent/user_memory.py`
- `src/embodied_online_agent/embodied_online_agent/user_preferences.py`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `tests/integration/test_speaker_memory_mock.py`

设计方式：

- 声纹识别被做成 sidecar：订阅 `/audio/clean_pcm` 和 `/audio/speech_ended`，发布 `/agent/speaker_identity`。
- 声纹录入通过 `/agent/speaker_enroll_request` 触发，sidecar 把后续语音段保存成 wav 样本并维护 `speakers.txt`。
- Agent 只消费稳定 JSON identity，不直接绑定某个模型库。
- `UserMemoryStore` 按 `speaker_id` 保存本地 profile，包括用户名、偏好、常用动作、最近交互。
- Agent 推理前把当前用户画像追加进 system prompt，但动作仍必须经过 ActionGuard。
- `user_preferences.py` 在动作发布出口统一应用确定性偏好，例如 `movement_speed=slow/fast`、
  `default_move_duration_s`、`default_turn_degrees`；这样 fallback、轻量 NLU、多命令队列和 LLM 输出
  都能得到一致的参数调整。
- 管理命令直接在 Agent 层处理，例如“记住我，我是小李”“我喜欢慢一点”“我是谁”“清除我的记忆”。

为什么这样设计：

- 声纹模型属于可替换能力，和 ASR/LLM/动作控制主链路解耦，降低演示风险。
- 用户画像是长期稳定信息，不适合无限追加到普通对话历史里。
- 记忆写入必须可控，不能完全交给 LLM 自行决定，否则容易把误识别或幻觉写入本地 profile。
- 低置信度声纹返回 `unknown`；`UserMemoryStore` 对写操作增加 `LowConfidenceSpeakerError`
  门控，Agent 捕获后跳过个人记忆写入，避免把 A 用户偏好误写到 B 用户。
- 偏好只改写低层运动参数，且只在 speaker identity 可信时生效；真正的速度/时长边界继续由
  C++ ActionGuard 兜底，避免“记忆”绕过安全策略。

方案对比：

- 直接接 mem0/Letta/Zep：记忆能力强，但偏 Web Agent/服务端框架，当前 ROS2 端侧项目会变重。
- 使用 SpeechBrain/pyannote：模型能力成熟，但依赖 PyTorch 或 HuggingFace 模型，部署复杂。
- 当前方案：mock 可自动验收，sherpa-onnx seam 可接真实端侧声纹，和已有离线技术栈一致。

验收方式：

```bash
bash scripts/acceptance_test.sh speaker-memory-mock
bash scripts/acceptance_test.sh speaker-enroll
```

该验收证明：speaker identity 进入 Agent、用户偏好落盘、偏好能影响后续动作参数、动作执行后更新用户行为统计，并且声纹录入 seam 能采集样本文件。

## 10. 离线 Agent：Sherpa、llama.cpp、Sherpa-TTS/SummerTTS 与双缓冲

关键代码：

- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_tts.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/summer_tts.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/summer_tts_ros.py`
- `src/embodied_agent_interfaces/srv/SynthesizeSpeech.srv`
- `src/embodied_agent_cpp/src/summer_tts_service_node.cpp`
- `src/embodied_offline_agent/embodied_offline_agent/pseudo_streaming_tts.py`
- `src/embodied_offline_agent/embodied_offline_agent/double_buffer.py`
- `src/embodied_offline_agent/embodied_offline_agent/latency.py`
- `scripts/start_llama_server.sh`
- `scripts/llama_cpp_preflight.py`
- `scripts/setup_summer_tts_runtime.sh`
- `scripts/summer_tts_smoke.py`
- `scripts/smoke_test_summer_pseudo_tts.py`
- `scripts/generate_offline_showcase_report.py`
- `scripts/audit_offline_showcase_evidence.py`

设计方式：

- Sherpa ZipFormer 负责流式 ASR。
- llama.cpp server 提供 OpenAI-compatible streaming completion，`LlamaCppLlm` 只暴露 `stream(messages)`，
  让 Offline Agent 不关心底层是 llama.cpp、云 API 还是测试 fake client。
- Sherpa-TTS 是默认稳定 TTS provider；SummerTTS 是新增 C++ 独立编译 TTS provider，可通过 `tts_provider:=summer` 切换。
- `SummerTts` provider 调用 `third_party/SummerTTS/build/tts_test`，输入文本文件和 `.bin` 模型，读取 16kHz mono wav 后返回 PCM16 bytes。
- `SummerTtsServiceNode` 是常驻 C++ ROS component：节点启动时加载 `single_speaker_fast.bin`，
  对外提供 `/tts/synthesize` service，Python 侧 `SummerTtsRosClient` 通过 `tts_provider:=summer_ros` 调用。
  service 内部对短文本做 LRU 缓存，缓存 key 包含文本、speaker_id 和 length_scale，避免不同音色或语速误复用。
- `PseudoStreamingTtsPipeline` 把 Sherpa/SummerTTS 这种“整句生成”的本地 TTS 包装成伪流式：
  LLM 文字增量先经 `SentenceChunker` 切成短句，TTS worker 合成 PCM，audio worker 再按小块发布。
- 双缓冲把 LLM 文本生成、TTS 合成与音频输出解耦。
- latency 模块记录离线端到端耗时。
- llama.cpp provider 额外记录首 token、token 数、tokens/s、错误原因，并合并到 `/offline_agent/metrics`。
- TTS pipeline 额外记录 `text_chunks`、`synth_calls`、`audio_chunks`、`first_text_to_first_audio_ms`，
  并合并到 `/offline_agent/metrics.tts_pipeline`。
- `llama_cpp_preflight.py` 把 binary、模型文件、`/health`、`/v1/models`、低 token 流式 chat 分层验证。
- `summer_tts_smoke.py` 把 SummerTTS 源码、二进制、模型和真实合成分层验证；`summer-pseudo-tts`
  再验证真实 SummerTTS 能接入项目双缓冲伪流式 pipeline。
- `generate_offline_showcase_report.py` 汇总模型资产、运行时版本、parser accuracy、可选 latency/ASR/TTS benchmark；
  `audit_offline_showcase_evidence.py` 再审计这份报告，输出 `claim_guidance`，明确哪些指标已有证据、
  哪些只能作为后续计划。

为什么这样设计：

- 端侧算力有限，离线链路必须控制模型体积和串行等待。
- llama.cpp、Sherpa、SummerTTS 都是轻量本地部署方案，适合 CPU/边缘端演示。
- 双缓冲可以减少“LLM 等 TTS / TTS 等 LLM”的卡顿。
- 本地 TTS 通常不是天然流式；伪流式的关键是尽早切短句、尽早开始合成、音频按 PCM 小块发布。
- 推理层独立预检可以快速判断问题在模型服务、ASR、TTS 还是 ROS 控制链路，避免完整 demo 失败时只能猜。
- 离线展示最怕“工程接口接了”和“指标已复现”混在一起讲；证据审计脚本把未运行的 latency、
  ASR/TTS benchmark、LoRA 训练指标显式标成 warning，帮助汇报时守住边界。
- SummerTTS 是 C++ 项目，适合展示“端侧 C++ 运行时嵌入”；命令行 provider 保证部署简单，
  常驻 ROS component 则展示了更工程化的低耦合封装，并减少每句进程启动和模型加载开销。
- 短文本缓存只覆盖“收到/好的/正在执行”等反馈语，长句不缓存，避免内存被长音频占满；这属于工程优化，
  不是模型推理加速，因此文档仍把 Sherpa-TTS 作为默认低延迟 gate。
- 请求失败后只在“尚未吐出 token”时重试；如果流式回复已经输出一半，就不能静默重试，否则上游 parser 会收到拼接污染的回复。

方案对比：

- 全部云端：效果强，但不体现端侧部署能力。
- Python 大模型框架直接推理：开发方便，但部署和性能压力更大。
- llama.cpp + Sherpa：工程味更强，适合展示端侧推理思路。
- SummerTTS 命令行封装：接入最快、易验收，但每句会启动进程并加载模型；适合先打通链路。
- SummerTTS C++ 组件化封装：可复用已加载模型，服务接口清晰，但需要维护 C++ wrapper、ROS2 service
  和组件生命周期；短文本缓存能显著优化重复反馈，未命中时仍受 SummerTTS CPU infer 本身限制。
- 直接绑定 llama.cpp C API：可控性更强，但 Python/ROS2 集成和维护成本高；本项目选择 OpenAI-compatible server，
  用网络 seam 换取更低耦合、更容易 mock 和更清晰的部署边界。

## 11. BehaviorTree.CPP 与 pluginlib 仿真执行

关键代码：

- `src/embodied_simulation/config/command_tree.xml`
- `src/embodied_simulation/src/command_behavior_tree.cpp`
- `src/embodied_simulation/include/embodied_simulation/robot_executor.hpp`
- `src/embodied_simulation/src/robot_executor_plugins.cpp`
- `src/embodied_simulation/src/simulation_controller.cpp`

设计方式：

- BehaviorTree 负责动作执行流程：校验动作、检查安全、执行、确认结果。
- `RobotExecutor` 是统一接口。
- pluginlib 提供 `GazeboRobotExecutor` 和 `MockRobotExecutor` 两种后端。
- `SimulationController` 负责把动作转换成 `/cmd_vel`，并处理基础安全逻辑。

为什么这样设计：

- BT 把流程从 if/else 里抽出来，更接近 Nav2 的工程风格。
- pluginlib 让 mock 和 Gazebo 后端可替换，测试不必依赖 Gazebo。
- executor 分层后，未来接真实硬件或 Nav2 行为树更自然。

方案对比：

- 单个节点写死所有逻辑：短期快，但难测试、难扩展。
- 直接引入完整 Nav2：功能强，但本项目目标不是复杂导航，成本过高。
- 轻量 BT + pluginlib：足够展示工程规范，同时保持项目可跑通。

## 12. 测试体系

关键代码：

- `tests/repository/test_repository_layout.py`
- `tests/integration/test_acceptance_cli.sh`
- `tests/integration/test_continuous_voice_control.py`
- `src/embodied_online_agent/test/`
- `src/embodied_offline_agent/test/`
- `src/embodied_agent_cpp/test/`
- `src/embodied_simulation/test/`
- `scripts/acceptance_test.sh`
- `scripts/showcase_release_gate.py`

设计方式：

- repository test 保证文件结构和文档入口不漂移。
- Python 单测覆盖 Agent 侧规则、会话、队列、补全。
- C++ 单测覆盖 validator、adapter、仿真执行器。
- smoke script 覆盖 ROS 2 topic/action/launch 组合。
- `release-gate` 默认固定 5 条聚合命令，并输出 `logs/acceptance_report.json`；
  `demo-gate` 输出 `logs/demo_acceptance_report.json`，更偏现场展示证据，例如 provider preflight、
  speaker memory、连续多命令、语音导航 mock 和离线展示报告。这样演示前有一份可复查报告，
  而不是在大量 smoke 脚本中临时挑命令。
- 真实麦克风用 `continuous-live-check` 做人工辅助证据统计。

为什么这样设计：

- 机器人项目很容易“单个模块能跑，整链路断掉”。
- 分层测试能快速定位问题发生在 ASR、Agent、ActionGuard、Action、Gazebo 哪一层。
- 人工真实语音无法完全自动化，所以需要明确“人工验收标准”。

方案对比：

- 只做单测：无法证明 Gazebo 真动了。
- 只做人工演示：不可复现，回归成本高。
- 单测 + smoke + 人工验收：更适合当前工程规模。

## 13. 语音目标点导航与多目标点巡航

关键代码：

- `src/embodied_online_agent/embodied_online_agent/navigation_phrases.py`
- `src/embodied_online_agent/embodied_online_agent/command_nlu.py`
- `src/embodied_online_agent/embodied_online_agent/command_fallback.py`
- `src/embodied_agent_interfaces/msg/RobotCommand.msg`
- `src/embodied_agent_cpp/src/action_validator.cpp`
- `src/embodied_agent_cpp/src/robot_command_adapter.cpp`
- `src/embodied_simulation/include/embodied_simulation/nav2_places.hpp`
- `src/embodied_simulation/config/places.yaml`
- `src/embodied_simulation/rviz/voice_nav2_demo.rviz`
- `src/embodied_simulation/src/simulation_control_node.cpp`
- `src/embodied_simulation/src/robot_executor_plugins.cpp`
- `src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py`
- `scripts/continuous_nav2_voice_control.sh`
- `scripts/audit_nav2_demo_assets.py`
- `scripts/publish_nav2_initial_pose.py`
- `tests/integration/test_navigation_sequence.py`
- `tests/integration/test_continuous_navigation_queue.py`
- `tests/integration/test_nav2_bridge_sequence.py`
- `tests/integration/test_nav2_turtlebot3_voice.py`
- `tests/integration/test_continuous_nav2_voice_control_script.py`

设计方式：

- Agent 层只解析“去哪里”和“经过哪些点”，输出 `navigate_to` 或 `follow_waypoints`，不直接写坐标。
- `navigation_phrases.py` 用语义地点词表把“门口/书桌/起点”等口语映射为 `door/desk/home`。
- `RobotCommand.msg` 新增 `NAVIGATE_TO / FOLLOW_WAYPOINTS / CANCEL_NAVIGATION` 和 `target/waypoints/number_of_loops` 字段。
- `ActionGuard` 继续作为安全边界：校验地点白名单、巡航点数量、循环次数，再转换为 typed command。
- `places.yaml` 维护语义地点到地图坐标的映射。
- `tests/repository/test_repository_layout.py` 会检查 Agent 地点词表、ActionGuard 白名单和
  `places.yaml` 的 canonical place 完全一致，避免“语音能解析但 Nav2 不认地点”的漂移。
- mock/Gazebo executor 用“运动窗口”模拟导航和巡航，确保 `/cmd_vel`、Action feedback、Action result 可观测。
- `Nav2RobotExecutor` 作为 pluginlib 插件调用 Nav2 `NavigateToPose / FollowWaypoints` action；
  `nav2-bridge` 用 fake Nav2 action server 自动验证 goal 内容。
- `RobotExecutor::external_action_update()` 让 Nav2 action result 反向驱动本项目
  `ExecuteRobotCommand` 的 result，避免只按本地 `duration_s` 假完成。
- `RobotExecutor::external_action_detail()` 是配套的可观测性 seam：Nav2 插件记录
  `server_unavailable`、`goal_rejected`、`aborted/canceled`、`error_code/error_msg`、
  `missed_waypoints` 等原因，`simulation_control_node` 再把这些 detail 写入
  Action feedback/result。这样现场演示失败时可以从终端直接区分“Nav2 没启动”
  和“目标执行失败”，而不是只看到笼统的 `blocked`。
- `voice_nav2_turtlebot3.launch.py` 复用官方 `nav2_bringup/tb3_simulation_launch.py`，
  再叠加本项目的 Agent、ActionGuard、typed action bridge 和 Nav2 executor。
- launch 暴露 `rviz_config_file/world/map/params_file`，默认 RViz 使用本项目的
  `voice_nav2_demo.rviz`，默认 map/world 使用项目内的 `voice_demo.yaml` 和
  `voice_demo.sdf.xacro`，方便面试时稳定展示 TF、map、scan、odom、global plan。
- `audit_nav2_demo_assets.py` 审计语义地点、Nav2 launch、RViz 配置、本地 map/world
  和验收脚本；加 `--require-local-assets` 时，可以把“项目自带固定 map/world”
  变成严格发布条件。world 文件仍通过 `model://turtlebot3_world` 复用 TurtleBot3 官方
  场景几何，这是为了降低 Gazebo/Nav2 bringup 的不确定性；项目负责维护演示入口、
  map/world 文件、目标点配置和语音到 Nav2 action 的链路。
- `nav2-turtlebot3` 重型验收会启动真实 TurtleBot3/Nav2 仿真，注入语音文本命令，
  等待目标点导航/巡航 result，并检查 `/odom` 运动证据。
- `test_continuous_navigation_queue.py` 是介于普通连续队列测试和真实 Nav2 重型测试之间的
  自动回归：它验证一次唤醒后，多目标点导航和巡航命令都能进入连续队列，并按 request_id
  对应到 ROS 2 Action result。
- `continuous_nav2_voice_control.sh` 面向现场真实麦克风演示：它在 TurtleBot3/Nav2
  bringup 之上打开在线/离线 Agent 的连续语音模式，让用户一次唤醒后连续说多个目标点命令。
- `publish_nav2_initial_pose.py` 在演示启动后重复发布 AMCL `/initialpose`，降低现场
  “Nav2 已启动但机器人还没有定位”的失败概率。
- 真实 Nav2 bringup 前需要给 AMCL 发布 `/initialpose`，否则 map->odom/base_link TF
  不成立；导航 action 也要使用较长 `nav_action_timeout_s`，不能沿用普通动作 12 秒超时。

为什么这样设计：

- 先做语义地点，而不是直接语音转坐标，可以减少 ASR/LLM 的自由度，方便测试和演示。
- 新增强类型字段，而不是继续塞字符串字段，可以体现 ROS2/C++ 工程能力，也让后续 Nav2 bridge 更自然。
- 把真实 Nav2 作为 executor 插件，避免把导航细节侵入 Agent、ActionGuard 和测试。
- 用 external result seam 连接 Nav2 与本项目 Action，能体现“长动作可反馈、可取消、可等待结果”，
  而不是仅发布一个 topic 后马上认为成功。
- 失败 detail 由 executor 层产生，而不是让 Agent 猜测，是因为 planner/controller/localization
  的真实状态属于导航运行时；语音层只负责把自然语言变成目标，不能把底层故障伪装成语义错误。
- AMCL 初始位姿和长动作超时放在验收/launch 层处理，而不是塞进 Agent，保持“语音语义层”和
  “导航运行时状态层”职责分离。
- 连续麦克风 Nav2 演示脚本保留 `VOICE_CONTROL_PROFILE`、VAD endpoint、ASR commit delay、
  session timeout 和 queue size 参数，原因是导航命令更长、更容易被噪声或尾部漏识别影响；
  把这些参数显式打印出来，比“没反应时猜原因”更适合工程验收。
- Nav2 自己会发布 `/cmd_vel`，所以 `RobotExecutor::publishes_cmd_vel()` 允许 Nav2 插件禁止
  simulation node 周期性发布速度，避免两个控制器抢同一个速度话题。

方案对比：

- 直接让 LLM 输出 `{x,y,yaw}`：灵活但不稳定，且每个地图都要改 prompt；本项目更适合展示工程闭环，所以先固定语义地点。
- 只做 fake Nav2 bridge：速度快、CI 稳定，但不能证明 controller server 真的驱动机器人；因此本项目同时提供 `nav2-turtlebot3` 重型验收入口，演示前单独跑。
- 只做文本注入 Nav2 验收：自动化更稳，但不能覆盖真实麦克风的 ASR/session/queue 体验；因此新增 `continuous-nav2-offline/online` 作为人工演示入口。
- 只做字符串 topic：实现快，但难体现可取消、带反馈、可测试的 ROS 2 Action 能力。

## 14. 面试讲法建议

可以用这条主线介绍项目：

> 我做的是一个 ROS 2 机器人智能语音控制系统。前端用 ASR 把语音转文本，Agent 负责唤醒、连续会话、命令归一化、LLM 动作解析和 TTS。动作不会直接控制机器人，而是先进入 C++ ActionGuard 做校验和限幅，再转换成自定义 RobotCommand 和 ROS 2 Action。仿真侧用 BehaviorTree.CPP 编排安全检查、执行和结果确认，用 pluginlib 切换 mock/Gazebo executor。最终在 Gazebo/TurtleBot3 里验证 `/cmd_vel` 和 odom 变化。

强调点：

- 不是只调 API，而是打通了 ROS 2 端到端控制链路。
- 不是 LLM 直接发速度，而是有 ActionGuard 和强类型 Action。
- 不是只写 demo，而是有连续语音、队列、急停、验收脚本和测试体系。
- 不是复杂导航项目，当前重点是语音到动作到仿真控制的闭环。
