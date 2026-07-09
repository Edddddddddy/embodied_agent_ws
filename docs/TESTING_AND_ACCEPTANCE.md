# 测试与验收手册

本文档用于回答两个问题：

1. 项目哪些能力可以自动验证？
2. 真实麦克风、在线接口、Gazebo 仿真应该如何人工验收？

Nav2 语音目标点导航与多目标点巡航的逐项完成度审计见
[NAV2_VOICE_ACCEPTANCE_AUDIT.md](NAV2_VOICE_ACCEPTANCE_AUDIT.md)。

所有命令默认在 WSL Ubuntu 中执行：

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
source install/setup.bash 2>/dev/null || true
```

## 1. 验收入口

查看当前脚本支持的模式：

```bash
bash scripts/acceptance_test.sh --help
```

常用模式：

| 模式 | 类型 | 说明 |
| --- | --- | --- |
| `mock` | 自动 | 构建、单测、无模型 ROS smoke 主链路 |
| `online` | 自动/联网 | DashScope 在线 ASR/LLM/TTS 最小 token 验证 |
| `offline` | 自动/本地模型 | Sherpa/llama.cpp/Sherpa-TTS 真实离线链路 |
| `offline-runtime-versions` | 自动/本地版本 | 检查 llama.cpp、SummerTTS、sherpa-onnx 是否匹配阶段固定版本 |
| `offline-showcase-report` | 自动/报告 | 汇总模型大小、运行时版本、指令解析准确率和 `claim_evidence` 指标证据矩阵，输出离线展示 JSON/Markdown 报告 |
| `offline-evidence-audit` | 自动/报告 | 审计离线报告的证据强度，透传缺失/未复现指标，输出哪些指标可宣称、哪些仍缺真实 benchmark |
| `llama-decode-benchmark` | 自动/本地模型 | 调用 llama.cpp `llama-bench` 测量 CPU decode tokens/s，输出 `logs/llama_decode_benchmark.json` |
| `offline-latency` | 自动/本地模型 | 检查 llama.cpp 首 token ≤ 1s、默认 Sherpa-TTS 首音频 ≤ 300ms |
| `instruction-eval-dataset` | 自动/数据集 | 校验轻量机器人指令评估集 schema、动作名和标签 |
| `instruction-parser-eval` | 自动/数据集 | 在评估集上计算 deterministic parser 动作准确率和 tag 维度分数 |
| `release-gate` | 自动/报告 | 求职展示版发布门禁，默认输出 `logs/acceptance_report.json` |
| `demo-gate` | 自动/报告 | 演示前自动证据门禁，默认输出 `logs/demo_acceptance_report.json` |
| `summer-tts-preflight` | 自动/本地模型 | SummerTTS 源码、二进制和模型文件预检 |
| `summer-tts-smoke` | 自动/本地模型 | 真实 SummerTTS C++ 二进制合成验证 |
| `summer-pseudo-tts` | 自动/本地模型 | SummerTTS 与项目伪流式双缓冲 pipeline 集成验证 |
| `summer-tts-service` | 自动/ROS2/C++ | 常驻 SummerTTS C++ ROS service 合成验证 |
| `sherpa-asr-preflight` | 自动/本地模型 | ASR-only 预检：检查 `sherpa_onnx` 和 ZipFormer 模型文件 |
| `sherpa-asr-smoke` | 自动/本地模型 | ASR-only 真实解码：用 ZipFormer test wav 验证 Sherpa provider |
| `offline-sherpa-typed` | 自动/本地模型/ROS2 | Sherpa ASR/TTS + llama.cpp 经过 ActionGuard、typed Action 和仿真 `/cmd_vel` |
| `navigation-demo` | 自动/仿真 | 语音风格目标点导航与多目标点巡航，覆盖 online/offline mock Agent |
| `nav2-bridge` | 自动/Nav2 seam | 用 fake Nav2 action server 验证语义地点会发成 NavigateToPose/FollowWaypoints goal |
| `nav2-preflight` | 自动/Nav2 | 检查 Nav2/TurtleBot3 voice launch 依赖和参数 |
| `nav2-assets` | 自动/Nav2 | 审计语义地点、Nav2 launch、RViz 展示配置和 map/world 资产边界 |
| `nav2-stage` | 自动/Nav2 | 语音导航阶段门禁：解析、连续队列、Nav2 bridge、preflight |
| `nav2-turtlebot3` | 重型/Nav2/Gazebo | 启动官方 Nav2 TurtleBot3 仿真，注入语音文本，验证目标点导航/巡航 result 与 odom |
| `gazebo` | 自动/仿真 | typed Action 到 Gazebo 运动验证 |
| `gazebo-voice` | 自动/仿真 | 离线合成语音到 Gazebo 动作 |
| `gazebo-voice-online` | 自动/联网/仿真 | 在线 provider 到 Gazebo 动作 |
| `continuous-endpoint` | 自动 | `/audio/speech_ended` endpoint 驱动 ASR commit |
| `continuous-mock` | 自动 | 一次唤醒、多条命令、队列和休眠 |
| `continuous-soak` | 自动 | 长会话连续命令稳定性 |
| `continuous-queue-full` | 自动 | busy 时队列满反馈 |
| `continuous-multi-command` | 自动 | 一条 ASR final 被轻量 NLU 解析成多条队列命令 |
| `continuous-navigation` | 自动/仿真 | 连续会话中目标点导航与多目标点巡航按队列顺序执行 |
| `continuous-navigation-natural` | 自动/仿真 | 自然多目标话术解析为多目标点巡航并按队列执行 |
| `speaker-memory-mock` | 自动 | mock 声纹身份事件、用户偏好记忆、prompt/action 记录 |
| `speaker-enroll` | 自动 | 声纹录入请求收集 wav 样本并生成 `speakers.txt` |
| `continuous-ttl` | 自动 | 过期命令丢弃 |
| `continuous-timeout` | 自动 | 会话超时后重新要求唤醒 |
| `voice-readiness` | 自动 | 麦克风/音频前端 readiness 检查 |
| `voice-calibration-report` | 自动/报告 | 汇总 provider/audio/KWS 校准，输出 `logs/voice_calibration_report.json/.md` 和可 `source` 的 env 文件 |
| `continuous-offline` | 人工 | 真实麦克风离线连续控制 |
| `continuous-online` | 人工/联网 | 真实麦克风在线连续控制 |
| `continuous-live-check` | 人工辅助 | 订阅 topic 并统计现场演示证据 |
| `continuous-live-report` | 自动/复盘 | 读取已保存的连续语音验收报告并重新判定 |
| `continuous-nav2-offline` | 人工/Nav2 | 真实麦克风离线连续目标点导航与多目标点巡航 |
| `continuous-nav2-online` | 人工/联网/Nav2 | 真实麦克风在线连续目标点导航与多目标点巡航 |
| `continuous-nav2-evidence` | 人工辅助/Nav2 | 一终端启动 Nav2 语音控制、现场计分并保存报告 |
| `continuous-nav2-live-check` | 人工辅助/Nav2 | 订阅 topic 并统计现场 Nav2 连续导航演示证据 |
| `continuous-nav2-live-report` | 自动/复盘/Nav2 | 读取已保存的 Nav2 连续语音验收报告并重新判定 |
| `all` | 自动/重型 | 全量自动 gate，不包含人工 microphone 模式 |

## 2. 推荐测试顺序

### 2.0 求职展示版 release gate

提交 PR 或录制演示前，优先运行聚焦版发布门禁：

```bash
bash scripts/acceptance_test.sh release-gate
```

该入口默认是 `core` profile，固定 5 条聚合命令：

1. Python repository/online/offline Agent 单测；
2. CLI 入口、指令数据集校验和指令解析准确率；
3. 连续语音 session、队列和多命令拆分；
4. 语音导航 demo 与 Nav2 bridge；
5. 离线延迟、SummerTTS service 和 C++/ROS2 单测。

脚本会把每个步骤的命令、耗时、退出码和尾部日志写入：

```text
logs/acceptance_report.json
```

15 分钟汇报或现场演示前，建议再运行演示证据 gate：

```bash
bash scripts/acceptance_test.sh demo-gate
```

该入口使用 `demo` profile，固定 5 条偏展示的自动证据：

1. CLI 入口可发现；
2. VAD/KWS provider preflight、语音 readiness 与 voice calibration report；
3. speaker identity、用户记忆和偏好影响动作参数；
4. 连续语音 session、多命令队列；
5. 语音导航 mock 与离线展示报告。

报告默认写入：

```text
logs/demo_acceptance_report.json
```

如果只想检查 gate 列表而不运行命令：

```bash
python3 scripts/showcase_release_gate.py --dry-run
python3 scripts/showcase_release_gate.py --profile demo --dry-run
```

如果需要更完整但更慢的本地门禁：

```bash
python3 scripts/showcase_release_gate.py --profile full
python3 scripts/showcase_release_gate.py --profile full --dry-run
```

动作解析评估可以单独运行：

```bash
bash scripts/acceptance_test.sh instruction-eval-dataset
bash scripts/acceptance_test.sh instruction-parser-eval
```

`instruction-parser-eval` 会读取 `training/robot_instruction_eval.jsonl`，当前代表集为
39 条，覆盖移动、转向、ASR 错词、多命令、组合动作、Nav2 导航/巡航和安全拒绝样例。
输出包含整体准确率、`source_counts`、tag 准确率、实际/期望动作和 `failed_cases`，
适合持续沉淀真实 ASR 错误样例。

真实麦克风演示时可以打开样本采集：

```bash
CONTINUOUS_SAMPLE_LOG=logs/asr_nlu_samples.jsonl \
  bash scripts/acceptance_test.sh continuous-offline
```

该 JSONL 会保存 ASR final、归一化/补全/NLU feedback、动作候选和 result。
演示后先把运行时事件流整理成待审核候选集：

```bash
ASR_NLU_SAMPLES_SYNTHETIC=false \
  ASR_NLU_SAMPLE_LOG=logs/asr_nlu_samples.jsonl \
  ASR_NLU_EVAL_OUTPUT=logs/asr_nlu_eval_candidates.jsonl \
  bash scripts/acceptance_test.sh asr-nlu-samples-to-eval
```

没有真实日志时，也可以运行默认合成样本检查转换工具是否可用：

```bash
bash scripts/acceptance_test.sh asr-nlu-samples-to-eval
```

输出候选集里的 `suggested_eval_case` 接近 `training/robot_instruction_eval.jsonl`
格式，但仍必须人工确认动作是否符合真实意图，再合入评估集并运行 `instruction-parser-eval`
做回归。
如果暂时不想合入正式评估集，也可以直接对候选集跑临时 accuracy：

```bash
ASR_NLU_CANDIDATE_SYNTHETIC=false \
  ASR_NLU_CANDIDATE_INPUT=logs/asr_nlu_eval_candidates.jsonl \
  ASR_NLU_CANDIDATE_REPORT=logs/asr_nlu_candidate_eval_report.json \
  bash scripts/acceptance_test.sh asr-nlu-candidate-eval
```

这个报告用于现场复盘和规则/轻量 NLU 迭代，不替代人工审核；只有确认过的样本才建议合入
`training/robot_instruction_eval.jsonl`。

### 2.1 无外部依赖基础验收

```bash
pytest -q tests/repository
bash tests/integration/test_acceptance_cli.sh
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh nav2-bridge
bash scripts/acceptance_test.sh nav2-preflight
bash scripts/acceptance_test.sh nav2-assets
bash scripts/acceptance_test.sh nav2-turtlebot3
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh continuous-navigation
bash scripts/acceptance_test.sh continuous-queue-full
bash scripts/acceptance_test.sh voice-readiness
```

通过后说明：仓库结构、脚本入口、连续语音会话、队列、endpoint、readiness 基本正常。

用户声纹与本地记忆 mock 验收：

```bash
bash scripts/acceptance_test.sh speaker-memory-mock
bash scripts/acceptance_test.sh speaker-enroll
```

通过后说明：

- `/agent/speaker_identity` 能驱动 Agent 绑定当前用户。
- “记住我，我是小李”“我喜欢慢一点”“我是谁”等管理命令能写入/读取本地用户画像。
- “我喜欢慢一点”会影响后续 `move` 动作候选参数，证明记忆不只是 prompt 上下文。
- 普通动作执行后会把动作统计写入当前用户 profile。
- `/agent/speaker_enroll_request` 能触发 sidecar 收集 3 段 wav 样本并维护 speaker-file。
- 该验收不依赖真实声纹模型；真实 sherpa-onnx 声纹需要另行准备 speaker embedding 模型和注册 wav。
`navigation-demo` 额外证明“去门口”和“依次去门口、书桌、起点”能被 online/offline
Agent 解析成 `navigate_to / follow_waypoints`，并通过 typed Action 驱动仿真 executor。

### 2.1.1 Sherpa-ONNX ASR-only 真实部署检查

如果只想先验证离线 ASR 推理框架，不想下载/编译完整离线 LLM/TTS 栈，运行：

```bash
bash scripts/setup_sherpa_asr_runtime.sh
bash scripts/acceptance_test.sh sherpa-asr-preflight
bash scripts/acceptance_test.sh sherpa-asr-smoke
```

通过后说明：

- `sherpa_onnx` Python 包可导入。
- ZipFormer encoder/decoder/joiner/tokens 文件存在且大小合理。
- `SherpaZipformerAsr` provider 可以加载真实模型，并对 test wav 输出 final 文本。

这只覆盖 ASR 层；完整离线链路仍使用：

```bash
bash scripts/setup_offline_runtime.sh
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh pseudo-tts
bash scripts/acceptance_test.sh offline-runtime-versions
bash scripts/acceptance_test.sh offline-latency
bash scripts/acceptance_test.sh summer-tts-preflight
bash scripts/acceptance_test.sh summer-tts-smoke
bash scripts/acceptance_test.sh summer-pseudo-tts
bash scripts/acceptance_test.sh offline
```

### 2.1.2 SummerTTS TTS-only 真实部署检查

SummerTTS 是独立 C++ 离线语音合成项目，源码和模型部署在 `third_party/SummerTTS`。
它和 Sherpa-TTS 的定位不同：Sherpa-TTS 通过 Python `sherpa_onnx` 包加载 ONNX/VITS
模型；SummerTTS 通过本地 C++ `tts_test` 二进制读取文本文件和 `.bin` 模型，输出
16kHz mono PCM wav。项目中的 `SummerTts` provider 会剥离 wav 头，返回 PCM16 bytes，
再交给 `PseudoStreamingTtsPipeline` 做双缓冲伪流式发布。

部署与验收：

```bash
bash scripts/setup_summer_tts_runtime.sh
bash scripts/acceptance_test.sh summer-tts-preflight
bash scripts/acceptance_test.sh summer-tts-smoke
bash scripts/acceptance_test.sh summer-pseudo-tts
bash scripts/acceptance_test.sh summer-tts-service
```

通过标准：

- `third_party/SummerTTS/build/tts_test` 存在且可执行。
- `third_party/SummerTTS/models/single_speaker_fast.bin` 存在。
- `summer-tts-smoke` 能输出非空 PCM。
- `summer-pseudo-tts` 的 `tts_pipeline.synth_calls` 为 2，且产生多个 audio chunks。
- `summer-tts-service` 能启动 `embodied_agent_cpp/summer_tts_service`，通过 `/tts/synthesize`
  返回 `sample_rate=16000` 和非空 PCM；probe 默认重复请求同一短文本，第二次应出现
  `cache_hit=true`，用于证明常驻服务的短反馈缓存生效。

常见失败定位：

- `uint16_t was not declared`：直接运行 `setup_summer_tts_runtime.sh`，脚本会为 Ubuntu
  24.04/GCC 13 自动补 `<cstdint>` 兼容 include。
- `SummerTTS runtime preflight failed`：检查 `third_party/SummerTTS` 是否完整克隆、构建是否完成。
- 完整离线 Agent 想切换 SummerTTS：启动时设置 `tts_provider:=summer`。
- 完整离线 Agent 想切换常驻 C++ service：启动时设置 `tts_provider:=summer_ros`。
- SummerTTS 命令行 provider 每句会启动进程并加载模型；`summer_ros` 已消除这部分开销，
  并缓存短文本反馈；但未命中的 SummerTTS CPU infer 仍明显高于 Sherpa-TTS，因此
  `offline-latency` 的 `<300ms` TTS 指标仍以默认 Sherpa-TTS provider 为准。

### 2.1.3 离线低延迟指标验收

```bash
bash scripts/acceptance_test.sh offline-latency
```

通过标准：

- `llm.first_token_ms <= 1000`。
- `tts.provider == "sherpa"`。
- `tts.first_audio_ms <= 300`。
- `ok == true`。

该模式会启动或复用 `llama-server`，发送一次极短流式请求，并在常驻 Sherpa-TTS
provider 上合成短句。它验证的是当前离线 Agent 默认低延迟路径，而不是 SummerTTS
命令行封装路径。

### 2.1.1 llama.cpp 推理层验收

llama.cpp 现在有独立分层验收，建议在排查离线链路时先跑：

```bash
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh pseudo-tts
```

通过标准：

- `third_party/llama.cpp/build/bin/llama-server` 存在且可执行。
- `models/Qwen3-0.6B-Q8_0.gguf` 存在。
- `llama-server` 的 `/health`、`/v1/models` 可访问。
- smoke 模式能从 `/v1/chat/completions` 收到流式 token，并打印 `first_token_ms`。
- `pseudo-tts` 能证明 token 流产生的短句被伪流式 TTS pipeline 异步合成并发布音频块。

常见失败定位：

- `MISSING llama-server binary`：先运行 `bash scripts/setup_offline_runtime.sh`。
- `MISSING GGUF model`：确认模型文件放在 `models/Qwen3-0.6B-Q8_0.gguf`，或设置 `LLAMA_MODEL`。
- `cannot connect` / health 超时：查看 `scripts/start_llama_server.sh` 输出，降低 `LLAMA_THREADS` 或缩小 `LLAMA_CONTEXT`。
- Offline Agent 已收到 ASR final 但无动作：查看 `/offline_agent/metrics` 的 `llm_provider` 字段和 Agent 日志。

如果只想验证“Sherpa-ONNX 语音模型参与的 ROS2 typed Action 控制闭环”，运行：

```bash
bash scripts/acceptance_test.sh offline-sherpa-typed
```

该模式会启动：

```text
Sherpa-TTS 合成命令音频
  -> /audio/clean_pcm
  -> Sherpa ZipFormer ASR
  -> Offline Agent + llama.cpp + Sherpa-TTS
  -> /agent/action_candidate
  -> C++ ActionGuard
  -> /robot/action_command_typed
  -> ExecuteRobotCommand Action
  -> simulation_control MockRobotExecutor
  -> /cmd_vel + /robot/action_result
```

详细说明见 [SHERPA_ONNX_DEPLOYMENT.md](SHERPA_ONNX_DEPLOYMENT.md)。

### 2.2 Python/C++ 单元测试

```bash
pytest -q src/embodied_online_agent/test src/embodied_offline_agent/test
colcon test --packages-select embodied_agent_cpp embodied_simulation --event-handlers console_direct+
colcon test-result --verbose
```

重点覆盖：

- 命令归一化、短命令补全、fallback parser。
- 连续语音 session、重复过滤、filler 过滤、队列。
- ActionGuard 限幅与 RobotCommandAdapter。
- BehaviorTree、executor、仿真控制逻辑。

### 2.2.1 C++ ROS 2 Action client 示例

```bash
bash scripts/acceptance_test.sh cpp-action-client
```

该模式启动 mock `simulation_control` action server，然后运行
`ros2 run embodied_agent_cpp typed_action_demo_client move 0.10 0.20`。
它专门用于展示 C++ `rclcpp_action` client 如何构造 `RobotCommand`、发送
`ExecuteRobotCommand` goal、接收 feedback/result，并根据 result 退出。

### 2.3 Gazebo 仿真验收

```bash
bash scripts/acceptance_test.sh gazebo
```

更接近端到端语音链路：

```bash
bash scripts/acceptance_test.sh gazebo-voice
bash scripts/acceptance_test.sh gazebo-voice-online
```

通过标准：

- `/agent/action_candidate` 产生动作候选。
- `/robot/action_command_typed` 产生强类型命令。
- `/robot/action_result` 返回 success。
- `/cmd_vel` 有速度输出。
- `/odom` 显示机器人位移或旋转变化。

## 3. 真实麦克风连续验收

离线优先：

```bash
bash scripts/acceptance_test.sh continuous-offline
```

在线补充：

```bash
bash scripts/acceptance_test.sh continuous-online
```

`continuous-offline/online` 会先检查 Gazebo/TurtleBot3 仿真 readiness：必须收到
`/odom`、`/scan`，并且 `/cmd_vel` 与 `/robot/execute_command` 链路在线。若检查失败，
脚本会直接停止，避免出现“语音在跑，但 Gazebo 里没有小车模型”的演示假阳性。
如果只想先排查仿真本身，运行：

```bash
bash scripts/acceptance_test.sh gazebo
```

如果 WSL 里残留了旧 Gazebo 进程，可能出现 GUI 空世界、模型不出现、odom 跳变等现象。
先运行：

```bash
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh
```

也可以让连续语音演示入口启动前自动清理：

```bash
SIMULATION_CLEANUP_STALE=true bash scripts/acceptance_test.sh continuous-online
```

推荐话术：

```text
小智
向前走一秒
左转九十度
后退一秒
绕圈
走正方形
停下
退出控制
```

另开一个终端做辅助统计：

```bash
bash scripts/acceptance_test.sh continuous-live-check offline
# 或
bash scripts/acceptance_test.sh continuous-live-check online
```

通过标准：

- 至少出现 6 条 `/agent/asr_final`。
- 至少出现 4 条 `/agent/action_candidate`。
- 至少出现 4 条成功 `/robot/action_result`。
- `/agent/session_state` 出现 awake 和 sleeping。
- `停下/急停` 能抢占，队列被清空。
- 结束后 `/cmd_vel` 为 0。

## 4. 语音目标点导航与多目标点巡航

自动验收：

```bash
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh nav2-bridge
bash scripts/acceptance_test.sh nav2-preflight
bash scripts/acceptance_test.sh continuous-navigation
```

覆盖链路：

```text
/agent/text_input 模拟 ASR final
  -> CommandNLU / fallback parser
  -> /agent/action_candidate
  -> C++ ActionGuard
  -> RobotCommand.NAVIGATE_TO / FOLLOW_WAYPOINTS
  -> ROS 2 ExecuteRobotCommand Action
  -> BehaviorTree + pluginlib executor
  -> /cmd_vel 与 /robot/action_result
```

当前支持的话术示例：

```text
去门口
前往书桌
回到起点
导航到厨房
依次去门口、书桌、起点
开始巡航
停止巡航 / 取消导航
```

通过标准：

- online 和 offline mock Agent 均 PASS。
- `去门口` 解析为 `{"name": "navigate_to", "arguments": {"target": "door"}}`。
- `依次去门口、书桌、起点` 解析为 `follow_waypoints`，waypoints 为 `door/desk/home`。
- `continuous-navigation` 会验证一次唤醒后，`去门口，然后前往书桌` 与
  `依次去门口、书桌、起点` 能排入同一个连续控制链路，且 result 与 request_id 对应。
- `/robot/action_result` 中对应 command_id success=true。
- `/cmd_vel` 能观察到目标导航的前进速度，以及巡航的线速度 + 角速度。

边界说明：语义地点仍由 `places.yaml` 维护，不在本阶段做自动建图或复杂任务规划。
仓库结构测试会锁住 Agent 地点词表、ActionGuard 地点白名单和 `places.yaml` 的
canonical place 一致性，防止某一层配置漂移。
`navigation-demo` 由仿真 executor 生成可观测运动；`nav2-bridge` 会启动 fake Nav2
action server，证明 `Nav2RobotExecutor` 已能把语义地点转换成真正的
`NavigateToPose / FollowWaypoints` goal，并且 action result 由 Nav2 result 驱动，
不是本地固定 duration 假完成。`nav2-preflight` 用于在启动重型 Gazebo/Nav2 前
确认依赖包、`voice_nav2_turtlebot3.launch.py`、`nav_action_timeout_s` 和关键参数可用；
`nav2-turtlebot3` 用于真实 TurtleBot3/Nav2 bringup，验证目标点导航/多目标点巡航
result 和 `/odom` 运动证据。Nav2 executor 会把 `server_unavailable`、`goal_rejected`、
`aborted/canceled`、`error_code/error_msg`、`missed_waypoints` 等 detail 透传到本项目
typed action feedback/result，便于现场判断是 action server 没起来、目标被拒绝、规划/控制失败
还是巡航点未到达。该模式会给 AMCL 发布 `/initialpose`，并把 Nav2 长动作
超时提高到演示级窗口，避免按普通短动作提前取消真实导航 goal。该模式耗时较长，
通常不放入 CI。
`nav2-assets` 会输出 `logs/nav2_demo_assets.json`，当前应能看到 `places/launch/rviz_config`
以及 `local_assets.local_maps/local_worlds/local_map_images` 为 present。项目默认 map 为
`src/embodied_simulation/maps/voice_demo.yaml`，默认 world 为
`src/embodied_simulation/worlds/voice_demo.sdf.xacro`；如果后续误删这些本地资产，
`python3 scripts/audit_nav2_demo_assets.py --require-local-assets` 会把缺失项作为发布 blocker。

真实麦克风连续 Nav2 演示：

```bash
bash scripts/acceptance_test.sh continuous-nav2-offline
# 或
bash scripts/acceptance_test.sh continuous-nav2-online
```

推荐的一终端留证方式：

```bash
CONTINUOUS_LIVE_CHECK_REPORT=logs/nav2-live-check.json \
  CONTINUOUS_LIVE_CHECK_DURATION=240 \
  bash scripts/acceptance_test.sh continuous-nav2-evidence offline
```

该模式会在同一个 `ROS_DOMAIN_ID` 下后台启动连续 Nav2 语音控制，前台运行
`continuous-nav2-live-check` 等价的现场统计，并在结束时清理 Gazebo/Nav2/Agent。
如果要分开观察日志和 topic，再使用下面的两终端方式。
正式启动前可用 dry-run 自检参数，不会占用麦克风或启动 Gazebo：

```bash
CONTINUOUS_NAV2_EVIDENCE_DRY_RUN=true \
  CONTINUOUS_LIVE_CHECK_REPORT=logs/nav2-live-check.json \
  bash scripts/acceptance_test.sh continuous-nav2-evidence offline
```

推荐另开一个终端做现场计分：

```bash
CONTINUOUS_LIVE_CHECK_DURATION=240 bash scripts/acceptance_test.sh continuous-nav2-live-check offline
```

`continuous-nav2-live-check` 会在通用连续语音统计基础上，额外要求至少出现一次
`navigate_to` 和一次 `follow_waypoints` action candidate。
如果需要留存验收证据，可以指定报告文件：

```bash
CONTINUOUS_LIVE_CHECK_REPORT=logs/nav2-live-check.json \
  CONTINUOUS_LIVE_CHECK_DURATION=240 \
  bash scripts/acceptance_test.sh continuous-nav2-live-check offline
```

报告文件可以作为阶段验收附件保存。复盘或发给他人确认时，可直接重新判定：

```bash
bash scripts/acceptance_test.sh continuous-nav2-live-report logs/nav2-live-check.json
```

推荐话术：

```text
小智
去门口
前往书桌
依次去门口、书桌、起点
停止巡航
退出控制
```

通过标准：

- 终端持续打印 session、ASR、queue、action/result 事件。
- `去门口/前往书桌` 能产生 `navigate_to`，并收到 Nav2 action result。
- `依次去门口、书桌、起点` 能产生 `follow_waypoints`，多个 waypoint 按顺序进入 Nav2。
- 执行过程中再次说目标点命令时，命令进入连续队列等待，而不是丢失。
- `停止巡航/取消导航/停下` 能抢占当前导航并让 `/cmd_vel` 归零。

## 5. 真实语音问题排查

### 5.1 ASR 完全没听到

运行：

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
bash scripts/acceptance_test.sh voice-readiness
bash scripts/acceptance_test.sh voice-calibration-report
python scripts/audio_frontend_calibration.py --duration 6
python scripts/audio_frontend_calibration.py --duration 6 --json > logs/audio_calibration.json
```

`voice-calibration-report` 会输出 `logs/voice_calibration_report.json/.md` 和
`logs/voice_calibration.env`，把 provider preflight、音频指标、KWS 阈值和下一条建议命令合并到同一份报告。
使用 `openwakeword/livekit` 时，如果采集到了 `/agent/kws_score`，
env 文件会额外写入 `OPENWAKEWORD_THRESHOLD` 或 `LIVEKIT_WAKEWORD_THRESHOLD`，
用于下一轮真实唤醒词阈值复测。
真实 topic 采集方式：

```bash
VOICE_CALIBRATION_COLLECT=true bash scripts/acceptance_test.sh voice-calibration-report
source logs/voice_calibration.env
bash scripts/acceptance_test.sh continuous-offline
```

如果不想手动 `source`，连续语音脚本默认 `APPLY_VOICE_CALIBRATION=auto`：当
`logs/voice_calibration.env` 存在时会自动加载；当你显式传入
`VOICE_CONTROL_PROFILE/SPEECH_START_THRESHOLD/VAD_PROVIDER` 等关键环境变量时，显式值优先。
也可以强制加载或关闭自动加载：

```bash
APPLY_VOICE_CALIBRATION=true bash scripts/acceptance_test.sh continuous-offline
APPLY_VOICE_CALIBRATION=false bash scripts/acceptance_test.sh continuous-offline
```

`audio_frontend_calibration.py` 的文本输出会给出更底层的 `recommended environment` 和 `next command`；
JSON 输出会保留 `recommended_environment`、`suggested_vad_threshold`、`next_command`，
可作为真实麦克风演示前的校准证据。

如果 `wsl-microphone-preflight` 录到的 `rms≈0.0000`、`peak` 只有个位数，说明
WSLg/PulseAudio source 存在但没有真实麦克风音频。这个问题发生在 ROS 音频前端之前，
需要先检查 Windows 麦克风权限、默认输入设备或 WSLg 音频转发。

如果 `wsl-microphone-preflight` 已 PASS，但 `continuous-offline` 中 C++ audio frontend
长期只显示近静音，优先确认终端是否显示：

```text
PULSE_CAPTURE_BRIDGE=auto（active=true）
enhancer=pulse_bridge
```

该 bridge 使用 `parecord` 从 WSLg PulseAudio 捕获 PCM，并发布项目既有
`/audio/clean_pcm`、`/audio/frontend_metrics`、`/audio/speech_started`、
`/audio/speech_ended`，用于绕过 WSL 中 PortAudio/ALSA 默认输入不可用的问题。

如果 speech ratio 很低，尝试：

```bash
VOICE_CONTROL_PROFILE=quiet bash scripts/acceptance_test.sh continuous-offline
```

如果终端持续出现类似：

```text
[audio] rms=0.0023 peak=180 speech=False
```

这表示音频链路有输入，但输入增益低于默认 VAD 阈值。优先使用低增益 profile：

```bash
VOICE_CONTROL_PROFILE=low_gain bash scripts/acceptance_test.sh continuous-offline
```

连续语音脚本默认 `VAD_PROVIDER=auto`：如果本机安装了 `silero-vad` 和 `onnxruntime`，
会启动 Silero sidecar；如果 Silero 不可用但安装了 `webrtcvad`，会启动更轻量的
WebRTC sidecar；两者都不可用时才打印 `vad:auto_fallback:energy:...` 并降级到
energy VAD。`provider-preflight` 会同时输出 `recommendations`，例如推荐执行
`bash scripts/setup_voice_vad_runtime.sh webrtc`，避免现场只看到缺包列表却不知道下一步。
想强制验证某个 provider，可运行：

```bash
VAD_PROVIDER=silero bash scripts/acceptance_test.sh provider-preflight
VAD_PROVIDER=silero bash scripts/acceptance_test.sh continuous-offline
VAD_PROVIDER=webrtc bash scripts/acceptance_test.sh provider-preflight
VAD_PROVIDER=webrtc bash scripts/acceptance_test.sh continuous-offline
```

WebRTC VAD 的可选安装命令：

```bash
bash scripts/acceptance_test.sh voice-vad-runtime-dry-run
bash scripts/setup_voice_vad_runtime.sh webrtc
VAD_PROVIDER=auto bash scripts/acceptance_test.sh provider-preflight
bash scripts/acceptance_test.sh webrtc-vad-sidecar
```

`webrtc-vad-sidecar` 会启动 C++ audio frontend 和 WebRTC VAD sidecar，要求当前环境已经安装
`webrtcvad`。它比 `vad-sidecar` 更接近真实运行时：`vad-sidecar` 只验证 Silero sidecar
的无依赖 seam，`webrtc-vad-sidecar` 则验证轻量成熟 VAD runtime 可以真正启动并接管
`/audio/speech_started` / `/audio/speech_ended` 端点事件。

如果要同时准备 Silero 和 WebRTC：

```bash
bash scripts/setup_voice_vad_runtime.sh all
```

底层等价方式是安装 `embodied_online_agent[webrtc-vad]` 或
`embodied_online_agent[silero-vad]` extra；项目脚本会在安装后自动跑 provider preflight。

声学唤醒 KWS 也有独立运行时准备入口。先 dry-run：

```bash
bash scripts/acceptance_test.sh voice-kws-runtime-dry-run
```

准备 openWakeWord：

```bash
bash scripts/setup_voice_kws_runtime.sh openwakeword
KWS_PROVIDER=openwakeword bash scripts/acceptance_test.sh provider-preflight
```

准备 sherpa-onnx KWS；脚本会复用 ASR ZipFormer 模型并生成默认关键词文件：

```bash
bash scripts/setup_voice_kws_runtime.sh sherpa
source logs/sherpa_kws.env
KWS_PROVIDER=sherpa bash scripts/acceptance_test.sh provider-preflight
bash scripts/acceptance_test.sh sherpa-kws-sidecar
```

`sherpa-kws-sidecar` 会用 `logs/sherpa_kws.env` 中的模型路径启动真实
`sherpa_onnx.KeywordSpotter`，验证声学 KWS runtime 至少可以完成模型加载和节点启动；
真实唤醒词召回率仍需要在麦克风现场通过 `/agent/kws_score` 和连续语音验收继续采样。

也可以直接套用 calibration/readiness 给出的阈值，例如：

```bash
SPEECH_START_THRESHOLD=0.0012 AEC_ENABLED=false bash scripts/acceptance_test.sh continuous-offline
```

如果环境噪声持续触发，尝试：

```bash
VOICE_CONTROL_PROFILE=noisy_room bash scripts/acceptance_test.sh continuous-offline
```

### 5.2 “左转90度”只识别成“左转”

当前链路有两层保护：

1. endpoint 更稳：`SPEECH_END_SILENCE_S` 和 `ASR_COMMIT_DELAY_MS`。
2. 命令补全：裸 `左转/右转/前进/后退` 补成默认演示动作。

建议：

```bash
SPEECH_END_SILENCE_S=0.85 ASR_COMMIT_DELAY_MS=500 bash scripts/acceptance_test.sh continuous-offline
```

如果 monitor 输出 `completed_missing_slot`，说明短命令补全已经生效。

### 5.3 一句话里多个命令没有顺序执行

当前连续控制链路增加了轻量 NLU 层。它会把一条 ASR final 解析为多个队列项：

```text
向右转，向前走一秒 -> turn -> move
向前走一秒再左转九十度 -> move -> turn
```

自动验收：

```bash
bash scripts/acceptance_test.sh continuous-multi-command
```

现场观察：

```bash
ros2 topic echo /agent/recognition_feedback
ros2 topic echo /agent/command_queue
ros2 topic echo /robot/action_result
```

通过时应看到 `nlu_parsed`、同一个 `batch_id` 下的多个 `enqueue`，以及与 `request_id` 对应的 `command_id` result。
如果现场出现新的 ASR 错词或多命令粘连，可用 `CONTINUOUS_SAMPLE_LOG=logs/asr_nlu_samples.jsonl`
保存事件流，演示后运行 `asr-nlu-samples-to-eval` 生成待审核候选集，再把确认过的失败样本
补进指令评估集。合入前也可以运行 `asr-nlu-candidate-eval` 先看候选集上的 parser accuracy。

### 5.4 ASR 有输出但动作没执行

依次观察：

```bash
ros2 topic echo /agent/session_state
ros2 topic echo /agent/command_queue
ros2 topic echo /agent/action_candidate
ros2 topic echo /robot/action_command_typed
ros2 topic echo /robot/action_result
ros2 topic echo /cmd_vel
```

判断方式：

- 卡在 session：检查唤醒词、会话超时、是否说了 `退出控制`。
- 卡在 queue：检查队列是否满、命令是否过期。
- 卡在 action candidate：检查 LLM/fallback parser 是否生成动作。
- 卡在 typed command：检查 ActionGuard 是否 active。
- 卡在 result/cmd_vel：检查 typed action bridge 和 simulation executor。

### 5.5 在线模式失败

检查 API Key：

```bash
grep DASHSCOPE_API_KEY .env
bash scripts/acceptance_test.sh online
```

在线真实语音受网络和云端服务波动影响，现场演示建议优先使用 `continuous-offline`，在线作为补充展示。

### 5.6 FastDDS SHM 端口锁报错

现象：

```text
RTPS_TRANSPORT_SHM Error ... Failed init_port fastrtps_port7000: open_and_lock_file failed
```

原因通常是 WSL 中 FastDDS shared-memory transport 的 `/dev/shm/fastrtps_port*`
锁文件或残留 ROS/Gazebo 进程冲突。当前项目的 `scripts/activate.sh` 会默认 source
`scripts/ros_dds_env.sh`，设置：

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

这会绕开 SHM transport，连续语音/Gazebo 本机演示仍可正常通过 DDS 通信。若要临时恢复
FastDDS SHM：

```bash
EMBODIED_ALLOW_FASTDDS_SHM=true bash scripts/acceptance_test.sh continuous-offline
```

如果恢复 SHM 后仍报错，先清理残留仿真进程：

```bash
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh
```

## 6. Release gate

完整自动门禁：

```bash
bash scripts/acceptance_test.sh all
```

说明：

- `all` 会运行构建、单测、mock、连续语音 smoke、typed action、仿真 smoke 等自动项。
- `all` 不包含 `continuous-offline` / `continuous-online`，因为它们需要人工真实说话。
- 如果本轮只修改文档和注释，可先跑轻量门禁；发布前再跑 `all`。

## 7. 当前完成度

已完成：

- 从 ASR final 到动作解析、动作校验、ROS 2 Action、Gazebo `/cmd_vel` 的闭环。
- 在线/离线 Agent 双链路入口。
- 连续语音 session、命令队列、急停抢占、会话休眠。
- Gazebo/TurtleBot3 仿真动作验收。
- 单元测试、集成 smoke、真实麦克风辅助验收。

边界：

- 实体硬件 UART/SPI 只保留 mock/协议预留，不作为当前验收结论。
- 离线模型训练流程不是当前交付重点。
- 复杂导航、地图、目标点规划不属于当前阶段。
