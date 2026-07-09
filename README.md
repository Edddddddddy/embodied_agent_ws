# Embodied Voice Agent for ROS 2

[![ROS 2 CI](https://github.com/Edddddddddy/embodied_agent_ws/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/Edddddddddy/embodied_agent_ws/actions/workflows/ros2-ci.yml)

这是一个面向 ROS 2 / C++ 求职展示的具身智能语音交互项目。项目目标是打通：

```text
真实/模拟语音输入 → ASR → 在线/离线 Agent → 动作解析 → ActionGuard → ROS 2 Action → Gazebo/TurtleBot3 仿真控制
```

当前项目以 Gazebo 仿真为主要验收平台，真实 UART/SPI 硬件控制保留为 mock/预留接口，不宣称已完成真实硬件闭环。

## 当前能力

- 在线 Agent：接入 DashScope/Qwen 兼容链路，支持在线 ASR、LLM、TTS 和流式响应。
- 离线 Agent：预留 Sherpa-onnx ZipFormer ASR、llama.cpp、Sherpa-TTS/SummerTTS 链路，支持 mock 和真实模型验收入口。
- 连续语音控制：一次“小智”唤醒后，可连续说多条命令；命令排队执行，`停下/急停` 可抢占。
- 识别鲁棒性：支持唤醒词别名、轻量 NLU 多命令识别、模糊命令归一化、短命令补全、重复 ASR final 过滤、语气词过滤、会话超时。
- ROS 2 工程化：自定义 msg/action、C++ ActionGuard、typed action bridge、Lifecycle、BehaviorTree.CPP、pluginlib executor。
- 仿真动作：前进、后退、左转、右转、停止、原地转圈、绕圈、走正方形、演示动作序列。
- 语音导航：支持“去门口/前往书桌/回到起点”等语义目标点导航，以及“依次去门口、书桌、起点/开始巡航”等多目标点巡航命令。
- 用户记忆：支持 `/agent/speaker_identity` 声纹身份事件、按用户保存本地偏好/行为习惯，并在 Agent 推理前注入用户画像；声纹 sidecar 支持 mock 和 sherpa-onnx 接入 seam。
- 验收脚本：提供 mock、在线、离线、Gazebo、真实麦克风连续控制等多层验收入口。

说明：当前导航能力分三层验收：`navigation-demo` 用 mock/Gazebo executor 做可观测运动；
`nav2-bridge` 用 fake Nav2 action server 证明语义地点已经能转换为真实 Nav2
`NavigateToPose / FollowWaypoints` goal；`nav2-turtlebot3` 启动官方 Nav2 TurtleBot3
仿真做重型端到端验收。日常开发优先跑前两层，演示前再跑完整 Nav2。

## 系统链路

```mermaid
flowchart LR
  Mic["麦克风 / mock 文本"] --> Audio["C++ audio_frontend\nVAD / endpoint / clean_pcm"]
  Audio --> ASR["在线 Qwen ASR\n或离线 Sherpa ASR"]
  Audio -.-> Speaker["speaker_identity sidecar\nmock / sherpa-onnx"]
  Speaker -.-> Memory["UserMemoryStore\n用户画像 / 行为习惯"]
  ASR --> Gate["Wake / Session Gate\n去重、语气词过滤、超时"]
  Gate --> Queue["连续命令队列\n普通命令 FIFO\n急停抢占"]
  Queue --> Agent["在线/离线 Agent\n轻量 NLU + LLM fallback\n短命令补全"]
  Memory -.-> Agent
  Agent --> Parser["动作解析\nprimitive / navigation"]
  Parser --> Guard["C++ ActionGuard\n动作校验、限幅、强类型转换"]
  Guard --> Bridge["Typed Action Bridge\nRobotCommand → ROS 2 Action"]
  Bridge --> Sim["Simulation Executor\nBT + pluginlib + /cmd_vel"]
  Sim --> Gazebo["Gazebo / TurtleBot3"]
```

## 项目结构

```text
embodied_agent_ws/
├── src/
│   ├── embodied_agent_interfaces/   # RobotCommand.msg 与 ExecuteRobotCommand.action
│   ├── embodied_agent_cpp/          # C++ 音频前端、ActionGuard、Action bridge、硬件 mock
│   ├── embodied_online_agent/       # 在线 Agent、Qwen ASR/LLM/TTS、连续语音控制、用户记忆/声纹 sidecar
│   ├── embodied_offline_agent/      # 离线 Agent、Sherpa/llama.cpp/Sherpa-TTS/SummerTTS 适配
│   └── embodied_simulation/         # Gazebo/TurtleBot3 执行器、BehaviorTree、pluginlib
├── scripts/                         # 一键验收、连续语音、校准、smoke test
├── tests/                           # repository / integration 测试
├── docs/
│   ├── ARCHITECTURE_AND_KNOWLEDGE.md
│   ├── TESTING_AND_ACCEPTANCE.md
│   ├── PROJECT_PRESENTATION_15MIN.md
│   ├── INTERVIEW_QA.md
│   ├── LEARNING_NOTES.md
│   ├── PROJECT_GAPS_AND_OPTIMIZATION.md
│   ├── OFFLINE_BENCHMARK_REPORT.md
│   ├── OFFLINE_RUNTIME_VERSIONS.md
│   └── CHANGELOG_AND_ROADMAP.md
└── README.md
```

## 环境准备

项目默认运行在 WSL Ubuntu 24.04 + ROS 2 环境中。

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
colcon build --symlink-install
source install/setup.bash
```

如果是第一次部署，建议先运行 bootstrap：

```bash
bash scripts/bootstrap.sh
```

在线模式需要配置 DashScope API Key：

```bash
cp .env.example .env
# 编辑 .env，填入 DASHSCOPE_API_KEY
```

离线真实模型模式需要额外准备 Sherpa-onnx、llama.cpp server、Sherpa-TTS 或 SummerTTS 模型。无模型时仍可使用 mock/offline smoke 验证工程链路。

```bash
bash scripts/setup_offline_runtime.sh
```

`setup_offline_runtime.sh` 会准备 `third_party/llama.cpp`、`third_party/SummerTTS`
以及 `models/` 下的 Sherpa/Qwen 模型文件。SummerTTS 使用开源仓库
[huakunyang/SummerTTS](https://github.com/huakunyang/SummerTTS)，本项目默认仍用
Sherpa-TTS 作为稳定 fallback，需要时可通过 `tts_provider:=summer` 切换。
当前离线运行时固定版本见 [docs/OFFLINE_RUNTIME_VERSIONS.md](docs/OFFLINE_RUNTIME_VERSIONS.md)。

llama.cpp 推理层可以先单独验收，避免把 ASR、TTS、Gazebo 的问题混在一起排查：

```bash
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh pseudo-tts
bash scripts/acceptance_test.sh offline-runtime-versions
bash scripts/acceptance_test.sh offline-latency
```

`llama-cpp-preflight` 会检查 `llama-server` binary、Q8 GGUF 模型、`/health` 和 `/v1/models`；
`llama-cpp-smoke` 会额外发送一次低 token 流式 chat 请求；`pseudo-tts` 不依赖真实
Sherpa/SummerTTS 模型，用假 PCM 验证“LLM token 流 -> 短句切分 -> 伪流式 TTS 双缓冲 -> 音频块发布”的工程链路。
常用调参环境变量：

```bash
LLAMA_THREADS=8 LLAMA_CONTEXT=2048 bash scripts/start_llama_server.sh
LLAMA_EXTRA_ARGS="--parallel 1" bash scripts/acceptance_test.sh llama-cpp-smoke
```

SummerTTS 可以单独部署和验收：

```bash
bash scripts/setup_summer_tts_runtime.sh
bash scripts/acceptance_test.sh summer-tts-preflight
bash scripts/acceptance_test.sh summer-tts-smoke
bash scripts/acceptance_test.sh summer-pseudo-tts
bash scripts/acceptance_test.sh summer-tts-service
```

`summer-pseudo-tts` 会使用真实 SummerTTS C++ 二进制合成短文本，再通过项目的
`PseudoStreamingTtsPipeline` 分块发布，验证“开源 C++ TTS 后端 + 双缓冲伪流式”的嵌入链路。
`summer-tts-service` 会启动常驻 C++ ROS service，模型在节点启动时加载，后续请求通过
`/tts/synthesize` 合成，不再每句启动命令行进程；短文本反馈默认启用缓存，重复请求会在
probe 输出中显示 `cache_hit=true`。

使用常驻 SummerTTS ROS 后端：

```bash
ros2 launch embodied_offline_agent offline_agent.launch.py \
  mode:=offline \
  tts_provider:=summer_ros
```

低延迟验收：

```bash
bash scripts/acceptance_test.sh offline-latency
```

当前低延迟默认路径为 `llama.cpp + Sherpa-TTS`：验收要求 LLM 首 token ≤ 1000ms、
TTS 首音频 ≤ 300ms。SummerTTS 目前通过命令行二进制接入，每句会重新启动进程并加载模型，
适合展示 C++ 离线 TTS runtime，但不作为 `<300ms` 低延迟默认 TTS。`summer_ros`
已经把 SummerTTS 做成常驻 C++ ROS 组件，消除了进程/模型重复加载，并对“好的/收到/正在执行”
这类短文本做请求级缓存；未命中的长句瓶颈仍主要是 SummerTTS CPU infer 本身，后续若要继续冲
`<300ms`，需要模型量化、真正流式合成或更快声码器等进一步优化。

如果只想先部署和验证 Sherpa-ONNX ZipFormer ASR，可运行更轻量的 ASR-only 入口：

```bash
bash scripts/setup_sherpa_asr_runtime.sh
bash scripts/acceptance_test.sh sherpa-asr-smoke
```

## 用户声纹与本地记忆

当前版本把“声纹识别”和“用户记忆”解耦：

- `speaker_identity` sidecar 订阅 `/audio/clean_pcm`、`/audio/speech_ended`，发布 `/agent/speaker_identity`。
- 录入声纹时，Agent 发布 `/agent/speaker_enroll_request`，sidecar 收集后续 3 句语音为 wav 样本并维护 `speakers.txt`。
- 在线/离线 Agent 订阅 `/agent/speaker_identity`，把当前用户画像从 `~/.ros/embodied_agent/users/` 加载进 prompt。
- 支持语音/文本命令：“记住我，我是小李”“我喜欢慢一点”“我是谁”“清除我的记忆”。
- 个性化偏好只作为 Agent 上下文，动作仍必须经过 ActionGuard 限幅和 ROS 2 Action 执行。

mock 验收：

```bash
bash scripts/acceptance_test.sh speaker-memory-mock
bash scripts/acceptance_test.sh speaker-enroll
```

真实 sherpa-onnx 声纹接入需要准备 speaker embedding 模型和注册样本文件，然后启动时开启：

```bash
ros2 launch embodied_online_agent online_agent.launch.py \
  speaker_identity_enabled:=true \
  speaker_identity_mode:=sherpa \
  speaker_identity_sherpa_model:=/path/to/speaker_model.onnx \
  speaker_identity_sherpa_file:=/path/to/speakers.txt
```

`speakers.txt` 每行格式为：

```text
lcy /path/to/lcy_enroll.wav
```

## 五分钟跑通主链路

无密钥、无麦克风、无真实模型的主链路验收：

```bash
bash scripts/acceptance_test.sh mock
```

Gazebo 仿真链路：

```bash
bash scripts/acceptance_test.sh gazebo
```

语音目标点导航与多目标点巡航 mock 验收：

```bash
bash scripts/acceptance_test.sh navigation-demo
```

连续语音会话中的目标点导航/多目标点巡航队列验收：

```bash
bash scripts/acceptance_test.sh continuous-navigation
```

Nav2 action bridge 验收（无需完整地图，用 fake Nav2 action server）：

```bash
bash scripts/acceptance_test.sh nav2-bridge
```

Nav2/TurtleBot3 完整 bringup 前置检查：

```bash
bash scripts/acceptance_test.sh nav2-preflight
```

语音导航阶段门禁（推荐提交前跑；不启动重型 Gazebo/Nav2）：

```bash
bash scripts/acceptance_test.sh nav2-stage
```

Nav2/TurtleBot3 真实仿真重型验收（会启动 Gazebo/Nav2，耗时数分钟）：

```bash
bash scripts/acceptance_test.sh nav2-turtlebot3
```

该模式会先发布 AMCL `/initialpose`，并使用较长的 `nav_action_timeout_s` 等待真实
Nav2 action result，避免按普通短动作提前取消导航。

在线接口最小 token 验证：

```bash
bash scripts/acceptance_test.sh online
```

离线真实模型链路：

```bash
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh pseudo-tts
bash scripts/acceptance_test.sh offline
```

离线 Agent 的 `/offline_agent/metrics` 会包含 `llm_provider` 字段，用于查看 llama.cpp
首 token 延迟、token 数和 tokens/s；同时包含 `tts_pipeline` 字段，用于查看伪流式
TTS 的文本块数、合成调用次数、音频块数和首文本到首音频耗时。如果失败信息指向 `cannot connect to llama-server`，
先单独运行上面的 `llama-cpp-preflight/smoke`。

Sherpa-ONNX 语音模型参与的 typed Action 仿真控制闭环：

```bash
bash scripts/acceptance_test.sh offline-sherpa-typed
```

查看所有验收模式：

```bash
bash scripts/acceptance_test.sh --help
```

日常开发最小回归：

```bash
bash scripts/acceptance_test.sh core
```

`core` 只跑仓库结构检查、Python Agent 单元测试和 C++/仿真 GTest；`mock`
会进一步启动依赖 mock 的 ROS smoke 链路，更适合提交前验收。

## 真实麦克风连续语音控制

离线优先，适合现场演示：

```bash
bash scripts/acceptance_test.sh continuous-offline
```

在线模式：

```bash
bash scripts/acceptance_test.sh continuous-online
```

Nav2/TurtleBot3 目标点导航连续语音演示：

```bash
bash scripts/acceptance_test.sh continuous-nav2-offline
# 或
bash scripts/acceptance_test.sh continuous-nav2-online
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

辅助计分终端：

```bash
CONTINUOUS_LIVE_CHECK_DURATION=240 bash scripts/acceptance_test.sh continuous-nav2-live-check offline
```

更推荐的一终端留证方式：

```bash
CONTINUOUS_LIVE_CHECK_REPORT=logs/nav2-live-check.json \
  CONTINUOUS_LIVE_CHECK_DURATION=240 \
  bash scripts/acceptance_test.sh continuous-nav2-evidence offline
```

它会自动启动连续 Nav2 语音控制、运行现场计分、保存证据报告，并在结束时清理
Gazebo/Nav2/Agent 进程。两终端方式仍适合调试 topic 和日志。
正式占用麦克风和 Gazebo 前，也可以先 dry-run 检查参数：

```bash
CONTINUOUS_NAV2_EVIDENCE_DRY_RUN=true \
  CONTINUOUS_LIVE_CHECK_REPORT=logs/nav2-live-check.json \
  bash scripts/acceptance_test.sh continuous-nav2-evidence offline
```

该计分脚本会要求现场至少出现一次 `navigate_to` 和一次 `follow_waypoints`，
避免只看到普通动作 result 就误判为 Nav2 导航演示通过。
如需保存现场证据：

```bash
CONTINUOUS_LIVE_CHECK_REPORT=logs/nav2-live-check.json \
  CONTINUOUS_LIVE_CHECK_DURATION=240 \
  bash scripts/acceptance_test.sh continuous-nav2-live-check offline
```

保存后可离线复核这份证据，不需要重新启动仿真或麦克风：

```bash
bash scripts/acceptance_test.sh continuous-nav2-live-report logs/nav2-live-check.json
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

另开一个终端做人工验收统计：

```bash
bash scripts/acceptance_test.sh continuous-live-check offline
# 或
bash scripts/acceptance_test.sh continuous-live-check online
```

通过标准：

- 终端持续打印 `[session] / [asr] / [queue] / [exec] / [action] / [result]`。
- 一次唤醒后能连续识别多条命令。
- 机器人在 Gazebo 中连续执行动作。
- busy 时后续命令进入队列，而不是静默丢失。
- `停下/急停` 能抢占并清队列。
- `退出控制` 后 session 进入 sleeping。
- 最终 `/cmd_vel` 归零。

## 语音识别调参

连续语音默认偏向“完整优先”，减少“左转90度”只识别成“左转”的尾部漏识别：

- `VAD_PROVIDER`：默认 `auto`，启动前优先检测 `silero-vad + onnxruntime`；可用时使用 Silero
  sidecar 接管 endpoint，不可用时自动降级到 energy VAD 并打印原因。
- `SPEECH_END_SILENCE_S`：VAD 判定一句话结束前等待的静音时长。
- `ASR_COMMIT_DELAY_MS`：收到 `/audio/speech_ended` 后，Agent 再延迟提交 ASR final 的时间。
- `VOICE_CONTROL_PROFILE`：`normal`、`quiet`、`low_gain`、`noisy_room` 四种预设。
  - `low_gain` 用于 WSL/笔记本麦克风输入很低的场景，例如 `rms≈0.002`、`peak<300` 且 `speech=False`。

示例：

```bash
VOICE_CONTROL_PROFILE=noisy_room bash scripts/acceptance_test.sh continuous-offline
VOICE_CONTROL_PROFILE=low_gain bash scripts/acceptance_test.sh continuous-offline
VAD_PROVIDER=silero bash scripts/acceptance_test.sh continuous-offline
ASR_COMMIT_DELAY_MS=500 bash scripts/acceptance_test.sh continuous-offline
```

如果 ASR 仍然只输出“左转/前进”，命令补全层会按演示默认语义执行：

- `前进 / 向前 / 往前` → `前进一秒`
- `后退 / 向后 / 往后` → `后退一秒`
- `左转 / 向左转` → `左转九十度`
- `右转 / 向右转` → `右转九十度`

## 常用验收命令

```bash
# 仓库结构与 CLI 入口
pytest -q tests/repository
bash tests/integration/test_acceptance_cli.sh

# 连续语音 mock / endpoint / queue
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh continuous-queue-full
bash scripts/acceptance_test.sh voice-readiness

# 声纹身份事件 + 用户行为记忆 mock 验收
bash scripts/acceptance_test.sh speaker-memory-mock
bash scripts/acceptance_test.sh speaker-enroll

# 语音导航 / 多目标点巡航
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh nav2-bridge
bash scripts/acceptance_test.sh nav2-preflight
bash scripts/acceptance_test.sh nav2-turtlebot3
bash scripts/acceptance_test.sh continuous-nav2-offline

# Gazebo 语音到仿真运动
bash scripts/acceptance_test.sh gazebo-voice
bash scripts/acceptance_test.sh gazebo-voice-online
```

完整 release gate：

```bash
bash scripts/acceptance_test.sh all
```

`all` 不包含需要人工说话的 microphone/continuous interactive 模式。

求职展示版推荐 release gate 会运行一组更聚焦的本地验收，并输出统一报告：

```bash
bash scripts/acceptance_test.sh release-gate
# 默认报告：logs/acceptance_report.json
```

阶段版本发布前建议额外执行：

```bash
pytest -q tests/repository src/embodied_offline_agent/test
bash tests/integration/test_acceptance_cli.sh
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh instruction-parser-eval
bash scripts/acceptance_test.sh offline-latency
bash scripts/acceptance_test.sh summer-tts-service
colcon test --packages-select embodied_agent_cpp embodied_simulation --event-handlers console_direct+
colcon test-result --verbose
```

## 常见问题

### ASR 没输出

先检查麦克风和 VAD：

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
bash scripts/acceptance_test.sh voice-readiness
python scripts/audio_frontend_calibration.py --duration 6
```

如果 `wsl-microphone-preflight` 的 `rms` 接近 `0.0000`、`peak` 只有个位数，说明
Windows/WSLg 没有把真实麦克风音频送进 WSL。此时优先检查 Windows 隐私设置里的麦克风权限、
默认输入设备、WSLg 音频 source，而不是继续调低 VAD。

如果 `wsl-microphone-preflight` 已经 PASS，但 `continuous-offline` 里的 `[audio]`
仍然长期接近 `rms=0.0000 peak=1`，说明 PortAudio/ALSA 默认输入没有路由到 WSLg
PulseAudio。脚本会在检测到 `PULSE_SERVER` 和 `parecord` 时自动启用
`pulse_audio_capture_bridge.py`，终端应显示：

```text
PULSE_CAPTURE_BRIDGE=auto（active=true）
enhancer=pulse_bridge
```

如需手动控制：

```bash
PULSE_CAPTURE_BRIDGE=true bash scripts/acceptance_test.sh continuous-offline
PULSE_CAPTURE_BRIDGE=false bash scripts/acceptance_test.sh continuous-offline
```

如果环境噪声大，尝试：

```bash
VOICE_CONTROL_PROFILE=noisy_room bash scripts/acceptance_test.sh continuous-offline
```

如果能看到 `[audio] rms/peak` 打印，但始终 `speech=False`，并且数值类似
`rms=0.0023 peak=180`，说明麦克风输入增益太低，默认 VAD 阈值过高。优先尝试：

```bash
VOICE_CONTROL_PROFILE=low_gain bash scripts/acceptance_test.sh continuous-offline
```

或按 readiness/calibration 输出的建议手动套阈值：

```bash
SPEECH_START_THRESHOLD=0.0012 AEC_ENABLED=false bash scripts/acceptance_test.sh continuous-offline
```

### 识别到“左转/前进”但漏掉数字

优先调大：

```bash
SPEECH_END_SILENCE_S=0.85 ASR_COMMIT_DELAY_MS=500 bash scripts/acceptance_test.sh continuous-offline
```

同时观察 monitor 是否出现 `completed_missing_slot`，出现则说明短命令补全已生效。

### 正确 ASR 后没有动作

观察这些 topic：

```bash
ros2 topic echo /agent/action_candidate
ros2 topic echo /robot/action_command_typed
ros2 topic echo /robot/action_result
ros2 topic echo /cmd_vel
```

如果 `/agent/action_candidate` 有输出但 `/robot/action_result` 没有，重点检查 ActionGuard、typed action bridge 和 simulation executor 是否已启动。

### continuous-online 启动后 Gazebo 里没有小车

先清理上一次残留的 Gazebo/ROS 仿真进程，再跑仿真自检：

```bash
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh
bash scripts/acceptance_test.sh gazebo
```

`continuous-offline/online` 启动时会自动做 TurtleBot3 readiness check：必须看到
`/odom`、`/scan`、`/cmd_vel` 和 `/robot/execute_command`。如果你想让脚本启动前自动清理残留进程：

```bash
SIMULATION_CLEANUP_STALE=true bash scripts/acceptance_test.sh continuous-online
```

如果只想后台无界面演示，可以设置：

```bash
GUI_ENABLED=false bash scripts/acceptance_test.sh continuous-online
```

### FastDDS SHM 报 `fastrtps_port7000`

如果看到类似：

```text
RTPS_TRANSPORT_SHM Error ... Failed init_port fastrtps_port7000: open_and_lock_file failed
```

这是 WSL 中 FastDDS 共享内存传输的常见锁文件/残留进程问题。项目默认在
`scripts/activate.sh` 中设置：

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

用于禁用 FastDDS SHM，真实麦克风/Gazebo 演示仍在本机 UDP 通信下运行。如果你确实要调试
FastDDS SHM，可临时恢复：

```bash
EMBODIED_ALLOW_FASTDDS_SHM=true bash scripts/acceptance_test.sh continuous-offline
```

### 在线模式失败

检查 `.env`：

```bash
grep DASHSCOPE_API_KEY .env
bash scripts/acceptance_test.sh online
```

在线真实麦克风受网络、云端 ASR 波动影响更大；现场演示优先使用 `continuous-offline`。

## 文档索引

- [架构与模块说明](docs/ARCHITECTURE_AND_KNOWLEDGE.md)
- [测试与验收手册](docs/TESTING_AND_ACCEPTANCE.md)
- [15 分钟汇报与代码走读稿](docs/PROJECT_PRESENTATION_15MIN.md)
- [面试问答：ROS 2 / C++ 项目追问](docs/INTERVIEW_QA.md)
- [项目不足与优化路线](docs/PROJECT_GAPS_AND_OPTIMIZATION.md)
- [Nav2 语音导航/巡航验收审计](docs/NAV2_VOICE_ACCEPTANCE_AUDIT.md)
- [学习笔记：关键技术点与设计取舍](docs/LEARNING_NOTES.md)
- [离线模型 Benchmark 报告模板](docs/OFFLINE_BENCHMARK_REPORT.md)
- [版本记录与路线图](docs/CHANGELOG_AND_ROADMAP.md)
- [Codex WSL + PowerShell 开发 Skill](docs/CODEX_WSL_POWERSHELL_SKILL.md)

## 当前边界

- 当前验收平台是 Gazebo/TurtleBot3 仿真，不是实体机器人。
- 离线 LoRA 训练数据集和真实训练流程有接口与说明，训练本身不是当前主线交付内容。
- 连续语音默认 `VAD_PROVIDER=auto`：Silero VAD 可用时优先使用成熟声学 VAD，不可用时降级 energy VAD；
  openWakeWord、LiveKit WakeWord、Sherpa KWS 仍是可选 seam/preflight/smoke，不是默认强依赖。
- 复杂导航、地图构建、目标点规划不是本阶段目标；当前重点是语音到动作到仿真控制的端到端链路。
