# ROS 2 具身智能语音 Agent

这是一个同时支持在线与离线推理的机器人语音交互、动作解析和硬件控制工程。

| 文档 | 内容 |
|---|---|
| [`docs/README.md`](docs/README.md) | 文档中心与推荐阅读顺序 |
| [`docs/KNOWLEDGE_NOTES.md`](docs/KNOWLEDGE_NOTES.md) | ROS 2、流式语音、LLM/TTS、并发、量化和硬件协议知识笔记 |
| [`docs/PROJECT_NOTES.md`](docs/PROJECT_NOTES.md) | 关键代码位置、设计与配置导航 |
| [`docs/TESTING_GUIDE.md`](docs/TESTING_GUIDE.md) | 单元测试文件、分层验收和实体设备检查清单 |
| [`docs/COMPLETION_REPORT.md`](docs/COMPLETION_REPORT.md) | 实测完成度、性能数据与尚未完成项 |

目标环境：Ubuntu 24.04 / ROS 2 Jazzy / C++17 / Python 3.12。工程按实时性和生态优势划分语言：音频前端、AEC/VAD、播放和动作安全使用 C++；云模型 SDK、流式文本协议、提示词和记忆使用 Python。两侧只通过 ROS 话题通信。

## 数据流

```text
麦克风 PCM16
  -> C++ 有界队列（PortAudio 回调不执行网络操作）
  -> C++ NLMS AEC
  -> C++ 能量 VAD + 0.4 s 静音断句
  -> Qwen3-ASR-Realtime
  -> 唤醒词门控
  -> Qwen 流式 LLM
  -> <speech>/<action> 增量解析
       |-> Qwen3-TTS-Realtime -> C++ 播放队列 + AEC 参考信号
       `-> 动作候选 -> C++ ActionGuard -> /robot/action_command
```

默认是 `mock` 模式，不需要 API Key、麦克风或扬声器，可以先验证 ROS 话题和动作链路。`online` 模式使用阿里云百炼 DashScope：

- ASR：`qwen3-asr-flash-realtime`，16 kHz PCM，通过本地 VAD 的 commit 实现 0.4 秒静音断句。
- LLM：OpenAI 兼容流式接口，默认 `qwen-plus`。
- TTS：`qwen3-tts-flash-realtime`，ServerCommit 双向流式合成，24 kHz PCM。

模型名、地域 URL、音色和所有阈值都在 `src/embodied_online_agent/config/online_agent.yaml` 中配置。

## 首次安装

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh
```

## 无密钥冒烟测试

推荐直接运行自动冒烟脚本。脚本会复用已经运行的 mock Agent；如果没有节点，则临时启动一个。随后发送测试指令并校验动作和延迟指标，脚本只会自动停止自己启动的节点：

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
bash scripts/smoke_test.sh
```

预期看到两条 `PASS`。如果需要手动观察消息，请使用三个终端；`ros2 topic echo` 是持续监听命令，在收到消息前保持等待是正常行为。

终端一（启动节点）：

```bash
source /home/ubuntu/embodied_agent_ws/scripts/activate.sh
ros2 launch embodied_online_agent demo.launch.py
```

终端二（监听动作；运行后保持等待）：

```bash
source /home/ubuntu/embodied_agent_ws/scripts/activate.sh
ros2 topic echo /robot/action_command
```

终端三（触发一轮 mock 对话）：

```bash
source /home/ubuntu/embodied_agent_ws/scripts/activate.sh
ros2 topic pub --once /agent/text_input std_msgs/msg/String "{data: '小智，向前走一秒'}"
```

终端三发布后，终端二应立即显示：

```text
data: '{"arguments":{"duration_s":1.0,"linear_x":0.2},"name":"move"}'
```

可观察话题：

- `/agent/asr_partial`、`/agent/asr_final`：识别结果。
- `/agent/response_delta`、`/agent/response_text`：流式增量和完整答复。
- `/agent/state`：`listening/thinking/speaking/error`。
- `/agent/metrics`：LLM 首 token、ASR 到首 token、TTS 首音频包延迟及目标是否达成。
- `/audio/clean_pcm`、`/audio/silence_timeout`：C++ 音频前端输出。
- `/agent/action_candidate`：Python 解析出的未校验动作候选。
- `/robot/action_command`：经过白名单和限幅后的 JSON 动作。
- `/robot/action_rejected`：C++ ActionGuard 拒绝动作的原因。
- `/robot/action_ack`：开发用硬件桩节点回执。

清空记忆：

```bash
ros2 topic pub --once /agent/clear_memory std_msgs/msg/Empty "{}"
```

## 在线模式

1. 在阿里云百炼创建 API Key。将 Key 和业务空间地址放入工作区根目录的 `.env`（参考 `.env.example`）；该文件已被 Git 忽略，`scripts/activate.sh` 会自动加载。
2. 业务空间专属地址分别配置为 `DASHSCOPE_BASE_URL` 和 `DASHSCOPE_WS_URL`，不要把 Key 写进 YAML 或提交到 Git。
3. WSL 设置中允许麦克风，使用 `pactl list short sources` 和 `pactl list short sinks` 确认 WSLg 音频设备。
4. 启动：

```bash
source /home/ubuntu/embodied_agent_ws/scripts/activate.sh
ros2 launch embodied_online_agent online_agent.launch.py \
  mode:=online microphone_enabled:=true speaker_enabled:=true
```

若 WSL 没有音频设备，先保持 `microphone_enabled:=false speaker_enabled:=false`，用 `/agent/text_input` 验证在线 LLM/TTS；ASR 必须在有可用输入设备后才能实测。

## 输出协议和动作安全

系统提示词要求模型输出：

```text
<speech>好的，向前走一秒。</speech>
<action>{"name":"move","arguments":{"linear_x":0.2,"duration_s":1.0}}</action>
```

Python 解析器会在 token 到达时增量取出 speech，按标点尽早送入 TTS。动作候选发送给 C++ `action_guard`；它严格检查每种动作的参数 schema，并对速度、角速度、持续时间和次数限幅后才发布 `/robot/action_command`。硬件层仍应保留急停、碰撞和电机限流等独立安全机制。

## 回声消除说明

当前 C++ 音频模块内置 NLMS AEC 基线：TTS PCM 进入播放线程前同时写入参考队列，麦克风帧进入 ASR 前执行自适应回声估计。真实机器人上的扬声器、麦克风距离和系统播放延迟不同，需要调整 `aec_taps/aec_step/aec_delay_ms`。量产环境建议在同一 interface 后替换为硬件 DSP 或 WebRTC AEC，并做双讲测试。

## 测试与性能

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

`<1 s` 和 `<300 ms` 是在线运行目标，不是静态代码能够保证的常量。节点逐轮发布真实测量：

- `llm_first_token_ms`：发起 LLM 请求到首 token。
- `asr_to_first_token_ms`：ASR 最终文本到 LLM 首 token。
- `tts_first_audio_ms`：首段文本送入 TTS 到首个 PCM 包。

建议在目标网络和机器人硬件上至少采集 100 轮 P50/P95；若不达标，优先检查地域 URL、连接复用、句子首标点、音频设备缓冲和网络 RTT。

## 目录

```text
src/embodied_agent_cpp/
  include/embodied_agent_cpp/
    audio_processing.hpp     # AEC、VAD、静音深模块 interface
    action_validator.hpp     # 动作安全深模块 interface
  src/
    audio_frontend_node.cpp  # PortAudio、工作队列、播放与 ROS seam
    action_guard_node.cpp    # 动作 schema、限幅与发布
    robot_action_stub_node.cpp
  test/                      # C++ GTest
src/embodied_online_agent/
  embodied_online_agent/
    providers/               # mock / Qwen ASR / LLM / Qwen TTS
    protocol.py              # 增量输出解析和 TTS 分块
    memory.py                # 有界持久化记忆
    online_agent_node.py     # Python 云 SDK 与对话编排
  config/                    # ROS 参数
  prompts/                   # 系统提示词
  launch/                    # 在线及 mock demo 启动文件
  test/                      # 单元测试
```

## 第二部分：端侧离线 Agent

离线链路复用 C++ 音频前端、AEC/VAD、0.4 秒静音断句及动作安全节点；新增 `embodied_offline_agent`，通过 sherpa-onnx 运行 ZipFormer ASR 和 VITS TTS，通过本机 `llama-server` 流式运行 Qwen3-0.6B Q8_0。LLM 输出按句进入容量为 2 的消息缓冲，TTS PCM 再进入容量为 2 的音频缓冲，模型推理和播放异步进行。

先验证不依赖模型的 ROS 全链路：

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
colcon build --symlink-install
bash scripts/smoke_test_offline.sh
```

安装原生运行时和模型（约需 2 GB 下载空间，编译 llama.cpp）：

```bash
bash scripts/setup_offline_runtime.sh
```

真实离线模式使用两个终端。终端一启动本地模型服务：

```bash
source scripts/activate.sh
bash scripts/start_llama_server.sh
```

终端二启动 ROS 链路；先用文字输入验证 LLM/TTS，确认 WSLg 音频设备后再打开麦克风和扬声器：

```bash
source scripts/activate.sh
ros2 launch embodied_offline_agent offline_agent.launch.py mode:=offline
ros2 topic pub --once /agent/text_input std_msgs/msg/String "{data: '小智，向前走一秒'}"

# 实机音频
ros2 launch embodied_offline_agent offline_agent.launch.py \
  mode:=offline microphone_enabled:=true speaker_enabled:=true
```

性能验证：

```bash
bash scripts/benchmark_offline.sh
ros2 topic echo /offline_agent/metrics
```

`benchmark_offline.sh` 分别输出 ASR finalization/实时率、TTS 实时率和 llama.cpp prompt/decode tokens/s；ROS 指标输出静音到 ASR final、LLM 首 token、静音到首音频和整轮耗时。`<0.6 s`、`8.6 tokens/s`、`<3.5 s` 与 `85%` 都是验收目标，只有在目标机器和独立评测集上实测通过后才能写成完成结果。

训练暂不执行。`training/` 包含机器人指令种子数据、LLaMA-Factory 数据注册和 Qwen3-0.6B LoRA 配置。Q8_0 通常将 FP16 权重压缩到约一半，不是四分之一；脚本会保留输入/输出文件大小供实际计算，不能同时把“Q8”与“压缩至 25%”当作天然成立的结论。
