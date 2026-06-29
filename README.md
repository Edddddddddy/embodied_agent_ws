# ROS 2 端侧在线流式 Agent

目标环境：Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12。工程把语音前端、实时 ASR、流式 LLM、实时 TTS、记忆和动作发布拆成可替换模块。

## 数据流

```text
麦克风 PCM16
  -> NLMS AEC
  -> 能量 VAD + 0.4 s 静音断句
  -> Qwen3-ASR-Realtime
  -> 唤醒词门控
  -> Qwen 流式 LLM
  -> <speech>/<action> 增量解析
       |-> Qwen3-TTS-Realtime -> 扬声器 + AEC 参考信号
       `-> /robot/action_command -> ROS 硬件控制节点
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

终端一：

```bash
source /home/ubuntu/embodied_agent_ws/scripts/activate.sh
ros2 launch embodied_online_agent demo.launch.py
```

终端二：

```bash
source /home/ubuntu/embodied_agent_ws/scripts/activate.sh
ros2 topic echo /robot/action_command
ros2 topic pub --once /agent/text_input std_msgs/msg/String "{data: '小智，向前走一秒'}"
```

可观察话题：

- `/agent/asr_partial`、`/agent/asr_final`：识别结果。
- `/agent/response_delta`、`/agent/response_text`：流式增量和完整答复。
- `/agent/state`：`listening/thinking/speaking/error`。
- `/agent/metrics`：LLM 首 token、ASR 到首 token、TTS 首音频包延迟及目标是否达成。
- `/robot/action_command`：经过白名单和限幅后的 JSON 动作。
- `/robot/action_ack`：开发用硬件桩节点回执。

清空记忆：

```bash
ros2 topic pub --once /agent/clear_memory std_msgs/msg/Empty "{}"
```

## 在线模式

1. 在阿里云百炼创建 API Key。不要把 Key 写进 YAML 或提交到 Git。
2. 如果使用业务空间专属域名，将 ASR/TTS URL 替换成对应地域的 WebSocket URL。
3. WSL 设置中允许麦克风，确认 `python -m pyaudio` 能看到输入/输出设备。
4. 修改配置：

```yaml
mode: online
microphone_enabled: true
speaker_enabled: true
```

5. 启动：

```bash
source /home/ubuntu/embodied_agent_ws/scripts/activate.sh
export DASHSCOPE_API_KEY='你的密钥'
ros2 launch embodied_online_agent online_agent.launch.py mode:=online
```

如需覆盖输入设备索引，设置 `input_device_index`。若 WSL 没有音频设备，先保持麦克风关闭，用 `/agent/text_input` 验证在线 LLM/TTS；ASR 必须在有可用输入设备后才能实测。

## 输出协议和动作安全

系统提示词要求模型输出：

```text
<speech>好的，向前走一秒。</speech>
<action>{"name":"move","arguments":{"linear_x":0.2,"duration_s":1.0}}</action>
```

解析器会在 token 到达时增量取出 speech，按标点尽早送入 TTS。动作仅允许 `stop/move/turn/wave/set_led`，速度、角速度、持续时间和次数会在发布前限幅。硬件层仍应保留急停、碰撞和电机限流等独立安全机制。

## 回声消除说明

当前自带依赖为零的 NLMS AEC 基线：TTS PCM 同时送往扬声器与参考队列，麦克风帧进入 ASR 前执行自适应回声估计。真实机器人上的扬声器、麦克风距离和系统播放延迟不同，需要调整 `aec_taps/aec_step`。量产环境建议在同一接口后替换为硬件 DSP 或 WebRTC AEC，并做双讲测试。

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
src/embodied_online_agent/
  embodied_online_agent/
    audio.py                 # AEC、VAD、静音、麦克风和播放
    providers/               # mock / Qwen ASR / LLM / Qwen TTS
    protocol.py              # 增量输出解析和 TTS 分块
    memory.py                # 有界持久化记忆
    actions.py               # 动作白名单和限幅
    online_agent_node.py     # ROS 2 编排节点
    robot_action_stub.py     # 硬件控制开发桩
  config/                    # ROS 参数
  prompts/                   # 系统提示词
  launch/                    # 在线及 mock demo 启动文件
  test/                      # 单元测试
```

