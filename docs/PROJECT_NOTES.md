# 具身智能机器人端侧交互与控制系统：代码与设计笔记

## 1. 总体边界

系统把实时、硬件敏感部分放在 C++，把模型 SDK 和对话编排放在 Python。各模块只通过 ROS 2 话题连接：

```text
麦克风 -> C++ AEC/VAD/静音断句 -> 在线或离线 ASR
       -> 唤醒词/记忆 -> 流式 LLM -> <speech>/<action> 解析
       -> TTS -> C++ 播放
       -> ActionGuard -> HardwareController -> UART/SPI -> MCU/外设
```

关键约束：模型输出不能直接碰硬件。动作必须先经过 C++ `ActionGuard` 的 schema、白名单和数值限幅，再由硬件控制节点编码发送。

## 2. 第一部分：在线流式 Agent

### C++ 音频前端

- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`：PortAudio 采集/播放、ROS 音频话题和有界工作队列。
- `src/embodied_agent_cpp/include/embodied_agent_cpp/audio_processing.hpp`：AEC、VAD、静音检测的稳定接口。
- `src/embodied_agent_cpp/src/audio_processing.cpp`：NLMS 回声消除、能量 VAD、0.4 秒静音触发实现。
- `src/embodied_agent_cpp/test/test_audio_processing.cpp`：静音阈值、无参考信号、回声收敛测试。

音频回调不执行网络请求或模型推理，只做轻量处理和入队，防止采集线程被阻塞。TTS 播放数据同时作为 AEC 参考信号。

### 在线模型与对话编排

- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`：ASR、LLM、TTS 全流式编排和 ROS 话题入口。
- `src/embodied_online_agent/embodied_online_agent/providers/`：Qwen 实时 ASR/TTS、OpenAI 兼容 LLM 和 mock provider。
- `src/embodied_online_agent/embodied_online_agent/protocol.py`：增量解析 `<speech>` 与 `<action>`，并按标点或最大字符数尽早切 TTS 文本。
- `src/embodied_online_agent/embodied_online_agent/command_fallback.py`：明确短命令的确定性降级和否定/疑问/危险组合拦截。
- `src/embodied_online_agent/embodied_online_agent/wakeword.py`：唤醒词激活窗口。
- `src/embodied_online_agent/embodied_online_agent/memory.py`：有界对话历史和原子落盘。
- `src/embodied_online_agent/prompts/system_prompt_zh.txt`：输出格式和动作约束。
- `src/embodied_online_agent/embodied_online_agent/metrics.py`：首 token、ASR 到首 token、TTS 首音频延迟。

LLM token 到达后立即解析；完整句子不必等待整段生成结束就可进入 TTS。动作 JSON 和可朗读文本走不同分支。

在线节点在发布 `listening` 前执行一次最小 LLM 预热并建立持久 TTS WebSocket。TTS 每轮使用 `commit()`，而不是会关闭连接的 `finish()`。动作标签不会边生成边执行：节点先收集整轮动作，明确命令由确定性解析覆盖，危险或否定语义禁止模型动作，然后才交给 C++ Guard。

## 3. 第二部分：离线模型 Agent

### 模型适配

- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py`：sherpa-onnx ZipFormer streaming transducer，接收 PCM16 并持续发布 partial；收到静音事件后 `input_finished` 并产出 final。
- `src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py`：连接本机 `llama-server /v1/chat/completions`，使用 Qwen3 非思考模式减少首 token 与无关输出。
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_tts.py`：Sherpa VITS 句级合成，句间构成伪流式输出。
- `src/embodied_offline_agent/embodied_offline_agent/providers/mock.py`：无模型测试替身。

### 并发与延迟

- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`：离线 ROS 节点和三段 worker 流水线。
- `src/embodied_offline_agent/embodied_offline_agent/double_buffer.py`：容量固定为 2 的消息/音频双缓冲，支持阻塞背压、统计和异常唤醒。
- `src/embodied_offline_agent/embodied_offline_agent/latency.py`：ASR finalization、LLM 首 token、静音到首音频、整轮耗时。

ROS 音频回调只向 ASR 队列写数据。LLM worker 产出句子，TTS worker 合成 PCM，audio worker 发布播放数据，三者异步并行。正常结束会排空缓冲；worker 异常会 `abort()` 唤醒消费者，避免线程永久等待。

### 模型、量化和训练占位

- `scripts/setup_offline_runtime.sh`：从官方 Hugging Face 仓库并行、断点下载 chunk-32 ZipFormer、FP32/int8 VITS、Qwen3 Q8_0，并编译 llama.cpp。
- `scripts/start_llama_server.sh`：启动本机模型服务。
- `scripts/benchmark_offline.py`、`scripts/benchmark_offline.sh`：ASR/TTS 实时率和 llama.cpp tokens/s。
- `scripts/quantize_qwen_q8.sh`：把 F16 GGUF 量化为 Q8_0，并显示前后文件大小。
- `training/robot_dialogue_seed.jsonl`：机器人动作、拒绝危险动作和闲聊种子数据。
- `training/qwen3_0_6b_lora.yaml`：LLaMA-Factory LoRA SFT 配置占位。

Q8_0 相对 FP16 通常约缩小一半，不能天然声称“压缩至 25%”。8.6 tokens/s、85% 指令遵循率与 3.5 秒端到端延迟必须在目标硬件和独立测试集上测量。

当前固定 seed 的 8 条种子集上，未经 LoRA 的模型动作准确率为 25%；语义仲裁后工程链路为 100%。后者仅证明这 8 条显式命令可用，不代表模型达到 85%。详细实测见 `docs/COMPLETION_REPORT.md`。

## 4. 第三部分：动作与硬件控制

### 动作解析与安全

- `src/embodied_agent_cpp/src/action_guard_node.cpp`：订阅 `/agent/action_candidate`，校验后发布 `/robot/action_command`。
- `src/embodied_agent_cpp/src/action_validator.cpp`：支持 `stop/move/turn/wave/set_led`；检查精确参数集合、类型、速度/持续时间范围和 LED 颜色白名单。
- `src/embodied_agent_cpp/test/test_action_validator.cpp`：非法动作、缺失/多余参数、限幅与颜色测试。

### MCU 协议和驱动

- `src/embodied_agent_cpp/include/embodied_agent_cpp/hardware_protocol.hpp`、`src/embodied_agent_cpp/src/hardware_protocol.cpp`：把动作 JSON 编码为二进制帧并计算 CRC16-CCITT。
- `src/embodied_agent_cpp/include/embodied_agent_cpp/hardware_transport.hpp`、`src/embodied_agent_cpp/src/hardware_transport.cpp`：统一 `mock/UART/SPI` 接口；UART 使用 Linux termios 8N1，SPI 使用 spidev mode 0。
- `src/embodied_agent_cpp/src/hardware_controller_node.cpp`：订阅可信动作、发送帧、发布 ACK/状态、处理急停和运动超时。
- `src/embodied_agent_cpp/test/test_hardware_protocol.cpp`：帧字段、整数缩放、CRC、序号和 watchdog 测试。
- `src/embodied_agent_cpp/test/test_hardware_transport.cpp`：用 Linux 伪终端验证真实 termios UART 配置和逐字节发送。

帧格式为：

```text
AA 55 | version | opcode | payload_len | sequence(u16 LE) | payload | CRC16(u16 LE)
```

`move` 将 m/s 乘 1000 编码为 `int16`，`turn` 将 rad/s 乘 1000 编码为 `int16`，持续时间以毫秒 `uint16` 表示。节点收到运动指令后启动 watchdog，持续时间到达自动发送 `STOP`。`/robot/emergency_stop` 可随时绕过对话流程直接发送停止帧。

硬件节点会对 `/robot/action_command` 再做一次完整校验，避免其他 ROS 发布者绕过 Guard。停止帧写入失败时每 100 ms 重试。ACK 表示帧已成功写入操作系统驱动或 mock transport，不等于电机已完成动作。真实 MCU 若需完成态，应在协议中增加 ACK/完成帧并实现 UART 读取或 SPI 状态轮询。

## 5. 关键话题

| 话题 | 方向 | 作用 |
|---|---|---|
| `/audio/clean_pcm` | C++ -> ASR | AEC 后的 PCM16 |
| `/audio/silence_timeout` | C++ -> ASR | 0.4 秒静音断句 |
| `/agent/action_candidate` | Agent -> Guard | 模型动作候选，尚未可信 |
| `/robot/action_command` | Guard -> Hardware | 已校验、已限幅动作 |
| `/robot/emergency_stop` | 任意安全源 -> Hardware | 无条件急停 |
| `/robot/action_ack` | Hardware/Simulation -> 上层 | 执行后端、动作、序号和接受状态回执 |
| `/robot/hardware_status` | Hardware -> 监控 | ready/rejected/io_error |

## 6. 运行与验证

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

无硬件冒烟：

```bash
bash scripts/smoke_test_hardware.sh
```

统一分层验收入口：

```bash
bash scripts/acceptance_test.sh mock
bash scripts/acceptance_test.sh online
bash scripts/acceptance_test.sh offline
bash scripts/acceptance_test.sh all
```

其中 `test_online_api.py` 验证真实云 LLM/TTS/ASR；`smoke_test_offline_real.sh` 验证真实本地 LLM/TTS/动作；`smoke_test_offline_voice_real.sh` 把本地合成语音送回 ZipFormer，验证 ASR 到硬件动作的完整链路；`evaluate_instruction_following.sh` 分开报告模型准确率和带安全仲裁的有效准确率。

单独启动硬件链路：

```bash
# mock
ros2 launch embodied_agent_cpp hardware_control.launch.py backend:=mock

# UART
ros2 launch embodied_agent_cpp hardware_control.launch.py \
  backend:=uart uart_device:=/dev/ttyUSB0 uart_baud_rate:=115200

# SPI
ros2 launch embodied_agent_cpp hardware_control.launch.py \
  backend:=spi spi_device:=/dev/spidev0.0 spi_speed_hz:=1000000
```

完整离线 Agent 默认使用 mock 硬件；接入串口时：

```bash
ros2 launch embodied_offline_agent offline_agent.launch.py \
  mode:=offline hardware_backend:=uart
```

使用 UART/SPI 前必须确认设备节点存在且当前用户有访问权限。电机驱动层仍应独立实现物理急停、碰撞保护、限流和通信失联停车，不能只依赖 Agent 软件。

## 7. 第四部分：TurtleBot3 仿真控制

- `src/embodied_simulation/include/embodied_simulation/simulation_controller.hpp`：与 ROS 解耦的控制器接口和配置。
- `src/embodied_simulation/src/simulation_controller.cpp`：手动定时命令、速度平滑、避障、沿墙 PID、雷达超时和急停优先级。
- `src/embodied_simulation/src/simulation_control_node.cpp`：订阅可信动作与 `/scan`，发布 `/cmd_vel`、`/robot/action_ack`、模式和安全状态。
- `src/embodied_simulation/config/turtlebot3_bridge.yaml`：使用标准 `geometry_msgs/Twist` 桥接 Gazebo DiffDrive。
- `src/embodied_simulation/launch/voice_turtlebot3.launch.py`：Gazebo Harmonic、TurtleBot3、bridge、在线/离线 Agent 和可选 RViz 总入口。
- `src/embodied_simulation/test/test_simulation_controller.cpp`：手动运动、避障、沿墙和安全逻辑单测。
- `scripts/smoke_test_simulation.sh`：无 Gazebo 的确定性 ROS 链路验收。
- `scripts/smoke_test_gazebo.sh`：真实雷达/里程计与物理位移验收。
- `scripts/smoke_test_gazebo_voice.sh`：ZipFormer、llama.cpp、动作 Guard 到 Gazebo 的真实语音闭环。
- `scripts/run_voice_simulation.sh`：面向使用者的一条命令在线/离线语音仿真入口。

语音增加 `set_mode` 动作，允许 `manual/obstacle_avoidance/wall_following`。模式动作与 move/turn 一样先经过 C++ schema 白名单；仿真 launch 关闭 UART/SPI hardware controller，只启动 simulation controller。完整使用说明见 `docs/SIMULATION_GUIDE.md`。
