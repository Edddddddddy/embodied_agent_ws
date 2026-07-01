# 测试与验收指南

本项目把测试分成四层：纯单元测试、ROS mock 冒烟、真实 provider 集成测试、实体硬件验收。越靠后越接近真实使用，也越受网络、模型、音频设备和外设影响。

## 1. 一键入口

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh

# 构建、全部单元测试和三个 mock 链路
bash scripts/acceptance_test.sh mock

# 极少量云 API 调用 + 在线 ROS 动作闭环
bash scripts/acceptance_test.sh online

# 本地模型基准、指令评测、文字与合成语音闭环
bash scripts/acceptance_test.sh offline

# 依次执行以上全部内容
bash scripts/acceptance_test.sh all
```

`online` 需要 `.env` 中的 DashScope 配置；`offline` 需要先运行 `scripts/setup_offline_runtime.sh`。日常开发优先运行单文件测试和 `mock`，发布前再运行 `all`，以减少 API 消耗和等待时间。

## 2. 单元测试文件清单

### C++：音频、动作与硬件

| 测试文件 | 覆盖功能 | 关键断言 |
|---|---|---|
| `src/embodied_agent_cpp/test/test_audio_processing.cpp` | Energy VAD、400 ms 静音检测、NLMS AEC | 语音/静音可区分；超时只触发一次；无参考时保真；重复回声可收敛 |
| `src/embodied_agent_cpp/test/test_action_validator.cpp` | 动作 schema、白名单和限幅 | move 被安全限幅；未知动作、缺参、多参、错误颜色被拒绝 |
| `src/embodied_agent_cpp/test/test_hardware_protocol.cpp` | 二进制帧、CRC、序号、watchdog | move/LED 编码稳定；非法命令拒绝；watchdog 仅过期一次 |
| `src/embodied_agent_cpp/test/test_hardware_transport.cpp` | UART Linux 系统调用 | PTY 收到逐字节一致帧；不支持的波特率被拒绝 |

单独运行：

```bash
colcon build --packages-select embodied_agent_cpp
colcon test --packages-select embodied_agent_cpp --event-handlers console_direct+

# 构建完成后也可直接运行某个 GTest
./build/embodied_agent_cpp/test_audio_processing
./build/embodied_agent_cpp/test_action_validator
./build/embodied_agent_cpp/test_hardware_protocol
./build/embodied_agent_cpp/test_hardware_transport
```

### Python：在线编排公共模块

| 测试文件 | 覆盖功能 | 关键断言 |
|---|---|---|
| `src/embodied_online_agent/test/test_protocol.py` | `<speech>/<action>` 增量协议、TTS 断句 | 跨 token 标签可解析；坏 JSON 不崩溃；标点/长度正确 flush |
| `src/embodied_online_agent/test/test_memory.py` | 有界记忆、持久化 | 历史裁剪正确；落盘后可恢复 |
| `src/embodied_online_agent/test/test_wakeword.py` | 唤醒词门控 | 唤醒后窗口内放行；未唤醒或过期时阻断 |
| `src/embodied_online_agent/test/test_command_fallback.py` | 明确指令仲裁、危险语义拦截 | 前进/后退/转向/挥手可确定解析；疑问、否定、复合高速旋转被阻断 |

运行：

```bash
python -m pytest -q src/embodied_online_agent/test
python -m pytest -q src/embodied_online_agent/test/test_protocol.py
```

### Python：离线并发与指标

| 测试文件 | 覆盖功能 | 关键断言 |
|---|---|---|
| `src/embodied_offline_agent/test/test_double_buffer.py` | 容量 2、背压、close/abort | 丢旧策略保留最新两项；无损 close 会排空；abort 唤醒消费者 |
| `src/embodied_offline_agent/test/test_latency.py` | 离线计时器 | ASR、首 token、首音频和整轮指标口径稳定 |

运行：

```bash
python -m pytest -q src/embodied_offline_agent/test
```

最新完整验收中，`colcon test-result --verbose` 汇总为 33 条测试记录、0 error、0 failure。测试记录数包含 CTest/GTest 的结果层级；按测试函数统计为 15 个 C++ GTest case 和 14 个 Python case。

## 3. ROS mock 冒烟测试

| 脚本 | 验证链路 | 外部依赖 |
|---|---|---|
| `scripts/smoke_test.sh` | mock 在线 Agent -> 动作命令 -> 延迟指标 | 无 API、无模型、无音频设备 |
| `scripts/smoke_test_offline.sh` | mock 离线 Agent -> 动作命令 | 无模型、无音频设备 |
| `scripts/smoke_test_hardware.sh` | ActionGuard -> HardwareController -> mock transport -> ACK/watchdog stop | 无实体串口/SPI |

这些测试使用真实 ROS 进程和 topic，能发现节点未启动、话题名错误、QoS 或 launch 配置问题。`ros2 topic echo` 本身是持续监听命令，等待消息不是卡死。

## 4. 真实 provider 与端到端测试

| 脚本 | 验证内容 | 通过条件 |
|---|---|---|
| `scripts/test_online_api.py` | 最小文本的云 LLM、TTS 音频、TTS 回灌 ASR | 三个 provider 都返回有效数据并输出延迟 |
| `scripts/smoke_test_online_real.sh` | 在线 LLM -> TTS -> ROS 动作 -> C++ Guard -> 硬件 mock | 收到动作 ACK 和 metrics |
| `scripts/benchmark_offline.sh` | ZipFormer、Melo-TTS、llama.cpp 单模块性能 | 输出 ASR/TTS RTF 与 token/s |
| `scripts/evaluate_instruction_following.sh` | 固定 seed 指令集 | 分开报告模型动作和语义仲裁后的成绩 |
| `scripts/smoke_test_offline_real.sh` | llama.cpp -> Sherpa-TTS -> 动作 -> 硬件 mock | 收到动作 ACK、音频和 metrics |
| `scripts/smoke_test_offline_voice_real.sh` | 合成语音 -> ZipFormer -> llama.cpp -> TTS -> 动作 -> 硬件 mock | ASR 含动作主体、move ACK、双缓冲无丢帧 |

真实语音测试关闭唤醒门控，是为了隔离验证 ASR 到硬件的模型链路；唤醒率需在真实麦克风测试中单独统计。云端测试默认不强制性能阈值，避免网络抖动把“功能失败”和“性能越界”混为一谈；使用 `test_online_api.py --enforce-targets` 才会把阈值越界作为退出失败。

## 5. 实体设备验收清单

自动测试无法替代以下项目：

- 麦克风：安静、噪声、电机运行、远近说话下的识别率。
- 唤醒：至少统计命中率、漏唤醒率、每小时误唤醒次数。
- AEC：不同音量和距离下的 ERLE、双讲、残留回声和额外延迟。
- 延迟：固定硬件连续至少 100 轮，分别报告冷启动、稳态 P50/P95。
- UART/SPI：先断开电机功率，用逻辑分析仪核对电平、时钟、帧、CRC 和重试。
- 安全：断网、模型超时、进程退出、串口断开、watchdog、急停和越界命令。
- 动作：低速架空轮测试后再落地，验证正负方向、持续时间和停止距离。

## 6. 新增功能时如何补测试

1. 算法或协议先写纯单元测试，不启动 ROS，不访问网络。
2. 节点连接关系使用 mock 冒烟脚本测试，并设置明确超时，避免永久等待。
3. provider 集成测试必须读取环境变量，禁止把 API Key 写进源码、日志或 fixture。
4. 硬件协议同时测试合法帧、边界值、畸形输入和故障停止路径。
5. 性能测试保存环境、模型、线程数和原始值，不只写一个“达标”布尔值。
6. 更新本文件的测试清单，并运行 `bash scripts/acceptance_test.sh mock`；发布前运行 `all`。

## 7. 常见问题

### `ros2 topic echo` 一直没有输出

它只负责订阅，需要另一个终端发布输入。先用 `ros2 topic info /robot/action_command -v` 检查 publisher，再执行对应 smoke 脚本。

### `colcon test-result` 出现第三方库测试

`third_party/COLCON_IGNORE` 应存在；`setup_offline_runtime.sh` 会自动创建它。验收脚本也通过 `--packages-select` 只测试三个 ROS 包。

### 单测通过但真实链路失败

依次检查 `.env`、模型文件、llama-server、ROS topic、音频采样率、WSLg 设备和实体外设权限。单元测试证明局部逻辑，不能证明外部服务或硬件可用。
