# 测试与验收

本文是唯一的测试入口。原则是先验证纯逻辑，再验证 ROS 通信，最后才消耗云 API、加载
本地模型或启动 Gazebo。

## 1. 一键入口

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh

bash scripts/acceptance_test.sh mock
bash scripts/acceptance_test.sh online
bash scripts/acceptance_test.sh offline
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh gazebo-voice
```

`mock` 是每次提交前的最低门槛；当前记录为 49 项测试、0 failure。`online` 使用少量
DashScope token；`offline` 会启动 llama-server；Gazebo 模式用 ACK 与里程计位移验真。

## 2. 分层测试矩阵

| 层级 | 主要文件/脚本 | 证明什么 |
|---|---|---|
| C++ 单元 | `embodied_agent_cpp/test/` | AEC/VAD、动作 schema、CRC、UART PTY |
| Python 单元 | `embodied_online_agent/test/` | 流式解析、记忆、唤醒、重试、fallback |
| 离线单元 | `embodied_offline_agent/test/` | 双缓冲、指标、ZipFormer 热词参数 |
| 仿真单元 | `embodied_simulation/test/` | 速度限制、雷达停车、避障、沿墙 PID |
| ROS mock | `smoke_test*.sh` | 话题发现、动作发布、ACK、watchdog |
| provider | `smoke_test_online_real.sh`、`benchmark_offline.sh` | 云/本地模型可用和真实延迟 |
| 物理仿真 | `smoke_test_gazebo*.sh` | Gazebo 执行动作并产生合理 `/odom` |
| 人工声学 | `accept_voice_simulation_microphone.sh` | WSLg 麦克风、真实人声和重试体验 |

直接运行测试：

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose

pytest -q src/embodied_online_agent/test src/embodied_offline_agent/test
bash scripts/smoke_test_recognition_retry.sh
```

## 3. 真实麦克风验收

先确认设备：

```bash
pactl list short sources
pactl list short sinks
```

核心链默认关闭唤醒词与扬声器，减少两个声学变量：

```bash
bash scripts/accept_voice_simulation_microphone.sh offline
bash scripts/accept_voice_simulation_microphone.sh online
```

系统依次要求：ASR final、action candidate、guarded command、simulation ACK、非零
`/cmd_vel` 和合理 `/odom` 位移。只看到识别文本不算通过。

单独验收唤醒词：

```bash
WAKE_WORD_ENABLED=true SPEAKER_ENABLED=false \
  bash scripts/accept_voice_simulation_microphone.sh offline
```

若门控未通过，`/agent/recognition_feedback` 会给出原因和次数，节点保持监听。热词表位于
`src/embodied_offline_agent/config/hotwords_zh.txt`。合成短词仍可能把“小智”识别成
“早日”，因此正式产品需要独立 KWS 并统计 false accept / false reject。

## 4. 当前实测基线

环境：WSL Ubuntu 24.04、ROS 2 Jazzy、16 vCPU、约 8 GB RAM；日期 2026-07-01。

| 指标 | 当前样本 | 结论 |
|---|---:|---|
| 在线 LLM 冷启动首 token | 1.63–2.34 s | 不达 1 s，必须预热 |
| 在线 LLM 热启动首 token | 460–537 ms | 当前样本达标 |
| 在线 TTS 首音频 | 231–301 ms | 临界，需要 P95 |
| 在线 ASR commit-to-final | 147–173 ms | 当前样本达标 |
| ZipFormer RTF | 0.0502–0.0525 | 远快于实时 |
| llama.cpp CPU decode | 20.14–30.47 token/s | 超过 8.6 目标 |
| Melo-TTS RTF | 0.53–0.60 | 快于实时 |
| 离线首音频 | 2.11–2.84 s | 当前样本达标 |
| 离线整轮 | 2.62–3.43 s | 当前样本接近 3.5 s 上限 |
| 消息/音频丢弃 | 0 / 0 | 当前样本无背压丢失 |

这些是少量样本，不应包装成稳定 SLA。正式数据需要固定硬件、冻结输入集、保存原始日志，
分别报告冷/热启动 100 轮 P50/P95。

## 5. 完成度与未验收项

已实测：在线/离线模型 adapter、0.4 秒断句、流式解析、双缓冲、动作 Guard、UART PTY、
Gazebo 位移、语音到仿真闭环、失败重试。

仍待实测：

- LoRA 训练及独立测试集上的模型指令遵循率；当前 8 条种子集纯模型仅 2/8。
- 真实扬声器/麦克风下的 AEC ERLE、双讲、距离和噪声测试。
- 目标机器上的 100 轮端到端 P95。
- 实体 `/dev/ttyUSB*`、`/dev/spidev*`、MCU、电机和急停。
- 独立 KWS 的误唤醒率、漏唤醒率和不同说话人测试。

## 6. 常见故障

### `ros2 topic echo` 没有输出

它是持续订阅命令，消息到来前等待是正常的。另开终端执行 `ros2 topic pub --once`，并
确认两个终端加载了同一 `scripts/activate.sh` 与 `ROS_DOMAIN_ID`。

### 正确 ASR 后没有动作

检查 `wake_word_enabled` 是否按预期传入节点，然后依次观察：

```bash
ros2 topic echo /agent/state
ros2 topic echo /agent/recognition_feedback
ros2 topic echo /agent/action_candidate
ros2 topic echo /robot/action_rejected
ros2 topic echo /robot/action_ack
```

### 一轮结束后持续识别

Agent 忙碌期间会丢弃音频、commit、partial 和重叠 final。若仍出现，确认没有旧 launch
残留，并使用验收脚本生成独立 `ROS_DOMAIN_ID`。

### 单测通过但真实链失败

按声卡 -> ASR -> LLM -> parser -> Guard -> executor -> odom 顺序定位，不要直接把问题
归因于模型。验收脚本的六阶段输出就是为这个目的设计的。
