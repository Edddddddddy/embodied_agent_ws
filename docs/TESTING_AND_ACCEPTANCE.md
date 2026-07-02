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
bash scripts/acceptance_test.sh demo
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh gazebo-voice
bash scripts/acceptance_test.sh all

# 交互式，不包含在 all 中
bash scripts/acceptance_test.sh microphone-offline
bash scripts/acceptance_test.sh microphone-online
```

`mock` 是每次提交前的最低门槛；`demo` 使用 mock executor 验证组合动作、accessory ACK
和弧线速度。当前记录为 130 项 colcon 测试、2 项仓库约束测试、
0 failure。`online` 使用少量
DashScope token；`offline` 会启动 llama-server；Gazebo 模式用 ACK 与里程计位移验真。
`all` 是完整自动 release gate，包含 mock、在线、离线、Gazebo 和在线/离线语音到 Gazebo，
但明确排除必须由真人说话的麦克风验收。运行 `--help` 可查看模式语义。

## 2. 分层测试矩阵

| 层级 | 主要文件/脚本 | 证明什么 |
|---|---|---|
| C++ 单元 | `embodied_agent_cpp/test/` | AEC/VAD、动作 schema、CRC、UART PTY |
| Python 单元 | `embodied_online_agent/test/` | 流式解析、记忆、唤醒、重试、fallback |
| 离线单元 | `embodied_offline_agent/test/` | 双缓冲、指标、ZipFormer 热词参数 |
| 仿真单元 | `embodied_simulation/test/` | 速度限制、雷达停车、避障、沿墙 PID |
| ROS mock | `smoke_test*.sh` | 话题发现、动作发布、ACK、watchdog |
| Lifecycle | `smoke_test_lifecycle.sh` | 未激活门控、激活执行、停用零速和 cleanup |
| 类型兼容 | `smoke_test_typed_action.sh` | 旧 JSON 与 typed command 同时发布且字段等价 |
| Action 状态 | `smoke_test_typed_action_server.sh` | 成功、反馈、取消、阻塞、超时和抢占 |
| BT 编排 | `test_command_behavior_tree.cpp` | 验证、反应式安全、取消、超时与恢复 |
| executor 插件 | `test_robot_executor_plugins.cpp` | 两个 pluginlib adapter 可发现且行为一致 |
| 参数与 lint | `test_node_configuration.cpp`、ament lint | 无效控制参数在 configure 前失败，产品 C++/CMake/XML 可静态检查 |
| mock 插件全链 | `smoke_test_mock_executor.sh` | 不改 Guard/BT 即可切换 backend |
| 组合演示 | `smoke_test_demo_sequence.sh` | `set_led → wave → move → turn → arc → stop` 顺序执行 |
| 组件化等价 | `smoke_test_composed_executor.sh` | 同一控制实现可在多线程 component container 中完成 Action/BT 链 |
| 命名空间 | `smoke_test_namespaced_executor.sh` | 相对名称、Action、BT、速度和 diagnostics 均隔离到 `/robot1` |
| Action 全链 | `smoke_test_typed_action_pipeline.sh` | JSON -> typed -> Action -> 仿真控制 |
| Action + Gazebo | `smoke_test_gazebo_typed_action.sh` | terminal result 与真实 `/odom` 位移 |
| provider | `smoke_test_online_real.sh`、`benchmark_offline.sh` | 云/本地模型可用和真实延迟 |
| 物理仿真 | `smoke_test_gazebo*.sh` | Gazebo 执行动作并产生合理 `/odom` |
| 人工声学 | `accept_voice_simulation_microphone.sh` | WSLg 麦克风、真实人声和重试体验 |
| 仓库约束 | `tests/repository/` | 用户脚本与集成探针分区、关键探针保持可发现 |
| 验收 CLI | `tests/integration/test_acceptance_cli.sh` | release gate 与交互式入口保持可发现、返回码稳定 |

直接运行测试：

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose

pytest -q src/embodied_online_agent/test src/embodied_offline_agent/test
pytest -q tests/repository
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

`voice_turtlebot3.launch.py` 默认启用 typed Action。`acceptance_test.sh gazebo` 会同时验证
默认链与显式 typed terminal result；`gazebo-voice` 要求离线语音链收到成功 result 后才通过。
typed Action 验收同时要求 `/robot/bt_status` 到达 `confirm/succeeded`，因此不是只验证 XML
能加载，而是验证真实动作确实穿过了整棵树。
所有主 launch 默认由 Nav2 lifecycle manager 自动配置并激活 Guard/仿真执行器；
`lifecycle_autostart:=false` 可用于手工检查未激活状态不会执行动作。
命名空间验收会等待 diagnostics 确认 Lifecycle 已进入 active 后再发目标，避免把“节点已
发现”误当成“控制器已就绪”的启动竞态。

单独验收唤醒词：

```bash
WAKE_WORD_ENABLED=true SPEAKER_ENABLED=false \
  bash scripts/accept_voice_simulation_microphone.sh offline
```

若门控未通过，`/agent/recognition_feedback` 会给出原因和次数，节点保持监听。热词表位于
`src/embodied_offline_agent/config/hotwords_zh.txt`。合成短词仍可能把“小智”识别成
“早日”，因此正式产品需要独立 KWS 并统计 false accept / false reject。

## 4. 当前实测基线

环境：WSL Ubuntu 24.04、ROS 2 Jazzy、16 vCPU、约 8 GB RAM；最新复验日期
2026-07-02。以下为本次单次/少量样本，不是 SLA。

| 指标 | 当前样本 | 结论 |
|---|---:|---|
| 在线 LLM 冷启动首 token | 1.474 s | 不达 1 s，必须预热 |
| 在线 LLM 热启动首 token | 350–384 ms | 当前样本达标 |
| 在线 TTS 首音频 | 222–242 ms | 当前样本达标，仍需 P95 |
| 在线 ASR commit-to-final | 174 ms | 当前样本达标 |
| ZipFormer RTF | 0.0416 | 远快于实时 |
| llama.cpp CPU decode | 34.10 token/s | 超过 8.6 目标 |
| Melo/Sherpa-TTS RTF | 0.4006 | 快于实时 |
| 离线首音频 | 2.274 s | 当前样本达标 |
| 离线整轮 | 2.313 s | 当前样本低于 3.5 s |
| 消息/音频丢弃 | 0 / 0 | 当前样本无背压丢失 |
| 原始模型 / fallback 指令通过 | 2/8 / 7/8 | 链路可用；模型本身仍需 LoRA |
| Gazebo 兼容链 / typed 链位移 | 0.088 / 0.330 m | 两条链均产生真实里程计位移 |
| 离线语音→typed Action→Gazebo | 0.162 m | 带 ASR 噪声仍完成动作并返回 success |

这些是少量样本，不应包装成稳定 SLA。正式数据需要固定硬件、冻结输入集、保存原始日志，
分别报告冷/热启动 100 轮 P50/P95。

### 2026-07-02 release gate 结果

| Gate | 结果 | 关键证据 |
|---|---|---|
| `mock` | PASS | 130 colcon + 2 repository tests，全链 smoke 通过 |
| `online` | PASS | 实际 DashScope ASR、LLM、TTS 与 Guard/hardware mock |
| `offline` | PASS | ZipFormer、Q8 llama.cpp、Sherpa-TTS、双缓冲与 hardware mock |
| `gazebo` | PASS | `/scan`、`/cmd_vel`、`/odom`、Action result 与 BT confirm |
| `gazebo-voice` | PASS | speech→ZipFormer→llama.cpp/fallback→Guard→Action/BT→Gazebo |
| `gazebo-voice-online` | PASS | speech→在线 ASR/LLM→Guard→typed Action/BT→Gazebo，位移 0.163 m |
| 真人麦克风 | 待人工复验 | WSLg 已发现 `RDPSource`，但自动任务不能代替真人发声 |

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
