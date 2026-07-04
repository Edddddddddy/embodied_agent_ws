# 测试与验收手册

本文档用于回答两个问题：

1. 项目哪些能力可以自动验证？
2. 真实麦克风、在线接口、Gazebo 仿真应该如何人工验收？

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
| `gazebo` | 自动/仿真 | typed Action 到 Gazebo 运动验证 |
| `gazebo-voice` | 自动/仿真 | 离线合成语音到 Gazebo 动作 |
| `gazebo-voice-online` | 自动/联网/仿真 | 在线 provider 到 Gazebo 动作 |
| `continuous-endpoint` | 自动 | `/audio/speech_ended` endpoint 驱动 ASR commit |
| `continuous-mock` | 自动 | 一次唤醒、多条命令、队列和休眠 |
| `continuous-soak` | 自动 | 长会话连续命令稳定性 |
| `continuous-queue-full` | 自动 | busy 时队列满反馈 |
| `continuous-multi-command` | 自动 | 一条 ASR final 被轻量 NLU 解析成多条队列命令 |
| `continuous-ttl` | 自动 | 过期命令丢弃 |
| `continuous-timeout` | 自动 | 会话超时后重新要求唤醒 |
| `voice-readiness` | 自动 | 麦克风/音频前端 readiness 检查 |
| `continuous-offline` | 人工 | 真实麦克风离线连续控制 |
| `continuous-online` | 人工/联网 | 真实麦克风在线连续控制 |
| `continuous-live-check` | 人工辅助 | 订阅 topic 并统计现场演示证据 |
| `all` | 自动 | release gate，不包含人工 microphone 模式 |

## 2. 推荐测试顺序

### 2.1 无外部依赖基础验收

```bash
pytest -q tests/repository
bash tests/integration/test_acceptance_cli.sh
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh continuous-queue-full
bash scripts/acceptance_test.sh voice-readiness
```

通过后说明：仓库结构、脚本入口、连续语音会话、队列、endpoint、readiness 基本正常。

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

## 4. 真实语音问题排查

### 4.1 ASR 完全没听到

运行：

```bash
bash scripts/acceptance_test.sh voice-readiness
python scripts/audio_frontend_calibration.py --duration 6
```

如果 speech ratio 很低，尝试：

```bash
VOICE_CONTROL_PROFILE=quiet bash scripts/acceptance_test.sh continuous-offline
```

如果环境噪声持续触发，尝试：

```bash
VOICE_CONTROL_PROFILE=noisy_room bash scripts/acceptance_test.sh continuous-offline
```

### 4.2 “左转90度”只识别成“左转”

当前链路有两层保护：

1. endpoint 更稳：`SPEECH_END_SILENCE_S` 和 `ASR_COMMIT_DELAY_MS`。
2. 命令补全：裸 `左转/右转/前进/后退` 补成默认演示动作。

建议：

```bash
SPEECH_END_SILENCE_S=0.85 ASR_COMMIT_DELAY_MS=500 bash scripts/acceptance_test.sh continuous-offline
```

如果 monitor 输出 `completed_missing_slot`，说明短命令补全已经生效。

### 4.3 一句话里多个命令没有顺序执行

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

### 4.4 ASR 有输出但动作没执行

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

### 4.5 在线模式失败

检查 API Key：

```bash
grep DASHSCOPE_API_KEY .env
bash scripts/acceptance_test.sh online
```

在线真实语音受网络和云端服务波动影响，现场演示建议优先使用 `continuous-offline`，在线作为补充展示。

## 5. Release gate

完整自动门禁：

```bash
bash scripts/acceptance_test.sh all
```

说明：

- `all` 会运行构建、单测、mock、连续语音 smoke、typed action、仿真 smoke 等自动项。
- `all` 不包含 `continuous-offline` / `continuous-online`，因为它们需要人工真实说话。
- 如果本轮只修改文档和注释，可先跑轻量门禁；发布前再跑 `all`。

## 6. 当前完成度

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
