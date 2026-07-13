# 测试与验收手册

本文只回答三个问题：改完代码该跑什么、现场演示怎样判定通过、失败先看哪一层。全部入口以
公共入口以 `bash scripts/acceptance_test.sh --help` 为准；全部高级和兼容模式以
`bash scripts/acceptance_test.sh --help-all` 为准。README 只保留最常用命令。

## 1. 证据分层

| 层级 | 证明内容 | 不能证明 |
| --- | --- | --- |
| repository/unit | 纯算法、接口、文件和参数契约 | ROS graph、真实设备 |
| mock ROS | topic/action/lifecycle/队列时序 | 麦克风、模型精度、物理仿真 |
| local runtime | 在线 API、离线模型、声学 Provider | Gazebo 运动 |
| Gazebo/Nav2 | TF、雷达、里程计、规划控制、停车 | 实体机器人 |
| live microphone | 当前声卡/噪声下连续控制体验 | 多场景统计泛化 |
| public rosbag | 指定序列上的真实轨迹误差 | 未测试环境的性能 |

所有报告写入 `logs/`，不提交大型模型、bag、地图临时产物或用户音频。

## 2. 提交前推荐顺序

### 2.1 日常门禁

```bash
source scripts/activate.sh
bash scripts/acceptance_test.sh core
bash tests/integration/test_acceptance_cli.sh
```

语音/队列修改追加：

```bash
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh continuous-queue-full
bash scripts/acceptance_test.sh voice-readiness
```

ROS 2/C++ 控制修改追加：

```bash
bash scripts/acceptance_test.sh cpp-action-client
bash scripts/acceptance_test.sh cpp-action-scheduler
bash scripts/acceptance_test.sh cpp-action-bridge-lifecycle
colcon test --packages-select embodied_agent_cpp embodied_simulation --event-handlers console_direct+
colcon test-result --verbose
```

### 2.2 发布和演示 gate

```bash
bash scripts/acceptance_test.sh release-gate
# logs/acceptance_report.json

bash scripts/acceptance_test.sh demo-gate
# logs/demo_acceptance_report.json

bash scripts/acceptance_test.sh robotics-gate
# logs/robotics_acceptance_report.json
```

聚合报告中的 `evidence_kind` 会区分 CI、mock、C++ ROS、本机真实模型、Gazebo 和公开 bag
证据。`robotics-gate` 固定覆盖连续多命令、Nav2 stage、SLAM 指标、OpenLORIS fixture 和动态
障碍 stage；自动 gate 通过后仍要按修改范围运行下面的重型/人工验收。

默认帮助只列出 12 个推荐公共入口；高级、诊断和兼容模式使用：

```bash
bash scripts/acceptance_test.sh --help-all
```

## 3. 在线与离线 Agent

### 在线最小 token

```bash
bash scripts/acceptance_test.sh online
bash scripts/acceptance_test.sh gazebo-voice-online
```

仅发送短请求；若失败先检查 `.env` 中 `DASHSCOPE_API_KEY`、网络和账户额度。

### 离线分层

```bash
bash scripts/acceptance_test.sh sherpa-asr-preflight
bash scripts/acceptance_test.sh sherpa-asr-smoke
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh offline-sherpa-typed
bash scripts/acceptance_test.sh offline
```

延迟证据分开测：

```bash
bash scripts/acceptance_test.sh offline-latency
bash scripts/acceptance_test.sh offline-voice-e2e-report
bash scripts/acceptance_test.sh llama-decode-benchmark
```

`offline-latency` 测 warm LLM 首 token 和短句整句合成；`offline-voice-e2e-report` 测
speech endpoint 到第一块 PCM。两者不能混用。SummerTTS 命令行 provider 每句重新启动进程，
不是低延迟默认路径；常驻服务用 `tts_provider:=summer_ros`：

```bash
bash scripts/acceptance_test.sh summer-tts-service
bash scripts/acceptance_test.sh summer-tts-cache-audit
ros2 launch embodied_offline_agent offline_agent.launch.py tts_provider:=summer_ros
```

## 4. Gazebo、Nav2、SLAM

### 动作与导航

```bash
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh nav2-turtlebot3
bash scripts/acceptance_test.sh nav2-resilience
```

判定：Action 成功或给出明确失败码；`/odom` 与目标一致；取消/结束后 `/cmd_vel` 线速度和角速度
均归零。`nav2-resilience` 还要求障碍注入后路径净空增加，地图外目标返回 aborted。

### 建图到定位规划

```bash
bash scripts/acceptance_test.sh mapping-stage
bash scripts/acceptance_test.sh slam-benchmark
bash scripts/acceptance_test.sh slam-gtsam-benchmark
bash scripts/acceptance_test.sh slam-ab-benchmark
bash scripts/acceptance_test.sh slam-navigation
```

判定：固定路线与漂移输入可比；Ceres/GTSAM 都降低 ATE/闭环误差；地图分辨率 0.05 m 且覆盖
达标；新进程加载地图后 AMCL 发布 `map→odom`，Nav2 到达目标并停车。

### ATE/RPE 与公开数据

```bash
bash scripts/acceptance_test.sh slam-evaluation-stage
bash scripts/acceptance_test.sh openloris-groundtruth
bash scripts/acceptance_test.sh openloris-replay-stage

# 推荐先用约 1.25 GB 的首序列快速模式完成真实数据闭环。
OPENLORIS_RANGE_ONLY=true bash scripts/acceptance_test.sh openloris-rosbag-setup

# 发布级来源审计再下载完整约 9.27 GB 归档。
bash scripts/acceptance_test.sh openloris-rosbag-setup

bash scripts/acceptance_test.sh openloris-bag-preflight
bash scripts/acceptance_test.sh openloris-slam-ab
```

`slam-evaluation-stage` 只验证指标数学；`openloris-replay-stage` 只用小 fixture 验证 ROS 1/2
读取、单调 `/clock`、TF、重复 odom 过滤、双后端启动与干净退出，两者都不代表真实精度。
真实 A/B 必须使用同一 bag 和前端参数，并检查匹配率、ATE RMSE/P95、1 秒 RPE、路径长度比、
最差窗口、终点漂移、回访恢复率、运动类别误差和两个 manifest 的哈希来源。验收报告必须保留
`archive_verification`，不能把 Range 快速模式表述为完整归档 SHA256 已验证。方法见
[REAL_WORLD_SLAM_EVALUATION.md](REAL_WORLD_SLAM_EVALUATION.md)。

### 动态障碍

```bash
bash scripts/acceptance_test.sh dynamic-obstacle-stage
bash scripts/acceptance_test.sh dynamic-obstacle-navigation
```

判定：tracker 输出稳定 ID/速度；未来位置在 costmap 成为 lethal cost；路径净空提升；track TTL
清除后机器人能继续规划并最终停车。

## 5. 真实麦克风连续验收

### 5.1 前置检查和校准

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
bash scripts/acceptance_test.sh voice-calibration-report
```

校准报告包含 `recommended_environment`、`next_command`，并生成：

```text
logs/audio_calibration.json
logs/voice_calibration_report.json
logs/voice_calibration.env
```

应用推荐值：

```bash
APPLY_VOICE_CALIBRATION=true \
CONTINUOUS_SAMPLE_LOG=logs/asr_nlu_samples.jsonl \
  bash scripts/acceptance_test.sh continuous-offline
```

### 5.2 固定话术和 PASS 标准

```text
小智
向右转，然后向前走一秒
后退一秒
绕圈
走正方形
停下
退出控制
```

通过标准：

- 终端持续显示 session、ASR、queue、execution、action result，而不是静默等待。
- 一句话中的多个动作带相同 batch，按 command/request ID 依次完成。
- 执行中收到的普通命令进入 FIFO；`停下/急停` 取消当前 action 并清队列。
- `退出控制` 后进入 sleeping；最终 `/cmd_vel` 为 0。

在线补充：

```bash
bash scripts/acceptance_test.sh continuous-online
```

三分钟留证：

```bash
bash scripts/acceptance_test.sh continuous-voice-evidence offline
# 或分终端运行 continuous-offline + continuous-voice-benchmark offline
```

报告未达门槛也必须生成现场事件和汇总文件，便于区分 ASR、NLU、队列或执行问题。

### 5.3 VAD/KWS 可选运行时

连续语音默认 `VAD_PROVIDER=auto`：Silero → WebRTC → energy。安装和验收：

```bash
bash scripts/setup_voice_vad_runtime.sh webrtc
bash scripts/acceptance_test.sh webrtc-vad-sidecar
bash scripts/acceptance_test.sh silero-vad-runtime
# Python extra 名称：embodied_voice_frontend[webrtc-vad]
```

声学唤醒可选：

```bash
bash scripts/setup_voice_kws_runtime.sh openwakeword
bash scripts/setup_voice_kws_runtime.sh sherpa
source logs/sherpa_kws.env
bash scripts/acceptance_test.sh sherpa-kws-sidecar
```

这些 Provider 不是基础 CI 强依赖；缺模型时必须降级并打印原因。

## 6. 故障定位

### 音频有日志但 ASR 为 0

1. 跑 `wsl-microphone-preflight`，确认 Pulse source 不是 monitor。
2. 看 `rms/peak/speech`；静音也 speech=true 表示阈值过低，讲话仍 false 表示阈值过高。
3. 应用校准 `recommended_environment`；再调整 `SPEECH_START_THRESHOLD`。
4. 漏掉“九十度/一秒”时增大 `SPEECH_END_SILENCE_S` 或 `ASR_COMMIT_DELAY_MS`。

### ASR 正确但动作没执行

按顺序看：`/agent/nlu_parse` → `/agent/action_candidate` → `/robot/action_command_typed` →
Action feedback/result → `/cmd_vel`。被拒绝时看 ActionGuard reason；旧 result 不应唤醒新 command ID。

### FastDDS SHM 锁

项目默认由 `scripts/ros_dds_env.sh` 设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。如仍有残留：

```bash
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh
```

若日志为 `Calculated port number is too high`，问题不是 SHM，而是 `ROS_DOMAIN_ID > 232`。
项目脚本的 PID 取模公式由仓库测试统一检查；手工覆盖时也应使用 `0..232`。

### Gazebo 没有小车

不要只启动 Agent launch。使用 `continuous-offline/online` 或 `nav2-turtlebot3` 一键入口，并检查
`ros2 topic echo --once /clock`、`/scan`、`/odom` 和 `ros2 action list`。

## 7. 完成度边界

- 自动 mock、真实模型、Gazebo、真实麦克风和公开 rosbag 是五类不同证据，不能相互替代。
- 当前真实硬件是 Adapter/mock；Gazebo PASS 不等于 UART/SPI 实机 PASS。
- LoRA 流水线 dry-run 不等于已训练并达到准确率。
- OpenLORIS 小 fixture 只验证接口；当前真实指标只覆盖 `office1-1`，不能外推到其他场景或回环能力。
- 完整功能完成后再 push/开 PR 触发 GitHub CI，避免为文档碎片频繁运行 CI。
