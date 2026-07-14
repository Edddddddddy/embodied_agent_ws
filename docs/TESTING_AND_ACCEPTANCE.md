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

# 短序列失败边界：office1-7 Range 下载约 1.43 GB。
bash scripts/acceptance_test.sh openloris-loop-evidence

# 本地重型实验：真实 office1-7 的 6 组 accepted-edge 前端阈值消融。
bash scripts/acceptance_test.sh openloris-loop-sweep

# 真值预筛选（小下载）与 corridor1-1 正式 2D 长回环证据（11.23 GB raw bag）。
bash scripts/acceptance_test.sh openloris-sequence-ranking
bash scripts/acceptance_test.sh openloris-long-loop-evidence

# 固定图后端鲁棒性与几何一致性门控；不重新回放 11 GB 原包。
bash scripts/acceptance_test.sh openloris-robust-kernel-ablation
bash scripts/acceptance_test.sh openloris-loop-consistency-ablation
bash scripts/acceptance_test.sh openloris-scan-overlap-ablation

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

`openloris-loop-evidence` 还要求生成 `revisit_catalog.json`、GTSAM accepted constraint JSONL 和
`gtsam_loop_constraints.json`。最终轨迹回访恢复率与 accepted-edge event recall 必须分开讲：前者
可以在没有非局部图边时仍然很高，不能据此宣称回环前端成功。

`openloris-loop-sweep` 首次运行会从公开 bag 流式复制 `/odom`、`/scan`、`/tf_static`，生成带
来源哈希的约 5 MB SLAM-only bag；后续可复用校验通过的 profile 结果。验收要求 6 组 manifest、
参数哈希、bag 哈希和轨迹覆盖可比，并生成 `logs/openloris/office1-7/loop_sweep/comparison.json`。
当前真实结果是 6 组均有 46 条相邻边、0 条非局部边、event recall 0；这是有效的失败边界证据，
不是测试失败。每个 profile 还必须生成 `gtsam_frontend.jsonl` 和
`gtsam_frontend_report.json`，并满足“诊断图节点数 = accepted graph edge 数 + 1”、JSONL 全部可
解析、closure begin/end 平衡。`failure_boundary` 用于区分 candidate generation、coarse、fine、
constraint insertion 和 accepted loop；没有 matcher callback 时不能猜成“响应阈值太高”。

`openloris-long-loop-evidence` 的通过含义是证据链完整，不是保证算法性能达标：默认推荐
`corridor1-1`，显式选择 `corridor1-2` 时则要求同一传感器契约、至少 100 m/90 s 和真实回访；
range/bag 大小与 SHA256 通过；派生 bag
绑定原包哈希；轨迹覆盖达标；按 360° LiDAR 位置口径存在 2 次至少相隔 60 秒的真值回访；
frontend trace 与 accepted-edge 报告均可解析。event recall 允许为 0，因为“真实回环存在但前端
未恢复”本身就是不能篡改的有效负结果。

`openloris-loop-consistency-ablation` 要求五组读取同一 graph SHA256、输入节点/约束数和真值匹配数。
门控组可以少用约束，但必须同时报告 `constraints_used` 和 `consistency_rejected_constraints`；公平性
检查比较的是不可变输入图，不会把主动拒绝异常边误判为换了数据。当前 2 m / π/4 门控拒绝 23 条边，
但该数值只对 `corridor1-1` 构成证据，门控仍默认关闭。

`openloris-scan-overlap-ablation` 还要求派生 bag、固定图和真值文件已存在。验收器先以时间戳关联
`/scan`，从 `/tf_static` 求出 `laser -> base_link` 变换，再为全部 858 条非局部边写入可选 overlap
字段。四组必须共享增强图 SHA256、1834/2751 图规模和 1828 个真值匹配位姿；任何门控组出现
`scan_overlap_unavailable_constraints > 0` 都判失败。当前 0.65/1 m 双证据组额外拒绝 11 条边，
但参数默认关闭，PASS 表示证据和公平性完整，不表示该阈值已跨场景泛化。

`openloris-scan-overlap-multisequence` 聚合 `corridor1-1/1-2` 两份 comparison。PASS 只表示输入
来自不同固定图、阈值一致、报告完整且扫描证据无缺失；是否启用由 `release_decision` 单独给出。
当前第二序列四组 ATE/P95 完全相同，因此即使平均 ATE 改善，决策仍必须是
`keep_disabled_collect_more_sequences`。

### 动态障碍

```bash
bash scripts/acceptance_test.sh dynamic-obstacle-stage
bash scripts/acceptance_test.sh dynamic-obstacle-navigation
bash scripts/acceptance_test.sh dynamic-obstacle-navigation-ablation
```

判定：tracker 输出稳定 ID/速度；未来位置在 costmap 成为 lethal cost；路径净空提升；track TTL
清除后机器人能继续规划并最终停车。

`dynamic-obstacle-stage` 自动生成 `logs/dynamic_obstacle_model_ablation.json/.md`，要求四种模型
使用相同 91 帧输入、没有轨迹丢失，并验证 CV 相对 current-only 的预测收益、IMM 在该固定机动
场景中的预测/遮挡误差和停车过冲。`dynamic-obstacle-navigation-ablation` 是本地重型证据：四轮
分别重新启动 Gazebo/Nav2，全部要求 future cell lethal、路径净空提升、导航成功和最终零速。
它依赖 `slam-benchmark` 生成的地图，不进入 GitHub CI。四份报告必须携带一致的场景、地图栅格
和 Nav2 参数 SHA256；任一哈希不同，汇总器判 FAIL。场景输入是确定性的 typed `PoseArray`，
因此这是“合成感知输入 + Gazebo/Nav2 真实规划控制”的闭环证据，不是物理动态 actor 证据。

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

五分钟留证：

```bash
bash scripts/acceptance_test.sh continuous-voice-evidence offline
# 或分终端运行 continuous-offline + continuous-voice-benchmark offline
# 在线模式同理，把 offline 替换为 online
```

每种模式分别生成 `continuous_voice_<mode>_live_report.json` 与
`voice_benchmark_<mode>_report.json`，不会互相覆盖。报告记录识别率、动作准确率/成功率、
误触发、queue reject、ignored/retry、P50/P95 延迟和最终零速；未达门槛也必须落盘。

两种模式完成后生成事实汇总：

```bash
bash scripts/acceptance_test.sh runtime-evidence-summary
# logs/runtime_evidence_summary.json
```

汇总中的 `proven/failed/missing` 不能互相替代；180 秒旧报告不会被升级成 5 分钟证据，
mock/fixture 也不会被标记为真人麦克风证据。

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
- LoRA dry-run 只验证入口；`lora-q8-comparison` 会独占两个端口重跑 43 条对照并进行哈希审计。
  合成 holdout 分数不等于真实麦克风准确率，动作、协议、严格总分与 fallback 必须分栏。
- OpenLORIS 小 fixture 只验证接口；office 短序列使用 OptiTrack，market 长序列使用官方离线
  LiDAR-SLAM 真值，证据独立性不同。任何序列 accepted 非局部回环为 0 时都不能宣称前端成功。
- 完整功能完成后再 push/开 PR 触发 GitHub CI，避免为文档碎片频繁运行 CI。
## 8. OpenLORIS 候选级回环检索

准备好 `corridor1-1`、`corridor1-2` 的派生 bag、真值和固定图后运行：

```bash
bash scripts/acceptance_test.sh openloris-lidar-loop-candidates
```

PASS 证明两条独立数据契约、候选评分和多序列公平性检查成立。它不会启动 Gazebo，也不会把候选
写入图；应同时查看 `docs/evidence/lidar_loop_candidates_multisequence.md` 中的 Recall、Precision
和 shadow-only 发布结论。

## 9. OpenLORIS 影子扫描匹配

在候选级验收完成后运行：

```bash
bash scripts/acceptance_test.sh openloris-lidar-shadow-matches
```

该入口提取两条序列的确定性 LaserScan corpus 与 `/odom` 偏航先验，把 Top-10 候选送入 C++
粗到细 ICP，再用官方轨迹离线统计 precision、conditional recall、相对位姿误差和事件恢复。
`PASS` 只表示数据覆盖、哈希、固定 profile 和跨序列公平性契约成立；是否允许下一阶段图边消融由
`docs/evidence/lidar_shadow_matches_multisequence.json` 的 `release_decision` 单独决定。当前状态是
`shadow_only_improve_geometric_verification`，`direct_graph_edge_insertion_enabled=false`。

## 10. OpenLORIS 局部子地图 A/B

先完成上一节、保留确定性 scan corpus 和 pair 文件，再运行：

```bash
bash scripts/acceptance_test.sh openloris-lidar-submap-ablation
```

该入口用短时 `/odom` 将中心帧前后各一帧变换到中心坐标系，形成三帧局部子地图；候选集合、
真值、ICP 配置和评估 profile 与单帧基线保持一致。`PASS` 只证明两序列 A/B 输入契约公平、报告
完整，不表示算法可进入正式位姿图。最终必须检查
`docs/evidence/lidar_submap_ablation_multisequence.json`：当前
`guarded_graph_edge_ablation_ready=false`、`direct_graph_edge_insertion_enabled=false`。

固定门槛要求每条序列同时达到 precision ≥ 80%、conditional recall ≥ 15%、平移中位误差
≤ 0.5 m。相对单帧有改善但未满足绝对门槛时，仍保持 shadow-only。
