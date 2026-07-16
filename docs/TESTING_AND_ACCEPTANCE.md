# 测试与验收手册

本文是项目唯一的验收契约。它说明每条命令的前置条件、实际覆盖的函数/接口、PASS 条件、证据和
失败定位；算法原理分别放在学习笔记和 SLAM 专题文档中。

## 1. 先区分五种证据

| 层级 | 证明什么 | 不能证明什么 |
| --- | --- | --- |
| 单元/仓库契约 | 函数、schema、路径和文档契约正确 | ROS 节点真的连通 |
| stage/mock | 状态机、队列、Action 接线可重复 | 当前麦克风、模型、Gazebo 可用 |
| Gazebo 重型 | ROS、物理仿真、SLAM/Nav2 形成闭环 | 实体硬件可靠 |
| 真人语音 | 当前声卡、VAD/ASR、Agent 和控制闭环 | 长期统计准确率 |
| 公开 bag/实体设备 | 数据适配或真实硬件证据 | 未跑过的环境和传感器 |

报告里出现 `passed: true` 只对报告声明的 `evidence_kind` 有效。禁止用 mock 通过冒充真人语音，
也禁止用 Gazebo 通过冒充 UART/SPI 实机。

## 2. 环境准备

首次部署：

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh
```

`bootstrap.sh` 默认完成系统/Python 依赖、colcon 构建和固定版本 Explore Lite 安装。日常新终端只需：

```bash
cd /home/ubuntu/embodied_agent_ws
unset WORKSPACE                    # 可选：验证自动 worktree 推导
source scripts/activate.sh
bash scripts/acceptance_test.sh --help
```

公共入口通过 `scripts/lifecycle_utils.sh:embodied_resolve_workspace()` 从自身位置推导仓库根目录并
export `WORKSPACE`。同一终端切换 worktree 时先 `unset WORKSPACE`；有意跨目录覆盖需要同时设置
`EMBODIED_ALLOW_WORKSPACE_OVERRIDE=true`，避免把残留变量误当成配置。

完整主演示部署检查：

```bash
embodied_workspace_doctor true
```

它必须确认：

- 当前 `install/setup.bash` 存在；
- `embodied_agent_interfaces` 与 `embodied_slam_tools` prefix 位于当前工作区；
- `ManageSlamSession.Goal.RUN_AUTOMATIC_MISSION` 已生成；
- `explore_lite` 位于当前工作区 install。

若失败，按输出的“修复”命令执行，不要靠重复 source 猜测。

## 3. 推荐门禁顺序

### 3.1 每次改代码：快速门禁

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh slam-autonomous-mission-stage
```

### 3.2 合并前：机器人阶段门禁

```bash
bash scripts/acceptance_test.sh robotics-gate
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh slam-nav-showcase-stage
bash scripts/acceptance_test.sh slam-evaluation-stage
bash scripts/acceptance_test.sh dynamic-obstacle-stage
```

### 3.3 演示前：本机真实门禁

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh gazebo
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

在线演示再补：

```bash
bash scripts/acceptance_test.sh continuous-online
```

## 4. 自动测试矩阵

| 命令 | 被测关键代码/接口 | PASS 条件 | 主要证据 |
| --- | --- | --- | --- |
| `core` | repository tests、Agent pytest、C++ gtest | 全部测试为零失败 | pytest/colcon 输出 |
| `continuous-multi-command` | `CommandNLU.parse()`→`AgentControlPlane.enqueue_command()`→queue | 一句多动作按 command_id 顺序完成 | queue/execution events |
| `slam-autonomous-mission-stage` | `_run_automatic_mission()` dry-run Adapter | 状态到 COMPLETED，cancel 回到 MAPPING，零速 | `automatic_mission_dry_run.json` |
| `nav2-stage` | `Nav2Places`、Nav2 bridge、队列 | navigate/patrol/cancel 顺序正确 | Action/result probe |
| `slam-nav-showcase-stage` | 场景 generator、语义地点、launch contract | world/map/places 同源且哈希/坐标合法 | audit 输出 |
| `slam-evaluation-stage` | ATE/RPE/回环评估器 | fixture 指标和阈值满足契约 | JSON/Markdown report |
| `dynamic-obstacle-stage` | association、CV/Kalman/IMM、costmap seam | 固定输入消融满足阈值 | ablation reports |
| `gazebo` | typed Action→BT→Gazebo executor | `/cmd_vel` 与 odom 变化，最终零速 | integration probe |
| `embodied_workspace_doctor true` | resolver、ROS package prefix、Action contract | 所有部署检查 PASS | 终端检查表 |

## 5. 语音控制验收

### 5.1 麦克风预检

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
```

PASS：列出 WSLg Pulse source，3 秒采样 `rms` 明显高于阈值，结果为 PASS。若 peak/rms 几乎为零，
先修 Windows 麦克风权限和 `PULSE_SERVER`，此时调 ASR 没有意义。

### 5.2 离线连续语音

```bash
bash scripts/acceptance_test.sh continuous-offline
```

推荐话术：

```text
小智
向右转，然后向前走一秒
左转九十度
绕圈
走正方形
停下
退出控制
```

关键调用：

```text
AudioFrontendNode endpoint
→ OfflineAgentNode._on_audio() / _on_speech_ended()
→ AsrEndpointRuntime.request()
→ OfflineAgentNode._accept_transcript()
→ AgentApplicationRuntime.accept_transcript()
→ AgentControlPlane / CommandNLU / queue
→ /agent/action_candidate
```

PASS 条件：

- 至少看到 `[asr]`、`[queue]`、`[exec]`、`[result]`；
- 一句多命令拆成多个 command_id，按序执行；
- 动作执行期间的新命令进入队列而不是丢失；
- `停下/急停` 抢占当前动作并清普通队列；
- `退出控制` 后 session sleeping；
- 最终 `/cmd_vel` 线速度、角速度都为 0。

### 5.3 在线连续语音

```bash
bash scripts/acceptance_test.sh continuous-online
```

前置：`.env` 中存在有效 API key，网络可达。PASS 条件与离线一致；额外观察 provider retry、首
token 和 TTS 指标。在线 ASR/LLM 的外部波动不应导致队列乱序或绕过 ActionGuard。

### 5.4 常见语音现象如何定位

| 现象 | 先看 | 调整/修复 |
| --- | --- | --- |
| audio rms/peak 为零 | Pulse source、Windows 权限 | 先跑 `wsl-microphone-preflight` |
| 有 audio、无 ASR final | `/audio/speech_ended`、VAD 状态 | 选 `quiet/normal/noisy_room`，检查 endpoint |
| “左转90度”只成“左转” | endpoint/commit event | 增大 silence/commit delay；短命令补全应给 feedback |
| ASR final 有、无动作 | `/agent/nlu_parse`、recognition feedback | 检查否定/缺槽位/低置信度 fallback |
| 第二条命令丢失 | queue/execution events | 检查 queue full、TTL、重复 final、command_id |
| 看似卡住 | Action feedback/result、readiness | 区分等待 Action、provider、生命周期未激活 |

### 5.5 成熟 VAD、KWS 与现场校准

默认 `VAD_PROVIDER=auto` 按 Silero→WebRTC→energy 降级。需要复现实例依赖时：

```bash
bash scripts/setup_voice_vad_runtime.sh webrtc
bash scripts/acceptance_test.sh webrtc-vad-sidecar
bash scripts/acceptance_test.sh silero-vad-runtime
bash scripts/setup_voice_kws_runtime.sh openwakeword
bash scripts/acceptance_test.sh openwakeword-sidecar
```

`setup_voice_vad_runtime.sh webrtc` 等价于在项目 venv 安装可选依赖
`embodied_voice_frontend[webrtc-vad]` 并做运行时检查；优先用封装脚本，避免漏掉 smoke。

Sherpa KWS 可用：

```bash
bash scripts/setup_voice_kws_runtime.sh sherpa
source logs/sherpa_kws.env
bash scripts/acceptance_test.sh sherpa-kws-sidecar
```

这些 sidecar 是可替换 provider，
不改变下游 session/queue/Action 接口；缺依赖时必须明确降级，不能把 energy VAD 冒充成熟模型。

现场调参先生成可复制建议：

```bash
VOICE_CALIBRATION_COLLECT=true \
  bash scripts/acceptance_test.sh voice-calibration-report
```

报告中的 `recommended_environment`、`next_command` 和 `logs/voice_calibration.env` 是下一次运行的
输入；可用 `APPLY_VOICE_CALIBRATION=true` 应用。默认不带 `VOICE_CALIBRATION_COLLECT=true`
时使用合成 low-gain 样本，只用于自动回归。不要只看一次 peak 就手工猜阈值。

## 6. 自动建图、定位与导航主演示

### 6.1 轻量前置

```bash
embodied_workspace_doctor true
bash scripts/acceptance_test.sh slam-nav-showcase-stage
bash scripts/acceptance_test.sh slam-autonomous-mission-stage
```

stage 的状态序列至少覆盖 mapping、exploring、saving、localizing、patrolling、completed，并验证
紧急停止。dry-run 只证明状态机，不证明真实地图质量。

### 6.2 真人一句话主演示

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

voice-SLAM 入口默认自动加载 `logs/voice_calibration.env`，并在 WSLg/Pulse 可用时优先使用 Pulse
capture bridge；它不应再与已经调通的 `continuous-offline` 使用两套麦克风参数。首次使用或输入
电平变化后先运行：

```bash
VOICE_CALIBRATION_COLLECT=true \
  bash scripts/acceptance_test.sh voice-calibration-report
```

说：

```text
小智，开始自动巡检建图
```

若 ASR final 精确截断为“开始自动”，白名单会恢复这条自动任务；“开始”、单独“自动”和完整识别
出的其他意图不会被模糊触发。若其他长句也被声学模型截成完全相同的 final，文本层无法区分；
终端应同时打印 partial/final，便于识别这一边界。

若需要先隔离麦克风/ASR、证明后半段真实机器人闭环，保持 Terminal 1 演示运行，并在 Terminal 2
执行：

```bash
bash scripts/voice_slam_nav_showcase.sh trigger-auto
```

此入口通过 typed `/slam/manage_session` Action 发送 `RUN_AUTOMATIC_MISSION`，不是文本 topic 或
旧 JSON 命令。它只绕过声学触发，不能计为真人语音 PASS；后续 Explore Lite、SLAM、map_saver、
AMCL/Nav2 和语义巡检仍是同一真实运行时。

核心函数链：

```text
SessionOrchestratorNode._on_asr_final()
→ parse_session_command()
→ _enqueue() → _worker_loop() → _execute_request()
→ _run_automatic_mission()
→ StageProcessManager.start("mapping") / start_explorer()
→ _wait_for_frontier_completion()
→ _save_map()
→ _start_navigation()
→ _run_agent_text_action()
```

现场必须观察：

1. Gazebo 是 `showcase_apartment` 四区域场景，机器人和激光雷达存在；
2. 机器人先自动脱离左下角充电位，随后 RViz `/map` 从未知逐渐变为已知，frontier goal 会变化；
3. `/slam/session_state` 按阶段推进，不是固定路线脚本输出；
4. 生成非空 YAML/PGM；
5. SLAM 阶段退出后，map_server/AMCL/Nav2 就绪，`map→odom` 存在；
6. 入口单点导航与厨房/办公室多航点巡检收到成功 result；
7. 最终 `/cmd_vel=0`。

任一步失败，任务应停止并给出阶段/原因；不能在 map 未保存时假装进入导航。
探索结束日志还应给出 `no_frontiers`、`coverage_plateau` 或 `time_budget_coverage`；后者必须
同时满足地图覆盖阈值，覆盖未达标的普通 timeout 必须 FAIL。

### 6.3 无麦克风重型门禁

```bash
bash scripts/acceptance_test.sh slam-autonomous-mission
```

它用确定性文本触发同一真实 Gazebo、frontier、SLAM、map_saver、AMCL 和 Nav2 运行时，隔离云
服务和声学波动。报告：

```text
logs/showcase/autonomous_runtime/automatic_mission_report.json
```

PASS 至少要求 `automatic_mission=true`、`map_saved=true`、final phase COMPLETED、已知/占用栅格
达到脚本阈值、导航/巡检成功、最终速度为零。多航点结果还必须同时满足
`ResultCode=SUCCEEDED`、`error_code=0`、`missed_waypoints=0`；Nav2 仅返回协议 `SUCCEEDED`
但存在漏点时按失败处理，不能把“尝试完全部目标”误报为“到达全部目标”。

### 6.4 失败隔离与保底

- `bash scripts/voice_slam_nav_showcase.sh mapping offline`：只定位建图问题；
- 另一个终端 `bash scripts/voice_slam_nav_showcase.sh save`：只定位 map_saver；
- `bash scripts/voice_slam_nav_showcase.sh navigation offline`：加载本次地图定位导航；
- `bash scripts/voice_slam_nav_showcase.sh navigation-static offline`：加载同源静态地图。

这些入口是故障隔离或现场保底。`navigation-static` 成功不能算自动建图成功，固定 mapping route 也
不能算 frontier 自主探索成功。

## 7. C++/ROS 2 专项验收

```bash
colcon test --packages-select \
  embodied_agent_cpp embodied_simulation embodied_slam embodied_navigation \
  --event-handlers console_direct+
colcon test-result --verbose
```

重点覆盖：

- `ActionValidator::validate()` 白名单、范围和危险组合；
- `ActionScheduler::enqueue()` FIFO、stop 抢占、失败清队列、结果关联；
- `CommandBehaviorTree::tick()` 安全阻断、执行、cancel；
- Gazebo/Nav2 executor 的 cancel 与零速；
- SLAM backend、回环门控和动态障碍算法 fixture。

单独运行 typed Action 生命周期：

```bash
bash scripts/acceptance_test.sh cpp-action-client
bash scripts/acceptance_test.sh cpp-action-scheduler
bash scripts/acceptance_test.sh cpp-action-bridge-lifecycle
```

## 8. 离线运行时验收

```bash
bash scripts/acceptance_test.sh sherpa-asr-preflight
bash scripts/acceptance_test.sh sherpa-asr-smoke
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh offline-latency
bash scripts/acceptance_test.sh summer-tts-service
```

前后调用关系：clean PCM→Sherpa stream decode→transcript→llama.cpp stream→sentence chunk→TTS
producer→双缓冲播放 consumer。ASR、LLM、TTS 分项通过不等于真实语音 E2E 通过；完整证据使用：

```bash
bash scripts/acceptance_test.sh offline-voice-e2e-report
```

模型体积、tokens/s、首 token、TTS 时延必须以当前生成报告为准，不复制历史宣传数字。

SummerTTS 命令行 provider 用于隔离验证第三方二进制和模型；配置 `tts_provider:=summer_ros` 的
常驻 C++ service 用于
避免逐句进程/模型加载并验证缓存命中。两者证明的是 Adapter 与服务化接入，当前低延迟默认链仍以
Sherpa-TTS 为主，不能把缓存命中耗时写成 SummerTTS 首次真实合成耗时。

## 9. SLAM 与动态障碍专项

工程闭环：

```bash
bash scripts/acceptance_test.sh mapping-stage
bash scripts/acceptance_test.sh slam-benchmark
bash scripts/acceptance_test.sh slam-gtsam-benchmark
bash scripts/acceptance_test.sh slam-ab-benchmark
bash scripts/acceptance_test.sh slam-navigation
```

公开数据和算法评估：

```bash
bash scripts/acceptance_test.sh slam-evaluation-stage
bash scripts/acceptance_test.sh openloris-replay-stage
```

动态障碍：

```bash
bash scripts/acceptance_test.sh dynamic-obstacle-stage
bash scripts/acceptance_test.sh dynamic-obstacle-navigation
bash scripts/acceptance_test.sh dynamic-obstacle-navigation-ablation
```

详细指标口径见 [SLAM 与导航工程笔记](SLAM_NAVIGATION_ENGINEERING.md) 和
[真实数据 SLAM 评估](REAL_WORLD_SLAM_EVALUATION.md)，本文不重复算法教程。

## 10. 失败清理与重复运行

验收脚本应通过 trap/进程组清理节点。若上次异常退出，先检查而不是盲目启动第二套：

```bash
ros2 node list
ros2 topic info /cmd_vel
ps -ef | grep -E 'gz sim|nav2|slam_toolbox|explore' | grep -v grep
```

WSL 出现 `Failed init_port fastrtps_port7000` 时，确认通过 `source scripts/activate.sh` 加载了
`scripts/ros_dds_env.sh`；默认 Fast DDS 使用 UDPv4，避免 SHM 锁冲突。

## 11. 合并与发布标准

一个完整功能完成后再触发 GitHub CI：

1. feature 分支运行相关单元、stage 和至少一个真实门禁；
2. `git diff --check`，确认未提交密钥、模型和 `logs/`；
3. PR 合并到 `dev`；
4. CI 通过且本机演示通过后，里程碑 PR 合并到 `main`；
5. release 说明必须区分自动证据与人工证据。

最终交付记录应至少包含：commit、环境、执行命令、PASS/FAIL、报告路径、已知边界和复现步骤。
