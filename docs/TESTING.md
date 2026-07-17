# 测试与验收

本文是项目唯一验收契约。命令实现以 `tools/acceptance/catalog.py` 为准；算法解释见 [三册学习笔记](learning/)，系统边界见 [架构文档](ARCHITECTURE.md)。

## 1. 证据分层

| 层级 | 能证明 | 不能证明 |
| --- | --- | --- |
| unit/repository | 领域规则、schema、文件契约 | ROS 图已连通 |
| stage/mock | 状态机、队列、Action 接线 | 麦克风、模型、Gazebo 真实可用 |
| Gazebo E2E | SLAM、Nav2、物理仿真闭环 | 实体硬件可靠 |
| 真人语音 | 当前声卡、VAD/ASR、Agent、控制闭环 | 长期准确率 |
| 公开 bag/实机 | 数据适配或设备证据 | 未运行的环境 |

`passed=true` 只对报告中的 `evidence_kind` 有效；mock 不能冒充真人语音，Gazebo 不能冒充 UART/SPI 实机。

## 2. 环境准备

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh
bash scripts/acceptance_test.sh --help
```

worktree 中先 `unset WORKSPACE`，再 `source scripts/activate.sh`。重型 SLAM 前运行：

```bash
embodied_workspace_doctor true
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh
```

doctor 必须确认当前 overlay、`ManageSlamSession.RUN_AUTOMATIC_MISSION` 和固定版本 Explore Lite 均来自当前工作区。

## 3. 稳定公开入口

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh slam-nav-e2e
bash scripts/acceptance_test.sh robotics-gate
```

`--help-all` 展示维护/实验模式，不应成为 README 主路径。

## 4. 合并前门禁

快速门禁：

```bash
pytest -q tests/repository
pytest -q src/embodied_online_agent/test src/embodied_offline_agent/test
bash tests/integration/control/test_acceptance_cli.sh
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh slam-autonomous-mission-stage
```

ROS 2/C++ 门禁：

```bash
colcon test --packages-select \
  embodied_agent_interfaces embodied_agent_middleware embodied_agent_cpp \
  embodied_simulation embodied_slam embodied_slam_tools embodied_navigation \
  --event-handlers console_direct+
colcon test-result --verbose
```

改变任务顺序、Action 关联、进程切换、frontier 或 Nav2 时必须补跑重型 E2E。

## 5. 完整 SLAM → Nav2 验收

```bash
bash scripts/acceptance_test.sh slam-nav-e2e
```

典型耗时约 3～5 分钟，最长门禁为 900 秒。启动后会立即给出 session、报告和运行日志路径；随后按
`runtime_startup → frontier_slam → map_save → localization_and_semantic_nav →
dynamic_obstacle_replan → evidence_validation` 输出里程碑，并每 15 秒打印心跳。只有心跳停止且进程
退出/达到超时才算异常；不要把真实 frontier 探索期间的数十秒等待误判为卡住。

调试时可缩短心跳间隔而不改变验收语义：

```bash
SLAM_NAV_PROGRESS_HEARTBEAT_S=5 bash scripts/acceptance_test.sh slam-nav-e2e
```

该入口只接受本次 session 新地图：

```text
showcase_apartment → bootstrap → Explore Lite frontier → SLAM Toolbox
→ map saver → 关闭 mapping stage → map_server + AMCL + Nav2
→ entrance → kitchen → office → 动态障碍横穿 → 重规划 → 停车
```

硬性 PASS：

- YAML/PGM 晚于 session 开始，路径、时间和 SHA256 写入报告；
- 至少一个 frontier goal，建图路径 `>=10m`；已知/占用栅格 `>=6000/150`；
- 结束原因仅允许 `no_frontiers`、`coverage_plateau`、`time_budget_coverage`；
- `map→odom`、AMCL pose 有观测，四个 Nav2 lifecycle 节点为 ACTIVE；
- 入口与厨房/办公室任务 `error_code=0` 且无遗漏 waypoint；
- 动态障碍进入预测 costmap，安全间距增大并产生多条唯一规划；
- 报告 `passed=true`，最终 `/cmd_vel` 为零。

证据：

```text
logs/acceptance/slam_nav/<session_id>/slam_nav_e2e_report.json
logs/acceptance/slam_nav/<session_id>/runtime.log
logs/acceptance/slam_nav/<session_id>/voice_built_map.{yaml,pgm}
```

进入 E2E 探针后的运行期失败必须生成带 `error`、最后状态和最后观测速度的 JSON，不能只表现为卡住。
只有成功报告才要求 `final_cmd_vel_zero=true`；失败报告中的速度用于诊断，不能当作已安全停车的证明。
如果 workspace doctor、依赖或 launch preflight 在探针启动前失败，终端会直接返回非零并指出缺失项，
此时不会伪造一份 session 报告。

## 6. 真人语音验收

### 6.1 麦克风预检

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
```

RMS/peak 几乎为零时先修 Windows 权限、WSLg Pulse source 和 `PULSE_SERVER`，不要先调 ASR。

### 6.2 连续控制

```bash
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
```

话术：`小智` → `向前走一秒` → `左转九十度` → `向右转，然后向前走一秒` → `停下` → `退出控制`。

PASS：ASR/NLU/queue/execution/result 连续可见；busy 时进 FIFO；急停清队列；退出后 sleeping；最终零速度。

### 6.3 一句话自动建图巡检

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

只说 `小智，开始自动巡检建图`。应看到地图增长、frontier、存图、AMCL/Nav2 切换和三个语义地点。
该高级交互入口可用 `--help-all` 发现。它会启用 predicted costmap layer，但不自动注入验收专用的
`crossing_cart` 或 synthetic detection，因此人工主演示的 PASS 是建图、定位和语义巡检完成，
不把“代价层已加载”说成“已经观察到动态重规划”。需要可重复的横穿、路径变化和安全间距证据时运行：

```bash
bash scripts/acceptance_test.sh slam-nav-e2e
```

## 7. 故障定位

| 现象 | 先看 | 处理 |
| --- | --- | --- |
| 无 audio | WSL 麦克风预检 | 修 source/权限 |
| 只识别前几个字 | VAD endpoint、commit | 校准 profile，提高尾静音/commit delay |
| 识别到但不执行 | session/nlu/queue | 检查 wake gate、queue full |
| 首命令偶发丢失 | readiness、ActionGuard health | 等 Agent→Guard→Scheduler DDS 全匹配 |
| 一直 executing 0% | Action feedback、Gazebo clock | 检查仿真时钟与 executor result |
| SLAM 看似卡住 | `[slam-nav-e2e] RUNNING`、session phase、runtime.log | 有心跳则继续等待；无心跳/超时再查 explorer、map growth 和显式 error |

```bash
bash scripts/acceptance_test.sh voice-calibration-report
APPLY_VOICE_CALIBRATION=true bash scripts/acceptance_test.sh continuous-offline
```

## 8. 在线、离线与发布边界

- 在线先验证 `.env` API key，禁止提交密钥；关注 retry 与首 token。
- 离线默认 Sherpa-ONNX + llama.cpp + Sherpa-TTS；SummerTTS 为可选组件。
- `offline-latency` 子指标不等于真人语音整链路时延。
- LoRA/Q8 只有真实报告后才能引用准确率、速度或压缩率。

PR 必须写明变更边界、测试命令、PASS 摘要、重型证据路径和已知限制。流程：

```text
feature/* or refactor/* → dev → main → version tag
```

`main` 只接收稳定里程碑；完整功能完成后再 push 触发 CI。CI 不强依赖模型、麦克风或 Gazebo GUI，由本机证据补充。
