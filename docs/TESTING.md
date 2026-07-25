# 测试与验收

本文是测试命令、PASS 条件、产物位置和故障分层的唯一说明。第一次阅读请先看
[文档阅读地图](README.md)；命令注册以 `tools/acceptance/catalog.py` 为准，稳定模块边界见
[系统架构](ARCHITECTURE.md)，算法原理和代码走读见对应的 [学习笔记](learning/)；已经产生的运行
事实见 [证据索引](evidence/README.md)。开发计划或历史记录不能替代本页的当前验收契约。

## 1. 证据分层

| 层级 | 能证明 | 不能证明 |
| --- | --- | --- |
| unit/repository | 领域规则、接口与文件契约 | ROS graph 已连通 |
| stage/mock | 状态机、队列、Action 接线 | 麦克风、模型、Gazebo 真实可用 |
| known-world Gazebo | 固定场景的 SLAM/Nav2 集成回归 | 未知环境自主探索完成 |
| unknown-world Gazebo | 真值隔离下的探索、建图、定位、导航闭环 | 实体硬件可靠 |
| 真人语音 | 当前声卡、VAD/ASR、Agent 和控制链 | 长期准确率或 SLAM 算法质量 |
| 公开 bag/实机 | 对应数据或设备证据 | 未运行的其他环境 |

`passed=true` 只对报告中的 `evidence_kind` 和本次 session 有效。历史地图、mock、known-world 演示和
Gazebo truth 都不能替换 unknown-world 正式证据。

## 2. 环境与接口重建

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh
bash scripts/acceptance_test.sh --help
```

在 Git worktree 中先 `unset WORKSPACE`，再 source 当前 worktree 的 `scripts/activate.sh`。重型测试前：

```bash
embodied_workspace_doctor true
```

本阶段新增 `FrontierExplorationEvidence.msg`、`SlamMappingCompletionEvidence.msg`、
`SlamNavigationGoalEvidence.msg`，并扩展
`SlamSessionState.msg`，因此 ROS 2 interface type hash 已变化。切换分支后需全量重建所有依赖包；
旧 overlay、旧节点或旧 rosbag 不能与新 schema 混用。重建后 doctor 必须确认接口、Explore Lite 和
关键 package prefix 都来自当前 worktree。

重型场景使用 `AcceptanceSession` 租用 ROS domain/Gazebo partition，并在结束时回收本会话进程组。
只有需要复现固定 domain 时才设置 `SLAM_NAV_ROS_DOMAIN_ID`。

## 3. 公开入口

当前稳定公开入口共 9 个：

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh slam-nav-e2e
bash scripts/acceptance_test.sh unknown-world-slam-e2e
bash scripts/acceptance_test.sh voice-unknown-world-slam-e2e offline
bash scripts/acceptance_test.sh robotics-gate
```

`--help-all` 展示内部回归和实验模式，不作为 README 主路径。

`gazebo` 会为每次运行租用独立 ROS domain/Gazebo partition，并复用重型门禁的
ROS overlay 隔离策略。宿主终端即使 source 过其它 `nav2_ws/ros2_ws`，验收仍只允许
当前 worktree、`/opt/ros/jazzy` 和白名单 Frontier 包参与运行。证据位于
`logs/acceptance/gazebo_typed_action/<session-id>/acceptance_session.json`；
readiness 失败时会直接附带 launch 日志，优先查看 Lifecycle Manager 是否完成配置。

## 4. 合并前门禁

轻量门禁：

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh slam-autonomous-mission-stage
```

`core` 已聚合仓库/评估、在线与离线 Agent、CLI 以及核心 C++ 回归，无需再手工重复执行两类 Agent
单测。需要定位失败时，再按 `core` 输出运行对应的 pytest/colcon 子集。

ROS 2/C++ 门禁：

```bash
colcon test --packages-select \
  embodied_agent_interfaces embodied_agent_middleware embodied_agent_cpp \
  embodied_simulation embodied_slam embodied_slam_tools embodied_navigation \
  --event-handlers console_direct+
colcon test-result --verbose
```

改变 unknown-world 任务策略、frontier 状态、地图判定、定位对齐或目标抽样时，必须补跑正式重型 E2E。

## 5. Unknown-world 正式 SLAM/Nav2 闭环

```bash
HEADLESS=true USE_RVIZ=true \
  bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

这是“未知场地自主探索 → 本次地图 → AMCL/Nav2 → 运行时采样导航目标 → 动态避障”的正式入口。机器人策略
进程只获得在线 scan/odom/TF/map 和本次 mission；场景真值、区域边界、Gazebo pose 只传给 evaluator。
前三个导航目标必须由本次地图运行时采样。随后由 acceptance probe 注入的固定 dynamic challenge
goal/actor 只是可重复的外部测试刺激，不进入探索/采样算法，也不计作自主目标。

长时可视化默认只打开 RViz，Gazebo server、物理、雷达和里程计仍全部运行。即使沿用旧命令
`HEADLESS=false USE_RVIZ=true`，启动前也会检查 renderer 和内存：双 GUI 在软件渲染、WSL 总内存
`<12 GiB` 或可用内存 `<6 GiB` 时自动降级为 RViz-only。若 WSLg 的 D3D12 探针可用，只对本次
AcceptanceSession 注入 `GALLIUM_DRIVER=d3d12`。`SLAM_NAV_ALLOW_DUAL_GUI=true` 是高配机器上的显式
调试 override，不是长时正式门禁默认值。

硬性 PASS：

- `mission_profile=UNKNOWN_WORLD`，`mission_sequence>0`，`mission_outcome=SUCCEEDED`，且阶段到达
  `MISSION_COMPLETED`；`phase` 表示运行阶段，不能替代独立终态 `mission_outcome`。
- YAML/PGM 晚于 session 开始，哈希和时间写入报告。
- 可达自由空间覆盖率 `>=90%`，每个可达区域 `>=85%`，可达区域 unknown 比例 `<=10%`。
- 障碍边界召回率 `>=60%`，真值障碍被误建为自由区的比例 `<=5%`。
- 探索完成必须满足以下两条路径之一，且两者使用相同地图质量门槛：
  - strict frontier：`FrontierExplorationEvidence.valid=true`，完成原因只接受
    `no_frontiers + no_reachable_frontiers`、
    `frontier_attempts_exhausted_recoverable + frontier_attempts_exhausted_below_material_gain`，或
    `frontier_attempts_exhausted_recoverable + frontier_attempts_exhausted_after_final_confirmation`。
    strict 路径要求 available/active 为零、accepted 全部 terminal；`no_frontiers` 必须 blacklisted=0，
    typed exhaustion 只允许与 detected 一致的残余 blacklist。
  - bounded saturation：硬预算后发布中性的 `time_budget_exhausted`，并由独立的
    `SlamMappingCompletionEvidence` 证明连续低收益、final probe、Action 总账排空和 typed STOP；
    `time_budget_exhausted` 本身没有成功语义。
- strict 和 bounded saturation 都必须在建图 stage 内返回首次运动前动态捕获的起点，证明返航 Action
  成功、XY/yaw 在容差内、返航发生在存图之前，并拿到返航后的新鲜零速度。map_saver 返回后还必须
  再执行一次 typed STOP 并取得新 generation 的零速，慢速存图不能靠放宽 freshness 阈值通过。
- AMCL 与 evaluator-only Gazebo truth 至少 20 个重叠时间范围、时间对齐且时间戳不重复的样本，位置误差
  P95 `<=0.25m`。
- 从本次已知自由连通区动态抽样至少 3 个目标；每个 typed goal 终态成功，producer 与 evaluator
  独立检查的全部 Nav2 plan 都没有 unknown、occupied 或 map 外采样点。
- Nav2 lifecycle active；动态障碍证据通过；报告须先证明本次发生过运动，再使用任务 terminal boundary
  之后收到的新鲜 `/cmd_vel` 证明线速度和角速度均为零。

强类型证据通过以下接口从生产侧发布，探针不解析 `detail` 日志猜状态：

- `SlamSessionState.msg`：profile、sequence、`mission_outcome`、frontier 和目标数组。
- `FrontierExplorationEvidence.msg`：frontier 计数、Action 生命周期和双层完成原因。
- `SlamMappingCompletionEvidence.msg`：strict/bounded mode、饱和摘要和逐项返航证据。
- `SlamNavigationGoalEvidence.msg`：目标、终态、plan 计数及 unknown/occupied/map-outside 统计。

all-blacklisted 不是完成或直接 BackUp 条件。`frontier_idle_grace_s=20.0` 给 provider 留出 goal terminal →
下一次 makePlan 的交接窗口；只有 typed attempts exhaustion/no-clearance、`active=0` 且
`accepted==terminal` 才能进入碰撞检查 BackUp，并以 odom 位移验证恢复确实改变了观察位置。扫描后的显著
增长必须同时达到 `>=40 cells` 与 `>=0.2%`，这两个生产门槛和上述 evaluator 门槛都不因本轮修复降低。

若最后一个有预算的恢复扫描仍达到双门槛，任务不会把“预算用完”写成失败或完成，而是运行一次
recovery-budget-neutral final confirmation epoch，让 Explorer 消费 post-scan 地图。确认轮不 BackUp、不
扫描，使用独立且只创建一次的 `240 s` 绝对 deadline；只接受 provider 的 `no_frontiers` 或 typed attempts
exhaustion，任何其他恢复原因均失败。失败报告仍在 `frontier.telemetry` 保存最后一帧 typed frontier 计数和 accepted/terminal Action
账本，不能只靠 `detail` 文本诊断。

场地总面积未知时，运行时不能计算“85%”。硬预算后的 bounded saturation 使用与场景大小无关的证据：
最近至少 `2` 个低收益 epoch、每轮至少 `3` 个 terminal goal、累计建图路径至少 `20m`、final probe
低增益、地图静默 `>=15s`、Action 总账排空以及 final probe
后的新鲜 typed STOP。恢复次数余量只作为诊断字段；若 provider 没有请求恢复，不会为了耗尽计数制造动作。
epoch 以单位 terminal goal 的地图增益判断；只有 cells 与 ratio 同时显著才继续探索。
任一条件不满足都失败，不能因外层等待时间到达而保存地图。

`residual_available_frontiers` 仍写入强类型证据，但 hard-budget 路径只把它作为诊断值：该值采自暂停
Explorer、执行 final probe 之前，provider 停止后不会依据新地图重算；同时 raw cluster 数会随墙角碎片和
栅格噪声波动，不是剩余信息量。`repeated_reachable_stall` 路径仍严格要求 residual 为零。最终 PASS 继续
由下述独立地图质量门禁否决不完整地图，不能把“时间到了”直接解释为完成。

生产进程只读取在线 scan/odom/TF/map；truth map、区域边界与场地可达面积只进入 evaluator。近似完成的
最终 PASS 仍要由 evaluator 独立满足 `90/85/10` 地图质量、障碍质量、返航、AMCL、路径安全、动态重规划
和最终零速门槛，因此不是对 strict evaluator 的降级。

证据路径：

```text
logs/acceptance/unknown_world_slam_nav/<session_id>/unknown_world_slam_e2e_report.json
logs/acceptance/unknown_world_slam_nav/<session_id>/runtime.log
logs/acceptance/unknown_world_slam_nav/<session_id>/unknown_world_map.{yaml,pgm}
logs/acceptance/unknown_world_slam_nav/<session_id>/acceptance_session.json
logs/acceptance/unknown_world_slam_nav/<session_id>/resource_samples.jsonl
```

正式报告必须满足 `schema_version=4`、
`evidence_kind=unknown_world_slam_nav_dynamic_replan`、`passed=true` 和所有 checks 为 true。

### 5.1 当前 Goal 的可视化验收

```bash
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh

HEADLESS=true USE_RVIZ=true \
  SLAM_NAV_PROGRESS_HEARTBEAT_S=10 \
  bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

可视化通过标准：

1. RViz 地图从未知栅格开始扩展，机器人没有读取真值或固定房间路线。
2. 探索正常运动；残余角落长期低收益时，终端明确显示 quiesce、ledger drained、final probe 和
   saturation decision，不能长时间无说明停住。
3. Explorer 停止派发后，机器人在 mapping stage 内回到初始扫描前动态捕获的起点附近；返航后才保存图，
   保存完成后再次确认新鲜零速才允许切换导航 stage。
4. 随后出现 AMCL/Nav2 阶段并完成 3 个本次地图运行时目标，以及动态障碍重规划。
5. 程序自行正常结束并输出 PASS；若由外层 timeout、Ctrl+C 或 cleanup 结束，一律不算通过。
6. 报告的 `return_to_start`、`frontier_complete`、地图/定位/导航/dynamic/final-stop checks 全部为 true，
   `/cmd_vel` 最终为零。
7. `acceptance_session.json` 的 `resources.failure=null`、`cleanup_complete=true`；长时曲线写入
   `resource_samples.jsonl`。可用内存持续低于 `768 MiB` 或 Swap 持续超过安全线时，watchdog 会以
   `resource_exhaustion` 失败并先走统一停车/清理，而不是等待 WSL 卡死。

验收失败时保留整个 session 目录，首先查看 `runtime.log` 最后一条 phase/detail，再对照报告中的
`approximate_completion`、`frontier.telemetry`、`return_to_start` 和失败 checks。不要只凭 RViz 中
“看起来覆盖很多”判断通过，也不要删除失败证据后重跑。若 Nav2 Lifecycle Manager 已打印
`CRITICAL FAILURE: SERVER ... IS DOWN`，探针会立即报告 `nav2_runtime_unhealthy`，不会继续等待完整任务
deadline。

### 5.2 取消与故障收口

取消不是“发出 cancel 请求”就完成，而是一项可审计的安全事务：

```text
等待 pending goal response（独立 safety budget）
→ 若 goal 已接受或迟到接受：cancel
→ 发布全新的、未继承 canceled token 的 typed priority STOP
→ 等待 Nav2 goal result 进入 terminal
→ 释放任务锁并发布 MISSION_CANCELED
```

为什么要等待 late goal response：请求 future 可能在用户取消后才返回 accepted；若此时直接释放任务锁，
后台 Nav2 goal 仍可能启动。为什么 `STOP` 必须是 fresh typed request：复用已取消 token 会让安全停车也被
短路，并绕过 Guard/Scheduler 的优先级语义。

pending response/result 未在安全预算内终态、cancel 或 typed STOP 失败时，orchestrator 必须停止整个
stage 并以 `MISSION_FAILED` 收口；这种情况下不能发布“已安全取消”。成功 E2E 的最终零速度只证明正常
终态，不覆盖取消分支；取消竞态由 `test_showcase_session_node.py` 的 pending/late-accept/stop-failure
回归单独证明；此前恢复边界、阶段切换与动态场景修复由 §5.3 的历史 fresh E2E 验证。

建图切换到保存图导航也是 fail-closed 事务：`manager.start("navigation")` 失败或 Nav2 readiness 超时后，
先回收半启动 stage，再发布 `FAILED`。不能根据 `manager.stage` 猜测并回写 `MAPPING/NAVIGATING`，否则
界面已经退出时监控仍会谎报可用状态；清理异常只作为首因的附注保留。

动态 challenge 按本次 baseline path 搜索具有双侧绕行净空的 anchor，不再依赖固定地图坐标；lethal
cost 在首次/最大观测时锁存，规划提交时已因 TTL 衰减的即时值只作诊断。事务退出必须取消活动 goal、归位
Gazebo 实体、清空 detection/tracker/costmap，并在本次动态 Action 的 terminal boundary 后观察新鲜零速。
这些改动既有确定性测试，也已由 §5.3 的 fresh 可视化长跑报告覆盖。

### 5.3 本轮 GUI 稳定性 fresh 证据

本地 session `20260725T120302Z-145519-3ec3df78` 使用用户原命令
`HEADLESS=false USE_RVIZ=true` 启动。策略识别到 D3D12/RTX 3060 Ti 硬件加速，但因 WSL 总内存约
`7.7 GiB` 自动选择 RViz-only；990 秒内完成 frontier 建图、动态起点返航、存图、AMCL、3 个运行时
采样目标和动态障碍重规划。

| 证据 | 实测值 | 门槛 |
| --- | ---: | ---: |
| 总体/最低区域可达自由覆盖率 | `99.7524% / office 98.3402%` | `>=90% / 各 >=85%` |
| reachable unknown | `0.2476%` | `<=10%` |
| 障碍边界召回 / false-free | `82.034% / 0.805%` | `>=60% / <=5%` |
| frontier available/active/blacklisted | `0 / 0 / 0` | 前两项为 0；账本完整 |
| accepted/terminal frontier goal | `34 / 34` | 全部有终态 |
| 建图路径 / 动态起点返航误差 | `153.403m / 0.0073m` | `>=20m / <=0.35m` |
| AMCL/Gazebo 对齐样本、位置误差 P95 | `217 / 0.1541m` | `>=20 / <=0.25m` |
| 运行时采样导航目标、最小间距 | `3/3 成功 / 5.604m` | `>=3 全成功 / >=1.50m` |
| producer/evaluator 路径检查 | 全部 known-free | unknown/occupied/map-outside 均为 0 |
| 动态重规划 / 终态速度 | PASS / 新鲜零速度 | 均须 PASS |
| 会话 RSS 峰值 / 最终 Swap 使用率 | `2056.5 MiB / 0.01%` | 无资源压力、watchdog 不触发 |

Manifest 记录 `requested_headless=false`、`effective_mode=rviz_only`、D3D12 renderer、199 个资源样本、
`resources.failure=null` 和 `cleanup_complete=true`。此前失败会话在约 535 秒出现 Xwayland page
allocation failure、2 GiB Swap 耗尽并丢失 `collision_monitor` 心跳；本次越过同一时点后 Swap 仍基本
为零。现有证据支持“WSLg 双 GUI 资源耗尽”，不支持把问题归因于某一个 ROS 节点的严格堆内存泄漏。
本地 `logs/` 不提交 Git，且 `source_dirty=true`，因此它是 PR 前开发证据，不冒充 clean release 证据。

### 5.4 失败样本与安全复核

诊断证据保留而不删除：

- `20260720T013943Z-1054618-0887ac81`：总体覆盖 `90.72%`，但厨房仅 `25%`，机器人进入约
  `0.605m` 盲袋，frontier 未终结。这促成 `0.33m` 可逃逸 traversal clearance。
- `20260720T025009Z-1101237-7f8143c7`：第一个近距 approach 停在约 `0.38m` 处，随后一次真实绕行
  仍被当成“连续静止”，恢复时地图增益 `35 < 40` 而正确 FAIL。地图增益门槛没有被降低。

最新安全复核锁定了四个边界：

- `0.40m` 是 Burger unknown-world profile 的传感器观测容差，不是 Nav2 成功容差；上游通用默认保持
  `0.30m`，避免把场景 profile 扩散到其他机器人。
- approach 进入观测半径后，必须先用最新地图复核目标仍安全；只请求取消 Action 并记 visited，
  不伪造 Nav2 succeeded。
- 一个 approach 即使最终 timeout，只要累计位移足以逃离上次卡点，就会清除连续静止计数；
  局部 approach 失败仍如实保留。
- 探索 session 实际生效的 Nav2 `SimpleProgressChecker=0.10m/30s` 由
  `prepare_frontier_nav2_params.py` 写入可留档 YAML，不依赖进程结束后丢失的临时参数。

本节列出的四项基线策略及此前恢复边界已通过补丁回放、单元/仓库门禁和 §5.3 fresh E2E 联合验证；
该结果仍不替代当前 Goal 或每次发布前的新会话验收。

### 5.5 真人语音 + unknown-world 联合门禁（待现场）

该入口不是把一次语音 PASS 和一次 SLAM PASS 手工拼接，而是在**同一 AcceptanceSession** 内等待真人
口令、完成严格 unknown-world 事务并生成一份联合报告：

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight

HEADLESS=true USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-unknown-world-slam-e2e offline
# 在线补充验收：
HEADLESS=true USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-unknown-world-slam-e2e online
```

终端进入 MAPPING 并打印“真人语音触发窗口已就绪”后再说：

```text
小智，开始自动巡检建图
```

不要在启动/依赖检查阶段提前说话：联合探针只采集 MAPPING 后打开的 volatile 事件窗口，启动阶段的旧
ASR final 不得成为本次证据。默认等待真人口令 120 秒，可用 `VOICE_TRIGGER_TIMEOUT_S` 显式调整。
在线模式还需提前配置当前 provider 所需 API key；离线模式需完成 Sherpa/llama.cpp 运行时部署。

linked worktree 中的 `models/`、`third_party/` 不属于 Git 跟踪内容。入口把当前代码目录与运行时资产目录
分离：默认用 Git common-dir 找到主工作区，也可显式设置 `EMBODIED_RUNTIME_ROOT`。最终解析值和受控
Nav2 包来源分别写入 `acceptance_session.json.environment.EMBODIED_RUNTIME_ROOT` 与
`acceptance_session.json.ros_environment`。因此不要把模型复制进功能 worktree，也不要依赖终端中偶然
source 的 `~/nav2_ws`。

真人路径的编排器参数为 `command_input_source=wake_event`，只消费 `/agent/wake_event` 中
`KIND_WAKE/KIND_CONTINUE + command_known=true + 非空 command`；裸 `/agent/asr_final` 仅供证据观察，
不能触发任务。无麦克风的 `unknown-world-slam-e2e` 则保持
`command_input_source=raw_asr`，由探针发布确定性文本。这种物理分离防止未唤醒语音、filler 或测试
publisher 绕过会话门，同时保留原 strict 核心门禁的可重复性。

联合报告路径：

```text
logs/acceptance/voice_unknown_world_slam_nav/<session_id>/
├── voice_unknown_world_slam_e2e_report.json
├── runtime.log
├── resource_samples.jsonl
├── acceptance_session.json
└── unknown_world_map.{yaml,pgm}
```

报告外层必须满足：

- `schema_version=1`、`evidence_kind=voice_unknown_world_slam_nav_e2e`、
  `trigger_source=live_voice`，`agent_mode` 与命令参数一致。
- `real_audio_observed`：与匹配命令 endpoint 同一窗口内存在 `speech=true`、`rms>0`、`peak>0` 的音频。
- `endpoint_observed`：至少形成一对 `speech_started → speech_ended`。
- `wake_accepted`：窗口内存在 Agent 发布的 `KIND_WAKE` 或 `KIND_CONTINUE` WakeEvent。
- `automatic_mission_asr_final`：endpoint 后的 ASR final 能解析为 `RUN_AUTOMATIC_MISSION`。
- `strict_core_schema_v4_passed`：内嵌 `core_report` 与外层 `session_id` 相同，并继续满足 §5 的 schema v4
  全部 checks/sections；外层 schema v1 不降低任何 SLAM/Nav2 门槛。

只有五个外层 checks 和内嵌 strict 核心全部为 true，`passed=true` 才成立。截至本文更新，该入口和
自动契约已实现，但**尚未生成真人现场 PASS**，因此不能把 §5.3 的 synthetic session 写成联合证据。

按外层 checks 从上游到下游定位：

| 失败项 | 先看 | 处理 |
| --- | --- | --- |
| `real_audio_observed` | `/audio/frontend_metrics`、WSLg source | 运行麦克风预检，修 `PULSE_SERVER`/权限/输入源 |
| `endpoint_observed` | `/audio/speech_started`、`/audio/speech_ended` | 校准 VAD profile、阈值和尾静音；不要先改 SLAM |
| `wake_accepted` | `/agent/wake_event` 的 kind/provider/command | 连贯说完整唤醒词和命令；检查 wake/session gate |
| `automatic_mission_asr_final` | `/agent/asr_final` 与 `voice_window.asr_finals` | 检查 ASR 截断、模型和 120 秒窗口；filler 不会唤醒任务 |
| `strict_core_schema_v4_passed` | 内嵌 `core_report`、`runtime.log` | 再按 map/frontier/AMCL/Nav2/dynamic/final-stop 分层定位 |

若能看到正确 ASR 文本但任务未启动，优先检查 WakeEvent 是否携带已授权的 `command`，不能通过切回
`raw_asr` 制造假 PASS。失败报告仍保留窗口快照和 core 诊断；不得用另一 session 的 v4 报告补写。

## 6. Known-world 回归与交互演示

```bash
bash scripts/acceptance_test.sh slam-nav-e2e
HEADLESS=true USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

`slam-nav-e2e` 是旧已知场景的确定性回归；`voice-slam-workplace-demo` 验证真人语音触发、阶段切换和
语义地点交互。它们可能使用 bootstrap 路线或预置地点，适合演示集成稳定性，但不能证明 unknown-world
自主覆盖或运行时无场景先验。旧证据仍保存在：

```text
logs/acceptance/slam_nav/<session_id>/slam_nav_e2e_report.json
```

两个证据目录和 `evidence_kind` 不可混用。

语音 demo 会加载预测代价层，但不自动注入验收专用 `crossing_cart`；动态横穿与重规划证据由重型
E2E 场景负责。需要区分“插件已加载”和“本次确实观察到重规划”。

## 7. 真人语音验收

麦克风预检：

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
```

连续控制：

```bash
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
```

话术：`小智` → `向前走一秒` → `左转九十度` → `向右转，然后向前走一秒` → `停下` →
`退出控制`。

PASS：ASR/NLU/queue/execution/result 连续可见；busy 时入 FIFO；急停清队列；退出后 sleeping；最终
零速度。该结果验证语音上游，不自动升级为 unknown-world SLAM/Nav2 PASS。

本阶段已通过 `offline-sherpa-typed`、`continuous-mock` 和 `continuous-multi-command` 自动回归；尚未
重新运行真人麦克风 `continuous-offline/online`，因此不把自动回归写成现场语音 PASS。在线静音
filler 导致的无效请求/token 消耗由独立 issue 跟踪，不属于本次 SLAM/Nav2 修复范围。

## 8. 故障定位

| 现象 | 先看 | 处理 |
| --- | --- | --- |
| 无 audio | WSL 麦克风预检 | 修 source/权限，不先调 ASR |
| 只识别前几个字 | VAD endpoint/commit | 校准 profile，提高尾静音或 commit delay |
| 识别到但不执行 | session/nlu/queue | 检查 wake gate、queue full、Action result |
| 一直 executing 0% | Action feedback、Gazebo clock | 检查仿真时钟和 executor 终态 |
| Gazebo/RViz 已出现但小车不动 | `runtime.log` 的 `session phase`、`WAIT: system readiness` | 必须先进入 `mapping`/`automatic_mapping`；若缺 `agent/action_guard`，看 Lifecycle configure/activate。验收会隔离外部 Nav2 overlay，禁止手工 source `~/nav2_ws` 绕过 |
| 真人离线入口没有出现界面 | `runtime.log` 最前面的 llama/model 路径 | launch 前模型服务失败会立即退出；确认 `EMBODIED_RUNTIME_ROOT` 指向含 `third_party/llama.cpp` 和 `models/Qwen3-0.6B-Q8_0.gguf` 的主工作区 |
| 探索不结束 | typed frontier evidence | 区分 active、reachable、blacklisted、`frontier_progress_stalled_recoverable` 和 provider/mission reason |
| 机器人反复卡在窄缝 | runtime goal/pose、诊断地图 | 检查 0.33m traversal clearance；不要误改成会切断整域的 0.40m goal clearance |
| 地图只覆盖局部 | schema v4 map_quality | 看总体/分区 coverage 与 reachable unknown，不看图片主观判断 |
| AMCL 看似正常但精度失败 | localization metrics | 看时间对齐样本、Gazebo truth callback 和 map/world 固定变换 |
| 目标被拒绝 | sampled_navigation | 看目标是否 known-free、plan 是否穿 unknown/occupied/map 外 |
| 取消后仍可能运动 | mission outcome、Nav2 result、typed STOP | 检查 cancel→fresh STOP→Nav2 terminal；任一步无终态都应停止 stage 并 FAILED |
| 重型任务似乎卡住 | `runtime.log`、心跳、session manifest | 有心跳继续等；无心跳再按阶段定位并确认清理结果 |

启动阶段如果子进程进入 `FAILED`，探针会立即携 `last_state_detail` 落失败报告；用户中断返回标准退出码
130，不再打印 `ExternalShutdownException` traceback。synthetic mock 冷启动的 readiness 上限为 45 秒，
正常情况下约数秒进入 `MAPPING`；真人离线模式还需等待一次模型 warmup。

进入 E2E 探针后的运行期失败仍应落盘带 `error`、最后业务状态和最后观测速度的报告。
只有成功报告才要求 `final_cmd_vel_zero=true`；失败报告中的速度是诊断事实，不是安全停车证明。若 workspace doctor
或依赖 preflight 在 probe 启动前失败，应直接非零退出，而不是伪造 session PASS。

## 9. PR 与发布边界

PR 必须写明测试命令、证据路径、已知限制和 interface schema 是否变化：

```text
feature/* → dev → main → version tag
```

`main` 只接收稳定里程碑；完整功能完成后再 push 触发 CI。CI 不强依赖模型、麦克风或 Gazebo GUI，
重型本机证据用于补充，但不能被 stage/mock 替代。
