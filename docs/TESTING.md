# 测试与验收

本文是当前项目的验收契约。命令注册以 `tools/acceptance/catalog.py` 为准；架构边界见
[ARCHITECTURE.md](ARCHITECTURE.md)，算法和代码走读见 [SLAM/Nav2 学习笔记](learning/SLAM_NAV2.md)。

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

本阶段新增 `FrontierExplorationEvidence.msg`、`SlamNavigationGoalEvidence.msg`，并扩展
`SlamSessionState.msg`，因此 ROS 2 interface type hash 已变化。切换分支后需全量重建所有依赖包；
旧 overlay、旧节点或旧 rosbag 不能与新 schema 混用。重建后 doctor 必须确认接口、Explore Lite 和
关键 package prefix 都来自当前 worktree。

重型场景使用 `AcceptanceSession` 租用 ROS domain/Gazebo partition，并在结束时回收本会话进程组。
只有需要复现固定 domain 时才设置 `SLAM_NAV_ROS_DOMAIN_ID`。

## 3. 公开入口

当前稳定公开入口共 8 个：

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh slam-nav-e2e
bash scripts/acceptance_test.sh unknown-world-slam-e2e
bash scripts/acceptance_test.sh robotics-gate
```

`--help-all` 展示内部回归和实验模式，不作为 README 主路径。

## 4. 合并前门禁

轻量门禁：

```bash
bash scripts/acceptance_test.sh core
pytest -q src/embodied_online_agent/test
bash scripts/acceptance_test.sh slam-autonomous-mission-stage
```

`core` 已聚合仓库/评估、离线 Agent、CLI 与核心 C++ 回归；这里只单独补它未覆盖的在线 Agent，避免同一
测试被手工重复运行。需要定位失败时再按 `core` 输出执行对应 pytest/colcon 子集。

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
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

这是“未知场地自主探索 → 本次地图 → AMCL/Nav2 → 运行时采样导航目标 → 动态避障”的正式入口。机器人策略
进程只获得在线 scan/odom/TF/map 和本次 mission；场景真值、区域边界、Gazebo pose 只传给 evaluator。
前三个导航目标必须由本次地图运行时采样。随后由 acceptance probe 注入的固定 dynamic challenge
goal/actor 只是可重复的外部测试刺激，不进入探索/采样算法，也不计作自主目标。

硬性 PASS：

- `mission_profile=UNKNOWN_WORLD`，`mission_sequence>0`，`mission_outcome=SUCCEEDED`，且阶段到达
  `MISSION_COMPLETED`；`phase` 表示运行阶段，不能替代独立终态 `mission_outcome`。
- YAML/PGM 晚于 session 开始，哈希和时间写入报告。
- 可达自由空间覆盖率 `>=90%`，每个可达区域 `>=85%`，可达区域 unknown 比例 `<=10%`。
- 障碍边界召回率 `>=60%`，真值障碍被误建为自由区的比例 `<=5%`。
- `FrontierExplorationEvidence.valid=true`；完成原因必须是 `no_frontiers + no_reachable_frontiers`，或
  `frontier_attempts_exhausted_recoverable + frontier_attempts_exhausted_no_map_gain`。后一种必须经过确认
  扫描并证明地图无新增信息；两种路径都要求 available/active/blacklisted 为 0，且所有 accepted goal
  都有 terminal 状态。
- AMCL 与 evaluator-only Gazebo truth 至少 20 个重叠时间范围、时间对齐且时间戳不重复的样本，位置误差
  P95 `<=0.25m`。
- 从本次已知自由连通区动态抽样至少 3 个目标；每个 typed goal 终态成功，producer 与 evaluator
  独立检查的全部 Nav2 plan 都没有 unknown、occupied 或 map 外采样点。
- Nav2 lifecycle active；动态障碍证据通过；报告须先证明本次发生过运动，再使用任务 terminal boundary
  之后收到的新鲜 `/cmd_vel` 证明线速度和角速度均为零。

强类型证据通过以下接口从生产侧发布，探针不解析 `detail` 日志猜状态：

- `SlamSessionState.msg`：profile、sequence、`mission_outcome`、frontier 和目标数组。
- `FrontierExplorationEvidence.msg`：frontier 计数、Action 生命周期和双层完成原因。
- `SlamNavigationGoalEvidence.msg`：目标、终态、plan 计数及 unknown/occupied/map-outside 统计。

证据路径：

```text
logs/acceptance/unknown_world_slam_nav/<session_id>/unknown_world_slam_e2e_report.json
logs/acceptance/unknown_world_slam_nav/<session_id>/runtime.log
logs/acceptance/unknown_world_slam_nav/<session_id>/unknown_world_map.{yaml,pgm}
logs/acceptance/unknown_world_slam_nav/<session_id>/acceptance_session.json
```

正式报告必须满足 `schema_version=4`、
`evidence_kind=unknown_world_slam_nav_dynamic_replan`、`passed=true` 和所有 checks 为 true。

### 5.1 取消与故障收口

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
回归单独证明；本轮系统集成已由 §5.2 fresh E2E 验证，后续发布仍需重生新会话证据。

### 5.2 当前修订版 fresh 实测

本地 session `20260720T031306Z-1114546-814430c3` 已完成六阶段真实 Gazebo 闭环：启动隔离 →
frontier 探索建图 → 存图并切换 AMCL → 3 个运行时采样导航目标 → 动态障碍重规划 → 终态停车与报告。

| 证据 | 实测值 | 门槛 |
| --- | ---: | ---: |
| 总体/最低区域可达自由覆盖率 | `99.6657% / office 97.7593%` | `>=90% / 各 >=85%` |
| reachable unknown | `0.3343%` | `<=10%` |
| 障碍边界召回 / false-free | `80.9322% / 0.2119%` | `>=60% / <=5%` |
| frontier available/active/blacklisted | `0 / 0 / 0` | 全部为 0 |
| accepted/terminal frontier goal | `20 / 20` | 全部有终态 |
| AMCL/Gazebo 对齐样本、位置误差 P95 | `246 / 0.154311m` | `>=20 / <=0.25m` |
| 运行时采样导航目标、最小间距 | `3/3 成功 / 5.570m` | `>=3 全成功 / >=1.50m` |
| producer/evaluator 路径检查 | 全部 known-free | unknown/occupied/map-outside 均为 0 |
| 动态重规划 / 终态速度 | PASS / 新鲜零速度 | 均须 PASS |

运行产物位于本机 `logs/acceptance/unknown_world_slam_nav/20260720T031306Z-1114546-814430c3/`，
`logs/` 不提交 Git；PR 只记录 session id、摘要和复现命令。该结果证明本次仿真 session，不外推为实体
硬件或真人语音准确率证据。

### 5.3 失败样本与安全复核

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

上述策略已通过补丁回放、单元/仓库门禁和本节 fresh E2E 联合验证；不替代每次发布前的新会话验收。

## 6. Known-world 回归与交互演示

```bash
bash scripts/acceptance_test.sh slam-nav-e2e
HEADLESS=false USE_RVIZ=true \
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
| 探索不结束 | typed frontier evidence | 区分 active、reachable、blacklisted、`frontier_progress_stalled_recoverable` 和 provider/mission reason |
| 机器人反复卡在窄缝 | runtime goal/pose、诊断地图 | 检查 0.33m traversal clearance；不要误改成会切断整域的 0.40m goal clearance |
| 地图只覆盖局部 | schema v4 map_quality | 看总体/分区 coverage 与 reachable unknown，不看图片主观判断 |
| AMCL 看似正常但精度失败 | localization metrics | 看时间对齐样本、Gazebo truth callback 和 map/world 固定变换 |
| 目标被拒绝 | sampled_navigation | 看目标是否 known-free、plan 是否穿 unknown/occupied/map 外 |
| 取消后仍可能运动 | mission outcome、Nav2 result、typed STOP | 检查 cancel→fresh STOP→Nav2 terminal；任一步无终态都应停止 stage 并 FAILED |
| 重型任务似乎卡住 | `runtime.log`、心跳、session manifest | 有心跳继续等；无心跳再按阶段定位并确认清理结果 |

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
