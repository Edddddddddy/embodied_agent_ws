# 多模态演示开发计划

## 1. 目标与边界

目标是在同一个持续运行的 Gazebo/RViz 场景中，分别演示并最终串联：

1. 未知环境前沿探索、SLAM 建图、返航和本会话地图保存。
2. AMCL 定位、Nav2 目标点规划、动态障碍重规划与安全停车。
3. 离线/在线自然语言控制：单命令、多命令队列、急停和会话反馈。
4. 键盘控制：W/S/A/D、deadman、接管、显式恢复和退出。

演示仍使用三条 typed ROS 2 边界：

- `ManageSlamSession.action`：建图、存图、定位和导航会话。
- `ExecuteRobotCommand.action`：可反馈、可取消的机器人动作。
- `SetControlAuthority.srv`：AUTONOMY、KEYBOARD、HOLD、ESTOP 控制权。

本轮不实现真实 UART/SPI，不复制第二套 SLAM/Nav2 状态机，不让键盘或 Agent
绕过 ActionGuard、控制权门或 Collision Monitor。

## 2. 当前架构

```text
麦克风 -> VAD/ASR -> NLU/队列 -> typed Action ----+
                                                   |
键盘 -> KeyboardTeleop -> typed authority --------+----> 会话编排
                                                   |
SLAM Toolbox/Explore <-> SessionOrchestrator <-> AMCL/Nav2
                                                   |
voice primitive/Nav2 -> twist_mux ----+            |
keyboard ---------------------------->+-> AuthorityGate
                                           -> Collision Monitor -> /cmd_vel -> Gazebo
```

所有权保持单一：

- `SessionOrchestratorNode` 是建图→导航的唯一业务状态机。
- 会话根持有唯一 authority manager。
- persistent base 持有 Gazebo、机器人、Agent、RViz、Nav2 common、typed bridge
  和速度安全管线。
- mapping stage 只持有 SLAM provider/executor；navigation stage 只持有本次地图的
  map server、AMCL、Nav2 executor 和动态障碍 tracker。
- `StageProcessManager` 负责 base/stage/explorer/map-saver 进程树，不把子阶段
  异常扩散成隐式重启整个世界。

阶段切换采用 fail-closed 事务：

```text
HOLD
 -> 旧 Explore/Nav2 terminal
 -> 同代 priority STOP result
 -> STOP 后新鲜零速
 -> 停旧 stage
 -> 启新 stage
 -> lifecycle ACTIVE
 -> /initialpose 匹配订阅并发布
 -> 本 navigation generation 新鲜 /map 与 /amcl_pose
 -> RESUME_AUTONOMY
```

任一步超时、进程退出、manager epoch 改变、KEYBOARD 或 ESTOP 都不得自动恢复。

### 本轮运行时重构（已通过重型验收）

本轮比较了三个不同深度的 seam：

1. 只提取 `Nav2MotionTransaction`：先把 Nav2 goal 从发送、迟到接受、取消、
   priority STOP 到可读 terminal wrapper 的完整安全事务收进一个深模块。
2. 建立 `DefaultDemoSession`：长期由一个会话 facade 拥有默认演示的启动、命令
   准入、提交、状态快照和关闭，ROS Node 只做 Adapter。
3. 建立通用 `MissionProgram`/phase DSL：用 phase registry 组合 mapping、
   navigation 和未来 demo。

当前已完成第 1 项。它直接收口重复最多、失败代价最高的 Nav2 运动事务，同时不改变
任何 ROS topic、Action、Service 或 evidence schema。代码锚点是：

```text
src/embodied_slam_tools/embodied_slam_tools/nav2_motion_transaction.py
src/embodied_slam_tools/embodied_slam_tools/nav2_motion_ros.py
src/embodied_slam_tools/test/test_nav2_motion_transaction.py
src/embodied_slam_tools/test/test_nav2_motion_ros.py
```

三个 Node caller 已迁移到同一 `execute(intent, ...)` seam，旧 Future/cleanup 私有
状态机及其白盒测试已删除。`DefaultDemoSession` 是长期方向，预期锚点为
`src/embodied_slam_tools/embodied_slam_tools/default_demo_session.py`。

暂缓 `MissionProgram`/phase DSL。当前只有 mapping 与 navigation 两个已经具有独立
不变量、恢复和证据语义的深 phase；为了一个尚未成形的第三 phase 提前公开 registry、
artifact 和 effect 类型，会增加调用者必须学习的 Interface，却还没有足够 Leverage。
等 demo phase 具有自己的事务、证据和恢复语义后再重新评估。

## 3. 版本与分支顺序

| 顺序 | 分支/版本 | 状态 | 完成条件 |
|---:|---|---|---|
| 0 | `main@v0.5.0` | 稳定基线 | strict unknown-world 发布门禁已通过 |
| 1 | `feature/demo-control-plane` | PR #89 已合入 `dev` | 控制权、键盘、mux/Gate、急停 |
| 2 | `feature/demo-persistent-session` | PR #92 已合入 `dev` | fresh heavy PASS、CI 全绿 |
| 3 | `refactor/repository-surface-cleanup` | PR #93 已合入 `dev` | 公开入口与文档表面已收口 |
| 4 | `refactor/showcase-session-runtime` | 本地闭环，待 PR/CI | 事务、Adapter、core、Gazebo 与 unknown-world E2E 已通过 |
| 5 | `feature/showcase-unified-entry` | 待创建 | 一个公开启动/状态/停止入口 |
| 6 | `feature/showcase-multimodal-handoff` | 待创建 | 语音、键盘、自治之间的接管闭环 |
| 7 | `feature/showcase-demo-profiles` | 待创建 | quick/strict 配置、证据和讲稿 |
| 8 | `dev -> main` | 待发布 | 集成 CI 与演示前人工门禁通过，打 `v0.6-multimodal-showcase` |

不得在当前分支未 PR 合入 `dev` 前创建下一功能分支。每个分支完成本地闭环后只
push 一次，创建 PR 到 `dev`；`main` 不接收未集成的 feature 分支或直接推送。

## 4. 当前分支完成定义

### 继承的已实现基线

- base/mapping/navigation launch 按进程所有权拆分。
- typed bridge 与 stage executor 关闭自启动/内置 manager，由编排器这个唯一
  Lifecycle 所有者读取状态并驱动到 ACTIVE。
- base 或 stage 提前退出时启动流程立即 fail-closed，释放排队 Action 并清理。
- AMCL 初始化由编排器完成：
  `AMCL ACTIVE -> /initialpose 订阅匹配 -> 发布返航终点 -> 当前代 /amcl_pose`。
- detached 子进程由 PID/starttime/PGID 校验后的 owned-process-tree 清理。
- runtime continuity 绑定实际 `offline_agent`/`online_agent` 进程身份，并从原始
  checkpoint 重新计算，不能复用缓存 PASS。

以下是 PR #92 的继承基线，不是当前事务重构结果：

```text
Python/仓库/集成组合测试：717 passed
embodied_slam_tools + embodied_simulation：最新 692 tests，0 failures
git diff --check / compileall / colcon build：PASS
```

当前事务分支已确认：

```text
embodied_slam_tools：411 passed
repository contracts：316 passed
事务 + ROS Adapter + Node seam：145 passed
embodied_slam_tools ROS 包测试：411 tests，0 failures
acceptance_test.sh core：547 repository/evaluation + 660 Agent tests，
                        C++/simulation 门禁 PASS
typed Gazebo Action -> cmd_vel -> odom -> terminal：PASS
```

这些结果证明接口、局部 ROS 行为和 typed Gazebo 运动闭环。clean commit
`bc65b8f` 又通过本分支独立的 unknown-world 长时门禁；详细失败闭环和指标见
[工程日志](ENGINEERING_LOG.md)。

### 本轮重构完成门槛

`Nav2MotionTransaction` 的完成定义如下：

1. 唯一行为 Interface 收紧为
   `execute(intent: SampledNavigate | MappingReturn | RecoveryBackup, *,`
   `is_cancelled, deadline_monotonic) -> None`；调用者不再了解 goal-response
   future、迟到接受或安全清理的内部次序。preflight、ROS `/plan` 采集与评分仍留在
   Node；事务读取同一个 ledger 并执行运行期安全门，不改变证据 owner。
2. 正常成功、拒绝、执行失败、用户取消、goal-response timeout、late accept、
   STOP 失败、terminal wrapper 缺失和 stage fail-safe 都有接口级回归。
3. 取消仍严格满足
   `cancel -> 独立 priority STOP -> fresh zero -> readable Nav2 terminal`；
   主错误不被 cleanup 错误覆盖。
4. `showcase_session_node.py` 的 ROS 名称、QoS、Action result/feedback 与三类 typed
   evidence 字段保持不变。
5. 包测试、repository contracts、`acceptance_test.sh core` 通过；合入或发布前再在
   clean commit 上跑 unknown-world 重型 E2E。

事务使用 `Nav2GoalRejected`、`Nav2MotionFailed`、`Nav2SafetyFailure` 和
`Nav2TransactionBusy` 区分失败；server 不可用和业务 deadline 耗尽使用
`TimeoutError`，用户取消继续使用 `AutomaticMissionCancelled`。成功返回 `None`，
不能靠一个含糊布尔值压平终态语义。

五项门槛均已通过；当前分支已完成本地闭环，下一步是 PR 与 CI。

### 继承的 PR #92 fresh 重型证据

session `20260724T053935Z-1431080-d6efcab6` 完整运行 `1515 s`，结果：

| 门禁 | 结果 |
|---|---:|
| reachable free coverage | `0.998` |
| 最弱区域 coverage | `0.985` |
| mapping path | `147.661 m` |
| frontier goals | `32` |
| AMCL P95 | `0.125 m` |
| sampled Nav2 goals | `3 / 3` |
| dynamic replan | PASS |
| runtime continuity | PASS |
| STOP 后最终新鲜零速 | PASS |

本地证据目录（`logs/` 被 Git 忽略，不把它伪装成 GitHub 可点击链接）：

```text
logs/acceptance/showcase_gazebo_e2e/20260724T053935Z-1431080-d6efcab6/
```

其中 `showcase_gazebo_e2e_report.json` 与 `acceptance_session.json` 记录了
顶层 `passed=true`、cleanup 完成以及 source revision/dirty provenance。

该 persistent 会话基线已经由 PR #92 与 CI 验证并合入 `dev`，但它不构成本轮
运行时事务重构的重型证据。本轮已使用 clean commit `bc65b8f` 独立通过
unknown-world E2E；后续仍需经过 PR/CI，再决定单独发布该纵向切片还是继续完成
统一演示入口。

## 5. 证据边界

2026-07-21 的 strict 基线：

```text
session: 20260721T072342Z-2344751-5452a492
elapsed: 1092 s
coverage: 0.998
minimum region: 0.988
frontier goals: 32 accepted / 32 terminal
return xy error: 0.019 m
AMCL P95: 0.120 m
sampled Nav2 goals: 3 / 3
dynamic obstacle replan: PASS
final fresh zero velocity: PASS
```

它使用 mock provider，且报告 schema 不含 persistent runtime continuity。用途仅是
证明修改前 strict 业务链路基线；不得写成“本分支持久会话 PASS”，也不得写成
“真实语音 PASS”。

当前分支继承的 PR #92 persistent 基线：

```text
session: 20260724T053935Z-1431080-d6efcab6
elapsed: 1515 s
coverage: 0.998
minimum region: 0.985
mapping path: 147.661 m
frontier goals: 32
AMCL P95: 0.125 m
sampled Nav2 goals: 3 / 3
dynamic obstacle replan: PASS
runtime continuity: PASS
final fresh zero velocity: PASS
source dirty: true
```

它证明 PR #92 当时的 dirty feature worktree 在真实 Gazebo/Nav2 中跨阶段连续，
不是当前事务分支的新证据，也不证明真实麦克风体验。

证据分层：

| 层级 | 输入/运行时 | 证明内容 |
|---|---|---|
| 单元/仓库 | fake/纯函数 | 状态机、解析、进程所有权、schema |
| ROS stage | 真实 ROS graph、无完整世界 | lifecycle、Action、QoS、控制权 |
| persistent heavy | synthetic/mock + 真实 Gazebo/Nav2 | 物理闭环和跨阶段进程连续性 |
| live offline/online | 真实麦克风 + Gazebo | endpoint、ASR、Agent 与仿真联合体验 |

后一级不能被前一级替代。

## 6. 后续统一演示接口

下一分支计划提供薄入口，底层仍复用现有 typed 深模块：

```bash
bash scripts/demo.sh run offline --profile quick
bash scripts/demo.sh run online --profile quick
bash scripts/demo.sh keyboard
bash scripts/demo.sh status
bash scripts/demo.sh stop
```

薄 shell 的长期内部 seam 是 `DefaultDemoSession`，而不是 shell 自己拼接
`save_map()`、`start_navigation()` 或 Nav2 goal loop。目标 Interface 为：

```text
start() -> None
admission(SessionRequest) -> Admission
submit(SessionRequest) -> SessionOperation
snapshot() -> SessionSnapshot
close() -> None
```

`SessionOperation` 只提供完成状态、进度、取消和有界等待。普通调用者使用
`SessionRequest.default_demo(source)` 或 `SessionRequest.stop(source)`，不学习存图、
readiness、navigation generation 和目标循环。该 facade 尚未实现；要等本轮
`Nav2MotionTransaction` 的 Interface 稳定并完成重型回归后再进入实现计划。

四个独立演示应可单独运行：SLAM、Nav2、语音/NLU、键盘；完整演示再复用同一
Gazebo 实例串联，不启动第二台机器人。

建议 15 分钟流程：

1. 冷启动与 readiness（1 分钟）。
2. 键盘移动、deadman 和恢复（1 分钟）。
3. 离线/在线语音多命令（2 分钟）。
4. 自动探索、地图增长、接管后继续（4 分钟）。
5. 返航、存图、AMCL 初始化与目标导航（4 分钟）。
6. 动态障碍、急停、最终零速和 evidence（3 分钟）。

`quick` 只服务现场时长；发布质量仍使用 strict profile。若使用预验证地图降级，
报告必须标记 `degraded=true`、`same_session=false`，不能冒充本次实时建图。
