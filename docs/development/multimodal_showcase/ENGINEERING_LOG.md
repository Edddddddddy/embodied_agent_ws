# 多模态演示工程日志

## 2026-07-21：strict unknown-world 用户验收基线

session：`20260721T072342Z-2344751-5452a492`

报告：

```text
/home/ubuntu/embodied_agent_ws_worktrees/voice-unknown-world-e2e/
  logs/acceptance/unknown_world_slam_nav/
  20260721T072342Z-2344751-5452a492/unknown_world_slam_e2e_report.json
```

可核验结果：

| 指标 | 结果 |
|---|---:|
| 运行时间 | 1092 s |
| reachable free coverage | 0.9981426 |
| 最弱区域 coverage | 0.9875519 |
| frontier accepted/terminal | 32 / 32 |
| 返航位置误差 | 0.0188 m |
| AMCL P95 位置误差 | 0.1196 m |
| 采样 Nav2 目标成功 | 3 / 3 |
| 动态障碍重规划 | PASS |
| STOP 后最终新鲜零速 | PASS |

证据边界：该 session 使用 mock provider，产生于 persistent base 开发之前，报告
不含 `runtime_continuity`。它只证明 strict SLAM→存图→定位→规划→动态重规划
业务链路，不证明本轮跨阶段进程连续性，也不证明真实麦克风。

## 2026-07-23：发布基线与控制平面

- unknown-world 稳定性修复在 PR #87 收口，没有降低原 `40 cells / 0.002`
  地图增益门槛。
- 在干净 detached worktree 的 exact revision 上运行发布 strict E2E：
  总覆盖率 `0.999`、最弱区域 `0.992`、路径 `160.543 m`、39 个 frontier
  goal、AMCL P95 `0.185 m`、3 个目标和动态重规划通过，cleanup 完成。
- PR #88 合入 `main`，merge commit 为
  `4125d4b5fba59fdb5f44700357cd8e32e4ffda16`，发布标签 `v0.5.0`。
- 控制平面独立开发：typed authority、C++ 键盘、自治 mux、AuthorityGate、
  manager epoch/lease、同代 quiescence ACK、急停锁存和 deadman。
- Python/C++ 对同 epoch 断租统一为 sticky fail-closed；RESUME 后先完成零速
  source-ready 握手，旧非零 DDS 帧不能复驶。
- 低内存门禁使用精确包选择和单线程构建；控制权 stage、在线/离线多命令回归通过。

## 2026-07-24：控制平面合并与 persistent 纵向切片

版本状态：

- PR #89 已合入 `dev`，merge commit：
  `2d91ffe49c996c08fd2aaf7bc4e4395533a3171d`。
- Issue #90 跟踪持久 mapping→navigation：
  <https://github.com/Edddddddddy/embodied_agent_ws/issues/90>。
- `feature/demo-persistent-session` 后续通过 PR #92 合入 `dev`，merge commit 为
  `3769150864c8e270442043110472f59fba5b7295`；`main` 继续保持 `v0.5.0`。

实现：

- 新增 persistent base、mapping stage、navigation stage 三层 launch。
- `StageProcessManager` 分离 base/stage/explorer/map-saver 所有权。
- `SessionOrchestratorNode` 以 opt-in persistent mode 执行
  HOLD→typed quiescence→stage switch→readiness→RESUME。
- session Nav2 参数在 base 前原子生成一次；navigation 只加载本次保存的地图。
- `showcase-gazebo-e2e` 复用 strict evaluator 并追加运行时连续性证据。

## 2026-07-24：重型门禁失败闭环

以下是实际失败，不是推测；每一项都新增了对应防回归测试。

### 1. Gazebo 运行时身份未找到

- 现象：第一次 heavy run 在约 5 秒内失败。
- 根因：Gazebo Harmonic Ruby wrapper 把 `gz sim -r -s ...` 暴露成不同于预期的
  `/proc/<pid>/cmdline` 形态。
- 修复：identity selector 兼容该 argv 形式，同时保留
  PID/start ticks/boot ID/executable/argv hash 校验，不退化为进程名匹配。

### 2. 长会话最终零速缺失

- 现象：第二次运行约 12 分钟，真实完成 `84.82 m` 和 23 个 frontier goal，
  最后却没有 STOP 边界之后的新鲜 `/cmd_vel=0`。
- 根因：Collision Monitor 默认 `stop_pub_timeout` 约 2 秒；机器人长时间静止后，
  上游虽然继续发零速，最终出口会抑制重复零样本。
- 修复：仅 persistent base 将零速心跳窗口设为可配置长会话值；Collision Monitor
  仍是唯一最终出口，STOP generation 与雷达安全门槛不变。

### 3. hard budget 被恢复原因遮蔽

- 现象：后续运行约 917 秒、`116.1 m`、31 个 goal 后失败；frontier
  `attempts_exhausted` 路径继续走恢复，掩盖了已达到硬时间预算。
- 根因：恢复原因的优先级高于 hard budget，造成有界任务变成额外长等待。
- 修复：`frontier_monitor.py` 先裁决硬预算，再解释可恢复原因；新增对应单测。

### 4. typed bridge 冷启动竞态

- 现象：最新一次运行约 49 秒、进入 MAPPING 前失败；
  persistent base code=1，typed Action bridge 未 ACTIVE。
- 根因：`LifecycleNode(autostart=True)` 通过 volatile transition event 串联
  configure/activate；冷启动时 service discovery 可能早于 topic matching，
  configure event 丢失后节点会停在 INACTIVE。
- 修复：
  - persistent 模式关闭 bridge 自启动；
  - orchestrator 成为唯一 Lifecycle 所有者，主动读取状态并驱动
    configure/activate；
  - service response 丢失时重新读取真实状态，不把未知响应冒充成功；
  - 等待期间持续检查 base liveness；
  - 任何提前退出都会拒绝新 goal、释放排队 goal、关闭会话和回收进程。

### 5. `/initialpose` 盲发导致假定位 readiness

- 现象：旧脚本在 navigation launch 前后台运行，等待固定时间后即使没有
  subscriber 也发布并退出 0。
- 根因：生产者“调用 publish”被误当成 AMCL 已消费并生成新定位。
- 修复：初始化事务移入 orchestrator，依次等待 AMCL ACTIVE、订阅匹配，发布本次
  返航终点，再等待当前 generation 的新鲜 `/amcl_pose`。独立 helper 没有
  subscriber 时现在非零退出。

### 6. launch wrapper 退出后遗留 detached child

- 现象：stage wrapper 已退出，AMCL/Gazebo 等后代仍可能留在 ROS graph，污染下次
  cold start。
- 根因：旧实现只信任根 PID/进程组；wrapper 生命周期不等于所有后代生命周期。
- 修复：运行期间记录 owned process tree，清理前校验 PID starttime 与 PGID；
  wrapper 已退出仍回收已登记后代，PID 复用时拒绝误杀。

### 7. runtime continuity 证据过弱

- 现象：只比较 launch parent 或缓存的 PASS，无法证明实际 Agent 未重启。
- 根因：证据 schema 没有约束 role/label/时间顺序，verifier 也未始终从原始
  checkpoint 重算。
- 修复：绑定实际 `offline_agent`/`online_agent`，要求 schema v1、精确 role、
  `mapping_ready`/`navigation_ready`、严格递增 wall time 和完整进程身份；验证器
  拒绝 cached PASS。

## 2026-07-24：fresh 重跑链——从失败到闭环

这一轮不是靠反复延长 timeout 得到 PASS。每次都先读取本次 session 的第一处致命
证据，再修正对应所有权或边界，并增加回归测试。下面按发生顺序保留最短、但足以
解释上下文的记录。

### `20260724T040928Z`：Python shebang 进程没有被识别

- 现象：会话刚开始做运行时连续性采样，就报告找不到 Agent。
- 根因：Agent 是带 shebang 的可执行 Python 入口，内核可直接执行脚本；
  `/proc/<pid>/cmdline` 不一定含有 `python3`。旧选择器把“argv 有 Python
  解释器”误当成 Python 进程的必要条件。
- 设计决策：在
  [`runtime_continuity.py`](../../../tools/acceptance/runtime_continuity.py)
  同时识别解释器启动与可信 shebang 入口，身份验证仍保留 PID、start ticks、
  boot ID、executable 和 argv hash，不退化为模糊进程名。
- 验证：新增 shebang Agent selector 回归后，后续 session 能取得
  `mapping_ready` 与 `navigation_ready` 两个 checkpoint。

### `20260724T041407Z`：有界探索到期，但最近仍有有效增益

- 现象：地图已经接近完成，硬预算到期时最近 epoch 仍有实质增益；立即结束可能
  过早，继续普通恢复则可能无界等待。
- 根因：完成判定只有“现在停止”与“继续恢复”两个分支，没有一个有上限的最终
  复核步骤。
- 设计决策：在
  [`mission_executor.py`](../../../src/embodied_slam_tools/embodied_slam_tools/mission_executor.py)
  增加一次最多 `240 s` 的最终确认机会；仅当近期历史仍有实质增益时触发，
  确认后仍必须通过连续低收益、剩余 frontier、最终 probe、地图质量、返航和
  typed STOP。
- 后续语义修订：hard-budget 的“剩余 frontier”只保留 pre-probe typed 诊断，
  不再使用 raw cluster 绝对上限；repeated-stall 仍要求 residual 为零。详见本日志
  `2026-07-25：未知环境 hard-budget 残余 frontier 假阴性`。
- 验证：最终 fresh session 的确认 probe 增益为 0，系统按
  `bounded_saturation` 正常收口，没有降低 coverage 阈值。

### `20260724T044459Z`：Nav2 收到小写 `false`

- 现象：建图、返航、存图已经完成，切到导航阶段后没有 map server/AMCL。
- 根因：launch 把字符串 `false` 传给 Nav2 的
  `PythonExpression(['not ', use_composition])`，Python 求值时报
  `name 'false' is not defined`。
- 设计决策：在
  [`persistent_navigation_stage.launch.py`](../../../src/embodied_simulation/launch/persistent_navigation_stage.launch.py)
  的边界把布尔参数统一为 Python 字面量 `True`/`False`，而不是要求下游猜测
  Bash 字符串语义。
- 验证：真实 20 秒 launch probe 观察到 map server、AMCL 和生命周期管理器
  启动并激活；对应 launch contract 测试通过。

### `20260724T051736Z`：typed bridge 自启动事件竞态

- 现象：typed bridge 完成 configure，却没有 activate，mapping stage 因 Action
  server 不可用而停止。
- 根因：自启动依赖 volatile `/transition_event`；service 已发现并不代表订阅已
  匹配，configure event 可能在冷启动窗口丢失。
- 设计决策：关闭 persistent typed bridge 自启动，由
  [`showcase_session_node.py`](../../../src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py)
  的 `SessionOrchestratorNode` 读取真实状态并驱动
  `configure -> activate`。响应未知时重新读状态；只有状态实际前进才继续。
- 验证：定向测试覆盖正常迁移、响应丢失但状态已前进，以及响应持续未知且状态不
  前进三种情况。

### `20260724T052728Z`：为验证启动链而人工停止

- 现象：该 session 已经进入 frontier exploration，但没有生成最终报告。
- 原因：这是一次有意停止的短探针，只用来确认上一步 bridge 修复已越过启动阶段；
  随后先补上“response unknown 必须重读状态”的防回归逻辑，避免再跑二十多分钟
  才发现边界漏洞。
- 结论：它不是失败证据，也不是 PASS 证据；工程记录明确标注为人工终止，不能与
  最终 fresh session 混用。

### `20260724T053240Z`：stage 内第二个 Lifecycle 所有者

- 现象：typed bridge 已 ACTIVE、mapping stage 已启动，但
  `simulation_control` 停在 INACTIVE；日志出现
  `change_state response timeout`。
- 根因：stage 内 Nav2 lifecycle manager 与 orchestrator 同时改变
  `simulation_control` 状态，形成第二个所有者。manager 等不到 response 后挂起，
  readiness 只看到节点存在，看不到 ACTIVE。
- 设计决策：
  - persistent mapping/navigation stage 关闭内部 lifecycle manager；
  - `simulation_control` 使用 `autostart=false`；
  - `SessionOrchestratorNode` 复用同一个 lifecycle 收敛函数，先确认 stage owner
    存活，再把 executor 驱动到 ACTIVE，之后才运行 SystemReadiness。
- 验证：launch contract、状态迁移顺序和两个 ROS 包测试通过；至此 typed bridge
  与 stage executor 都由同一个业务状态机拥有。

### `20260724T053935Z-1431080-d6efcab6`：最终 fresh PASS

修复后的权威命令完整运行 `1515 s`，同一个 Gazebo、机器人状态发布器和
`offline_agent` 从建图持续到导航，没有重启。结果如下：

| 指标 | 结果 |
|---|---:|
| reachable free coverage | `0.998` |
| 最弱区域 coverage | `0.985` |
| 建图路径 | `147.661 m` |
| frontier goal | `32` |
| AMCL P95 位置误差 | `0.125 m` |
| 采样 Nav2 目标 | `3 / 3` |
| 动态障碍重规划 | PASS |
| runtime continuity | PASS |
| STOP 后最终新鲜零速 | PASS |

报告的顶层 `passed=true`，所有发布门禁检查均为 true；其中探索以
`bounded_saturation` 合法完成，地图保存、返航、定位、规划、动态重规划、实际
运动和最终停车均有本 session 的新鲜证据。本地事实源位于：

```text
logs/acceptance/showcase_gazebo_e2e/20260724T053935Z-1431080-d6efcab6/
├── showcase_gazebo_e2e_report.json
├── acceptance_session.json
└── runtime.log
```

`logs/` 是被 Git 忽略的运行产物，因此这里保留可复现路径，而不提供推送后会失效
的 Markdown 链接。

证据边界：报告绑定 revision `2d91ffe49c996c08fd2aaf7bc4e4395533a3171d`
且记录 `source_dirty=true`。它证明当时 feature worktree 的实现已经闭环；同一
实现随后通过 PR #92 的完整 CI 并进入 `dev`。它仍不能替代 clean commit 上的
`main` 发布级重型证据。

## 2026-07-24：提交前本地门禁

已运行：

```text
pytest（slam tools + repository + integration）：717 passed
colcon build --packages-select embodied_slam_tools embodied_simulation：PASS
最新 colcon test：692 tests，0 errors，0 failures，0 skipped
stage process manager 定向测试：16 passed
compileall：PASS
git diff --check：PASS
```

这些门禁覆盖 fail-close、Lifecycle、AMCL generation、孤儿进程与 evidence
schema；完整真实 Gazebo 长时门禁也已由上述 fresh session 覆盖。复现入口：

```bash
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh
HEADLESS=true USE_RVIZ=false SLAM_NAV_PROGRESS_HEARTBEAT_S=15 \
  bash scripts/acceptance_test.sh showcase-gazebo-e2e
```

PR #92 已完成上述流程并合入 `dev`。下一步在独立
`refactor/repository-surface-cleanup` 分支删除有明确替代者的旧入口、补齐门禁和
文档导航；不在清理分支重写已通过的 SLAM 状态机。

## 2026-07-24：会话运行时三方案比较与本轮决策

仓库表面收口后，下一处明显风险是
`showcase_session_node.py` 同时拥有 ROS Adapter、Nav2 goal 生命周期、安全清理和
整场 mission 编排。只按文件大小拆分会产生很多浅 Interface，因此本轮先比较 seam，
再决定代码改动。

### 方案 A：`Nav2MotionTransaction`

范围是一笔 Nav2 运动：发送 goal、读取响应、处理拒绝或迟到接受、取消、独立
priority STOP、新鲜零速、读取 terminal wrapper 和最终 fail-safe。它不拥有地图
采样、候选 preflight、ROS `/plan` 采集/评分、mission phase 或 ROS wire schema；
但会读取 Node 写入的 ledger 并强制执行运行期路径安全门。

唯一行为 Interface 计划为：

```text
execute(
  intent: SampledNavigate | MappingReturn | RecoveryBackup,
  *,
  is_cancelled,
  deadline_monotonic,
) -> None
```

成功返回 `None`；错误按 `Nav2GoalRejected`、`Nav2MotionFailed`、
`Nav2SafetyFailure`、`Nav2TransactionBusy` 分类；server 不可用和业务 deadline
耗尽使用 `TimeoutError`，取消沿用 `AutomaticMissionCancelled`。

优点是改动集中，当前 sampled goal、返航 goal、恢复 BackUp 已经共享同一组安全
不变量；删除该模块后，这些复杂性会重新散回多个调用者，因而它能形成深模块。

预期代码锚点：

```text
src/embodied_slam_tools/embodied_slam_tools/nav2_motion_transaction.py
src/embodied_slam_tools/test/test_nav2_motion_transaction.py
```

### 方案 B：`DefaultDemoSession`

长期让一个会话 facade 向语音、typed Action 和统一演示入口提供同一 Interface：

```text
start() -> None
admission(SessionRequest) -> Admission
submit(SessionRequest) -> SessionOperation
snapshot() -> SessionSnapshot
close() -> None
```

`SessionOperation` 只暴露完成、进度、取消和有界等待。caller 不应直接调用
`save_map()`、`start_navigation()`、`wait_navigation_ready()` 或逐个 Nav2 goal
方法。预期长期锚点：

```text
src/embodied_slam_tools/embodied_slam_tools/default_demo_session.py
src/embodied_slam_tools/embodied_slam_tools/ros_demo_runtime_adapter.py
```

该方案能获得更大的 Locality，但当前一次迁移会同时触及启动、队列、控制权、证据和
关闭语义，风险高于先收口 Nav2 事务。

### 方案 C：`MissionProgram`/phase DSL

该方案用 phase registry、typed artifact 和 effect Adapter 组合 mapping、
navigation 与未来 demo。它对未来扩展最有上限，但现在只有 mapping/navigation
两个已经有独立事务深度的 phase。尚未出现第三个有自己的不变量、恢复和证据语义的
phase，此时建立 DSL 会先扩大 Interface，再等待未来需求证明其 Leverage。

### 当前决定和实现结果

本轮选择并已实现方案 A；方案 B 作为长期方向；方案 C 暂缓。新增：

```text
src/embodied_slam_tools/embodied_slam_tools/nav2_motion_transaction.py
src/embodied_slam_tools/embodied_slam_tools/nav2_motion_ros.py
src/embodied_slam_tools/test/test_nav2_motion_transaction.py
src/embodied_slam_tools/test/test_nav2_motion_ros.py
```

`run_recovery_backup()`、`run_navigation_goal()` 与
`run_mapping_return_goal()` 已迁移到同一个 `execute(intent, ...)` seam。ROS
`/plan` 的采集和 occupancy 评分仍由 Node 写入 ledger，事务负责执行期安全门；
恢复位移、返航 TF 容差和成功后的 typed STOP 仍由 Node 证明。原 Node 内部
goal-response、late accept、cancel/STOP/terminal 私有状态机及对应白盒测试已经
删除，不保留永久转发层。

本轮实现必须保持：

- 正常执行共享调用者的绝对 deadline；异常清理只创建一次不可续期的 cleanup
  deadline，late accept、cancel、STOP 与 terminal 共用该预算。
- 同时只有一笔事务；并发进入显式失败。
- pending cancel/timeout 必须同步收口迟到 accepted handle。
- accepted goal 故障严格执行
  `cancel -> 独立 priority STOP -> readable terminal`。
- terminal 不可证明时停止 navigation stage 并 fail-closed。
- quiescence 仅在明确拒绝或可读 terminal 后结算；sampled ledger terminal 恰好
  一次且晚于安全收口。
- primary error 不被 cleanup error 覆盖。
- ROS 名称、QoS、Action result/feedback 和 typed evidence 不变。

当前验证结果：

```text
embodied_slam_tools：411 passed
repository contracts：316 passed
事务 + ROS Adapter + Node seam：145 passed
embodied_slam_tools ROS 包：411 tests，0 failures
acceptance_test.sh core：547 repository/evaluation + 660 Agent tests，C++/simulation PASS
typed Gazebo Action -> cmd_vel -> odom -> terminal：PASS
```

`core` 首次曾报告 live benchmark 没有生成报告；直接运行后确认当前 worktree
只有 `--packages-up-to embodied_slam_tools` 的半安装层，缺少
`embodied_agent_core`。完成全 workspace build 后，原命令无代码修改即通过。
该经验已写入 `WSL_POWERSHELL.md`，避免把环境构建不完整误判成业务回归。

## 2026-07-24：clean commit 重型门禁与最终停车证据闭环

本轮第一次进入长时 unknown-world 收口时，Nav2 Action 和 typed STOP 都已有
terminal，但 verifier 没有收到 STOP 之后的新鲜 `/cmd_vel=0`。这不是机器人仍在
运动，而是 legacy `voice_nav2` 启动入口遗漏了 `stop_pub_timeout` 覆盖：Nav2 官方
Collision Monitor 默认只在停车后的约 `2 s` 内继续转发零速，未知环境探索静止数十秒
后再发 STOP 时，控制面已经完成，最终速度边界却没有本次操作的新鲜见证。

修复没有放宽 verifier，而是统一速度出口的契约：

- legacy `voice_nav2` 与 persistent profile 都把 `stop_pub_timeout` 设为
  `86400 s`，覆盖整场长任务，保证晚到 STOP 仍能穿过 Collision Monitor 留下新鲜
  零速；
- 把 Nav2 `docking_server` 的速度 writer 从底盘 `/cmd_vel` remap 到
  `/control/docking/cmd_vel`，继续保持 Collision Monitor 是唯一最终 writer；
- 增加 launch contract 测试，防止旧入口再次漏掉长时零速心跳或恢复旁路 writer。

clean commit `bc65b8f` 随后完成独立重型验收：

```text
session: 20260724T143855Z-274112-d0619407
elapsed: 1032 s
reachable coverage: 0.997
minimum region coverage: 0.983
mapping path: 175.305 m
frontier goals: 33
AMCL P95: 0.160 m
sampled Nav2 goals: 3 / 3
dynamic obstacle replan: PASS
STOP 后 fresh final zero: PASS
```

这次证据同时证明事务重构没有破坏未知环境建图、返航、保存图、定位和规划闭环，
并验证了 Action terminal 与物理速度见证必须同时成立。ADR-016 至此完成重型验证。
未来实现 `DefaultDemoSession` 时，再增加
`RUN_DEFAULT -> MISSION_COMPLETED -> STOP -> STOPPED` 的会话级证据。

## 2026-07-25：公开 Gazebo 门禁隔离宿主 Nav2 ABI

`acceptance_test.sh gazebo` 曾能看到 `/clock`、`/odom` 和 `/scan`，却同时缺少
`/cmd_vel` publisher 与 `/robot/execute_command` Action server。launch 日志证明
三个 Lifecycle Manager 都错误地来自宿主 `/home/ubuntu/nav2_ws/install`，其
`libnav2_lifecycle_manager_core.so` 与当前 Jazzy `diagnostic_updater` ABI 不一致，
因此在 configure 前以 exit 127 退出。两个 readiness blocker 实际属于同一条
Lifecycle 配置链断裂。

修复让公开 Gazebo smoke 进入独立 `AcceptanceSession`，复用 unknown-world 门禁已经
验证的 `RosEnvironmentIsolation`：只保留当前 worktree、系统 ROS 和白名单 Frontier；
同一外部 workspace 的 `install` 与 `build` 路径会在 C++/Python 环境变量中一起清除。
readiness 失败时还会打印 launch 日志，避免以后只看到表层图谱 blocker。

本地连续两次运行原公开命令均通过，Action progress 约 `0.98`，结果为
`succeeded`，最终 ACK 为 `gazebo:velocity_zero`；session manifest 证明
`nav2_lifecycle_manager=/opt/ros/jazzy` 且 cleanup 无残留进程。

## 2026-07-25：未知环境 hard-budget 残余 frontier 假阴性

现场会话 `20260725T071407Z-72290-630add20` 在探索约 900 秒后被
`too_many_residual_frontiers` 单项否决：暂停 Explorer 时 `available=6`，旧配置绝对上限为 `4`。失败地图
经项目自己的 evaluator 复评，实际已达到 `99.78%` 可达覆盖、`98.55%` 最低分区覆盖，障碍召回和
false-free 也全部通过；路径为 `139.58m`，34 个 Frontier Action 均已终态。

根因不是“把 4 调得不够大”，而是证据语义错误：任务先暂停并停止 Explorer，再执行 final 360° probe。
final probe 会更新地图，但停止的 provider 不会重算 cluster，因此 residual 永远是 probe 之前的快照；
raw cluster 数本身也受墙角碎片影响，不等价于剩余信息量。

修复只调整 hard time-budget 的 bounded-saturation 语义：

- residual 继续写入 typed evidence，供诊断和 producer/evaluator 一致性复核；
- hard-budget 不再用陈旧绝对 cluster 数单项否决；
- repeated reachable stall 仍严格要求 residual 为零，evaluator 也按 trigger
  独立复核，不能只相信 producer 的 `valid` 位；
- 连续低收益 epoch、单位 terminal-goal 增益、建图里程、final probe、地图静默、Action 账本、新鲜 STOP、
  返航以及独立 `90/85/10` 地图质量门禁全部保留。

新增领域层和 MissionExecutor 调用层回归测试，精确覆盖 `pre-probe available=6` 的现场模式。修复后的
dirty-worktree 开发证据 `20260725T075144Z-136143-407995d3` 又完成一次无界面全链路：

```text
elapsed: 1022 s
completion: frontier_attempts_exhausted_below_material_gain
reachable coverage: 0.998
minimum region coverage: 0.984
mapping path: 167.963 m
frontier goals: 34
AMCL P95: 0.138 m
sampled Nav2 goals: 3 / 3
dynamic obstacle replan + final fresh zero: PASS
```

该次自然走 strict low-gain 路径，证明完整链路没有回归；hard-budget `available=6` 分支由上述两个确定性
红→绿测试覆盖。由于 `source_dirty=true`，它是提交前开发证据，不冒充 clean release evidence。

当前 typed schema 记录 final probe 的实际增益，但没有同时携带本次生效的 cells/ratio 阈值；因此
evaluator 能复核数值存在、trigger/residual、地图质量、返航和停车，细粒度收益阈值仍由 producer 的纯函数
assessment 保证。后续若扩展证据 schema，应把阈值及配置 provenance 一并传输，再由 evaluator 独立比较；
本轮不复制硬编码常量，避免生产与验收两套阈值漂移。

## 2026-07-25：GUI 长时建图资源耗尽与安全降级

用户使用 `HEADLESS=false USE_RVIZ=true` 时，地图扫描到后期出现“机器人不再移动、终端长期等待”，最终
WSL 非正常退出。失败 session `20260725T082402Z-200546-b46208ea` 的业务日志与上一 boot kernel journal
对齐后确认：

- 默认 OpenGL renderer 是 llvmpipe，Gazebo GUI 与 RViz 同时走 WSLg/Xwayland 软件渲染；
- 约 535 秒时 Xwayland 首次 page allocation failure；
- WSL 匿名内存约 7 GiB、可用内存约 80–100 MiB、2 GiB Swap 全部耗尽；
- 一秒后 `collision_monitor` 4 秒 bond 心跳丢失，Lifecycle Manager 关闭 Nav2；
- Explorer/探针没有感知该终态，manifest 留在 `running`，表现为“卡死”。

地图只有约 3.1 万栅格；观察器的轨迹列表在成功会话中也只有数万 tuple，数量级为 MiB。当前证据能确认
资源耗尽，但不能证明某一个 ROS 节点存在严格堆内存泄漏。

修复：

1. `visual_runtime.py` 检测 renderer；D3D12 可用时为本次子树注入驱动。长时双 GUI 在 8 GiB WSL 中
   自动降级为 Gazebo server + RViz。
2. `resource_watchdog.py` 每 5 秒流式写 JSONL，只在内存保留 latest/peak；连续低资源触发
   `resource_exhaustion` 并复用 AcceptanceSession 的停车/清理。
3. `runtime_log_health.py` 增量识别 Lifecycle `CRITICAL FAILURE`，立即报告
   `nav2_runtime_unhealthy`，不再等待 3690 秒 mission deadline。
4. StageProcessManager 的 `/proc` 子树观察从稳定期 100 Hz 降到 4 Hz；启动首秒仍高频捕获脱组进程。

用户原命令随后自动选择 D3D12 RViz-only，并生成 fresh 证据
`20260725T120302Z-145519-3ec3df78`：

```text
elapsed: 990 s
reachable / minimum-region coverage: 0.998 / 0.983
mapping path / frontier accepted-terminal: 153.403 m / 34-34
AMCL P95 / sampled goals: 0.154 m / 3-3
dynamic replan + final fresh zero: PASS
peak session RSS: 2056.5 MiB
final swap used: about 0.01%
resource failure / cleanup: null / complete
```

该次在旧故障时点之后仍无 Nav2 heartbeat failure，mapping stage 完成后 RSS 下降，支持“图形资源组合”
而非“地图持续泄漏”的根因结论。

## 下一轮安排

persistent 功能和 repository surface 已进入 `dev`，接下来顺序开发：

1. 将 `fix/gui-slam-runtime-stability` 通过 PR 合入 `dev`，由 CI 复核轻量门禁。
2. 在 clean commit 上按发布需要重跑一次 strict E2E；本地 dirty run 只作为开发证据。
3. 重新评估 `DefaultDemoSession`，不提前建立 phase DSL。
4. `feature/showcase-unified-entry`：统一 run/status/keyboard/stop 入口。
5. `feature/showcase-multimodal-handoff`：语音、键盘、自治任务接管与恢复。
6. 集成完成后由 `dev -> main` 发布下一阶段展示版本。
