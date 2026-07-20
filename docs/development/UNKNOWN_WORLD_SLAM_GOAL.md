# Unknown-world SLAM/Nav2 修复 Goal

> 状态：**活动中（fresh E2E 与发布回归已 PASS，待 PR 与 CI）**
> 工作分支：`feature/unknown-world-slam-navigation`
> 稳定入口：`slam-nav-e2e`（known-world 回归）
> 本 Goal 入口：`unknown-world-slam-e2e`（unknown-world 严格验收）

## 1. 可验证目标

把当前依赖场景先验的演示升级为可审计的未知环境自主闭环：

```text
未知 Gazebo 场景
  → 激光/里程计/TF 在线探索
  → SLAM 建图与无增益收敛判定
  → 保存本次地图
  → AMCL 定位
  → 从本次已知自由连通区动态抽取目标
  → Nav2 规划、避障、重规划
  → 证据报告与安全停车
```

只有**基于当前候选代码新生成**的真实 Gazebo schema v4 报告满足 [测试手册 §5](../TESTING.md) 的全部硬门槛，
才允许把 Goal 标记为完成；该章节是阈值与实测值的唯一事实源。本 Goal 只保留不会随参数漂移的语义：

- robot policy 不读取静态真值、固定路线或语义坐标；真值只进入独立 evaluator。
- frontier 必须有合法双层完成原因和完整 accepted→terminal 生命周期，timeout 不等于收敛。
- 保存图、AMCL 与目标抽样来自同一 navigation generation；定位样本须时间范围重叠、时间对齐且不重复。
- 本次地图运行时抽样目标必须全部成功，producer/evaluator 路径均不得穿越
  unknown/occupied/map-outside；外部 dynamic challenge 不计作自主目标。
- 正常完成须先证明发生过运动，再用 terminal boundary 之后的新鲜零速证明停车。
- 取消须完成 `cancel → fresh typed priority STOP → Nav2 terminal`；任一步无法证明终态时必须 fail-safe
  停止 stage，并以 `MISSION_FAILED` 而不是 `MISSION_CANCELED` 收口。

## 2. 不可降低的边界

- evaluator 可以读取 Gazebo 真值；机器人探索、采样和规划策略不可以。
- 不用固定路线、场景坐标、真值地图或“已知哪里没扫完”来改善覆盖率。
- 不降低覆盖、定位、导航成功率或 unknown-path 门槛来制造 PASS。
- `slam-nav-e2e` 继续保留并诚实标记为 known-world 确定性回归，不冒充自主建图。
- Nav2 接受一个目标后，失败结果必须真实记为失败；不能静默换目标伪造“全部成功”。
- 取消不能仅以 cancel request 已发送为成功；任务锁在 Nav2 terminal 之前不得释放。
- 在线 ASR 静音 filler 成本问题继续由独立 issue 管理，不混入本 Goal。

如果只有引入场景先验或放宽上述门槛才能继续，停止开发并请求用户决策。

## 3. 精简且可扩展的架构

采用少量深 Module，避免把每个 topic 包成一个薄类：

| Module | 拥有的规则/状态 | 对外 Interface |
|---|---|---|
| `mission_executor.py` | 探索 epoch、恢复预算、地图增益门禁、阶段状态机 | 任务事件 → 明确决策 |
| `mapped_goal_sampler.py` | 已知自由连通区、净空、确定性候选排序 | 地图+起点 → 候选目标 |
| `showcase_session_node.py` | ROS 生命周期、Action 调用、消息适配 | 领域 Module 与 ROS/Nav2 的 Adapter |
| `unknown_world_evidence.py` | 覆盖、定位、路径、frontier、停车联合裁决 | 运行证据 → PASS/FAIL 报告 |
| `session_observer.py` 及三个证据 Module | 跨 topic 关联、时间对齐、代际隔离 | ROS 观测 → typed evidence |

设计规则：

- 纯策略写成无 ROS 依赖函数/类，能用小型 fixture 单测。
- ROS Node 只负责 wiring、时钟、QoS、Lifecycle 和 Action/Topic 适配。
- 只有存在两个真实实现时才增加 seam；不为未来假想需求预建空接口。
- 配置只表达可调策略，不承载场景坐标。
- typed msg/action 是主链路，不恢复旧 JSON 控制接口。

## 4. 已实现修复与证据边界

触发本轮修复的上一轮真实运行已完成 100% 地图覆盖和 3 个运行时导航目标抽样，但第二个目标在第一个目标执行后以 Nav2 `NO_VALID_PATH(208)` 失败。根因不是地图不完整，而是：

1. ROS 地图分辨率为 float32 近似值，旧净空计算遗漏临界栅格。
2. 目标过于贴近膨胀层，第一个目标结束后机器人落入不可重规划区域。
3. “静态可达”与“Nav2 当前代价地图可规划”之间缺少正式准入。
4. 目标采样应绑定导航阶段加载的权威保存地图，避免 SLAM 阶段旧消息串入。

本轮采用通用修复，没有加入场景特例：

- 用抗 float32 误差的栅格几何计算净空，并增加边界回归测试。
- 目标净空提高到覆盖机器人半径、栅格误差和定位误差的 `0.40 m`。
- 先生成过量、确定性候选；再用 Nav2 `ComputePathToPose` 做准入，准入后才登记恰好 3 个验收目标。
- `GridBased.allow_unknown=false`，规划约束与路径证据契约保持一致。
- 每个已登记目标执行前仍重新预检，避免准入与执行之间的时变竞态。
- 最终 frontier 耗尽必须经过一次确认扫描和地图增益判定；无增益才允许收敛。
- 地图稳定采用 soft quiet + hard budget：quiet 吸收尾部更新，绝对 deadline 不随地图增长延长，避免持续
  噪声或增长把等待变成无限续期。
- Nav2 取消采用 pending response 安全预算，并对已接受/迟到接受目标执行
  `cancel → fresh typed priority STOP → Nav2 terminal`；无法证明终态时停止 stage 并失败。
- frontier 正常完成可使用 `no_frontiers + no_reachable_frontiers`；尝试耗尽只能使用
  `frontier_attempts_exhausted_recoverable + frontier_attempts_exhausted_no_map_gain`，
  且两种路径都要求 available/active/blacklisted 为 0、accepted goal 全部已有终态。

session `20260719T235926Z-978092-3c6b24eb` 是**上一修订版**的代表性 schema v4 PASS，证明当时的六阶段
闭环。它早于本节新增的 hard budget 与取消安全事务，因此不能证明最新代码；当前发布候选证据已更新为 §4.3。

### 4.1 首轮 fresh E2E 的失败证据

最新代码已真实运行一次 unknown-world E2E：

```text
session: 20260720T013943Z-1054618-0887ac81
result:  FAIL（探索阶段 900 s hard budget）
reason:  available=7, active=1, blacklisted=3
map:     failed_exploration_map.yaml / failed_exploration_map.pgm
```

这次失败没有被包装成 PASS，严格 evaluator 正确阻止了保存地图与后续导航。诊断结果是：

- 全局可达覆盖率为 `90.72%`，但厨房只有 `25%`，违反“每个区域至少 85%”的硬门槛。
- 后半程机器人卡在办公桌与东墙之间约 `0.605 m` 的盲袋；13 个 goal 因 progress timeout 结束，
  此后远处厨房 frontier 虽被发现却无法到达。
- 当前 `0.25 m` frontier 净空只保证目标点附近可站立，没有为机器人半径、控制误差和退出通道留下足够余量。
- 旧进展判定只承认“到目标的直线距离下降”；合法横移或 U 型绕行会被误判为无进展。
- progress timeout 是某个 approach 的局部执行失败，不足以证明整个 frontier identity 永久不可达。

因此本轮不增加时间预算，也不放宽覆盖率，而是修复探索策略本身。

### 4.2 本轮 frontier 修复策略

1. **可逃逸净空**：把 frontier traversal clearance 调整为 `0.33 m`。该值可排除 `0.605 m` 盲袋，
   同时实测仍保持主要房间与约 `1.0 m` 门洞连通；`0.35/0.40 m` 会错误切断 office 连通域，所以不采用。
2. **实际位移进展**：使用 progress anchor 记录机器人真实位移。机器人只要产生足够空间位移，即使暂时远离目标，
   也应刷新 progress；只有长期近似静止才触发 timeout。
3. **分层失败记忆**：progress timeout 只记录失败 approach；只有规划器明确给出不可达证据时才抑制 frontier identity，
   避免一个局部落点失败就永久屏蔽整段未知边界。
4. **可恢复早停**：连续 2 次真正 progress timeout 后发布
   `frontier_progress_stalled_recoverable`。必须先完成 Action cancel/terminal，再由任务层执行旋转扫描、地图增益判断和新 epoch；
   禁止逐个耗尽所有候选后才恢复。

`0.33 m` frontier traversal clearance 与 `0.40 m` navigation goal clearance 是不同契约：前者约束探索期间整个
clearance-safe 连通域，过大会割裂房间；后者只约束最终抽样目标，较大净空能提高 AMCL/Nav2 重规划稳定性。

第二次 fresh FAIL `20260720T025009Z-1101237-7f8143c7` 进一步暴露：一个 approach 已到达目标约
`0.38m` 内，另一个 approach 产生了真实绕行，却被当成两次连续静止；恢复时地图增益
`35 < 40` 因而正确 FAIL。修复没有降低增益或最终验收门槛，而是补充两项运行语义：

- Burger unknown-world profile 将“进入传感器可观测半径”设为 `0.40m`，上游通用默认仍为
  `0.30m`。reached 之前必须用最新地图复核 approach 仍安全，且只取消 Action/记 visited，
  不增加 Nav2 succeeded 计数。
- progress timeout 只描述动作末段。若该 approach 的累计位移已足以逃离上次卡点，则清除
  “连续静止”计数，但仍保留这次 approach 失败记录。

最后复核了真正生效的 Nav2 progress checker：`0.10m/30s` 由参数生成器写入会话 YAML，
避免只改 launch 临时参数而无法事后审计。

### 4.3 当前修订版 fresh PASS

```text
session: 20260720T031306Z-1114546-814430c3
schema:  v4 / unknown_world_slam_nav_dynamic_replan
result:  PASS
```

该 session 对修复后代码完成六阶段真实 Gazebo 闭环：总体覆盖 `99.6657%`，最低分区 office
`97.7593%`，reachable unknown `0.3343%`，障碍边界召回/false-free 为 `80.9322% / 0.2119%`；
frontier `available/active/blacklisted=0/0/0`，`accepted/terminal=20/20`；AMCL/Gazebo 246 个对齐样本
P95 `0.154311m`；3/3 运行时采样目标成功、最小间距 `5.570m`，路径
unknown/occupied/map-outside 均为 0；动态重规划和 terminal 后新鲜零速均通过。

这是当前发布候选证据；它不抹除上述两次 FAIL，也不替代后续提交的回归、CI 和新会话验收。

## 5. 阶段计划

- [x] 建立独立工作树，保留原工作区和 known-world 基线。
- [x] 建立严格 evaluator，并证明旧局部地图不能通过。
- [x] 实现无场景先验 frontier 探索、typed telemetry 和运行时导航目标抽样。
- [x] 修复净空栅格边界、目标候选排序、Nav2 准入及权威地图代际绑定。
- [x] 上一修订版运行真实 Gazebo 六阶段 E2E，并保留代表性报告。
- [x] 为 map settle soft quiet/hard budget 与 Nav2 取消竞态补充专项单元回归。
- [x] 对最新安全语义运行首轮 fresh Gazebo E2E；严格 evaluator 在厨房覆盖不足和 frontier 未终结时正确 FAIL。
- [x] 修复 `0.33 m` frontier 可逃逸净空，并用盲袋拒绝/门洞保留测试锁定几何边界。
- [x] 修复合法绕行的进展语义、approach/identity 失败记忆和连续卡死后的可恢复早停。
- [x] 对修复后代码重跑 fresh Gazebo 六阶段 E2E：探索 → 保存 → 定位 → 3 目标导航 → 动态障碍 →
  停车/报告。
- [x] 通过 `offline-sherpa-typed`、`continuous-mock`、`continuous-multi-command` 自动回归。
- [x] 同步 README、架构、测试验收、SLAM/Nav2 学习笔记和本 Goal。
- [x] fresh E2E 后把新 session 与安全语义同步到 15 分钟汇报材料；未预写尚未生成的 PASS。
- [ ] 真人麦克风 `continuous-offline/online` 本轮未重跑；作为现场演示证据保留，不阻塞本次
  SLAM/Nav2 功能合并，也不把自动回归写成真人语音 PASS。
- [x] 在最新安全修改后重跑 `core` 与 `robotics-gate`；只按当前自有包结果汇总，不把 vendor lint 或
  历史 xUnit 误算成本项目回归。
- [ ] 提交完整功能节点并创建 feature → dev PR；CI 通过后才进入 main。

在线静音 filler 的无效请求/token 消耗由独立 issue 跟踪，不混入本 Goal。

## 6. 测试与证据门禁

开发期先跑最小相关门禁：

```bash
pytest -q src/embodied_slam_tools/test/test_mapped_goal_sampler.py
pytest -q src/embodied_slam_tools/test/test_mission_executor.py
pytest -q src/embodied_slam_tools/test/test_showcase_session_node.py
pytest -q tests/repository/test_repository_navigation.py
```

随后跑结构、Evaluator 与 C++ 核心回归。最终现场门禁为：

```bash
bash scripts/cleanup_simulation_processes.sh
bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

报告字段和阈值以 [测试手册 §5](../TESTING.md) 为准；其中路径必须同时审计
unknown/occupied/map-outside，定位必须审计重叠时间范围与唯一时间戳，停车必须来自 terminal 之后的
新鲜样本。终端出现阶段完成日志但没有最终 schema 报告，不算通过。

当前发布候选证据：

```text
logs/acceptance/unknown_world_slam_nav/20260720T031306Z-1114546-814430c3/
└── unknown_world_slam_e2e_report.json  # schema v4，passed=true
```

`logs/` 为本地运行产物，不提交 Git；PR 引用 session id、摘要和复现命令。

当前修改已通过 `core`、`robotics-gate`、相关专项回归和 fresh `unknown-world-slam-e2e`；精确 test
count 不写入 Goal，因为会随测试增删快速漂移。PR 必须写入新 session id、报告摘要和复现命令。
真人麦克风仍按上文边界单独现场验收。

## 7. 中文注释与文档同步

- 中文注释只写在关键风险和策略位置，解释“为什么”，不逐行翻译代码。
- 优先覆盖：真值隔离、frontier 收敛、地图 epoch、float32 栅格净空、Nav2 准入、Action 结果关联、定位误差和安全停车。
- 每次改变验收语义，同步检查：
  - `README.md`
  - `docs/ARCHITECTURE.md`
  - `docs/TESTING.md`
  - `docs/learning/SLAM_NAV2.md`
  - `docs/development/UNKNOWN_WORLD_SLAM_GOAL.md`
- 完整功能节点才 commit/push/触发 CI，避免未闭环的小提交污染远端进度。
