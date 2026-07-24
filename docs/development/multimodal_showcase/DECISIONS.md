# 多模态演示架构决策

这里仅保留会影响模块边界、安全语义或验收可信度的决定。

## ADR-001：业务控制权与速度仲裁分层

状态：已接受。

决定：

- C++ `ControlAuthorityManager` 管理 AUTONOMY、KEYBOARD、HOLD、ESTOP。
- `twist_mux` 只仲裁语音原语和 Nav2 两个自治速度源。
- C++ `VelocityAuthorityGate` 按 typed authority 正授权自治或键盘，并校验
  TurtleBot3 平面轴与速度包络。

理由：数值优先级不能表达 Action 取消、人工接管或按状态选择来源；把业务状态塞进
mux 会产生第二套不可测试的状态机。

## ADR-002：Collision Monitor 是唯一最终速度出口

状态：已接受。

决定：所有速度依次经过自治 mux、AuthorityGate 和 Collision Monitor，只有
Collision Monitor 发布最终 `/cmd_vel`。

理由：多个直接发布者没有确定的业务优先级；任何语音或键盘旁路都会绕过雷达安全。

## ADR-003：撤权与恢复必须显式、同代、fail-closed

状态：已接受。

决定：

- 接管/急停先撤销旧自治任务并生成 `autonomy_revocation_sequence`。
- 只有相同 `manager_epoch + revocation_sequence` 的 Explore/Nav2 terminal、
  priority STOP result 和 STOP 后新鲜零速组成 typed ACK。
- deadman 和 RESET 只进入 HOLD；用户显式 RESUME 后才恢复 AUTONOMY。
- manager lease 在同一 epoch 中断后保持失效；迟到 heartbeat 不能重新授权。

理由：松键、固定 sleep、进程退出或旧零速都不能证明旧 Action 已终止。显式同代
确认可阻止 ghost motion、late result 和 manager 重启绕过安全边界。

## ADR-004：`SessionOrchestratorNode` 是唯一 SLAM/Nav2 业务状态机

状态：已接受。

决定：建图、返航、存图、定位和导航只由 `SessionOrchestratorNode` 编排；控制权
节点只负责授权、取消和 STOP，不复制阶段 FSM。

理由：两个 FSM 同时拥有阶段生命周期会产生竞态。现有 strict unknown-world
状态机已有地图、定位和规划质量门禁，应扩展其窄接口而非重写。

## ADR-005：持久 base 与可替换 stage 分开拥有进程

状态：已接受，已通过重型门禁验证。

决定：

- base 只启动一次，持有 Gazebo、机器人/RSP、RViz、Agent、Nav2 common、
  typed bridge 和速度安全管线。
- mapping stage 持有 SLAM provider/executor；navigation stage 持有本次地图的
  map server、AMCL、Nav2 executor 和动态 tracker。
- 会话根持有唯一 authority manager；stage 不得隐式重建它。

理由：阶段脚本若拥有整个世界，会重置 odom、Agent 会话、RViz 和控制权；明确
所有权后，stage 故障可局部清理，base 故障则整场 fail-closed。

## ADR-006：Persistent 会话只保留一个 Lifecycle 所有者

状态：已接受，已通过重型门禁验证。

决定：

- persistent 模式把 typed bridge 与 `simulation_control` 的 `autostart` 关闭，并
  关闭 stage 内部的 lifecycle manager。
- `SessionOrchestratorNode` 是唯一 Lifecycle 所有者：它读取真实状态，再按
  `configure -> activate` 收敛到 ACTIVE。
- service 返回未知时不立即判失败，也不假装成功；编排器重新读取节点状态。只有
  状态真的前进才继续，否则在期限内重试，超时后 fail-closed。
- 每次等待都同时检查 base/stage 进程是否已退出。
- readiness 超时或进程退出会拒绝新 goal、完成排队 goal 为失败并关闭会话。

理由：Lifecycle 节点自启动、stage lifecycle manager 和会话编排器同时发状态
变更，会形成三个所有者。冷启动时 service response 或 volatile transition event
可能丢失，“请求已发出”也不等于 ACTIVE。单一所有者加最终状态观察，使阶段切换
成为可重试、可验证的事务。

## ADR-007：AMCL 初始化是带代际的事务

状态：已接受，已实现。

决定：

1. 等待 map server 与 AMCL ACTIVE。
2. 等待 `/initialpose` 至少一个匹配订阅者。
3. 使用本次返航终点（兼容手工流程时才回退到显式地图原点）重复发布初始位姿。
4. 只接受当前 navigation generation、发布时间晚于切换边界的 `/amcl_pose`。
5. `/initialpose` 使用 RELIABLE + VOLATILE，不依赖旧 transient-local 样本。

理由：后台脚本在 AMCL 启动前盲发一次并退出 0，会把“消息已写出”冒充“定位已
建立”；旧 `/map`、`/amcl_pose` 缓存也会造成假 readiness。

## ADR-008：进程清理按 owned process tree，而非只杀 launch wrapper

状态：已接受，已实现。

决定：

- `StageProcessManager` 在进程存活期间持续记录 descendant 的 PID 与 starttime。
- 清理前校验 starttime、PGID 和所有者，随后按 explorer→stage→base 顺序终止。
- wrapper 已退出也要回收其留下的 detached child；PID 已复用时禁止误杀。

理由：ROS launch/Gazebo wrapper 可提前退出或派生独立进程组。只杀根 PID 会留下
AMCL/Gazebo 孤儿；只按进程名清理又可能杀死其他会话。

## ADR-009：会话参数只有一个不可变快照

状态：已接受，已实现。

决定：base 启动前原子生成一次 Nav2 参数文件，base、mapping、navigation 只读
同一快照；动态障碍参数可预置，但 tracker 只在 navigation stage 启动。

理由：常驻 planner/controller 不会因环境变量改变而重新读取参数；阶段间替换
文件会造成配置漂移，也可能让建图阶段的动态数据污染静态地图。

## ADR-010：运行时连续性必须绑定真实进程身份

状态：已接受，已通过重型门禁验证。

决定：

- mapping/navigation checkpoint 记录 schema、role、label 和严格递增 wall time。
- Gazebo、RSP、实际 `offline_agent`/`online_agent`（启用时含 RViz）通过
  PID、`/proc` start ticks、boot ID、executable 和 argv SHA-256 识别。
- 使用 `ROS_DOMAIN_ID`/`GZ_PARTITION` 唯一筛选本次会话；报告不保存原始 argv。
- verifier 从原始 checkpoint 重新计算，不接受缓存的 `passed=true`。

理由：PID 可复用，launch parent 可退出，缓存 PASS 可与原始数据矛盾；这些情况
都不能证明 mapping→navigation 期间同一机器人和 Agent 未重启。

## ADR-011：strict evaluator 复用，persistent 只增加连续性门

状态：已接受。

决定：`showcase-gazebo-e2e` 复用 strict unknown-world 的地图质量、返航、定位、
采样目标、动态重规划和最终停车判断，只在其上附加 runtime continuity。

理由：另建宽松 evaluator 会让主演示轻易 PASS 却丢失发布级质量。持久化是新增
不变量，不应降低既有不变量。

## ADR-012：停车证据必须位于本次 STOP 之后

状态：已接受，已实现。

决定：

- 不使用历史零速或“机器人看起来没动”作为停车证据。
- persistent base 延长 Collision Monitor 的零速心跳窗口，但仍由它唯一发布
  `/cmd_vel`，不绕过碰撞裁决。
- verifier 要求 STOP 边界之后收到新鲜最终零速。

理由：Collision Monitor 默认在停车约 2 秒后停止重复零速；长会话末尾若没有
因果边界后的样本，就无法证明本次 STOP 已穿过最终执行边界。

## ADR-013：首次自动任务取消后不承诺断点续跑

状态：已接受。

决定：人工接管或急停取消当前 Explore/Nav2 事务；恢复后发起新任务，不自动恢复
旧 goal。

理由：真正暂停需要冻结 deadline、goal generation、阶段状态和恢复点，不能混入
控制平面最小安全闭环。

## ADR-014：快速演示、严格仿真和真人语音证据分离

状态：已接受。

决定：

- `quick` profile 服务 15 分钟现场演示。
- strict unknown-world/persistent heavy 服务发布门禁。
- live offline/online 才证明麦克风、endpoint、ASR 与 Agent 体验。
- 预验证地图降级必须记录 `degraded=true`、`same_session=false`。

理由：mock ASR 可稳定验证编排，但不能证明真实识别；旧地图可演示导航，但不能
冒充本次未知环境建图。证据必须与它实际覆盖的边界一致。

## ADR-015：探索接近饱和时只允许一次有界最终确认

状态：已接受，已通过重型门禁验证。

决定：探索已触达硬时间预算、但最近 epoch 仍有实质地图增益时，允许一次最多
`240 s` 的最终确认。确认结束后必须依据连续低收益 epoch、剩余 frontier、最终
probe、地图质量和 typed STOP 共同裁决，不能无限延长。

理由：立即停止可能把“还有有效增益”误判为完成；不断恢复又会把有界任务变成无界
等待。一次有上限的确认既保留最后一段有效探索，又保证验收能确定结束。

## ADR-016：先提取 Nav2 运动事务，长期收口默认演示会话

状态：已接受、已实现，并已通过 clean commit 重型门禁。

比较：

1. **`Nav2MotionTransaction`**：把一个 Nav2 goal 从发送、响应、接受、取消、
   priority STOP 到 terminal 的完整安全事务藏在一个小 Interface 后面。改动集中，
   能直接消除当前最危险的重复清理路径。
2. **`DefaultDemoSession`**：让一个深会话模块拥有默认演示的启动、命令准入、提交、
   状态和关闭。它能给语音、typed Action 和未来统一 shell 一个共同 caller
   Interface，但迁移范围大于一笔 Nav2 事务。
3. **`MissionProgram`/phase DSL**：通过 phase registry、artifact 和 effect 组合
   mapping、navigation 和未来 demo，扩展上限最高；但当前还没有第三个具有独立
   不变量、恢复和证据语义的深 phase。

决定：

- 当前分支已实现 `nav2_motion_transaction.py:Nav2MotionTransaction`，并由
  `nav2_motion_ros.py:RosNav2MotionAdapter` 适配到 rclpy ActionClient、ROS clock、
  priority STOP、stage fail-safe、quiescence ledger 与 evidence callback。
  接口级测试位于 `test_nav2_motion_transaction.py`，ROS Adapter 测试位于
  `test_nav2_motion_ros.py`。
- 该模块只拥有单次 Nav2 运动事务，不拥有整场 mission、语音解析、地图采样策略或
  ROS wire Interface。候选 preflight、ROS `/plan` 采集和 occupancy 评分仍由
  Node 拥有；事务读取 ledger 并强制执行运行期路径安全门。
- 唯一行为 Interface 是
  `execute(intent: SampledNavigate | MappingReturn | RecoveryBackup, *,`
  `is_cancelled, deadline_monotonic) -> None`。成功返回 `None`；拒绝、运动失败、安全
  失败和并发分别使用 `Nav2GoalRejected`、`Nav2MotionFailed`、
  `Nav2SafetyFailure`、`Nav2TransactionBusy`；server/业务 deadline 使用
  `TimeoutError`，用户取消继续使用 `AutomaticMissionCancelled`。
- `DefaultDemoSession` 是长期方向，预期锚点是
  `src/embodied_slam_tools/embodied_slam_tools/default_demo_session.py`。目标
  Interface 是 `start()`、`admission()`、`submit()`、`snapshot()`、`close()`；
  异步操作只暴露完成、进度、取消和有界等待。
- 暂缓 `MissionProgram`/phase DSL。等出现至少第三个有实质行为的 phase，且两个
  现有 phase 的共同需求已经由代码事实证明，再重新设计其 seam。

`Nav2MotionTransaction` 的 Interface 不变量：

- 每次执行最多拥有一个 Nav2 goal；并发进入明确抛出
  `Nav2TransactionBusy`。
- 正常执行共享调用者给出的绝对 deadline；进入故障清理后只创建一次独立
  cleanup deadline，late accept、cancel、STOP 与 terminal 共用这份预算且不得续期。
- pending goal 在取消或 response timeout 后也必须同步收口迟到的 accepted handle。
- accepted goal 的故障严格执行
  `cancel -> 独立 priority STOP -> readable terminal`；用户取消不能短路 STOP。
- terminal 不可证明时停止 navigation stage 并以 `Nav2SafetyFailure` fail-closed。
- primary error 保持为主错误，清理错误只附加诊断。
- quiescence 只在明确拒绝或可读 terminal 后结算；sampled ledger terminal 恰好
  一次且晚于安全收口。
- ROS topic、Action、Service、QoS、`SlamSessionState` 和 typed evidence schema
  全部保持不变。

采用门槛：

1. 成功、拒绝、执行异常、取消、response timeout、late accept、STOP 失败和
   terminal 缺失均有接口级测试。
2. 现有 Node 白盒测试只有在等价接口测试建立后才删除，避免只叠加一层转发模块。
3. 包测试、repository contracts、`acceptance_test.sh core` 通过。
4. 合入或发布前，在 clean commit 上完成 unknown-world 重型 E2E；若长期
   `DefaultDemoSession` 落地，还必须新增显式 STOP 到 STOPPED 的证据。

当前代码迁移、接口测试、包测试、repository contracts、`acceptance_test.sh core`
和 typed Gazebo 门禁均已通过。clean commit `bc65b8f` 的 unknown-world E2E 也已
验证建图、返航、定位、三点导航、动态重规划与最终新鲜零速，ADR-016 至此完成验证。
