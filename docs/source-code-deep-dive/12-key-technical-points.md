# 12. 关键技术点凝练

## 一句话技术主线

项目把在线/离线语音输入转换成强类型机器人命令，通过 C++ ActionGuard、带 TTL 的启动 outbox、单执行槽 Action scheduler、BehaviorTree 和可替换 Gazebo/Nav2 executor，形成可取消、可超时、可观测的 ROS 2 执行闭环。

## 十个最关键技术点

### 1. 控制面与 provider 数据面分离

`AgentControlPlane`、`AgentApplicationRuntime`、`AgentExecutionRuntime`、`AgentLifecycleRuntime` 被 online/offline 复用；ASR/LLM/TTS 只是 Adapter。收益是相同输入在在线/离线模式下拥有一致的唤醒、记忆、队列、NLU、取消和事件语义。

### 2. 连续语音状态机

一次唤醒进入多轮会话，支持 timeout、filler、重复 final、sleep、短命令补全、partial 恢复、有界 FIFO 和 stale TTL。急停/取消导航绕过普通队列并跨三层取消。

### 3. 确定性 NLU + LLM fallback

字符 n-gram 做意图相似度，规则抽取速度、距离、角度、时长和地点槽位。固定域动作优先确定性路径，LLM 处理开放表达，但疑问、否定和危险语义阻断模型动作。

### 4. 安全流式协议

`<speech>` 可增量进入 TTS；`<action>` 必须闭标签完整且 JSON 合法后才形成候选。协议损坏有明确语音反馈，但不会从半个流猜动作。

### 5. 强类型动作和 C++ Guard

跨节点控制不用 JSON，统一 `RobotCommand`。Guard 做动作白名单、字段互斥、finite 检查、速度/时长限幅、地点白名单和 priority 约束。typed schema 与业务校验形成两层保障。

### 6. DDS 发现窗口补偿

Reliable/volatile 不能重放 discovery 前消息。Guard 的 bounded TTL outbox 等 scheduler 匹配，ready 后按序发送，超时显式拒绝，兼顾启动可靠性和陈旧动作安全。

### 7. 单 active Action scheduler

纯 C++ `ActionScheduler` 独占 FIFO、重复 ID、失败清队列和 priority cancel。Node Adapter 才做 Action Client 副作用。取消 watchdog 与 command ID 防止失联和迟到 result 卡死/串单。

### 8. BT + pluginlib + 运行时安全

Action server 提供跨进程生命周期；BT 每 tick 编排 validate/safety/execute/confirm；pluginlib 替换 Mock/Gazebo/Nav2 executor；20 Hz controller 做加速度斜坡、雷达 stale fail-safe、近障停车和沿墙 PID。

### 9. 真实 Nav2 result 驱动终态

语义地点 key 通过 YAML 转 `PoseStamped`，Nav2 executor 调用 `NavigateToPose/FollowWaypoints`。外层完成以 Nav2 result 为准，本地计时仅负责 hard timeout；generation 主动取消迟到 goal handle。

### 10. Lifecycle 和 readiness 闭环

Lifecycle 固化安全启停，STOP 在 managed publisher 停用前发送；bringup contract 统一参数类型和 VAD 互斥；组件持续 health heartbeat，SystemReadiness 聚合 missing/degraded/stale 依赖。

## 三个最有含金量的代码设计

### 状态所有权

会话、busy、endpoint timer、C++ FIFO、active action、控制器和用户身份分别有唯一拥有者。面试中可用“避免多个层同时推进同一状态”解释为什么做这些模块，而不只说“做了解耦”。

### 异步迟到保护

系统多处使用 ID/generation：

- endpoint timer generation。
- Python sequence cancel generation。
- Action command ID。
- cancel watchdog 后 stale result 检查。
- Nav2 goal generation。

共同目标是：取消/超时之后，旧异步回调不能修改新请求状态。

### 纵深安全

```text
语义阻断
-> typed conversion
-> C++ Guard
-> scheduler 单执行
-> Action server/BT 再校验
-> controller 雷达安全
-> Lifecycle STOP
-> hardware watchdog
```

任何单层都有遗漏可能，多层边界让错误以 rejected/blocked/canceled/timed_out 收敛，而不是变成无界运动。

## 三个可量化工程点

1. 端到端延迟拆成 ASR endpoint、LLM first token、TTS first audio 和 action result，不用单一“响应快”概括。
2. 队列和 buffer 都有 capacity、TTL/timeout、dropped/high-watermark 指标，避免无限积压。
3. Action、Nav2、Lifecycle、硬件都有 timeout/watchdog，避免等待外部依赖永久悬挂。

## 面试讲述结构

### 30 秒技术版

我做的是 ROS 2 端侧语音机器人控制系统。语音经 VAD/ASR 后先进入共享会话和本地 NLU，固定控制命令不依赖 LLM；模型只产生强类型候选动作，再经过 C++ Guard、FIFO Action scheduler、BT 和 Gazebo/Nav2 executor。系统重点解决连续命令、急停抢占、异步迟到回调、Lifecycle 安全停机和执行证据闭环，而不是让 LLM 直接发 `/cmd_vel`。

### 深挖顺序

1. 先画 `candidate -> Guard -> Action -> executor -> result`。
2. 解释 Topic/Service/Action/QoS 选型。
3. 展开连续语音状态所有权与急停。
4. 展开 Guard、scheduler、cancel watchdog。
5. 展开 BT/controller 或 Nav2 真实 result。
6. 最后讲测试证据和事实边界。

## 不要夸大的五句话

| 不准确说法 | 应改为 |
| --- | --- |
| “做了完整降噪” | “实现 NLMS AEC；VAD 可减少非语音触发，通用 NS/AGC 尚未激活” |
| “实现了自主导航” | “语义地点已桥接 Nav2 Action，并按 mock/fake server/完整 Nav2 分层验收” |
| “硬件已经跑通” | “完成 UART/SPI Adapter、帧协议和 mock/pseudo-terminal 验收，真实硬件闭环待验证” |
| “模型动作准确率 100%” | “工程出口有 deterministic fallback；模型严格协议分数单独评估” |
| “满足生产实时 SLA” | “当前机器少量样本达到阶段指标，需更多场景和长稳测试” |

## 最后记住的源码锚点

| 关键词 | 源码对象 |
| --- | --- |
| 统一控制面 | `AgentControlPlane.accept_transcript/enqueue_command` |
| 并发和 busy | `AgentExecutionRuntime` |
| 安全停机 | `AgentLifecycleRuntime.deactivate` |
| VAD 滞回 | `StreamingVadEndpoint._update` |
| 回声消除 | `NlmsEchoCanceller.process` |
| 模型协议 | `TaggedStreamParser`、`StreamingTurnRuntime` |
| 动作安全 | `ActionValidator.validate` |
| 启动窗口 | `GuardedCommandOutbox.drain` |
| FIFO/急停 | `ActionScheduler.enqueue/complete` |
| Action 执行 | `SimulationControlNode.handle_accepted/control_tick` |
| BT | `CommandBehaviorTree.tick` |
| 雷达控制 | `SimulationController.step` |
| Nav2 | `Nav2RobotExecutor.send_navigate_goal/send_follow_goal` |
| 用户一致性 | `UserContextRuntime.snapshot` |
| 硬件安全 | `HardwareProtocol`、`MotionWatchdog` |

## 最终自检

不看文档，尝试完整回答：

> 用户说“小智，先右转九十度，然后去门口”，执行中又说“取消导航”。请从 PCM、VAD、ASR、会话、NLU batch、用户快照、候选消息、Guard、scheduler、外层 Action、BT、Nav2 goal、取消和最终 result，按时间顺序讲完整调用链，并指出每层失败会在哪里留下证据。

能讲清这道题，基本就掌握了整个项目的核心设计。
