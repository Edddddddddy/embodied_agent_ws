# 08. Lifecycle、Launch 契约与系统 Readiness

## 先给结论

启动成功不等于系统可执行。项目用 Lifecycle 控制资源开放顺序，用 bringup contract 收敛在线/离线公共参数和节点拓扑，用 `ComponentHealth -> SystemReadiness` 判断关键依赖是否持续可用。

## Agent Lifecycle 的资源边界

核心文件：[agent_lifecycle_runtime.py](../../src/embodied_agent_core/embodied_agent_core/agent_lifecycle_runtime.py)

online/offline 节点仍是各自的 `LifecycleNode`，但业务资源顺序由同一个 `AgentLifecycleRuntime` 组合管理。它持有 endpoint、execution runtime 的引用，并调用 action sequencer、ROS I/O 和 event adapter。

### configure

节点创建 provider、`AgentExecutionRuntime` 和 `AsrEndpointRuntime` 后，调用 `bind()` 一次性移交所有权。重复 bind 直接报错，防止多套 worker/timer 共存。

### activate

顺序是：

1. `ros_io.set_lifecycle_active(true)` 开启 health heartbeat。
2. `_active=true`。
3. 启动/重置 ASR 输入 provider。
4. 启动 command worker。

如果旧 turn 未静默，`execution.start()` 返回 false，activate 失败。不能在旧线程仍可能发布动作时开放新会话。

### deactivate 的安全顺序

源码固定为：

```text
active=false，关闭新输入门
-> cancel pending ASR endpoint timer
-> cancel Python action sequence wait
-> clear natural-language queue
-> reset wake/session/partial/retry state
-> publish priority STOP
-> stop provider input
-> stop and join execution workers
-> publish inactive/STOPPED
-> deactivate managed publishers
```

STOP 必须在 publisher 停用前发布。若先调用 publisher `on_deactivate()`，安全命令会被 managed publisher 丢弃。

`LifecycleQuiescence` 同时记录 input 和 execution 是否静默。provider stop 抛异常也不会跳过 worker 停止；只有全部成功才把状态标记 inactive。

### cleanup 和 shutdown

cleanup 关闭 endpoint 并停止 worker，静默成功后才释放引用。`begin_shutdown()` 用 `_stopping` 保证只执行一次，并在原来 active 时利用仍可用 publisher 发 STOP。

## C++ Lifecycle 节点

### ActionGuard

- configure 创建 publisher/subscription/outbox/timer。
- activate 激活 publisher、启动 flush timer、health=STARTING。
- deactivate 停 timer、清 outbox、先发 STOPPED health，再停 publisher。

### SimulationControl

- configure 加载 plugin/BT，创建 Action server和 control timer。
- activate 激活 `SimulationRosIo` 并启动控制循环。
- deactivate 取消 active action、executor stop、发布零速度、停 timer，再停 I/O。

两者都让“资源已创建”和“允许工作”分开。

## Launch contract 为什么独立成包

`embodied_agent_bringup` 是装配根，不放领域逻辑。它解决的是 online/offline 重复 launch 参数、语音前端 provider 互斥、Lifecycle 顺序和硬件 Adapter 选择。

### Agent 参数契约

[agent_launch_contract.py](../../src/embodied_agent_bringup/embodied_agent_bringup/agent_launch_contract.py) 从 `agent_parameters.default_parameter_values(profile)` 生成 Launch 默认值，避免节点 schema、online launch、offline launch 各维护一份默认参数。

Launch 替换值天然是字符串，所以 `agent_control_parameter_overrides()` 用 `ParameterValue(..., value_type=...)` 明确 bool/int/float 类型。否则字符串 `"false"` 被错误当真是常见部署问题。

只转发现场常调的控制参数，模型路径等 provider 私有配置留在各自 YAML，防止公共 launch 膨胀为万能参数管道。

### 语音前端契约

[voice_frontend_launch_contract.py](../../src/embodied_agent_bringup/embodied_agent_bringup/voice_frontend_launch_contract.py) 统一创建：

- speaker identity sidecar
- C++ audio frontend
- WebRTC 或 Silero VAD sidecar
- keyword wake sidecar

关键互斥规则：`vad_provider` 为 silero/webrtc 时，C++ `endpoint_events_enabled=false`。这样全系统只有一个 endpoint 事件拥有者。

### 部署契约

[agent_deployment_launch_contract.py](../../src/embodied_agent_bringup/embodied_agent_bringup/agent_deployment_launch_contract.py) 创建：

- ActionGuard LifecycleNode
- Nav2 lifecycle manager
- 可选 hardware controller

manager 的 `node_names=[action_guard, agent]` 是启动顺序；停止按逆序，使 Agent 先停止产生命令，再关闭 Guard。

## Health 与 readiness

### ComponentHealth

组件周期发布：

```text
component, state, detail, stamp
```

状态包括 STARTING、READY、DEGRADED、ERROR、STOPPED。使用 state QoS，让晚加入 readiness 节点能立即获得最近状态；同时组件持续心跳，避免一条旧 latched READY 永久代表已经崩溃的进程。

### SystemReadiness

核心文件：[system_readiness_node.cpp](../../src/embodied_agent_middleware/src/system_readiness_node.cpp)

`ComponentHealthRegistry` 按 profile 的 required components 评估：

- 是否收到每个组件。
- 是否 READY。
- 距最后心跳是否超过 `stale_timeout_s`。
- 哪些 missing/degraded。

节点默认每 250 ms 发布 `SystemReadiness`，包含 required/ready/missing/degraded 列表和 detail。

### 为什么 readiness 不是启动 sleep 3 秒

固定 sleep 依赖机器速度，不能区分模型慢加载和组件崩溃，也无法发现运行后断连。健康心跳提供语义状态，readiness 还能持续从 ready 退化为 not ready。

## ActionGuard 与 bridge 的健康状态

ActionGuard 通过 subscription count 判断 scheduler 是否已匹配：

- 从未匹配：STARTING/waiting。
- 匹配：READY。
- 曾匹配后断开：DEGRADED。

TypedActionBridge 通过 `action_server_is_ready()` 判断 SimulationControl：逻辑同样区分首次等待和运行后断连。二者串起来，readiness 可以表达“Guard 已接上 scheduler，scheduler 已接上 Action server”。

## Lifecycle manager 与 readiness 的区别

- Lifecycle manager 负责调用 configure/activate/deactivate 的状态迁移。
- readiness 负责观察组件实际依赖是否可用。

节点显示 ACTIVE 不代表下游 Action server 已 ready；反之，某普通节点无 Lifecycle 也可以发布 READY health。两套机制互补。

## 参数与默认值的事实边界

语音前端 contract 暴露 `noise_suppression_enabled` 和 `auto_gain_enabled`，但暴露参数不等于后端实现。判断能力必须继续看 `audio_frontend_node.cpp` 的实际 enhancer 和 metrics active 字段。

同样，`hardware_backend=uart/spi` 只表示已有 Adapter 入口；是否存在真实设备、下位机协议兼容和闭环反馈，要由运行证据证明。

## 启动故障推演

### Agent active，但命令没有执行

检查 `system/readiness`：

- action_guard missing：Lifecycle/launch 问题。
- action_guard STARTING：scheduler topic 尚未匹配。
- typed_action_bridge STARTING：ExecuteRobotCommand server 未 ready。
- simulation_control missing/stale：仿真节点未激活或进程退出。

### deactivate 卡住

看 `LifecycleQuiescence` 对应日志：是 provider input 未停，还是 execution worker 未 join。运行时会发布 `deactivate_timeout`，不会假装 inactive。

### endpoint 在重新激活后异常提交旧句子

检查 provider reset、`AsrEndpointRuntime.cancel_pending()` generation，以及 deactivate 是否真的 quiesced。

## 自测问答

### 问：为什么需要 Lifecycle，普通 Node 加 enabled 参数不行吗？

答：enabled 参数通常没有标准资源迁移和错误语义。Lifecycle 明确 configure/activate/deactivate/cleanup，并能由 manager 编排。对声卡、模型、Action server 和运动 publisher，受控停机比一个布尔分支更可靠。

### 问：安全停机最关键的顺序是什么？

答：先关闭输入和取消任务，再清队列并发布 priority STOP，等待 worker 静默，最后停 managed publisher。STOP 不能排在 publisher 停用之后。

### 问：ACTIVE 是否等于 READY？

答：不等于。ACTIVE 说明 Lifecycle 状态允许工作；READY 还要求 provider、topic match、Action server 等实际依赖可用且心跳新鲜。

### 问：为什么 health 既用 transient-local 又要心跳？

答：transient-local 让晚加入者立即看到状态；心跳让 registry 能判断进程是否仍活着。只有 latched 状态无法区分旧 READY 和当前存活。
