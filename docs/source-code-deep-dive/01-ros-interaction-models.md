# 01. ROS 2 节点交互模式与 QoS

## 先给结论

项目没有把所有通信都做成 Topic。选择标准是数据语义：连续流和广播事件用 Topic；短请求响应用 Service；持续任务用 Action；需要受控启停的资源用 Lifecycle；运行参数用 Parameter；可替换 C++ 实现用 pluginlib。

## 六种交互模式

| 模式 | 当前项目实例 | 优点 | 缺点 | 适用判断 |
| --- | --- | --- | --- | --- |
| Topic | `/audio/clean_pcm`、`/agent/action_candidate`、状态与事件 | 解耦、可多订阅者、适合流式数据 | 无原生请求终态；订阅者未发现时可能错过 volatile 消息 | 数据流、广播、遥测 |
| Service | `/tts/synthesize` | 一问一答简单，调用者明确拿到响应 | 不适合持续反馈和标准取消 | 短时、有限、请求响应操作 |
| Action | `robot/execute_command`、Nav2 action | goal、feedback、result、cancel 完整 | 接口和状态管理更复杂 | 长任务、可取消、需要进度 |
| Lifecycle | Agent、ActionGuard、SimulationControl | configure/activate/deactivate 可控，便于安全停机 | 需要 manager 和严格状态迁移 | 持有外部资源或运动能力的节点 |
| Parameter | VAD 阈值、队列容量、模型路径、速度上限 | 部署可配置，类型可校验 | 动态修改可能引入运行时一致性问题 | 环境和 profile 差异 |
| pluginlib | Gazebo/Mock/Nav2 `RobotExecutor` | 上游接口不变即可换后端 | 插件装载和 ABI 更复杂 | 同一职责的 C++ 运行时替换 |

## Topic：为什么 PCM 和命令都是 Topic，但 QoS 不同

### PCM

`/audio/clean_pcm` 是 20 ms 左右的连续块。若消费者落后，旧音频已经没有价值，继续可靠重传只会把实时语音变成延迟语音。因此 [ros_qos.py](../../src/embodied_agent_core/embodied_agent_core/ros_qos.py) 和 [qos_profiles.hpp](../../src/embodied_agent_middleware/include/embodied_agent_middleware/qos_profiles.hpp) 都把 audio 定义为：

```text
KEEP_LAST + small depth + BEST_EFFORT + VOLATILE
```

优点是低延迟且不反压；代价是允许丢帧，所以 ASR/VAD 必须能容忍少量缺失。

### 控制命令

`/agent/action_candidate` 和 `/robot/action_command_typed` 使用：

```text
KEEP_LAST + RELIABLE + VOLATILE
```

命令需要在已匹配节点之间可靠传输，但不能 transient-local。若重启后重放旧的“前进 5 秒”，会形成危险的陈旧动作。

### 当前状态

`/agent/state`、`system/component_health`、`system/readiness` 使用：

```text
RELIABLE + TRANSIENT_LOCAL + shallow history
```

新加入的监控应立即看到最新状态，不必等待下一次状态变化。它们表示“现在是什么”，不是不可覆盖的事件历史。

### 事件

Action result、队列事件、BT 状态使用 reliable/volatile。事件必须有序到达，但晚加入者不应该把旧事件当成当前动作。

## QoS 速查

| 命名 profile | Reliability | Durability | 默认深度 | 语义 |
| --- | --- | --- | ---: | --- |
| `command_qos` | reliable | volatile | 50 | 控制命令 |
| `event_qos` | reliable | volatile | 50 | 生命周期事件 |
| `state_qos` | reliable | transient local | 1 | 可覆盖当前状态 |
| `sensor_qos` | best effort | volatile | 5 | 高频传感器 |
| `audio_qos` | best effort | volatile | 5 | 实时 PCM |
| `diagnostics_qos` | reliable | volatile | 10 | 低频诊断 |

Python/C++ 两份实现名称和语义一致，是跨语言中间件契约，而不是各节点临时选择。

## Reliable 为什么仍然需要 outbox

可靠 QoS 只保证已完成 DDS discovery、已经匹配的 publisher/subscriber 之间可靠。发布时如果没有订阅者，volatile 消息不会在未来补发。

因此 [action_guard_node.cpp](../../src/embodied_agent_cpp/src/action_guard_node.cpp) 先把已通过校验的命令放入 `GuardedCommandOutbox`：

1. 每 50 ms 检查 `get_subscription_count()`。
2. 下游匹配后按序发布。
3. 超过 `downstream_wait_timeout_s` 则过期拒绝。
4. 队列有最大容量，避免等待期间无限积压。

这不是替代 DDS，而是补偿启动 discovery 窗口，同时用 TTL 防止旧动作迟到执行。

## Service：SummerTTS 为什么适合

[SynthesizeSpeech.srv](../../src/embodied_agent_interfaces/srv/SynthesizeSpeech.srv) 的请求是文本、speaker 和语速，响应是 PCM、采样率、耗时和缓存命中。一次句级合成有明确结果，当前无需中途 feedback，所以 Service 比 Action 简单。

[summer_tts_service_node.cpp](../../src/embodied_agent_cpp/src/summer_tts_service_node.cpp) 在构造时加载模型一次，后续请求复用模型；用 mutex 串行保护未声明线程安全的 `SynthesizerTrn`，再用 LRU 缓存短反馈。

它的不足是长文本合成无法标准取消，也没有增量反馈。若未来做真正逐帧 TTS、可打断播报，Action 或 streaming transport 会更合适。

## Action：为什么运动不能用 Service

“前进 5 秒”在调用返回前有中间过程，用户可能在第 2 秒喊停，控制器也可能在第 1 秒检测到障碍。Service 只有请求和响应，缺少标准 feedback/cancel 状态机。

[ExecuteRobotCommand.action](../../src/embodied_agent_interfaces/action/ExecuteRobotCommand.action) 定义了：

- Goal：完整 `RobotCommand`。
- Result：`success/status/message`。
- Feedback：`phase/progress/detail`。

Action Client 位于 `TypedActionBridgeNode`，Action Server 位于 `SimulationControlNode`。Client 的 result callback 再把终态发布为 `/robot/action_result`，供 Python 动作批次按 `command_id` 唤醒。

## Lifecycle：为何不是普通 Node

普通 Node 一创建就开始工作，不适合“先装资源、等依赖、再开放输入”。Lifecycle 把过程拆成：

```text
unconfigured -> inactive -> active -> inactive -> cleaned up
```

项目中的安全语义是：

- configure：创建 provider、publisher、subscription、Action server 和 timer，但不产生运动。
- activate：先激活 managed publisher，再开放输入/控制 timer。
- deactivate：关闭输入、取消任务、清队列、发布 STOP、等待 worker，最后停 publisher。
- cleanup：释放 provider 和 ROS entity。

详细顺序见 [08-lifecycle-launch-and-readiness.md](08-lifecycle-launch-and-readiness.md)。

## pluginlib：为何不是在节点里写 if/else

`SimulationControlNode` 只依赖 `RobotExecutor` 接口，通过参数 `executor_plugin` 加载：

- `GazeboRobotExecutor`
- `MockRobotExecutor`
- `Nav2RobotExecutor`

优点是 Action server、BT、状态发布和超时逻辑不变，只替换执行策略；测试可以用 mock，演示可用 Gazebo，导航可用 Nav2。代价是插件 XML、链接依赖和运行时错误更复杂，所以 configure 阶段捕获 `PluginlibException` 并拒绝激活。

## 节点与接口表

| 节点 | 订阅/服务输入 | 发布/客户端输出 |
| --- | --- | --- |
| `audio_frontend` | `/audio/tts_pcm`、PortAudio input | `/audio/clean_pcm`、endpoint、metrics、health |
| `webrtc_vad` / `silero_vad` | `/audio/clean_pcm` | endpoint、`/audio/vad_event` |
| `keyword_wake` | PCM 或 mock text | `/agent/wake_event_input`、KWS event/score |
| online/offline Agent | PCM、endpoint、text、wake、speaker、action result | ASR、response、candidate、metrics、TTS PCM |
| `action_guard` | `/agent/action_candidate` | `/robot/action_command_typed`、rejection、health |
| `typed_action_bridge` | guarded command | Action goal、feedback/result topic、diagnostics |
| `simulation_control` | Action goal、scan、mode、emergency stop | `/cmd_vel`、Action feedback/result、state、BT、ACK |
| `system_readiness` | component health | aggregate readiness |
| `hardware_controller` | guarded command、emergency stop | UART/SPI frame、ACK、status |

## 自测问答

### 问：Topic、Service、Action 怎么选？

答：先看数据是否持续以及是否需要取消。广播流和事件用 Topic；短时一问一答用 Service；持续任务且需要 feedback/result/cancel 用 Action。不能因为 Service 写起来简单就让长运动失去取消语义。

### 问：为什么 `/cmd_vel` 用 reliable 是否一定更安全？

答：不一定。安全取决于实时性、控制器 watchdog 和停止策略。旧速度可靠排队后迟到，可能比丢失一帧更危险。本项目上游命令使用 reliable，`SimulationRosIo` 当前的 `cmd_vel` 也使用 command profile，但控制器以 20 Hz 重发当前值并在停机发布零速度；面试中应说明 QoS 要结合下游兼容性和实时控制语义评估。

### 问：为什么状态用 transient-local，命令不能用？

答：状态是可覆盖快照，新监控需要立即拿到最新值；命令是一过性意图，重启后重放会执行陈旧动作。两者的时间语义完全不同。

### 问：Action result topic 是否重复了 Action 自带 result？

答：传输上有适配，但用途不同。C++ bridge 是 Action Client，它接收原生 result；Python Agent 不是这个 Action Client，所以 bridge 把统一终态转换成 typed topic，供 `SequentialActionPublisher` 按 command ID 关联。这样 Python 不需要拥有第二套 Action Client 和 FIFO。
