# ROS 2 与机器人软件框架追问

## Q1. Node 是什么？为什么项目要拆成多个节点？

口述：

Node 是 ROS 2 中对一组职责、通信接口和运行状态的封装，不等同于操作系统进程。这个项目把音频采集、语音 Agent、动作校验、动作调度、机器人执行和系统健康拆开，是因为它们的实时性、故障范围和生命周期不同。例如音频回调不能被 LLM 推理阻塞，模型异常也不应该绕过 C++ 安全边界。部署时，节点既可以独立进程运行，也可以通过 component 放进同一容器。

源码：

- 音频节点：[audio_frontend_node.cpp](../../../src/embodied_agent_cpp/src/audio_frontend_node.cpp)
- 动作桥接节点：[typed_action_bridge_node.cpp](../../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)
- 执行节点：[simulation_control_node.cpp](../../../src/embodied_simulation/src/simulation_control_node.cpp)
- component 部署：[simulation_control.launch.py](../../../src/embodied_simulation/launch/simulation_control.launch.py)

运行：

1. 音频节点发布清洗后的 PCM 和语音端点事件。
2. Python Agent 生成结构化候选动作。
3. C++ 校验节点发布可信命令。
4. 动作桥接节点排队并发送 Action goal。
5. 执行节点驱动 Gazebo 或 Nav2。

取舍：拆节点增加了通信和部署成本，但换来了故障隔离、独立测试和后端替换能力。高频且强耦合的代码仍保留在同一进程内函数或同一 component 中。

## Q2. Topic、Service 和 Action 有什么区别？项目如何选择？

口述：

Topic 适合持续数据和广播，发送方不等待某个订阅者给出任务终态；Service 适合较短的一次请求响应；Action 适合需要反馈、取消和最终结果的长任务。项目中音频、激光和状态使用 Topic，离线 TTS 合成使用 Service，机器人动作和导航使用 Action。选型依据是交互形状，不是简单地认为哪一种“更高级”。

| 模式 | 项目例子 | 优点 | 局限 |
| --- | --- | --- | --- |
| Topic | `/audio/clean_pcm`、`/scan`、候选命令、健康状态 | 异步、多订阅者、适合连续流 | 没有内建的一次任务终态 |
| Service | `/tts/synthesize`、地图保存服务 | 请求与响应对应清楚 | 不适合长时间反馈和取消 |
| Action | `ExecuteRobotCommand`、`NavigateToPose`、`FollowWaypoints` | goal、feedback、result、cancel 完整 | 状态和迟到回调处理更复杂 |

运行例子：导航目标由 Action client 发送，Nav2 接受后持续执行；用户说“停下”时 client 请求取消；Nav2 最后返回成功、取消或失败。若改成 Service，调用线程要长时间等待；若改成 Topic，需要自行重造目标关联、反馈、取消和结果协议。

## Q3. 为什么候选命令先走 Topic，真正执行再走 Action？

口述：

候选动作和执行任务是两个不同阶段。模型可能连续产生动作，也可能产生非法字段，所以先用 Topic 把候选交给独立 C++ 边界校验；校验后的命令再由调度器转成 Action goal。这样模型只负责表达意图，执行层才拥有排队、反馈、取消和终态。两层之间通过 `command_id` 关联结果。

源码：

- 候选消息：[RobotCommand.msg](../../../src/embodied_agent_interfaces/msg/RobotCommand.msg)
- 字段校验：[action_validator.cpp](../../../src/embodied_agent_cpp/src/action_validator.cpp)
- 排队与派发：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)
- Action 定义：[ExecuteRobotCommand.action](../../../src/embodied_agent_interfaces/action/ExecuteRobotCommand.action)

运行：

```text
1. Python 发布 /agent/action_candidate
2. C++ 检查动作类型、字段组合、速度、时长和地点
3. 可信命令进入 ActionScheduler
4. TypedActionBridgeNode 发送 ExecuteRobotCommand goal
5. feedback 和 result 按 command_id 返回 Python
```

取舍：可以让 Python 直接建 Action client，但模型适配和安全策略会耦合，且其他候选来源也要重复校验。

## Q4. Parameter 适合保存什么？为什么不能用它传运行事件？

口述：

Parameter 适合节点配置，例如速度上限、队列长度、超时和 provider 类型；运行中的传感器帧、任务结果和急停属于事件，应使用 Topic 或 Action。项目通过 YAML 给出默认配置，Launch 在部署时覆盖，节点启动后把关键参数读成成员或不可变配置。这样一次任务使用的阈值不会在执行中途无意变化。

源码：

- 参数入口：[agent_parameters.py](../../../src/embodied_agent_core/embodied_agent_core/agent_parameters.py)
- 音频部署参数：[voice_frontend_launch_contract.py](../../../src/embodied_agent_bringup/embodied_agent_bringup/voice_frontend_launch_contract.py)
- 导航参数覆盖：[voice_nav2_turtlebot3.launch.py](../../../src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py)

关键代码含义：

```python
ParameterValue(configs[name], value_type=value_type)
# Launch 的替换值原本是字符串，这里显式恢复 bool、int、float 类型，
# 防止 "false" 被节点当成非空字符串处理。
```

边界：项目主要采用启动期配置，没有把全部参数做成运行时动态重配置。若允许动态修改安全阈值，还需要校验回调、原子更新和审计记录。

## Q5. Launch 文件解决什么问题？

口述：

Launch 不只是同时启动几个可执行文件，它还负责参数覆盖、条件启动、命名空间、组合部署和生命周期启动顺序。项目的在线与离线 Agent 共用语音前端和安全控制部署契约，避免两个入口逐渐出现不同默认值。导航演示再组合 Gazebo、Nav2、控制节点和 Agent，并统一传入仿真时间。

源码：

- Agent 部署契约：[agent_deployment_launch_contract.py](../../../src/embodied_agent_bringup/embodied_agent_bringup/agent_deployment_launch_contract.py)
- 在线入口：[online_agent.launch.py](../../../src/embodied_online_agent/launch/online_agent.launch.py)
- Nav2 总入口：[voice_nav2_turtlebot3.launch.py](../../../src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py)

运行：生命周期管理器按 `action_guard`、Agent 的顺序激活，停机时按逆序处理。这样上游 Agent 先停止产生命令，下游安全节点最后退出。

## Q6. Lifecycle Node 和普通 Node 有什么区别？

口述：

普通 Node 创建后通常立即工作，Lifecycle Node 把资源准备、对外服务和释放分成明确状态。项目在 `configure` 阶段创建调度器、订阅者、Action client 和 timer；在 `activate` 后才发布状态并接受命令；`deactivate` 时先清队列、取消活动目标，再关闭 publisher。这样“进程存在”不等于“节点已经可以安全接单”。

源码：[typed_action_bridge_node.cpp](../../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)

关键逻辑的简化注释：

```cpp
on_configure() {
  scheduler_ = std::make_unique<ActionScheduler>(...); // 只准备资源
  // 创建 subscription、Action client、publisher 和 watchdog
}

on_activate() {
  result_pub_->on_activate();                           // 开始对外服务
}

on_deactivate() {
  events = scheduler_->clear_all("lifecycle_deactivated"); // 先取消和清队列
  process_events(events);
  result_pub_->on_deactivate();                         // 再关闭输出
}
```

取舍：生命周期增加状态转换代码，但能把启动失败、停用后置条件和恢复过程变成可测试协议。

## Q7. QoS 怎样选？能只背 Reliable 和 Best Effort 吗？

口述：

QoS 要根据数据语义选择。传感器和音频频率高、旧数据价值低，因此允许丢旧帧并兼容传感器常见配置；命令和结果数量低且不能随意丢，使用可靠传输；系统状态还要让后启动的监控节点立即拿到最近快照。重点不是背名词，而是说明丢失、积压和晚加入订阅者分别会造成什么后果。

源码：[qos_profiles.hpp](../../../src/embodied_agent_middleware/include/embodied_agent_middleware/qos_profiles.hpp)

```cpp
// 传感器：小队列，处理不过来时保留新数据，避免延迟越来越大。
sensor_qos = KeepLast(5).best_effort().durability_volatile();

// 命令：数量少且每条都有业务意义，但节点重启后不重放旧动作。
command_qos = KeepLast(depth).reliable().durability_volatile();

// 状态：可靠发送，并保存最近快照给晚加入的监控节点。
state_qos = KeepLast(depth).reliable().transient_local();
```

追问：为什么命令不用持久重放？因为机器人节点重启后自动执行历史运动命令有安全风险，恢复应由上层重新确认当前任务。

## Q8. 为什么机器人动作接口使用自定义消息？

口述：

自定义消息把动作类型、速度、时长、地点、优先级和任务标识变成可检查字段，生成的 C++/Python 类型和
ROS introspection 工具使用同一接口定义。接口不匹配可以在构建或反序列化边界暴露，字段也能被
`ros2 topic echo` 直接观察。项目仍然需要业务校验，因为“字段类型正确”不代表“字段组合合法”：
例如 STOP 不应携带速度，导航地点也必须在白名单中。

源码：[RobotCommand.msg](../../../src/embodied_agent_interfaces/msg/RobotCommand.msg)

```text
string command_id       # 贯穿候选、Action、结果和日志
string source           # 标识 online、offline 或其他来源
bool priority           # 只有停止和取消类命令允许为 true
uint8 action_type       # 使用枚举值限制动作集合
float32 linear_x
float32 angular_z
float32 duration_s
string target
string[] waypoints
```

运行：Python 把 `ActionCommand` 映射为该消息；C++ 再检查无关字段是否为空、数值是否有限、速度是否
越界、地点是否在白名单；合法消息才进入执行队列。当前主链从候选到 Action/result 都使用 typed
interface，不再保留另一套兼容协议。

## Q9. `command_id` 和 ROS Action goal UUID 有什么区别？

口述：

goal UUID 只标识一次 ROS Action 传输，而 `command_id` 是项目端到端的业务标识。一个命令在进入 Action 前已经经过 Topic、校验和队列，结果还要回到 Python 动作批次，所以不能只依赖 Action 内部 UUID。调度器还用 `command_id` 拒绝重复请求和忽略迟到结果。

源码：

- ID 生成与等待：[action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)
- 去重与终态关联：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)

运行：`SequentialActionPublisher` 生成 `agent-action-N`，结果放入 `_results_by_id`；C++ 只在返回 ID 等于当前活动命令时推进队列。旧 Action 的迟到结果不会结束新任务。

## Q10. Callback Group 和 Executor 为什么会影响正确性？

口述：

ROS 回调并不是自动并行安全的。项目把命令订阅放在互斥组，保证入队顺序；Action 的 goal response、feedback、result 和取消 watchdog 放在可重入组，避免长任务事件互相饿死。调度状态、goal handle 和诊断文本分别使用不同锁，调用 ROS Action API 前先释放调度锁，减少重入死锁和长临界区。

源码：[typed_action_bridge_node.cpp](../../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)

运行：

1. `on_command()` 在命令组内修改纯调度状态。
2. 它取出 `SchedulerEvent` 后释放 `scheduler_mutex_`。
3. `process_events()` 再执行发送 goal、取消 goal 或发布结果。
4. Action 回调可以进入可重入组，并用 `goal_mutex_` 保护 handle。

边界：callback group 只规定“哪些回调可以并发”，真正使用多线程还需要 `MultiThreadedExecutor` 或多线程 component container。

## Q11. 项目中的 Service 为什么适合 TTS？

口述：

离线 TTS 的一次输入是文本，一次输出是完整 PCM，时长有上限且不需要中途取消，因此项目提供了 `SynthesizeSpeech` Service。C++ 服务节点在启动时加载模型，避免每次请求重复初始化；Python client 异步调用，并用单调时钟做超时。在线 TTS 是持续音频增量，所以使用持久 WebSocket，而不是复用这个 Service。

源码：

- 服务端：[summer_tts_service_node.cpp](../../../src/embodied_agent_cpp/src/summer_tts_service_node.cpp)
- Python client：[summer_tts_ros.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/summer_tts_ros.py)
- 接口：[SynthesizeSpeech.srv](../../../src/embodied_agent_interfaces/srv/SynthesizeSpeech.srv)

取舍：如果离线合成也要逐块播放、反馈进度和取消，更合适的接口会是 Action 或流式 Topic，而不是返回巨大响应的 Service。

## Q12. Colcon 多包构建解决了什么问题？

口述：

工作空间按接口、中间件、核心逻辑、在线离线 Agent、控制、仿真、导航和 SLAM 拆包。每个包在 `package.xml` 声明运行依赖，CMake 或 `setup.py` 声明构建与安装规则；Colcon 根据依赖图按顺序构建，并能选择单包测试。接口包必须先生成消息和 Action 代码，使用它们的 C++、Python 包才能编译。

源码：

- 接口生成：[embodied_agent_interfaces/CMakeLists.txt](../../../src/embodied_agent_interfaces/CMakeLists.txt)
- C++ 目标：[embodied_agent_cpp/CMakeLists.txt](../../../src/embodied_agent_cpp/CMakeLists.txt)
- CI 构建：[ros2-ci.yml](../../../.github/workflows/ros2-ci.yml)

常用回答：

```bash
colcon build --symlink-install
colcon test --packages-select embodied_agent_cpp
colcon test-result --verbose
```

边界：多包不是越细越好。拆分依据应是依赖方向、发布边界和独立测试需要，而不是每个类建一个包。
