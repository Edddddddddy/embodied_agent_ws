# 架构与知识笔记

本文是项目唯一的结构与原理说明。它回答三个问题：数据怎样流动、关键实现在哪里、
为什么这样划分 C++ 与 Python。

## 1. 系统分层

```text
音频输入层       C++ PortAudio -> AEC -> VAD -> silence timeout
模型交互层       Online(Qwen) / Offline(ZipFormer + llama.cpp + Sherpa-TTS)
对话编排层       唤醒与重试 -> 记忆 -> LLM -> speech/action parser
动作可信层       C++ ActionGuard -> schema / clamp / reject
执行层           Gazebo controller / UART / SPI / mock
观测与验收层     state / metrics / ACK / odom / tests
```

核心 seam 是 ROS 话题和 provider interface。声学/控制的实时实现不会依赖云 SDK，
模型 adapter 也不直接操作电机。这种隔离让在线、离线和 mock 能复用相同动作安全链。

## 2. ROS 包与关键代码

### `embodied_agent_interfaces`

`RobotCommand.msg` 将 move、turn、stop、wave、LED 和模式切换表达为强类型字段，并携带
command id、source 和时间戳。`ExecuteRobotCommand.action` 定义后续执行所需的 goal、
feedback、result、取消、超时和阻塞状态。迁移期间 ActionGuard 同时发布旧 JSON 和新
`/robot/action_command_typed`，因此既不破坏现有执行器，也为 Action/BT 链提供稳定 seam。

`typed_action_bridge` 将 typed topic 转成 `/robot/execute_command` Action goal，并把
feedback/result 转发为可观察话题。仿真 Action server 复用纯 C++ `ActionExecution`
状态机，统一成功进度、用户取消、目标抢占、雷达阻塞和硬超时语义；任一终止路径都会
调用 `SimulationController::stop()`。

`CommandBehaviorTree` 是执行编排的深模块。外部只提交 typed command，并在每个控制周期
给出 safety/ActionExecution snapshot；内部由 `command_tree.xml` 的 ReactiveSequence 执行
`ValidateCommand -> CheckSafety -> ExecuteCommand -> ConfirmResult`。ReactiveSequence 会在
动作 RUNNING 时重新检查安全条件，因此新障碍可以 halt Execute；树的阶段和终态发布到
`/robot/bt_status`。`use_behavior_tree:=false` 保留旧 ActionExecution 直连路径。

`RobotExecutor` 是 BT 下方的 pluginlib seam：Lifecycle 节点只调用 configure、execute、
stop、update_scan 和 step，不知道具体 backend。`GazeboRobotExecutor` 封装原有
SimulationController、LaserScan 安全和 PID；`MockRobotExecutor` 不依赖仿真或传感器，供
CI 和插件教程使用。插件名由 `executor_plugin` 参数选择，新增 executor 不需修改 Guard、
BT XML 或 Action server。

ActionGuard 与 SimulationControl 都是 `rclcpp_lifecycle::LifecycleNode`。configure 阶段
创建 publisher、subscription、Action server 和 timer；activate 后才转发动作和发布速度；
deactivate 会终止活动目标并在 publisher 停用前发送零速；cleanup 释放 ROS interface。
launch 使用 `nav2_lifecycle_manager` 自动执行配置和激活，`bond_timeout=0` 用于管理原生
rclcpp lifecycle node，而不是 Nav2 自带 bond 的节点基类。

### `embodied_agent_cpp`

| 文件 | 责任 |
|---|---|
| `audio_processing.hpp/.cpp` | NLMS AEC、能量 VAD、静音检测，纯计算可单测 |
| `audio_frontend_node.cpp` | PortAudio 回调、有界队列、PCM 发布和播放 |
| `action_validator.hpp/.cpp` | 动作白名单、参数 schema、速度与时长限幅 |
| `action_guard_node.cpp` | Lifecycle Guard；仅 active 时将 candidate 转成可信动作 |
| `hardware_protocol.hpp/.cpp` | 版本、序号、payload 和 CRC 帧 |
| `hardware_transport.hpp/.cpp` | mock、UART、SPI adapter |
| `hardware_controller_node.cpp` | 发送、ACK、watchdog 和急停 |

PortAudio 回调只搬运数据，不执行网络、日志或模型推理；这是实时音频最重要的约束。
动作校验是深模块：调用方只提交 JSON，schema、限幅和拒绝原因隐藏在统一 interface 后。

### `embodied_online_agent`

| 文件 | 责任 |
|---|---|
| `online_agent_node.py` | ROS 与一轮在线对话的编排 |
| `providers/` | mock、Qwen 实时 ASR/TTS、OpenAI-compatible LLM adapter |
| `protocol.py` | `<speech>/<action>` 增量解析与按句 TTS 分块 |
| `command_fallback.py` | 有限机器人命令的确定性语义兜底 |
| `wakeword.py` | 文本唤醒窗口与兼容别名 |
| `recognition_retry.py` | 可观测重试计数，不锁死监听 |
| `memory.py` | 有界、原子写入的对话记忆 |
| `metrics.py` | LLM 首 token 与 TTS 首音频时延 |

`providers/base.py` 是真实 seam：同一 interface 至少有 mock 与云端两个 adapter，测试可以
不访问外网。`types.py` 中的 `ActionCommand` 和 `LatencySnapshot` 分别作为动作解析与
延迟观测的值对象；二者都有实际调用方，并通过回归测试保护。

### `embodied_offline_agent`

| 文件 | 责任 |
|---|---|
| `offline_agent_node.py` | 离线 ASR、LLM、TTS 并发编排 |
| `providers/sherpa_asr.py` | ZipFormer 流式识别与 hotword biasing |
| `providers/llama_cpp.py` | 本机 llama-server 流式接口 |
| `providers/sherpa_tts.py` | VITS/Melo-TTS 合成 |
| `double_buffer.py` | 容量为 2 的消息/音频缓冲与背压 |
| `latency.py` | 静音到 final、首 token、首音频及整轮耗时 |

离线的“伪流式”不是模型逐帧生成音频，而是 LLM 按句输出、TTS 逐句合成，再将 PCM
切成 80 ms 块播放。两个双缓冲使 LLM、TTS 和播放重叠执行，同时限制内存增长。

### `embodied_simulation`

| 文件 | 责任 |
|---|---|
| `simulation_controller.hpp/.cpp` | 手动运动、雷达停车、避障、沿墙 PID |
| `command_behavior_tree.hpp/.cpp` | BT blackboard、异步状态与统一终态 |
| `config/command_tree.xml` | Validate → Safety → Execute → Confirm 编排 |
| `robot_executor.hpp` | Gazebo/mock 共用的小型执行 interface |
| `robot_executor_plugins.cpp` | pluginlib Gazebo 与 mock adapters |
| `simulation_control_node.cpp` | Lifecycle Action server、LaserScan、Twist、ACK 的 ROS seam |
| `simulation_control_main.cpp` | 独立进程入口与双线程 MultiThreadedExecutor |
| `simulation_control_factory.hpp` | 独立入口复用 component 实现的工厂 seam |
| `node_configuration.cpp` | configure 前的控制参数与插件名交叉校验 |
| `executor_diagnostics.cpp` | 从线程安全快照构造标准 diagnostics，统一等级与字段契约 |
| `simulation_control.launch.py` | 独立/组合部署、namespace 与 Lifecycle 管理 |
| `voice_turtlebot3.launch.py` | Agent、Guard、Gazebo、bridge 的组合启动 |

当前产品主链只承诺 `move / turn / stop`；避障和沿墙用于展示安全控制模块，不继续扩展
成完整导航栈。Gazebo ACK 和 `/odom` 位移共同证明命令确实经过了仿真执行器。

`SimulationControlNode` 只实现一次并注册为 `rclcpp_components` component；独立可执行文件
通过 factory 创建同一个类，因此两种部署不会形成两份控制逻辑。独立模式使用双线程
`MultiThreadedExecutor`，diagnostics timer 位于单独的互斥 callback group；控制快照通过
mutex/atomic 读取，诊断发布不会与 Action 控制状态发生数据竞争。

控制节点所有执行相关 topic 和 Action 都使用相对名称，launch 的 `namespace` 会同时作用于
component container、Lifecycle manager、控制组件和 typed bridge。速度/动作/状态使用
reliable QoS，LaserScan 使用 sensor-data QoS。标准 `/diagnostics`（加 namespace 后为
`/<namespace>/diagnostics`）报告 lifecycle state、executor plugin/backend、control mode、
active action、sensor stale、safety stopped 与原因。

## 3. 关键话题

| 话题 | 生产者 -> 消费者 | 含义 |
|---|---|---|
| `/audio/clean_pcm` | AudioFrontend -> ASR | AEC 后 PCM16 |
| `/audio/silence_timeout` | AudioFrontend -> Agent | 连续静音 0.4 秒 |
| `/agent/asr_final` | ASR -> 观测者 | 最终识别文本 |
| `/agent/recognition_feedback` | Agent -> UI/验收器 | 失败原因、次数和重试提示 |
| `/agent/action_candidate` | parser -> ActionGuard | 未可信动作 JSON |
| `/robot/action_command` | ActionGuard -> executor | 已校验、已限幅动作 |
| `/robot/action_command_typed` | ActionGuard -> 新 executor | 等价的强类型可信动作 |
| `/robot/execute_command` | Action client -> SimulationController | 可取消、反馈、超时的 ROS Action |
| `/robot/action_feedback` | typed bridge -> 观测者 | command id、阶段、进度和执行原因 |
| `/robot/action_result` | typed bridge -> 观测者 | 成功、拒绝、取消、阻塞或超时结果 |
| `/robot/bt_status` | SimulationController -> 观测者 | command id、BT 阶段、终态与原因 |
| `/robot/action_rejected` | ActionGuard -> 观测者 | schema 或安全拒绝原因 |
| `/robot/action_ack` | executor -> 观测者 | 接受、执行结果及实际 backend |
| `/cmd_vel` | SimulationController -> Gazebo | 标准差速速度 |
| `/scan`, `/odom` | Gazebo -> controller/probe | 雷达与物理位移证据 |

## 4. 流式语音与延迟

- AEC：NLMS 用播放 PCM 作为参考估计回声；真实设备需校准 delay、step、taps，并测双讲。
- VAD：能量阈值负责区分语音/静音，静音检测器累计 400 ms 后只提交一次 utterance。
- ASR：在线采用实时 WebSocket；离线 ZipFormer 采用流式 transducer 和 modified beam search。
- 唤醒：当前文本门控前有热词偏置，失败会回到 `retry_listening`。生产级应增加声学 KWS。
- LLM：token 到达即进入标签解析器；动作必须等完整 JSON，speech 可按标点提前送 TTS。
- TTS：在线保持连接并流式返回 PCM；离线用按句合成模拟流式体验。

需要区分四种延迟：ASR finalization、LLM first token、TTS first audio、end-to-first-audio。
平均值不足以证明体验，正式报告应记录至少 100 轮 P50/P95 和冷/热启动。

## 5. 动作与安全原则

模型输出格式：

```text
<speech>好的，向前走一秒。</speech>
<action>{"name":"move","arguments":{"linear_x":0.2,"duration_s":1.0}}</action>
```

LLM 输出永远视为不可信输入。ActionGuard 只接受白名单动作，拒绝多余/缺失字段，限制
速度、角速度、持续时间和枚举值。硬件控制器还有 watchdog；真实机器人仍必须保留独立
急停、碰撞、电机限流和 MCU 侧超时。

## 6. 量化、训练与事实边界

- Qwen3-0.6B Q8_0 通过 llama.cpp CPU 推理；Q8 相对 FP16 通常约一半，不是四分之一。
- `training/` 只有种子数据、dataset 注册和 LoRA 配置；没有执行训练就不能宣称 85%。
- 小数据集上的确定性 command fallback 提升的是工程链正确率，不等于模型准确率。
- 模型、`third_party/llama.cpp`、构建目录与密钥均被 Git 忽略，仓库保持轻量。

## 7. 结构审计与下一次重构

本轮保留在线和离线两个 ROS 节点，因为它们的 provider 生命周期、ASR threading 和
TTS 策略确实不同。二者仍重复“唤醒 -> busy -> turn -> publish -> metrics”编排，下一轮
应先定义一个小型 `TurnCoordinator` interface，通过 mock adapter 写端到端行为测试，
然后替换重复代码，而不是再叠一层工具函数。

集成探针已统一迁到 `tests/integration/`，`scripts/` 只保留用户命令与负责进程生命周期的
runner。仓库约束测试会阻止 `scripts/test_*` 再次出现。最大的剩余热点仍是
`simulation_control_node.cpp`；本轮先提取 diagnostics 深模块，使等级、字段和并发快照可
独立测试，后续只在出现第二个明确职责簇时继续拆分，避免产生一批浅层转发类。

## 8. 新增 RobotExecutor 插件教程

新增机器人后端不应修改 Action server、BT XML 或 Guard。最小步骤如下：

1. 新建类继承 `embodied_simulation::RobotExecutor`，实现 `configure`、`execute`、`stop`、
   `update_scan`、`step`、`mode_name` 和 `backend_name`。ROS 节点负责通信，插件只负责控制
   语义，避免在插件内再创建一套 node/executor。
2. 在实现文件末尾使用 `PLUGINLIB_EXPORT_CLASS` 注册类，并在
   `robot_executor_plugins.xml` 增加 `<class>`；插件名采用
   `项目名/后端名RobotExecutor`，例如 `my_robot/MyRobotExecutor`。
3. 将实现编译进共享库，通过 `pluginlib_export_plugin_description_file` 导出描述文件，
   package.xml 声明 pluginlib 与消息依赖。
4. 先扩展 `test_robot_executor_plugins.cpp`：要求 pluginlib 能真实发现并实例化插件，同一
   `RobotCommand` 能执行、step 并 stop。再增加一条无模型 ROS smoke，验证 Action result、
   BT confirm 和 diagnostics 中的 backend 名称。
5. 启动时传入
   `executor_plugin:=my_robot/MyRobotExecutor`；若必须修改 BT/Guard 才能切换，说明插件
   interface 泄漏了机器人细节，应先重新收紧 seam。

插件必须遵守三条契约：`stop()` 可重复且立即归零；每个 terminal path 最终停车；
`step()` 不阻塞 ROS executor。UART/SPI 若要迁入该 seam，应让 transport 继续作为更底层
adapter，而不是把串口重试、CRC 和 ROS Action 全塞进一个类。

## 9. 求职版项目描述

**具身智能机器人端侧语音交互与仿真控制系统｜ROS 2 Jazzy / C++17 / Python**

- 打通麦克风、流式 ASR、LLM 结构化动作、C++ 安全 Guard、ROS 2 Action、
  BehaviorTree.CPP、pluginlib executor 到 TurtleBot3 Gazebo `/cmd_vel`/`odom` 的在线与
  全离线闭环，并保留 UART/SPI adapter。
- 将控制链规范化为 LifecycleNode 与可取消 Action；通过 BT 实现 Validate→Safety→Execute
  →Confirm，支持取消、抢占、障碍、超时和急停；同一实现支持独立进程、component
  container 和 namespace 隔离。
- 部署 Qwen3-0.6B Q8/llama.cpp、ZipFormer 与 Sherpa-TTS；当前环境 CPU decode
  34.10 token/s、离线整轮 2.313 s、在线热启动首 token 350–384 ms；建立 130 项 colcon
  测试、2 项仓库约束测试及
  mock/online/offline/Gazebo 分层 release gates。

面试时必须主动说明：LoRA 尚未训练；2/8 是原始模型成绩，7/8 是 fallback 后系统成绩；
冷启动在线 LLM 不达 1 秒；所有性能数字均是当前机器少量样本，不是生产 SLA。

## 10. 面试问题与回答要点

### 为什么模型不能直接发布 `/cmd_vel`？

LLM 输出是不可信文本，可能格式错误、越界或幻觉。项目先转为强类型 command，经 C++
schema/限幅，再由 Action/BT 检查生命周期与安全状态；真正的速度只由 executor 发布。

### ROS topic、service、Action 在这里如何取舍？

音频、雷达和状态是连续数据，用 topic；Lifecycle transition 是短请求，用 service；运动有
持续时间、反馈、取消和 terminal result，用 Action。旧 JSON topic 仅作为兼容 gateway。

### 为什么同时需要 Lifecycle 和 Behavior Tree？

Lifecycle 管节点资源与启动顺序，回答“控制器是否可以工作”；BT 管一次任务的业务阶段，
回答“这个动作如何验证、执行和确认”。两者状态机作用域不同，不能互相替代。

### MultiThreadedExecutor 如何避免数据竞争？

主控制 callback 使用默认互斥组，diagnostics timer 使用独立互斥组；跨组只共享 atomic
动作标志与 mutex 保护的 `ControllerOutput`/后端快照，诊断线程不直接调用可变 executor。

### pluginlib 带来了什么，不只是“为了设计模式”吗？

它让 BT/Action 只依赖小型 `RobotExecutor` interface。Gazebo 与 mock 经过同一动态发现
测试和同一 command 行为测试；新增后端通过参数选择，不需要改核心节点，seam 有可验证
的替换价值。

### 如何证明机器人真的执行了，而不是话题发成功？

验收同时要求非零 `/cmd_vel`、Gazebo `/odom` 物理位移、Action success、BT
`confirm/succeeded` 和 ACK backend；任何单一消息都不足以算闭环完成。

### ASR 有错别字为什么还能动作？风险是什么？

热词、唤醒别名和有限命令 fallback 可恢复常见同音/噪声文本，失败则继续监听。fallback
只覆盖白名单动作并仍经过 Guard；它提高系统可用率，但不能冒充模型准确率，也不能无限
扩充字符串规则替代声学 KWS。
