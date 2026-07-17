# ROS 2 / C++ 控制学习笔记

覆盖工作区、typed 接口、ActionGuard、ActionScheduler、BehaviorTree.CPP、pluginlib、QoS、Lifecycle、readiness 与测试证据。

## 1. WSL 工作区、colcon overlay 与可重复激活

### 【功能】

把 ROS 2 Jazzy、Python 虚拟环境、当前工作区的 colcon 产物和 `.env` 组合为同一运行环境。
显式 `WORKSPACE` 让主仓库与 Git worktree 不会误用彼此的安装树或配置。

### 【关键文件/类/函数】

- `scripts/bootstrap.sh`：安装依赖、创建 `.venv`、安装 requirements、执行 colcon build。
- `scripts/activate.sh`：依次 source ROS、venv、当前 overlay 和 `.env`。
- `scripts/ros_dds_env.sh`：统一 DDS transport 环境变量。
- `package.xml`、`CMakeLists.txt`、`setup.py`：ament/colcon 包发现与安装契约。

### 【上游 → 处理 → 下游】

```text
/opt/ros/jazzy/setup.bash（underlay）
→ .venv + PYTHONPATH
→ $WORKSPACE/install/setup.bash（overlay）
→ .env
→ ros2 run / ros2 launch / pytest / acceptance_test.sh
```

### 【为什么这样设计】

ROS 2 生成的 Python console script 常使用系统 Python shebang，而模型依赖安装在 venv。
激活脚本显式暴露 venv site-packages，避免“当前 shell 能 import，ros2 run 却不能 import”。
`WORKSPACE` 作为唯一根目录，也比在每个脚本中猜 `pwd` 更适合 worktree。

### 【与替代方案区别】

- 只 source `/opt/ros/jazzy`：看不到项目接口和节点。
- 只激活 venv：ament index 找不到 ROS package。
- 全局 pip 安装：短期简单，容易污染系统 ROS Python ABI。
- 容器：隔离更强，但 WSLg 麦克风和 Gazebo GUI 接线成本更高。

### 【失败/安全边界】

`install/setup.bash` 不存在表示尚未构建；worktree 未设置 `WORKSPACE` 可能加载主工作区旧产物。
不要把删除整个 `build/install/log` 作为第一反应，应先确认路径、overlay 和包版本。

### 【对应测试】

```bash
export WORKSPACE="$PWD"
source scripts/activate.sh
bash scripts/acceptance_test.sh core
```

## 2. C++ ActionGuard 与 ActionScheduler

### 【功能】

在模型与执行器间建立可信边界：Guard 校验和限幅候选动作；Scheduler 管理单 active goal、FIFO、
优先取消、失败清队列、watchdog 和 command ID 结果关联。

### 【关键文件/类/函数】

- `src/embodied_agent_cpp/src/action_guard_node.cpp`：`ActionGuardNode::on_candidate()`。
- `src/embodied_agent_cpp/src/action_validator.cpp`：`ActionValidator::validate()`。
- `src/embodied_agent_cpp/src/action_scheduler.cpp`：`ActionScheduler::enqueue()`、`complete()`、
  `clear_all()`、`dispatch_next()`。
- `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp`：`on_command()`、`process_events()`、
  `dispatch_goal()`、`request_cancel()`。
- `src/embodied_agent_cpp/src/typed_action_demo_client.cpp`：独立 `rclcpp_action` client 示例，展示
  goal、feedback、result、cancel 和 timeout 的标准调用方式。
- `src/embodied_agent_cpp/include/embodied_agent_cpp/guarded_command_outbox.hpp`：`GuardedCommandOutbox`。

### 【上游 → 处理 → 下游】

```text
/agent/action_candidate → ActionGuardNode::on_candidate()
→ ActionValidator::validate() → /robot/action_command_typed 或 rejection
→ TypedActionBridgeNode::on_command() → ActionScheduler::enqueue()
→ ExecuteRobotCommand goal/cancel → typed feedback/result/diagnostics
```

### 【为什么这样设计】

模型层变化快且可能幻觉，控制 policy 需要稳定、可单测、低延迟。Guard 回答“能否执行”，Scheduler
回答“何时执行、取消谁、结果属于谁”。outbox 只覆盖 discovery 短窗口，并用容量和 TTL 防旧命令回放。

### 【与替代方案区别】

LLM 直接 `/cmd_vel` 没有 schema、限幅和取消；Python 调度开发快，但 C++ 更贴近 ROS 2 执行生命周期；
把校验与调度塞进一个节点会让纯策略和并发状态难以独立测试。

### 【失败/安全边界】

Guard 不能修复“左”被识别成“右”这种合法但错误的语义，只保证参数边界。deactivate/cleanup 必须
取消 active goal、清 pending、发布终态并停车。priority 明确区分用户急停和计划 STOP。

### 【对应测试】

```bash
colcon test --packages-select embodied_agent_cpp --event-handlers console_direct+
bash scripts/acceptance_test.sh cpp-action-client
bash scripts/acceptance_test.sh gazebo
```

## 3. BehaviorTree.CPP、pluginlib 与 Gazebo 执行层

### 【功能】

Action server 接收长动作，BehaviorTree 依次做校验、安全检查、执行和确认；pluginlib 在 Mock、
Gazebo、Nav2 Executor 间切换，上游接口保持不变。

### 【关键文件/类/函数】

- `src/embodied_simulation/src/simulation_control_node.cpp`：`handle_goal()`、`control_tick()`、
  `update_active_action()`、`finish_active_action()`。
- `src/embodied_simulation/src/command_behavior_tree.cpp`：`CommandBehaviorTree::start()`、`tick()`、
  `cancel()`；`ValidateCommandNode`、`CheckSafetyNode`、`ExecuteCommandNode`、`ConfirmResultNode`。
- `src/embodied_simulation/src/robot_command_policy.cpp`：`is_executable_robot_command()`，定义
  Action goal 与 BT 共同使用的执行层命令契约。
- `src/embodied_simulation/include/embodied_simulation/robot_executor.hpp`：`RobotExecutor`。
- `src/embodied_simulation/src/gazebo_robot_executor.cpp`：`GazeboRobotExecutor::execute()`、`step()`。
- `src/embodied_simulation/src/simulation_controller.cpp`：`SimulationController::update_scan()`、`step()`。
- `src/embodied_simulation/src/simulation_ros_io.cpp`：`publish_velocity()`、`publish_zero_velocity()`。

### 【上游 → 处理 → 下游】

```text
ExecuteRobotCommand goal → SimulationControlNode::handle_goal()
→ ActiveActionRuntime + CommandBehaviorTree::tick()
→ RobotExecutor::execute()/step() → GazeboRobotExecutor
→ /cmd_vel → Gazebo /odom + /scan → Action result
```

### 【为什么这样设计】

Action server 管外部协议，BT 表达业务顺序，Executor 隐藏后端，Controller 封装速度和雷达安全。
mock 验证状态机、Gazebo 验证物理运动、Nav2 验证规划控制，无需复制 Action/Lifecycle 逻辑。
命令合法性不能分别写在 Action Server 和 BT 节点里，否则启用/关闭 BT 后可能出现两套安全行为；
共享 policy 让动作类型、时长、模式和 accessory 白名单只有一个权威实现。

### 【与替代方案区别】

switch/case 在小动作域简单，扩展取消和终态后难维护；Nav2 BT 适合一次导航，不适合销毁整套 mapping
graph；每个后端独立节点隔离强，却会重复 Action server、诊断和超时逻辑。

### 【失败/安全边界】

雷达无效、紧急障碍、取消、超时或插件异常都必须发布零速度。`/cmd_vel` 有数据不等于成功，还需
odom、Action result 和最终停车。mock PASS 不代表 Gazebo 时钟、TF 和物理插件正常。
执行层 policy 不是 ActionGuard 的替代品：Guard 负责来源、payload、限幅和 `ARC→MOVE` 规范化，
policy 只防止未经规范化或数值非法的命令进入具体 Executor。

### 【对应测试】

```bash
colcon test --packages-select embodied_simulation --event-handlers console_direct+
bash scripts/acceptance_test.sh gazebo
```

## 4. 可观测性、分层测试与事实证据

### 【功能】

把音频、ASR、session、NLU、队列、Action、BT、仿真、SLAM 和 Nav2 变成可观察事件；以单测、
repository contract、stage、Gazebo、真人麦克风和公开 bag 分层证明。

### 【关键文件/类/函数】

- `scripts/continuous_voice_monitor.py`：`ContinuousVoiceMonitor`、`MonitorStats.format_summary()`、
  `format_advice()`、`AsrNluSampleRecorder`。
- `src/embodied_agent_middleware/src/system_readiness_node.cpp`：`publish_readiness()`。
- `src/embodied_simulation/src/simulation_ros_io.cpp`：ACK、BT status、health、diagnostics。
- `scripts/showcase_release_gate.py`：`GateCommand`、`_run_command()`、`main()`。
- `scripts/generate_architecture_facts.py`：`build_facts()`、`render_markdown()`。
- `scripts/acceptance_test.sh`、`tests/repository/`、各包 `test/`、`tests/integration/`。

### 【上游 → 处理 → 下游】

```text
typed events + TF/map/odom/cmd_vel
→ monitor/readiness/integration probes
→ command ID、状态迁移和指标聚合
→ terminal + JSON/Markdown report
→ release/demo/robotics gate → CI 或人工结论
```

### 【为什么这样设计】

链路跨音频、网络、Python、DDS、C++、Gazebo 和 Nav2，仅看最终“不动”无法定位。typed 状态能区分
没录音、ASR 拒绝、队列满、Guard 拒绝、Action 失败、Nav2 aborted 和 TF 缺失；失败报告也要保留。

### 【与替代方案区别】

只看 INFO 日志难做断言；只做单测不能证明 DDS/TF/Gazebo；只做重型 E2E 反馈慢。测试金字塔让
单测定位、stage 验接口、重型/真人/公开数据提供最终证据。

### 【失败/安全边界】

mock、fixture、Gazebo、真人麦克风、公开 bag 和实体硬件不能互相替代。报告 PASS 可能只表示契约
完整，算法发布仍要检查 metric/release decision。历史报告必须绑定日期、commit、配置和输入来源。

### 【对应测试】

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh robotics-gate
bash scripts/acceptance_test.sh slam-nav-showcase-stage
```

## 5. DDS、QoS、Lifecycle 与系统 readiness

### 【功能】

为命令、事件、状态、传感器、音频和诊断定义不同 DDS 语义；用 Lifecycle 管理 provider 和 Action
资源；用组件健康聚合判断整套 launch 是否真正可以接收任务。

### 【关键文件/类/函数】

- `src/embodied_agent_middleware/include/embodied_agent_middleware/qos_profiles.hpp`：
  `command_qos()`、`event_qos()`、`state_qos()`、`sensor_qos()`、`audio_qos()`。
- `src/embodied_agent_core/embodied_agent_core/ros_qos.py`：Python 同名 QoS。
- `src/embodied_agent_core/embodied_agent_core/agent_lifecycle_runtime.py`：
  `AgentLifecycleRuntime.activate()`、`deactivate()`、`release()`。
- `src/embodied_agent_middleware/include/embodied_agent_middleware/component_health_registry.hpp`：
  `ComponentHealthRegistry`。
- `src/embodied_agent_middleware/src/system_readiness_node.cpp`：`SystemReadinessNode`。
- `src/embodied_agent_cpp/src/action_guard_node.cpp`：`flush_outbox()` 同时验证
  Agent→Guard 与 Guard→Scheduler 两段 DDS discovery 后才发布 READY。

### 【上游 → 处理 → 下游】

```text
节点 configure/activate/deactivate
→ ComponentHealth + typed state/event
→ ComponentHealthRegistry
→ SystemReadinessNode::publish_readiness()
→ launch、会话编排器和验收探针决定是否放行任务
```

命令/事件使用 reliable + volatile；当前状态使用 reliable + transient-local；scan/PCM 使用
best-effort + volatile；diagnostics 使用 reliable。命令不使用 transient-local，避免节点重启后
重放旧动作。

### 【为什么这样设计】

状态晚加入订阅者需要拿到最新快照，所以用 transient-local；高频 PCM/scan 更重视低延迟，消费
落后时应丢旧帧。Lifecycle 让“进程存在”和“资源已准备”成为不同状态，停机可以先停车、取消
goal，再释放线程和 provider。

### 【与替代方案区别】

- 所有 topic 都 reliable：音频积压会放大端到端延迟。
- 所有 topic 都 best-effort：控制结果丢失会破坏队列关联。
- 普通 Node：启动简单，无法表达资源 configure/activate/deactivate 边界。
- 只检查 PID：进程活着不代表模型、Action server 或 TF 已就绪。

### 【失败/安全边界】

DDS discovery 有时间窗；ActionGuard readiness 必须同时看到上下游端点，不能只把 Lifecycle ACTIVE
当作通信链路 ready。已校验命令只由有容量和 TTL 的 outbox 暂存。Fast DDS SHM 锁和过大的
`ROS_DOMAIN_ID` 属于环境故障，不应通过放宽安全规则规避。readiness 快照不是永久有效心跳，
重型冷启动要使用合适的 stale window。

### 【对应测试】

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh robotics-gate
```

## 6. Typed msg/srv/action 作为跨进程契约

### 【功能】

用 rosidl schema 表达动作、队列、执行、VAD/KWS、健康、SLAM 回环和动态障碍，使 Python/C++ 节点
在编译与 discovery 阶段共享字段、枚举和时间戳语义。

### 【关键文件/类/函数】

- `src/embodied_agent_interfaces/msg/RobotCommand.msg`、`RobotCommandFeedback.msg`、
  `RobotCommandResult.msg`。
- `src/embodied_agent_interfaces/msg/CommandContext.msg`、`CommandQueueEvent.msg`、
  `CommandExecutionEvent.msg`、`NluParseEvent.msg`。
- `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action`、`ManageSlamSession.action`。
- `src/embodied_agent_interfaces/srv/SynthesizeSpeech.srv`。
- `src/embodied_agent_core/embodied_agent_core/ros_action_transport.py`：`action_command_to_message()`。
- `src/embodied_agent_core/embodied_agent_core/ros_event_transport.py`：
  `queue_event_to_message()`、`execution_event_to_message()`、`nlu_parse_to_message()`。

### 【上游 → 处理 → 下游】

```text
Python 领域 ActionCommand / queue event
→ transport Adapter
→ rosidl Python/C++ message
→ DDS
→ C++ Guard、Scheduler、simulation 或 monitor
→ typed feedback/result
→ Adapter 恢复报告字典
```

### 【为什么这样设计】

跨进程协议不能靠每个节点自行拼字典。typed message 固定字段、数组和枚举，C++/Python 由同一 IDL
生成。报告层仍可序列化 JSON，不让文件格式反向污染实时控制协议。

### 【与替代方案区别】

- `String + JSON`：原型快，无编译期 schema，字段漂移运行时才发现。
- protobuf/gRPC：跨语言强，不是 ROS graph 原生工具链。
- ROS service：适合短请求响应，不适合可取消长动作。
- ROS Action：有 goal/feedback/result/cancel，适合移动、导航和会话切换。

### 【失败/安全边界】

typed 只保证结构，不保证语义安全；速度、地点和任务状态仍要校验。修改 IDL 后必须重建所有依赖
overlay，避免 Python 读取旧生成类型。JSON 只保留在报告、数据集和硬件协议边界。

### 【对应测试】

```bash
pytest -q src/embodied_agent_core/test/test_ros_action_transport.py \
  src/embodied_agent_core/test/test_ros_event_transport.py
bash scripts/acceptance_test.sh core
```
