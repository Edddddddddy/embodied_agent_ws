# 项目经历逐句追问

本章直接对应简历中的项目介绍和 5 条个人工作。每个回答先保证能口述，再给源码和调用关系。涉及算法公式或 ROS 2 基础时，进入前八册继续深挖。

## 0. 项目介绍的基础讲法

简历原句：

> 面向端侧机器人开发语音交互与自主导航系统，实现从语音输入、指令解析到机器人执行的完整流程。系统基于 ROS 2 连接 Gazebo、SLAM Toolbox 和 Nav2，支持在线/离线语音交互，主要解决连续指令处理、任务抢占、长任务取消、异步结果管理和安全执行等问题。

### Q1. 这个项目到底解决了什么问题？

口述：

它不是单独做一个语音聊天节点，而是把自然语言变成可验证的机器人任务。用户可以连续说运动命令，也可以一句话启动探索建图、存图、定位和巡检。系统重点解决模型输出不确定、多个命令连续到达、长任务可取消、结果异步返回以及机器人必须安全停车的问题。主要验收环境是 Gazebo 和 TurtleBot3。

主链：

```text
麦克风和 ASR
会话及任务队列
NLU 或 LLM 候选动作
C++ 规则校验和调度
ROS 2 Action
Gazebo 或 Nav2
typed result 和运行证据
```

### Q2. 为什么选择 ROS 2？

口述：

项目同时有高频音频和传感器流、短请求、长导航任务、坐标变换和可管理节点状态。ROS 2 的 Topic、Service、Action、TF、Launch 和 Lifecycle 正好覆盖这些交互形状，并能直接接入 SLAM Toolbox、Nav2、Gazebo 和 RViz。项目自己补充的是语音应用、安全边界、任务调度和验收，而不是重新开发整套机器人中间件。

代码入口：[voice_nav2_turtlebot3.launch.py](../../../src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py)

### Q3. 在线和离线模式共用了什么，差异在哪里？

口述：

两种模式共用会话状态机、连续队列、动作序列、ROS 接口、安全校验和执行链。差异被限制在 provider 和 turn runtime：在线使用 Qwen WebSocket ASR/TTS 与云端 LLM，离线使用 sherpa-onnx ASR、本机 llama-server 和本地 TTS。这样切换模型不会复制机器人控制逻辑。

源码：

- 共用应用层：[agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)
- 在线节点：[online_agent_node.py](../../../src/embodied_online_agent/embodied_online_agent/online_agent_node.py)
- 离线节点：[offline_agent_node.py](../../../src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py)

### Q3.1 RAG 在项目里解决什么问题？

口述：

RAG 只解决部署手册、系统原理和排障问答，不参与运动授权。本地 NLU 已识别的控制命令直接进入
typed ROS 2 Action；知识问题才检索只读文档，限制 top_k 和字符预算，并要求回答带 source_id。
在线和离线 Agent 共用同一个 Prompt 构造模块，因此切换 LLM 不会改变检索安全策略。

源码：[prompt_context.py](../../../src/embodied_agent_core/embodied_agent_core/prompt_context.py)

边界：当前是适合小型知识库的稀疏检索基线，不宣称已部署向量数据库、Self-RAG 或达到未实测准确率。

### Q4. 项目最难的部分是什么？

口述：

难点不是把几个组件启动起来，而是跨异步边界保持任务一致。语音可能重复，旧 timer 和旧 Action result 可能迟到，急停可能发生在 goal 尚未接受时，Nav2 的协议成功也可能包含漏点。项目通过唯一任务 ID、generation、单活动槽、分层超时和业务结果检查收口这些竞态。

代码证据：

- [asr_endpoint_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py)
- [action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)
- [nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)
- [nav2_result_policy.cpp](../../../src/embodied_simulation/src/nav2_result_policy.cpp)

## 1. Python 应用层、上下文和任务队列

简历原句：

> 基于 Python 和 rclpy 开发机器人应用层，负责用户上下文、会话状态和任务队列管理；通过任务 ID、状态同步和异步回调机制处理连续语音输入、重复请求和执行结果关联。

### Q5. Python 应用层具体负责什么？

口述：

Python 层接收 ASR final，判断是否已唤醒、是否为语气词或重复命令，再决定直接执行、进入连续队列、休眠或急停。它还在入队时冻结用户身份和偏好，并把模型动作发布成候选消息。它不直接发布 `/cmd_vel`，最终安全校验和任务调度在 C++ 层完成。

源码：[agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)

调用：

```text
OnlineAgentNode 或 OfflineAgentNode
AgentApplicationRuntime.accept_transcript()
AgentControlPlane.accept_transcript()
连续队列或模型 turn
publish_actions()
RobotCommand candidate
```

### Q6. 会话状态怎样管理？

口述：

连续语音会话至少有未唤醒、已唤醒和休眠语义。第一次包含唤醒词的文本激活会话，后续指令在超时窗口内无需重复唤醒；休眠词清除上次命令并关闭会话。急停词在会话忙碌时仍走优先路径，不能因普通命令去重或队列满而被丢弃。

源码：[continuous_voice.py](../../../src/embodied_agent_core/embodied_agent_core/continuous_voice.py)

关键顺序：先识别休眠和急停，再做普通重复过滤。否则连续两次“停下”可能被误当重复文本。

### Q7. 连续命令队列为什么有界？

口述：

机器人命令有时效性，不能像离线任务一样无限积压。有界队列限制内存和最坏等待时间；命令入队还记录创建时间，过期后产生 `expired` 事件而不是继续执行。只有一个 worker 串行消费，保证“前进、左转、停止”等动作顺序明确。

源码：

- 队列与事件：[continuous_voice.py](../../../src/embodied_agent_core/embodied_agent_core/continuous_voice.py)
- worker：[agent_execution_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_execution_runtime.py)

队列满时普通命令被拒绝并可提示用户；急停走旁路并清队列。

### Q8. 重复请求怎样判断？

口述：

文本先去除空白和常见标点，再在短时间窗口内比较最近普通命令。重复普通命令不再入队，避免 ASR 重复 final 导致机器人执行两次；优先停止命令不做这种抑制，因为重复停止比漏掉停止更安全。动作调度层还会按 `command_id` 做第二层重复检查。

源码：

- 文本级去重：[continuous_voice.py](../../../src/embodied_agent_core/embodied_agent_core/continuous_voice.py)
- 命令级去重：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)

这两层对象不同：第一层去重自然语言，第二层去重已经生成的业务命令 ID。

### Q9. 用户上下文为什么不能等执行时再读取？

口述：

命令可能在队列中等待，等待期间新的声纹或偏好会改变“当前用户”。因此入队时保存 `UserContextSnapshot` 和 provider 上下文，执行时恢复这份快照，保证命令、偏好、回复和结果属于同一用户 turn。快照是不可变值对象，减少共享状态竞态。

源码：

- [agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)
- [user_context_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/user_context_runtime.py)

边界：默认演示使用 mock 身份来确定性验证这条上下文链。Sherpa embedding provider 已有模型加载、
注册样本、阈值和 margin seam，但没有完成真实多人、多噪声条件的准确率验收；因此不能把 mock 身份
快照写成“已实现生产级声纹识别”。

### Q10. 任务 ID 是怎样生成和关联的？

口述：

Python 动作批次为每个候选动作生成 `agent-action-N`，并把结果按 ID 存入字典。自定义消息携带同一个 ID 经过 C++ 校验、调度和 ROS Action；结果消息再带 ID 返回。等待线程只消费自己的 ID，调度器也只允许当前活动 ID 的终态推进队列。

源码：

- ID 生成与结果等待：[action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)
- C++ 关联：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)

### Q11. “状态同步”具体同步了什么？

口述：

这里不是把所有状态复制到多个节点，而是给每类状态明确所有者。Python 拥有会话、用户上下文和语句队列；C++ 调度器拥有 active 与 pending；Action server 拥有执行终态。节点间通过 typed event、feedback 和 result 同步观察结果，而不是多处同时修改同一份状态。

重要不变量：Python 不决定 C++ 下一条何时派发，C++ 不修改用户会话；两层通过命令和终态接口协作。

### Q12. 异步回调如何避免串结果？

口述：

结果回调带 `command_id`，没有 ID 或不是当前等待目标就不能结束本次任务。对于没有业务 ID 的 timer、连接和 goal response，使用 generation 标识生命周期代次。取消后旧回调仍可能到达，但只能被忽略或主动取消其远端副作用。

源码：

- [action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)
- [asr_endpoint_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py)
- [nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)

## 2. C++ 音频前端

简历原句：

> 使用 C++ 开发音频前端模块，基于 PortAudio 实现实时音频采集，通过缓冲队列和线程隔离降低采集与算法处理之间的影响；结合 VAD、语音端点检测和回声抑制，提高连续语音交互过程中的稳定性。

### Q13. 为什么音频前端使用 C++？

口述：

采集回调需要稳定的帧级时延和明确资源生命周期，C++ 便于直接接入 PortAudio、控制内存复制和线程同步。回调只做入队，DSP 放到独立线程，避免 Python 模型推理或 ROS 回调阻塞采集。上层 Agent 仍使用 Python，保留模型和业务编排效率。

源码：[audio_frontend_node.cpp](../../../src/embodied_agent_cpp/src/audio_frontend_node.cpp)

### Q14. 缓冲队列如何降低相互影响？

口述：

采集者按音频设备节奏生产帧，处理线程按 DSP 能力消费；队列吸收短时抖动，让两者不直接互相调用。队列达到 50 帧时丢最旧帧并增加丢帧统计，避免延迟不断扩大。条件变量让处理线程没有数据时休眠。

调用：

```text
PortAudio callback
input_queue_ 和 condition_variable
processing_loop()
audio_enhancer_->process()
VAD 与 endpoint
ROS publish
```

### Q15. 处理线程具体执行哪些步骤？

口述：

处理线程取出麦克风帧后，先调用音频增强器做 NLMS 回声抵消，再计算 RMS 等帧指标，VAD 判断是否有语音，端点状态机累计开始和尾部静音，最后发布清洗 PCM 及开始、结束事件。它不会运行完整 ASR 或 LLM，后者属于独立 provider 和 worker。

源码：

- [audio_frontend_node.cpp](../../../src/embodied_agent_cpp/src/audio_frontend_node.cpp)
- [audio_processing.cpp](../../../src/embodied_agent_cpp/src/audio_processing.cpp)

### Q16. 回声抑制为什么需要播放参考？

口述：

机器人播放 TTS 时，麦克风会再次收到扬声器声音。播放线程把同一 PCM 写入 reference 缓冲，NLMS 根据参考估计回声并从麦克风中减去；没有已知参考就无法区分机器人自己的声音和用户声音。参考还要重采样并补偿播放到采集之间的延迟。

边界：这是回声抵消，不是完整环境降噪。通用 noise suppression 和 AGC 当前未激活。

### Q17. VAD 和端点怎样提高连续交互稳定性？

口述：

VAD 判断当前帧像不像语音，端点状态机再要求连续语音达到开始门槛，并在尾部静音达到阈值后结束一句话。最短句长过滤瞬时噪声，最长句长防止麦克风一直不提交。端点后还有可取消的延迟 commit，用户短暂停顿继续说话时不会被硬切成两句。

源码：

- [audio_processing.cpp](../../../src/embodied_agent_cpp/src/audio_processing.cpp)
- [asr_endpoint_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py)

### Q18. 音频线程异常或停机怎样处理？

口述：

节点持有 stop 标志并唤醒条件变量，停止 PortAudio stream 后等待处理和播放线程退出，再释放设备与缓冲。后台 timer 和 provider 也在 Lifecycle 停用时取消，避免节点销毁后继续回调。RAII 负责资源最后释放，显式关停负责业务顺序。

## 3. 结构化动作、安全校验和 Action

简历原句：

> 设计自然语言指令到机器人动作的转换流程，将 LLM 输出限制为候选动作生成，通过结构化 ROS 2 消息和 C++ 校验模块约束动作类型、参数、速度和导航目标；基于 ROS 2 Action 实现任务执行、状态反馈、取消处理、急停抢占和异常结果过滤。

### Q19. 怎样限制 LLM 只生成候选动作？

口述：

应用层要求模型或 NLU 输出有限动作名和参数，再转换为 `ActionCommand` 值对象及 `RobotCommand` 消息。模型不能获取 `/cmd_vel` publisher，也不能直接访问 Nav2 client。即使结构化输出格式正确，C++ 仍按独立规则重新校验。

源码：

- [command_nlu.py](../../../src/embodied_agent_core/embodied_agent_core/command_nlu.py)
- [types.py](../../../src/embodied_agent_core/embodied_agent_core/types.py)
- [ros_action_transport.py](../../../src/embodied_agent_core/embodied_agent_core/ros_action_transport.py)

### Q20. C++ 校验具体检查什么？

口述：

它检查动作是否在允许集合、优先级是否只用于停止类动作、数值是否有限、字段组合是否互斥，并限制速度、时长、waypoint 数量、巡检轮数、颜色、模式和导航地点。无关字段不为空也会拒绝，防止一条消息同时携带多种动作语义。合法候选才发布到可信命令 Topic。

源码：[action_validator.cpp](../../../src/embodied_agent_cpp/src/action_validator.cpp)

例子：MOVE 可以有线速度、角速度和时长，但不应同时带导航 target；FOLLOW_WAYPOINTS 需要非空地点列表，数量和轮数受限。

### Q21. 为什么 C++ 校验后执行端还要再检查？

口述：

边界校验负责清洗 Agent 候选，执行端策略负责定义所有入口都必须满足的规范命令。测试工具、其他节点或未来网关可能绕过候选 Topic，Action server 不能默认上游永远正确。两层共享部分规则，但职责不同：上游规范化，执行端防旁路。

源码：

- [action_validator.cpp](../../../src/embodied_agent_cpp/src/action_validator.cpp)
- [robot_command_policy.cpp](../../../src/embodied_simulation/src/robot_command_policy.cpp)

### Q22. 为什么任务执行使用 Action？

口述：

运动、导航和巡检持续时间不固定，需要知道 goal 是否接受、执行到哪一步、最后成功还是失败，并支持中途取消。Action 原生提供 goal、feedback、result 和 cancel，适合这种长事务。Topic 适合候选流，但单独使用 Topic 需要自定义完整任务协议；Service 又会让调用方长时间等待。

接口：[ExecuteRobotCommand.action](../../../src/embodied_agent_interfaces/action/ExecuteRobotCommand.action)

### Q23. 急停抢占完整经过哪些层？

口述：

会话层优先识别“停下”，取消 Python 动作批次并清语句队列；随后发布带 priority 的 STOP。C++ 调度器清除 pending、请求取消 active，并把 STOP 放在队首；执行端取消行为树或 Nav2 goal，并立即归零。验收还检查最终速度为零。

源码：

- [agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)
- [action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)
- [simulation_controller.cpp](../../../src/embodied_simulation/src/simulation_controller.cpp)

### Q24. Action cancel 一定会立刻成功吗？

口述：

不会。cancel request 只是请求，server 可能稍后返回，网络或节点异常也可能没有结果。桥接节点记录取消开始时间，watchdog 超时后为旧任务生成超时终态并继续派发优先命令；之后到达的旧 result 因 ID 不匹配被忽略。Nav2 goal 尚未接受时，晚到的 handle 还会被主动取消。

源码：

- [typed_action_bridge_node.cpp](../../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)
- [nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)

### Q25. “异常结果过滤”指什么？

口述：

第一类是未知或迟到的 `command_id`，不能修改当前任务；第二类是 Action 协议成功但业务不完整，例如多点巡航有 missed waypoint；第三类是模型流已经产生部分输出后的重试结果，不能与旧输出拼接。过滤依据必须来自当前状态和业务字段，而不是只看某个 success 布尔值。

源码：

- [action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)
- [nav2_result_policy.cpp](../../../src/embodied_simulation/src/nav2_result_policy.cpp)
- [llama_cpp.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py)

## 4. 行为树、插件、建图和导航

简历原句：

> 使用 BehaviorTree.CPP 和 pluginlib 组织机器人任务流程，将任务逻辑与执行模块分离；接入 Gazebo 和 Nav2，实现 frontier 探索、地图保存、AMCL 定位、多点巡检和动态障碍避障，并根据导航状态和机器人实际速度判断任务完成情况。

### Q26. BehaviorTree.CPP 在项目中不是只为了写在简历上吗？

口述：

执行节点真的创建了 `ReactiveSequence`，每个控制周期更新 blackboard 并 tick。四个节点分别做规范命令检查、安全检查、执行状态传播和结果确认；取消时 `haltTree()`。它目前规模不大，但已经把失败阶段和细节变成可观测状态，也为以后添加恢复节点保留结构。

源码：

- [command_behavior_tree.cpp](../../../src/embodied_simulation/src/command_behavior_tree.cpp)
- [command_tree.xml](../../../src/embodied_simulation/config/command_tree.xml)

### Q27. pluginlib 实际替换了哪些后端？

口述：

控制节点按参数加载 Gazebo、Nav2 或 mock 的 `RobotExecutor` 实现。Gazebo 后端计算并发布速度，Nav2 后端只发送导航 Action，mock 用于快速验证流程。上层 Action server 和行为树不需要知道具体后端。

源码：

- [robot_executor_plugins.xml](../../../src/embodied_simulation/robot_executor_plugins.xml)
- [robot_executor.hpp](../../../src/embodied_simulation/include/embodied_simulation/robot_executor.hpp)
- [nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)

### Q28. 自动 frontier 建图完整流程是什么？

口述：

unknown-world 自动任务先等待 scan/TF 并动态捕获首次运动前的起点，再通过 typed action 做初始
扫描，随后由 Explore Lite 选择 frontier 并交给 Nav2。优先使用 `strict_frontier` 强类型原因收口；硬预算
或反复可达停滞时，必须再用多轮低收益、final probe、地图静默、Action 总账排空和 typed STOP 证明
`bounded_saturation`。两条路径都先在 mapping stage 回到首次运动前动态捕获的起点，确认 Action、位姿
和新鲜零速度，再保存图并切换定位导航；不能把 timeout、平台期或“地图看起来差不多”直接写成完成。

源码：

- [mission_executor.py](../../../src/embodied_slam_tools/embodied_slam_tools/mission_executor.py)
- [frontier_monitor.py](../../../src/embodied_slam_tools/embodied_slam_tools/frontier_monitor.py)
- [exploration_saturation.py](../../../src/embodied_slam_tools/embodied_slam_tools/exploration_saturation.py)
- [mapping_return.py](../../../src/embodied_slam_tools/embodied_slam_tools/mapping_return.py)

### Q29. 地图保存怎样避免假成功？

口述：

任务层调用 map saver 后检查进程返回值，还检查 YAML 和图像文件真实存在，并由验收会话确认是本次运行新生成的文件。只有日志说“saved”不够，因为旧地图可能仍在目录中。存图成功后才允许状态机进入 `MAP_SAVED`。

源码：

- [stage_process_manager.py](../../../src/embodied_slam_tools/embodied_slam_tools/stage_process_manager.py)
- [showcase_session.py](../../../src/embodied_slam_tools/embodied_slam_tools/showcase_session.py)

### Q30. AMCL 和 Nav2 启动后怎样判断可用？

口述：

系统先等待组件 readiness，再等待 `navigate_to_pose` 和 `follow_waypoints` Action server；随后通过 Lifecycle `GetState` Service 检查 `bt_navigator` 和 `waypoint_follower` 为 active。只有这些条件都满足才发送任务。节点进程存在或 Topic 偶尔有消息都不足以证明导航已就绪。

源码：[showcase_session_node.py](../../../src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py)

### Q31. 多点巡检怎样实现？

口述：

语义 waypoint 在 YAML 中映射为 `map` 坐标，Nav2 插件构造 `FollowWaypoints::Goal`。项目处理了业务总轮数与 Nav2 额外轮数的差异，并在结果中检查 error code 和 missed waypoints。任意漏点都会按巡检业务失败处理。

源码：

- [nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)
- [nav2_result_policy.cpp](../../../src/embodied_simulation/src/nav2_result_policy.cpp)

### Q32. 动态障碍避障是谁完成的？

口述：

本项目的跟踪器负责把检测点关联成稳定轨迹并预测未来位置，预测 costmap 插件把这些位置写入 Nav2 代价地图；实际路径调整和速度控制仍由 Nav2 planner/controller 完成。演示使用 Gazebo 中的确定性输入验证整条链路，不声称已完成真实动态目标感知。

源码：

- [dynamic_obstacle_tracker.cpp](../../../src/embodied_navigation/src/dynamic_obstacle_tracker.cpp)
- [predicted_obstacle_layer.cpp](../../../src/embodied_navigation/src/predicted_obstacle_layer.cpp)

### Q33. “根据导航状态和实际速度判断完成”准确吗？

口述：

运行时任务终态主要来自 Nav2 Action result，并进一步检查错误码和漏点；速度不是单独宣布导航成功的依据。Odometry 或 `/cmd_vel` 的零速度用于验收“成功或取消后机器人确实停下”，也用于排除仍在运动的假完成。因此更准确的说法是：以 Action 业务终态判断任务结果，以速度和路径证据验证执行效果及安全后置条件。

源码：

- 终态：[nav2_result_policy.cpp](../../../src/embodied_simulation/src/nav2_result_policy.cpp)
- 验收：[slam_nav_evidence.py](../../../tools/acceptance/slam_nav_evidence.py)

## 5. GTSAM、LiDAR 回环和自动化测试

简历原句：

> 基于 GTSAM 验证位姿图优化流程，完成 LiDAR 回环约束和局部子图匹配实验；针对机器人任务流程编写 pytest、gtest 自动化测试，覆盖 ROS 2 节点运行、任务取消、超时处理和导航结果验证。

### Q34. GTSAM 位姿图中放了什么？

口述：

节点是二维位姿 `Pose2`，边是相邻运动约束或非局部回环约束，最早位姿用 Prior 固定坐标基准。约束协方差先处理成正定矩阵，再构造 Gaussian noise；回环可以增加 Huber、Cauchy 鲁棒核或 switch variable。最后使用 Levenberg-Marquardt 优化并输出轨迹和被抑制约束。

源码：[gtsam_pose_graph.cpp](../../../src/embodied_slam/src/gtsam_pose_graph.cpp)

### Q35. LiDAR 回环约束怎样生成？

口述：

当前 scan 先生成极坐标描述子和 ring key，从历史中召回 Top-K 候选；候选不能直接入图，还要对局部子图做 scan matching，检查重叠率、RMSE、可观测性和歧义。连续多帧的一致性通过后，约束门控才记录或提交。默认策略更偏向 shadow 观察，避免错误回环污染地图。

源码：

- [lidar_loop_candidates.cpp](../../../src/embodied_slam/src/lidar_loop_candidates.cpp)
- [lidar_loop_verifier.cpp](../../../src/embodied_slam/src/lidar_loop_verifier.cpp)
- [lidar_loop_constraint_gate.cpp](../../../src/embodied_slam/src/lidar_loop_constraint_gate.cpp)

### Q36. 为什么使用局部子图，不只匹配单帧 scan？

口述：

单帧激光点少、视角变化大，在走廊或转角容易退化。局部子图融合相邻多帧，提供更多稳定几何结构，提高重叠和可观测性判断。代价是构建和匹配更耗时，还要严格处理每帧位姿与时间。

源码：[lidar_submap_builder.cpp](../../../src/embodied_slam/src/lidar_submap_builder.cpp)

### Q37. 鲁棒核能替代前端回环验证吗？

口述：

不能。鲁棒核根据残差连续降权，适合抑制少量离群约束，但错误回环如果初始残差不大仍可能误导优化。项目先做描述子召回、几何和时序门控，后端鲁棒核和 switch variable 只是最后保护。前端和后端承担不同层次的错误控制。

源码：[gtsam_pose_graph.cpp](../../../src/embodied_slam/src/gtsam_pose_graph.cpp)

### Q38. gtest 主要覆盖哪些 C++ 逻辑？

口述：

重点覆盖动作字段校验、调度器取消和迟到结果、行为树状态、Nav2 漏点结果、音频算法、硬件帧、动态目标关联与滤波、GTSAM 优化。测试尽量针对无 ROS 的纯类，因此速度快且容易构造边界条件。

例子：

- [test_action_scheduler.cpp](../../../src/embodied_agent_cpp/test/test_action_scheduler.cpp)
- [test_nav2_result_policy.cpp](../../../src/embodied_simulation/test/test_nav2_result_policy.cpp)
- [test_gtsam_pose_graph.cpp](../../../src/embodied_slam/test/test_gtsam_pose_graph.cpp)

### Q39. pytest 主要覆盖哪些 Python 逻辑？

口述：

pytest 验证连续会话、队列满和过期、ASR endpoint 取消、provider 回调、自动任务状态机、frontier 结束、进程清理和验收证据。单条失败、取消和超时都可以通过 fake clock、fake provider 和依赖注入稳定复现。

例子：

- [test_frontier_monitor.py](../../../src/embodied_slam_tools/test/test_frontier_monitor.py)
- [test_action_sequence.py](../../../src/embodied_agent_core/test/test_action_sequence.py)
- [test_process_supervisor.py](../../../tests/repository/test_process_supervisor.py)

### Q40. 如何测试 ROS 2 节点和完整导航？

口述：

纯单元测试之后，参数化 probe runner 启动真实节点检查 Lifecycle、Topic、Service 和 Action 接线；Gazebo E2E
再运行传感器、SLAM、AMCL 和 Nav2，记录地图、TF、路径、Action 结果和最终速度。正式
unknown-world E2E 只给机器人在线 scan/odom/TF/map，本次地图以外的真值只给 evaluator；known-world
真人语音演示验证交互链，但不能替代这份自主证据。验收会话为每次运行分配独立 ROS domain 和证据目录，
并清理全部进程组，避免旧环境造成假通过。面试演示时只需要记住统一入口：
`verify voice` 讲 Agent、`verify control` 讲 C++ Action/安全、`verify gazebo` 讲物理执行、
`verify slam-nav` 讲未知地图闭环；`verify all` 生成一份汇总报告。

入口：

- [TESTING.md](../../TESTING.md)
- [project_verification.py](../../../tools/acceptance/scenarios/project_verification.py)
- [unknown_world_slam_e2e.py](../../../tools/acceptance/scenarios/unknown_world_slam_e2e.py)
- [process_supervisor.py](../../../tools/acceptance/process_supervisor.py)

### Q41. 这条项目经历最需要主动说明的边界是什么？

口述：

主要运行环境是 Gazebo，不等同于真实底盘交付；音频实现的是 NLMS 回声抵消、VAD 和端点，不是完整
通用降噪；默认声纹证据是 mock，Sherpa mode 尚未完成真实多人准确率验收；动态障碍使用确定性仿真
输入；新增 LiDAR 回环默认谨慎观察；在线离线模型接入不等于完成 LoRA 训练或生产准确率。主动说明这些
边界不会削弱项目，反而能说明我知道证据能证明到哪里。
