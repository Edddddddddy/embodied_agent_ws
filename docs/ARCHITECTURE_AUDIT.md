# 阶段架构审计

更新时间：2026-07-13

## 1. 当前结论

项目已形成清晰的领域、输入 Adapter、部署、控制和仿真边界，适合继续作为 ROS 2/C++
求职展示工程维护。后续优化应由真实变化触发，不再单纯按文件行数拆分。

当前依赖方向：

```text
embodied_agent_interfaces
  ├── embodied_agent_middleware
  ├── embodied_agent_core
  │     └── embodied_voice_frontend
  └── embodied_agent_cpp

embodied_agent_bringup
  ├── embodied_agent_core
  ├── embodied_voice_frontend
  └── embodied_agent_cpp

embodied_online_agent / embodied_offline_agent
  ├── embodied_agent_core
  └── embodied_agent_bringup

embodied_simulation
  ├── embodied_agent_middleware
  ├── embodied_agent_bringup
  └── online/offline Agent（仅组合演示 launch）
```

领域 core 不再依赖 launch、具体 Agent 或仿真包；bringup 是系统装配根，simulation 是演示集成根。

## 2. 已收敛的关键边界

- `embodied_agent_core`：连续会话、NLU、执行、Lifecycle、用户上下文、记忆和 ROS I/O。
- `embodied_voice_frontend`：Silero/WebRTC VAD、KWS、声纹输入 Adapter。
- `embodied_agent_bringup`：Agent 参数、语音前端节点和安全部署拓扑的 launch contract。
- `embodied_agent_middleware`：Python/C++ 共用的命名 QoS 语义与系统 readiness。
- `ActionGuard + ActionScheduler`：强类型动作安全、FIFO、取消、超时和结果关联。
- `ActiveActionRuntime`：仿真长动作的唯一生命周期所有者。
- `SimulationRosIo`：仿真 managed publisher、QoS、ACK/BT/diagnostics 状态映射。
- `RobotExecutor`：Gazebo、Mock、Nav2 三个独立 pluginlib 后端。
- `voice_control_profile.sh`：真实语音 profile 的唯一默认值解析器。
- `tests/repository`：按 architecture、delivery、voice runtime 分区的仓库契约。

## 3. 当前保留的大文件

### online/offline Agent 节点

两个节点仍约 700～900 行，但共享状态机、Lifecycle、ROS I/O、执行、端点和用户上下文已经下沉。
剩余内容主要是 provider 创建、在线/离线 ASR 差异、TTS/LLM 调用和 ROS callback 适配。

只有出现以下变化时再继续拆分：

- 新增第三种 Agent provider，需要复用现有节点 Adapter；
- provider 初始化/关闭再次出现在线离线重复；
- 单元测试必须实例化完整 ROS 节点才能覆盖 provider 逻辑。

### `acceptance_test.sh`

该脚本约 90 个模式，当前定位是稳定 CLI router，具体实现已经分散在独立脚本和 Python 探针中。
暂不为了行数改成动态注册；当新增模式需要同时修改 help、case 和 release gate 三处时，再引入声明式模式表。

### `command_nlu.py`

当前同时包含意图原型、小型字符 n-gram 模型和槽位解析。下一次扩充大量语料时，应把原型数据迁移到
版本化 JSON/YAML；在现有命令规模下保留单模块更方便审查安全语义。

## 4. 阶段验收证据

```bash
bash scripts/acceptance_test.sh mock
```

2026-07-13 结果：

- 9 个 ROS 2 包完成构建，包含独立 bringup 包；
- 仓库契约 56 项通过；
- Python 快速测试 226 项通过；
- ROS/C++ 汇总 419 项，0 error、0 failure、0 skipped；
- 连续语音、端点提交、短命令补全、多命令队列、急停、Nav2 mock、pluginlib、硬件 mock、
  Lifecycle 和雷达安全冒烟全部通过；
- 最终控制链路能回到零速度。

真实麦克风、云 API、Gazebo GUI 和完整 Nav2 TurtleBot3 仍属于演示前人工/重型验收，不能由 mock
结果替代。

## 5. 下一阶段触发条件

优先处理可量化问题：真实 ASR 样本准确率、真实麦克风连续 3～5 分钟稳定性、离线模型延迟、
Gazebo/Nav2 重型演示回归。若这些证据稳定，架构层不再主动扩大改动面。
