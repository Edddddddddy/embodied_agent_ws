# 阶段架构审计

决策审计基线：2026-07-13。当前可计算规模以自动生成的
[架构事实报告](evidence/architecture_facts.md) 为准；本文不再固化会随提交变化的行数和测试数量。

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

两个节点的当前行数由架构事实报告从源码计算；共享状态机、Lifecycle、ROS I/O、执行、端点和用户
上下文已经下沉。剩余内容主要是 provider 创建、在线/离线 ASR 差异、TTS/LLM 调用和 ROS callback
适配。

只有出现以下变化时再继续拆分：

- 新增第三种 Agent provider，需要复用现有节点 Adapter；
- provider 初始化/关闭再次出现在线离线重复；
- 单元测试必须实例化完整 ROS 节点才能覆盖 provider 逻辑。

### `acceptance_test.sh`

该脚本的公开/高级/router mode 数量由架构事实报告计算，当前定位是稳定 CLI router，具体实现已经
分散在独立脚本和 Python 探针中。默认帮助固定为 12 个公共入口；新增模式必须同时满足帮助可发现、
case 可路由和 release-gate 契约，repository test 会检查三者是否漂移。

### `command_nlu.py`

当前同时包含意图原型、小型字符 n-gram 模型和槽位解析。下一次扩充大量语料时，应把原型数据迁移到
版本化 JSON/YAML；在现有命令规模下保留单模块更方便审查安全语义。

## 4. 阶段验收证据

```bash
bash scripts/acceptance_test.sh mock
```

长期有效的验收契约：

- GitHub Actions 构建矩阵必须与全部 ROS 2 `package.xml` 一致；
- `robotics-gate` 必须覆盖仓库/Agent/C++、连续多命令、Nav2、SLAM、OpenLORIS fixture 和动态障碍；
- 公开帮助保持 12 个入口，高级帮助中的每个入口都必须可路由；
- 精确包数、模式数、节点行数和 typed interface 数见自动生成报告；单次测试通过数量只保留在 CI/终端
  证据中，不再复制成会过期的架构事实；
- 最终控制链路必须回到零速度。

真实麦克风、云 API、Gazebo GUI 和完整 Nav2 TurtleBot3 仍属于演示前人工/重型验收，不能由 mock
结果替代。

## 5. 下一阶段触发条件

优先处理可量化问题：真实 ASR 样本准确率、真实麦克风连续 3～5 分钟稳定性、离线模型延迟、
Gazebo/Nav2 重型演示回归。若这些证据稳定，架构层不再主动扩大改动面。
