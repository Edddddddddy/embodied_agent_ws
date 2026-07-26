# 简历技术追问题库索引

本目录按简历中的 8 条技能和 5 条项目经历组织。目标不是背术语，而是做到：

1. 先用几句话说明知识点解决什么问题。
2. 能指出项目中哪段代码真正使用了它。
3. 能从上游输入讲到下游结果。
4. 能解释为什么这样设计，以及替代方案的代价。
5. 能主动说明仿真、实验和真实工程之间的边界。

事实来源是当前检出 revision 的代码、`docs/ARCHITECTURE.md` 和 `docs/TESTING.md`。同一 Git 仓库可以有
多个 worktree；文件所在目录不是版本身份，回答前应先确认 `git branch --show-current` 和
`git rev-parse HEAD`。本目录不把个人了解的技术包装成项目成果。

## 1. 文档对应关系

| 简历内容 | 面试笔记 | 重点 |
| --- | --- | --- |
| ROS 2 与机器人软件框架 | [01-ros2-framework.md](01-ros2-framework.md) | 通信方式、QoS、自定义接口、Launch、Lifecycle、Colcon |
| 机器人控制与自主导航 | [02-control-navigation.md](02-control-navigation.md) | 速度、LaserScan、Odometry、TF、SLAM、AMCL、Nav2 |
| 软件架构与可靠性 | [03-architecture-reliability.md](03-architecture-reliability.md) | 状态机、行为树、插件、调度、健康检查、超时和一致性 |
| 现代 C++ 与 Linux | [04-cpp-linux.md](04-cpp-linux.md) | RAII、所有权、线程同步、异步回调、文件描述符和进程管理 |
| 语音与具身智能 | [05-voice-agent.md](05-voice-agent.md) | PortAudio、AEC、VAD、ASR、LLM、TTS、连续会话 |
| 网络通信与分布式系统 | [06-network-distributed.md](06-network-distributed.md) | HTTP 流、WebSocket、超时重试，以及非项目实做项 |
| 硬件通信与工程工具 | [07-hardware-tooling.md](07-hardware-tooling.md) | UART、SPI、帧协议、看门狗、Gazebo、测试和 CI |
| 算法与数据结构 | [08-algorithms.md](08-algorithms.md) | 队列、匈牙利匹配、滤波、图优化，以及 BFS/A* 的使用边界 |
| 5 条项目经历逐句追问 | [09-project-line-by-line.md](09-project-line-by-line.md) | 每句话如何展开、如何举代码、哪些说法要收口 |
| ROS 2 功能设计场景题 | [10-ros2-system-design.md](10-ros2-system-design.md) | 从需求到接口、状态、安全、异常和验证的完整设计 |
| 语音运行时部署与 RAG | [11-voice-deployment-rag.md](11-voice-deployment-rag.md) | 近年论文、部署 profile、检索安全、指标与取舍 |
| Git worktree 与发布治理 | [12-git-worktree-release-governance.md](12-git-worktree-release-governance.md) | 一个仓库多工作目录、分支职责、CI、Tag、Release 与安全清理 |

## 2. 每题怎么使用

每个问题采用相同结构：

- 口述：面试现场先说这一段，通常控制在 4 至 6 句。
- 源码：给出文件和函数，回答“你具体怎么做的”。
- 运行：按真实先后顺序说明上游、当前模块和下游。
- 取舍：解释采用原因、异常处理和当前边界。

建议先遮住“源码”和“运行”，只看问题口述一遍；再打开代码，确认自己能解释每个状态变量。不要背文件名，文件名只用于证明回答来自真实实现。

## 3. 贯穿全部问题的任务

用“开始自动巡检建图”贯穿项目最容易形成连续讲述：

1. PortAudio 回调取得 20 ms PCM，只复制到有界队列。
2. C++ 处理线程完成回声抵消、VAD 和端点判断。
3. 在线 ASR 通过 WebSocket 接收音频，离线 ASR 在单独 worker 中解码。
4. Python 会话层处理唤醒、重复文本、连续命令和急停。
5. NLU 或 LLM 只产生结构化候选动作，不直接控制底盘。
6. C++ 校验动作字段和参数，调度器维护一个活动任务和等待队列。
7. ROS 2 Action 把任务交给行为树及执行插件，并返回反馈、取消和终态。
8. unknown-world 自动任务用 frontier 探索；优先严格终结，硬预算或反复可达停滞时才评估有界饱和。
9. 两条收口路径都先排空 Action、最终探测、typed STOP，并回到动态捕获的起点，再保存本轮地图。
10. 系统切换到 map_server、AMCL 和 Nav2，执行本次地图运行时采样目标与动态障碍挑战。
11. 独立验收检查地图质量、Action 结果、TF、路径、定位误差、返航和最终零速度。

记住三个层级：

```text
语音文本只是输入
结构化动作只是候选
执行端返回并通过验收，才是任务完成
```

对应的代码证据统一从一个入口展开：

```bash
bash scripts/acceptance_test.sh verify voice
bash scripts/acceptance_test.sh verify control
bash scripts/acceptance_test.sh verify gazebo
HEADLESS=true USE_RVIZ=true bash scripts/acceptance_test.sh verify slam-nav
```

这四个 profile 分别回答“Agent 是否正确理解和排队”“C++ 安全与调度是否正确”“仿真底盘是否真实运动
并停稳”“未知地图是否完成建图、定位、导航和动态避障”。真人麦克风是单独的现场体验证据，不混入这些
可重复门禁。

## 4. 面试时的回答顺序

遇到任何设计题，按下面五句话组织：

1. “这个模块解决的是……”
2. “输入来自……，代码先……，再交给……”
3. “这里选择 Topic、Service、Action 或独立线程，是因为……”
4. “如果出现超时、取消或迟到结果，系统会……”
5. “当前通过……验证；项目边界是……”

面试官继续追问时再进入类、函数、锁和数据结构。不要一开始把所有包名和英文术语全部报出来。

## 5. 当前事实边界

| 内容 | 当前可以说 | 不要说成 |
| --- | --- | --- |
| 音频 | 实现 PortAudio、有界缓冲、NLMS 回声抵消、VAD 和端点检测 | 已实现完整通用降噪和自动增益 |
| 声纹与记忆 | 已有 typed 身份/录入接口、Sherpa embedding 接入 seam、低置信拒写和用户画像；默认演示仍用 mock 身份 | 默认链路已完成真实多用户声纹准确率验收 |
| 导航 | 在 Gazebo 中接入 SLAM Toolbox、AMCL、Nav2、frontier 和多点巡航 | 已完成真实机器人量产导航 |
| 动态障碍 | 用确定性仿真输入验证跟踪、预测代价层和重规划链路 | 已完成真实视觉或 LiDAR 动态目标感知 |
| SLAM | 实现回环候选、几何验证和 GTSAM 后端实验 | 新回环约束已在任意现场默认自动入图 |
| 硬件 | 实现 UART、SPI、CRC 帧和 mock 看门狗验证 | 已完成 CAN、EtherCAT 或真实 MCU 闭环 |
| 网络与中间件 | 项目使用 HTTP 流和 WebSocket；个人了解 gRPC、Redis、Kafka、MySQL | 本项目使用了这些中间件 |
| 工程部署 | 项目有 Colcon、gtest、pytest、Gazebo E2E、GitHub Actions、Dockerfile 与 Compose 门禁 | 已完成带模型/麦克风/sidecar 的生产容器部署 |
| 模型 | 在线、离线 provider 已接入 | LoRA 已训练或当前数据代表生产准确率 |
| RAG | 已实现小型本地稀疏检索、路由、引用和 Prompt 预算 | 已部署向量数据库、Self-RAG 或达到未实测召回率 |

## 6. 三轮复习法

第一轮只读每题“口述”，目标是能连续说明项目。

第二轮打开源码，对每题回答三个问题：谁调用这个函数，状态由谁保存，结果交给谁。

第三轮只看 [10-ros2-system-design.md](10-ros2-system-design.md)，在没有项目原题提示的情况下完成接口选型、异常设计和验证方案。

准备讲版本和交付时再读
[12-git-worktree-release-governance.md](12-git-worktree-release-governance.md)：先说“一个仓库、多份工作目录”，
再说明 `feature/* → dev → release/* → main` 的证据如何由 PR、CI、Tag 和 GitHub Release 串起来。
