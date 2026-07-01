# 版本记录、同类项目对比与路线图

## 1. 版本迭代记录

| 阶段 | 代表提交 | 主要结果 |
|---|---|---|
| 在线原型 | `a7aa586` | ROS 2 在线 Agent、mock provider、结构化动作 |
| 冒烟修复 | `ce57588` | 修复 topic echo 验收误解，形成自动冒烟入口 |
| 实时路径 C++ 化 | `e80fd3a` | 音频、AEC/VAD、播放和动作安全下沉 C++ |
| 云模型接入 | `f19a742` | DashScope ASR/LLM/TTS 与低 token 接口测试 |
| 离线 Agent | `e93c7e9` | ZipFormer、llama.cpp、Sherpa-TTS、双缓冲 |
| 硬件控制 | `0a37345` | ActionGuard、UART/SPI、CRC、ACK、watchdog |
| 总体验收 | `c064f7e` | 分层测试、真实性能和完成度报告 |
| 仿真闭环 | `06bc00d` | TurtleBot3、LaserScan、Twist、PID 与 Gazebo |
| 聚焦语音动作 | `50263aa` | 目标收敛到语音 -> move/turn/stop -> 仿真 |
| 麦克风验收 | `4f83218` | 六阶段交互式验收器 |
| 停滞修复 | `301b74f` | launch 参数贯通、busy 时抑制重叠 ASR |
| 识别恢复 | `0469723` | 热词、别名、失败反馈和持续重试 |
| 文档与结构收敛 | 当前 | 七份重叠笔记合并为三份，完成模块依赖与入口审计 |

## 2. 当前结论

项目已达到“可演示、可测试、可继续接真实机器人”的工程原型：在线/离线语音、动作
安全、Gazebo、UART/SPI interface 均有实现和分层测试。它还不是生产系统，不能把少量
样本延迟、未执行的 LoRA 或未接实体 MCU 写成已完成成果。

最有辨识度的能力是：同一个 ROS 2 安全动作 seam 同时连接在线语音、全离线 CPU 模型、
Gazebo 物理仿真和硬件传输。这条纵向闭环应成为项目对外叙事中心。

## 3. 近期同类高星项目对比

GitHub 星数采样于 2026-07-01，会随时间变化；功能依据各项目官方 README。

| 项目 | 星数约 | 强项 | 相对不足/与本项目关系 |
|---|---:|---|---|
| [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) | 27.8k | 流式语音、离线唤醒、MCP、70+ 硬件、OTA、多语言与强演示内容 | 不以 ROS 2/Gazebo 和机器人运动安全为核心；其产品化、硬件生态和传播远强于本项目 |
| [LeRobot](https://github.com/huggingface/lerobot) | 25.4k | 统一 Robot interface、标准数据集、预训练策略、Hub、PyPI、丰富硬件和社区 | 重点是数据/训练/VLA，不是语音 Agent；本项目缺少它的标准数据与可安装 SDK 体验 |
| [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) | 13.3k | ASR/TTS/KWS/VAD/增强、多平台、多语言、NPU 和完整示例 | 是语音基础设施而非机器人闭环；本项目应复用其 KWS/增强，不应重复造轮子 |
| [RAI](https://github.com/RobotecAI/rai) | 532 | ROS 2 原生 agentic framework、ASR/TTS、仿真、benchmark、多模态、正式文档与 ROS 社区连接 | 与本项目最接近；本项目链路更小更易读，但缺少工具扩展、多机器人配置和 benchmark 产品化 |

四者共同做对的事情：README 首屏立即说明价值；一条命令产生可见结果；有稳定 interface
连接多种硬件/模型；提供视频、教程、release、CI、贡献指南和社区入口。高星不是靠继续
添加“沿墙/灯光”等零散功能，而是降低第一次成功的成本，并让别人容易扩展和展示。

## 4. 本项目主要不足

### P0：别人很难在十分钟内相信它

- 没有 README 首屏 GIF/视频，语音、终端六阶段和 Gazebo 位移无法一眼看见。
- 安装仍依赖 WSL、ROS、模型下载和本地编译，没有 Docker/devcontainer 或缓存 release。
- 缺少 GitHub Actions、正式根目录 LICENSE、CONTRIBUTING、issue/PR 模板和版本 release。
- 没有公开的 benchmark 原始日志和可重复硬件规格。

### P1：扩展成本仍偏高

- 在线/离线节点重复一轮对话编排，新增 provider 需要理解两个大节点。
- 动作协议是 JSON 字符串，缺少正式 ROS msg/action 定义、schema 版本与生成文档。
- 配置散布在 YAML、launch 参数、环境变量和脚本默认值，缺少启动前校验。
- 集成测试在 `scripts/` 中，CI 可发现性不如标准 `tests/integration/`。

### P2：缺少社区可复用资产

- 机器人指令数据只有种子规模，没有公开评测集、噪声集与排行榜。
- 只展示 TurtleBot3，尚未证明 executor interface 可以被第二种机器人复用。
- 没有插件教程，例如“30 分钟增加一个动作/provider/机器人”。
- 中英文文档、架构图、演示场景和故障排查素材不足。

## 5. 高星改进路线

### 第一个里程碑：让陌生人 10 分钟跑通（最高优先级）

1. 录制 30–60 秒 GIF：说“向前走一秒” -> ASR 文本 -> 动作 JSON -> Gazebo 移动。
2. 提供 `docker compose up demo` 或 devcontainer，缓存 ROS 依赖；mock demo 不下载模型。
3. GitHub Actions 自动跑 build、49 项测试和 headless mock/simulation smoke。
4. 补 Apache-2.0 LICENSE、CONTRIBUTING、release notes、issue 模板和架构图。
5. 发布 `v0.1.0`，README 只保留一个主 CTA：Run the voice-to-Gazebo demo。

验收指标：全新 Ubuntu 机器按 README 操作，10 分钟内得到 PASS；CI 始终可见；演示 GIF
首屏加载后不读正文也能理解项目。

### 第二个里程碑：把亮点变成可比较数据

1. 建立 100–500 条公开中文机器人命令评测集，包含同音词、口音、噪声和拒识样本。
2. 报告在线/离线冷热启动 P50/P95、RTF、内存、CPU、动作准确率、误唤醒/漏唤醒率。
3. 接入 sherpa-onnx open-vocabulary KWS，替换不断增加文本别名的策略。
4. 保存机器可读 JSON/CSV 结果，在 CI 或 release 中生成对比表。

验收指标：每个性能声明都能由一个命令重跑；README 的数字链接到原始结果和硬件环境。

### 第三个里程碑：形成可扩展生态

1. 提取 `TurnCoordinator`，让在线/离线只提供 provider adapter。
2. 将字符串 JSON 升级为自定义 ROS 2 interface，并保留外部 JSON gateway。
3. 写三篇插件教程：新增模型 provider、新增动作、新增机器人 executor。
4. 接入第二种机器人或机械臂，证明 interface 不是只为 TurtleBot3 定制。
5. 可选增加 MCP gateway，与小智类设备或外部 Agent 生态互通。

验收指标：贡献者无需修改核心节点即可新增一个 provider 和 executor；至少有一个外部复现
或贡献 PR。

## 6. 不建议优先做的事情

- 在没有真实评测前继续堆 LoRA 宣传数字。
- 在基础 demo 不稳定时增加复杂导航、视觉和多 Agent。
- 为每个模型复制一个 ROS 节点。
- 使用模糊字符串匹配无限扩充短唤醒词；短词应由声学 KWS 解决。
- 把模型、编译产物或 llama.cpp 源码提交进主仓库。
