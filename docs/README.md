# 项目文档索引

README 是安装与演示入口。本目录只保留分层资料，避免每份文档重复部署命令。

## 权威文档

| 需求 | 文档 | 维护边界 |
| --- | --- | --- |
| 快速理解、部署、主演示 | [README](../README.md) | 对外入口，只放稳定命令 |
| 架构、模块职责、上下游接口 | [ARCHITECTURE_AND_KNOWLEDGE.md](ARCHITECTURE_AND_KNOWLEDGE.md) | 系统结构事实源 |
| 自动/人工验收与故障定位 | [TESTING_AND_ACCEPTANCE.md](TESTING_AND_ACCEPTANCE.md) | 验收标准事实源 |
| 关键技术、文件、函数与方案比较 | [LEARNING_NOTES.md](LEARNING_NOTES.md) | 学习与面试技术笔记 |
| 15 分钟讲解顺序 | [PROJECT_PRESENTATION_15MIN.md](PROJECT_PRESENTATION_15MIN.md) | 演示脚本 |

上述文档发生冲突时，以代码、`tools/acceptance/catalog.py` 和自动生成的 `docs/evidence/architecture_facts.json` 为准。

## 专题资料

- [VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md](VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md)：从麦克风到仿真执行的文件、函数和 topic/action 调用链。
- [SLAM_NAVIGATION_ENGINEERING.md](SLAM_NAVIGATION_ENGINEERING.md)：frontier、slam_toolbox、AMCL、Nav2、GTSAM 与动态障碍。
- [REAL_WORLD_SLAM_EVALUATION.md](REAL_WORLD_SLAM_EVALUATION.md)：公开 bag、ATE/RPE、回环与后端消融证据。
- [SHERPA_ONNX_DEPLOYMENT.md](SHERPA_ONNX_DEPLOYMENT.md)：离线 ASR 模型和 WSL 音频部署。
- [OFFLINE_BENCHMARK_REPORT.md](OFFLINE_BENCHMARK_REPORT.md)：llama.cpp/Q8/TTS 的报告口径与事实边界。
- [CODEX_WSL_POWERSHELL_SKILL.md](CODEX_WSL_POWERSHELL_SKILL.md)：Windows/WSL 开发命令约定与常见陷阱。

## 图与面试材料

- [FINAL_ARCHITECTURE_DIAGRAMS.md](FINAL_ARCHITECTURE_DIAGRAMS.md)：Mermaid 架构图和端到端时序。
- [source-code-deep-dive/README.md](source-code-deep-dive/README.md)：按 ROS、音频、Action、Nav2、生命周期分章的代码深读。
- [interview/04-ros2-cpp-resume.md](interview/04-ros2-cpp-resume.md)：ROS 2/C++ 简历表述。

## 历史与路线

- [CHANGELOG_AND_ROADMAP.md](CHANGELOG_AND_ROADMAP.md)：阶段版本与后续工作。
- [PROJECT_GAPS_AND_OPTIMIZATION.md](PROJECT_GAPS_AND_OPTIMIZATION.md)：已知缺口，不代表已实现能力。

其余报告型文档保留作为历史证据，不应再复制到 README。后续整理遵循：先把独有内容并入权威文档，再删除重复文件。

## 最短阅读路径

首次运行：

```text
README → TESTING_AND_ACCEPTANCE.md
```

代码走读：

```text
ARCHITECTURE_AND_KNOWLEDGE.md → VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md
→ LEARNING_NOTES.md
```

SLAM/Nav2 面试：

```text
SLAM_NAVIGATION_ENGINEERING.md → REAL_WORLD_SLAM_EVALUATION.md
→ PROJECT_PRESENTATION_15MIN.md
```

## 维护规则

- 新命令只先加入 `tools/acceptance/catalog.py`，稳定后才进入 README。
- README 不展开 OpenLORIS/消融步骤；实验命令放 `tools/evaluation/README.md` 或专题文档。
- mock、dry-run、公开 bag、Gazebo、真实麦克风、真实模型证据必须明确区分。
- 文档中的文件/命令移动后，必须运行 `pytest -q tests/repository` 和 `architecture-facts`。
- 关键中文注释解释设计原因与风险保护，不做逐行翻译。
