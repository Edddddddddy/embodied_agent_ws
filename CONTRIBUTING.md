# 贡献指南

感谢你改进本项目。仓库优先保证“语音输入 → 动作解析 → C++ 安全链 → Gazebo”可复现，
请不要在主链稳定前引入复杂导航、视觉或多 Agent 功能。

## 开发环境

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh
```

不要提交 `.env`、模型、`third_party/llama.cpp`、`build/`、`install/` 或测试日志。

## 修改原则

- C++ 负责实时音频、动作安全、Lifecycle/Action/BT 与控制；Python 负责模型 adapter 和编排。
- 新模型或机器人应实现现有 interface，不应复制完整 Agent 或控制节点。
- LLM 输出必须经过 ActionGuard；任何终止路径都必须让 executor 停车。
- 注释解释线程、安全、时序和设计原因，不复述显而易见的代码。
- 行为变更先写通过公开 interface 的测试，再做最小实现。

## 提交前检查

分支流程：

- `main` 只保存稳定可验收版本。
- `dev` 是日常集成分支。
- 新功能从 `dev` 拉 `feature/<name>`，修复从 `dev` 拉 `fix/<name>`。
- 功能分支测试通过后合并回 `dev`；里程碑验收通过后再从 `dev` 合并到 `main`。

```bash
# 最低门槛：仓库结构、130+ ROS/单元测试和无外部模型 smoke
bash scripts/acceptance_test.sh mock

# 按改动范围增加真实依赖验收
bash scripts/acceptance_test.sh online
bash scripts/acceptance_test.sh offline
bash scripts/acceptance_test.sh gazebo
```

测试探针统一放在 `tests/integration/`，用户可运行的命令放在 `scripts/`，包内纯逻辑测试
放在对应 `src/<package>/test/`。提交信息建议采用 `feat:`、`fix:`、`refactor:`、`test:`、
`docs:` 前缀，并保持一次提交只表达一个可回滚意图。

## 新增 RobotExecutor

实现与验收步骤见
[架构与知识笔记](docs/ARCHITECTURE_AND_KNOWLEDGE.md#8-新增-robotexecutor-插件教程)。
至少需要动态发现 GTest 和一条 Action/BT/diagnostics 集成测试。
