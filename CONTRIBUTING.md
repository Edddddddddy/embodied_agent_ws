# 贡献指南

感谢你改进本项目。仓库优先保证“语音输入 → 动作解析 → C++ 安全链 → Gazebo/Nav2/SLAM”
可复现；新增能力必须保留 typed 接口、安全停车和可审计验收证据。

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
- 新功能从 `dev` 拉 `feature/<name>`，修复从 `dev` 拉 `fix/<name>`，纯架构整理使用
  `refactor/<name>`。
- 功能分支测试通过后合并回 `dev`；里程碑验收通过后再从 `dev` 合并到 `main`。

`dev` 和 `main` 已启用同一套分支保护：所有变更都通过 PR 合并，并且必须通过
`repository-layout`、`frontier-patch-replay`、`build-and-test` 和
`Build and test container` 四项 required checks。两条分支都禁止 force-push 和删除，
管理员也遵守这些规则。仓库当前采用单人维护流程，因此 required approving reviews 为
`0`；不要求维护者给自己的 PR 审批，但 PR 和四项检查仍然不可跳过。

```bash
# 最低门槛：仓库契约、Python/C++ 单测与无外部模型 smoke
bash scripts/acceptance_test.sh verify core

# 按改动范围增加确定性关键功能验收
bash scripts/acceptance_test.sh verify voice
bash scripts/acceptance_test.sh verify control
bash scripts/acceptance_test.sh verify gazebo
bash scripts/acceptance_test.sh verify slam-nav
```

`continuous-offline/online` 是可选真人交互验收，仅在 `--help-all` 中展示，不属于合并前自动门禁。

pytest 断言和 fixture 放在 `tests/`，可执行 ROS graph probe 放在
`tools/acceptance/probes/<domain>/`，用户稳定入口放在 `scripts/`，包内纯逻辑测试放在对应
`src/<package>/test/`。Probe 文件不使用 `test_` 前缀，也不得反向导入 `tests.*`；环境激活、domain
隔离和进程清理由 smoke/handler 拥有。提交信息建议采用 `feat:`、`fix:`、`refactor:`、`test:`、
`docs:` 前缀，并保持一次提交只表达一个可回滚意图。

## 新增 RobotExecutor

实现与验收步骤见
[ROS 2/C++ 控制学习笔记](docs/learning/ROS2_CPP_CONTROL.md)。
至少需要动态发现 GTest 和一条 Action/BT/diagnostics 集成测试。
