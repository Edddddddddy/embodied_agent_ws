# 验收入口模块

`scripts/acceptance_test.sh` 只保留仓库定位和转发；本目录是验收接口的唯一事实源。

- `catalog.py`：声明模式名称、公开级别、领域和 handler，不执行副作用。
- `cli.py`：处理帮助、参数校验和模式选择，可注入 fake runner 做单元测试。
- `runner.py`：安全传递位置参数，按需激活 ROS 环境并调用领域 handler。
- `progress.py`：为重型门禁提供阶段里程碑和定时心跳；完整 ROS 输出仍写证据日志。
- `dynamic_route.py`：在失败恢复后选择第一条真正可重规划的候选路线，并记录尝试审计。
- `slam_nav_evidence.py`：无 ROS 的几何与 E2E 证据深模块，统一动态重规划和自动任务 PASS 语义。
- `handlers/`：保留必须依赖 Bash/ROS setup 的 control、voice、slam_nav 实现。

公开模式固定为 7 个。内部回归仍可通过 `--help-all` 发现，但不能写进新手必跑步骤。
新增模式时必须同时补充注册表测试；不要在 `acceptance_test.sh` 中重新增加 `case`。
运行时间超过 30 秒的模式必须输出可定位的日志路径，并使用 `AcceptanceProgress` 或等价机制提供
周期心跳，避免“进程正常但终端无输出”的现场假卡死。
