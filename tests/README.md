# 测试目录

- `repository/`：仓库结构与交付约束，不依赖 ROS graph。
- `integration/`：由 `scripts/smoke_test_*.sh` 驱动的 ROS 探针，只通过 topic、Action、
  diagnostics 和 odom 等公开 interface 验证行为。
- `src/<package>/test/`：C++ GTest 或 Python 单元测试，随 `colcon test` 运行。

集成探针本身不负责启动系统，也不自行选择 ROS domain；对应 smoke runner 负责隔离
`ROS_DOMAIN_ID`、启动进程、收集日志与清理资源。这样同一个探针可以复用于独立进程、
component container 或 Gazebo launch。

统一入口：

```bash
bash scripts/acceptance_test.sh --help
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh mock
```

建议日常开发优先跑 `core`：它覆盖仓库结构护栏、在线/离线 Agent Python
单元测试，以及 C++/仿真包的 GTest。`mock` 会在 `core` 之上继续启动 ROS
smoke runner，适合提交前回归；`gazebo`、`continuous-*` 和真实麦克风模式用于
链路验收。

脚本整理原则：

- `scripts/acceptance_test.sh` 是用户入口，只暴露稳定验收模式。
- `scripts/run_core_tests.sh` 是开发入口，只收口最典型、最快反馈的单元/结构测试。
- `scripts/smoke_test_*.sh` 是 acceptance 后端 runner，仍被统一入口调用时不能删除。
- `tests/integration/test_*.py` 只做探针，不负责启动系统；对应 smoke runner 负责
  `ROS_DOMAIN_ID`、进程清理和日志收集。
