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
bash scripts/acceptance_test.sh mock
```
