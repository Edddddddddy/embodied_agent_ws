# 参考资料与工具

## 上游资料

- Navigation2：<https://github.com/ros-navigation/navigation2>
- BehaviorTree.CPP：<https://github.com/BehaviorTree/BehaviorTree.CPP>
- twist_mux：<https://github.com/ros-teleop/twist_mux>
- teleop_twist_keyboard：<https://github.com/ros2/teleop_twist_keyboard>
- Nav2 Velocity Smoother：<https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html>
- Nav2 Collision Monitor：<https://docs.nav2.org/tutorials/docs/using_collision_monitor.html>

使用边界：

- 参考上游接口和成熟组件职责，不复制其仓库结构。
- 键盘核心自行实现为可测试 C++ 模块；不把 Python teleop 当作生产节点。
- `twist_mux` 与 Nav2 安全组件通过 rosdep/APT 安装，不把第三方源码复制进本仓库。

## 本仓库关键资料

- `src/embodied_agent_interfaces/`：typed msg/action/srv。
- `src/embodied_agent_cpp/src/action_guard_node.cpp`：动作白名单与限幅。
- `src/embodied_agent_cpp/src/action_scheduler.cpp`：单 active Action、FIFO 和 priority stop。
- `src/embodied_simulation/src/simulation_control_node.cpp`：BT 与 pluginlib 执行入口。
- `src/embodied_simulation/src/simulation_ros_io.cpp`：原语速度发布边界。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py`：SLAM/Nav2 会话状态机。
- `tools/acceptance/`：进程租约、探针、场景和证据校验。

## 开发工具

| 工具 | 用途 | 典型命令 |
|---|---|---|
| Git worktree | 隔离 release、功能和旧重构分支 | `git worktree list` |
| GitHub CLI | PR、CI、分支状态 | `gh pr checks`, `gh run view` |
| rosdep | 安装 ROS 包依赖 | `rosdep install --from-paths src` |
| colcon | ROS 2 构建和测试 | `colcon build`, `colcon test` |
| pytest | Python/仓库契约测试 | `python3 -m pytest -q tests/repository` |
| ros2 CLI | topic/action/service/lifecycle 诊断 | `ros2 topic info`, `ros2 action list` |
| actionlint | GitHub Actions 静态检查 | `actionlint` |
| shellcheck | Shell 脚本静态检查 | `shellcheck scripts/...` |
| Gazebo/RViz | 物理运动与可视化 | 由 showcase launch 统一启动 |

## WSL 调用规范

PowerShell 直接拼接包含 `$`、管道和 here-doc 的 `bash -lc` 容易二次解释。优先：

1. 简单命令使用明确的 Linux 绝对路径。
2. 复杂逻辑写入仓库脚本并用 ShellCheck 测试。
3. 避免在 PowerShell 字符串内构造 Bash 变量。
4. 详细模板见 `docs/development/WSL_POWERSHELL.md`。

