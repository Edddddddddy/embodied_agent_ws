# 验收入口模块

`scripts/acceptance_test.sh` 只保留仓库定位和转发；本目录是验收接口的唯一事实源。

- `catalog.py`：每个 mode 显式声明 `HandlerDomain`、公开级别和 handler；名称只用于 CLI 展示，
  不参与执行库推断。
- `cli.py`：处理帮助、参数校验和模式选择，可注入 fake runner 做单元测试。
- `runner.py`：按需激活 ROS 环境并调用领域 handler；Bash `$1` 始终是首个用户参数，mode 名不会
  作为隐藏参数泄漏到 handler。
- `run_probe.sh`：可执行 Python probe 的统一启动 Interface；相对路径固定按仓库根解析，并使用
  `python3 -u` 保证管道或重定向下实时输出，避免 stdout 缓冲造成重型验收“假卡死”。它不负责
  激活 `.venv`、ROS overlay 或选择 domain，这些环境前置条件仍由调用它的 handler/smoke 脚本拥有。
- `paths.py`：Python 验收工具共享的仓库根定位 Interface；`repository_root()` 从任意仓库内路径
  向上查找 `scripts/acceptance_test.sh`、`tools/acceptance` 和 `src` 三个稳定标记，不依赖脆弱的
  `parents[n]`。因此 probe 迁移目录或运行在 Git worktree 中时，日志、配置和地图路径不会静默漂移。
- `probes/control/`：typed Action、Lifecycle、调度和 Gazebo 的独立 ROS graph 验收 Adapter；文件名
  不使用 `test_`，因为它们由 runner 执行而不是 pytest 断言。
- `probes/slam_nav/`：mapping、localization、Nav2、动态障碍的可执行 ROS graph probe，以及 canonical
  `slam-nav-e2e` 的会话编排 Module；各 probe 按需要复用仓库定位、typed transport 和证据边界。
- `typed_action_probe_utils.py`：复用生产 transport 构造/读取 typed 消息，禁止 probe 手写第二套协议。
- `progress.py`：为重型门禁提供阶段里程碑和定时心跳；完整 ROS 输出仍写证据日志。
- `dynamic_route.py`：在失败恢复后选择第一条真正可重规划的候选路线，并记录尝试审计。
- `dynamic_scenario_transaction.py`：ROS-free 清理策略；通过回调依次取消导航、归位障碍、清空检测、
  验证 tracker/costmap 已清除并确认零速度，且不会用清理错误覆盖原始场景异常。
- `slam_nav_evidence.py`：无 ROS 的几何与 E2E 证据深模块，统一业务观测、阈值和报告 `passed`；
  资源后置条件由事务在报告生成前独立强制。
- `handlers/`：保留必须依赖 Bash/ROS setup 的 control、voice、slam_nav 实现。

公开模式固定为 7 个。内部回归仍可通过 `--help-all` 发现，但不能写进新手必跑步骤。
新增模式时必须同时补充注册表测试；不要在 `acceptance_test.sh` 中重新增加 `case`。
一个 handler 只能归属一个领域库。需要兼容旧名称时优先迁移调用方，不得为同一重型 E2E 保留
两个可独立输出 PASS 的入口。
`tools/acceptance/probes` 不得导入 `tests.*`；依赖方向只能是测试验证工具，而不是运行工具依赖测试。
Python probe 不得通过固定父目录层级推导仓库根；确需从模块位置寻找仓库资源时必须复用
`paths.repository_root()`，CLI 已显式传入路径时则直接尊重调用者输入。
运行时间超过 30 秒的模式必须输出可定位的日志路径，并使用 `AcceptanceProgress` 或等价机制提供
周期心跳，避免“进程正常但终端无输出”的现场假卡死。
