# Codex WSL + PowerShell 开发 Skill

这份文档记录在 Windows PowerShell 中驱动 WSL Ubuntu 开发本项目时的固定流程和坑。

## 1. 推荐命令模板

优先使用 `--%` 阻止 PowerShell 继续解析后面的管道、括号和引号：

```powershell
wsl -d Ubuntu-24.04 --% bash -lc "cd /home/ubuntu/embodied_agent_ws && source scripts/activate.sh && pytest -q tests/repository"
```

简单命令可以不用 `--%`：

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'cd /home/ubuntu/embodied_agent_ws && git status --short --branch'
```

经验规则：

- 命令里有 `|`、`$()`、`[]`、中文 here-doc、复杂正则时，用 `--%`。
- 能用单条简单命令就不要嵌套 here-doc。
- 纯逻辑断言写入 `tests/`；需要真实 ROS/provider 运行时的可执行探针写入
  `tools/acceptance/probes/<domain>/`，再由 handler 调用。不要用带 `main()` 的文件冒充 pytest。

### Codex `exec` 中更稳定的形式

Codex 工具的外层 shell 仍可能是 Windows PowerShell 5；它不支持裸 `&&`，而且会先解析
`|`、正则里的 `[]` 和嵌套引号。对简单命令不要再套一层 `bash -lc`：

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/ubuntu/embodied_agent_ws git status --short --branch
wsl.exe -d Ubuntu-24.04 --cd /home/ubuntu/embodied_agent_ws git grep -n DeclareLaunchArgument -- src
```

需要激活 ROS 环境和执行多步 Bash 时，把完整命令放在同一对单引号内，并使用分号：

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/ubuntu/embodied_agent_ws bash -lc 'source scripts/activate.sh; pytest -q tests/repository'
```

- 不把 `| head` 写在 `wsl.exe ...` 外面；PowerShell 会把它当成自己的管道。
- 即使 `|` 看似位于 `bash -lc` 字符串内，只要还嵌套了双引号正则，也不要直接执行。例如
  `grep -E "explore|gz sim|nav2"` 曾被外层拆开，并意外执行 `gz sim`，留下孤儿 Gazebo server。
  改为多次 `grep`/`pgrep -af NAME`，或把诊断逻辑写入仓库脚本后调用。
- 带复杂正则的搜索优先拆成多次固定字符串 `git grep`，不要让 PowerShell、WSL Bash、
  grep 三层同时解释引号。
- 多个互不依赖的简单检查由工具层分别调用；不要为省一行把 `&&` 拼到 PowerShell 命令中。
- 不在 Codex 的 PowerShell 命令字符串里临时写依赖 `$mode`、`$file` 等 Bash 变量的
  `for` 循环。变量可能在 PowerShell、`wsl.exe` 和 `bash -lc` 三层传递中丢失，最终把空参数
  交给脚本。验收模式应使用独立的固定命令；确实需要循环时，把循环写进仓库中的 `.sh`
  文件并直接执行该文件。
- 同样不要在一条 Windows 包装命令里先赋值 `SESSION_DIR=...`，随后又拼接
  `$SESSION_DIR/report.json` 或在 `trap` 中引用 `$LAUNCH_PID`。外层可能在 Bash 执行前展开变量，
  导致文件意外写到根目录或清理空进程组。长生命周期任务应使用仓库脚本管理变量、trap 和 PID；
  临时诊断则拆成“启动一个 PTY session”和“使用固定路径运行探针”两个调用。

例如，下列固定调用比跨 Shell 动态循环更容易审计退出码：

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/ubuntu/embodied_agent_ws bash scripts/acceptance_test.sh continuous-multi-command
wsl.exe -d Ubuntu-24.04 --cd /home/ubuntu/embodied_agent_ws bash scripts/acceptance_test.sh navigation-demo
```

Gazebo 重型测试重跑前先确认没有孤儿 server；多个默认 partition 的 server 会让 entity 创建、
bridge 和 `/odom` 落到不同进程，表现为“创建成功但 odom/TF 永远不存在”：

```powershell
wsl.exe -d Ubuntu-24.04 -- pgrep -af "gz sim"
wsl.exe -d Ubuntu-24.04 -- pgrep -af voice_slam_session_orchestrator
```

仓库提供显式、两阶段确认的定向清理；它同时识别 Gazebo/ROS 节点和被中断后仍在等待
超时的 unknown-world acceptance/probe，避免后者继续占内存或写回陈旧报告：

```bash
bash scripts/cleanup_simulation_processes.sh
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh
```

脚本先列出 PID 与完整命令，再按已审计模式逐个终止。不要改用全局 `pkill`，避免伤到
用户正在运行的其他演示。

## 2. 搜索文件

不要在仓库根目录无脑 grep，会扫到 `.venv`、`third_party/llama.cpp`、`__pycache__`，输出会爆炸。

如果 `command -v rg` 指向 Windows Codex App 目录，例如
`/mnt/c/Program Files/WindowsApps/.../app/resources/rg`，WSL 里直接调用可能报
`Permission denied`。此时不要继续调 PowerShell 引号，先改用 `grep/find`，或在 WSL
内安装 `ripgrep` 后显式使用 WSL 版本。

推荐：

```bash
find src tests scripts docs -type f \
  ! -path '*/__pycache__/*' \
  ! -name '*.pyc' \
  -print
```

或者限定目录：

```bash
grep -R "ContinuousCommandQueue" -n src tests scripts --exclude='*.pyc'
```

## 3. Python/ROS 测试前先激活环境

直接跑 `pytest` 可能找不到 ROS 包：

```bash
source scripts/activate.sh
pytest -q src/embodied_agent_core/test src/embodied_voice_frontend/test src/embodied_offline_agent/test
```

如果涉及构建后的包或 ROS topic/action：

```bash
source scripts/activate.sh
embodied_workspace_doctor true   # 自动建图/导航任务需要 Explore Lite 时
```

`activate.sh` 已加载 `install/setup.bash`，不要重复 source。公共入口会从自身路径推导并 export
`WORKSPACE`。同一终端从主仓库切到 worktree 时，旧 `WORKSPACE` 会被安全拒绝：

```bash
unset WORKSPACE
cd /path/to/feature-worktree
source scripts/activate.sh
```

若看到 `WORKSPACE 与当前入口所属仓库不一致`，不要强行 source 主仓库 install。只有 CI 明确需要
跨目录覆盖时才同时设置：

```bash
export WORKSPACE=/absolute/repo
export EMBODIED_ALLOW_WORKSPACE_OVERRIDE=true
```

自动任务启动前的 doctor 会检查两个核心 package prefix、`RUN_AUTOMATIC_MISSION` 生成接口和
`explore_lite` 都属于当前 install；失败输出本身就是修复命令。

### 不要用“半安装层”运行整仓门禁

`colcon build --packages-up-to embodied_slam_tools` 只保证目标包及其依赖进入当前
`install/`，不会安装与它无依赖关系的 Agent 包。此时 `install/setup.bash` 虽然存在，
`acceptance_test.sh core` 里的在线/离线 benchmark 仍可能报
`ModuleNotFoundError: embodied_agent_core`，随后表现为“报告文件没有生成”。

这不是报告逻辑故障。运行整仓 `core` 前先建立完整安装层：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
bash scripts/acceptance_test.sh core
```

定向开发可以继续使用 `--packages-up-to`，但它的结果只能证明该依赖闭包，不能替代整仓
门禁。排障时先检查缺失模块是否出现在 `install/`，不要为了掩盖环境问题修改业务代码。

### ROS 2 接口 hash 变化后的完整重建

增加 msg 字段或新增 msg/action 会改变 ROS 2 interface type hash。旧 overlay 中的 Python/C++ 类型支持
可能仍能 import，却无法与新节点正确反序列化；这种故障常表现为 topic 存在但 callback 永远不触发。
不要只重建 `embodied_agent_interfaces`，必须重建所有下游依赖：

```bash
unset WORKSPACE
cd /path/to/current-worktree
source /opt/ros/jazzy/setup.bash
# 确认当前目录后再清理该 worktree 自己的生成目录。
rm -rf build install log
colcon build --symlink-install --executor sequential
source install/setup.bash
embodied_workspace_doctor true
```

本阶段 `FrontierExplorationEvidence`、`SlamNavigationGoalEvidence` 和扩展后的
`SlamSessionState` 就属于这种变更。旧 rosbag 只可作历史证据，不能证明当前 strong typed schema。

## 4. GitHub CLI 与 git push

WSL 内优先使用 `gh`：

```bash
gh auth status
gh repo view
gh issue list --limit 5
gh pr checks
```

调用 `gh api` 时不要把 PowerShell 生成的 JSON 直接通过管道送入 stdin。Windows
PowerShell 的管道编码会在 JSON 开头加入 BOM，GitHub 可能把它识别为非法首字符；因此避免
`ConvertTo-Json | gh api --input -` 和 `Get-Content request.json | gh api --input -`。
需要复杂 JSON 时，先显式写成无 BOM 的 UTF-8 文件，再用 `--input` 传递：

```powershell
$json = @{ key = "value" } | ConvertTo-Json -Depth 10
$path = Join-Path $PWD "gh-api-request.json"
$utf8 = New-Object System.Text.UTF8Encoding -ArgumentList $false
[System.IO.File]::WriteAllText($path, $json, $utf8)
gh api --method PATCH repos/OWNER/REPO/... --input $path
```

如果 `gh` 运行在 WSL，就在 WSL 内创建同样的 UTF-8 JSON 文件并传 Linux 路径。只有简单字段时，
优先使用 `gh api --method ... -f key=value`；这两种方法都比跨 PowerShell 管道传 JSON stdin
稳定。

首次登录：

```bash
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
```

如果 WSL 的 HTTPS git 凭据失效，而 Windows 侧有 Git Credential Manager，可以临时用 Windows git 推送：

```powershell
git -C "\\wsl.localhost\Ubuntu-24.04\home\ubuntu\embodied_agent_ws" push
```

注意：Windows git 查看 WSL 工作区可能因为权限/行尾显示大量假修改。最终状态以 WSL 内为准：

```bash
git status --short --branch
```

## 5. 开发分支流程

```bash
git status --short --branch
git checkout dev
git pull --ff-only
git checkout -b feature/<name>
```

每个功能阶段：

```bash
git add <files>
git commit -m "type: concise message"
git push -u origin feature/<name>
```

用 GitHub issue/PR 记录进度：

```bash
gh issue create --title "..." --body "..."
gh pr create --base dev --head feature/<name> --draft --title "..." --body "..."
gh pr checks --watch
```

## 6. 本项目常用验证命令

轻量：

```bash
pytest -q tests/repository
bash tests/integration/control/test_acceptance_cli.sh
pytest -q src/embodied_agent_core/test src/embodied_voice_frontend/test src/embodied_offline_agent/test
```

连续语音：

```bash
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh voice-readiness
```

自动建图导航：

```bash
bash scripts/acceptance_test.sh slam-nav-showcase-stage
bash scripts/acceptance_test.sh slam-autonomous-mission-stage
# known-world 稳定回归：
bash scripts/acceptance_test.sh slam-nav-e2e
# unknown-world 正式自主闭环；完整功能收口时再跑，不为零散编辑频繁触发 CI：
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

`slam-nav-e2e` 是允许 bootstrap/语义地点的 known-world 确定性回归；
`unknown-world-slam-e2e` 才是禁止真值/固定路线进入 robot policy 的正式自主门禁。先用
`bash scripts/acceptance_test.sh --help` 确认当前分支已注册两个模式；若帮助中没有后者，说明终端仍
停留在旧分支、旧 worktree 或旧脚本，而不是 ROS 运行时故障。

不要在同一终端额外 `source ~/nav2_ws/install/setup.bash`。公开 unknown-world 会在 session 内剔除此外部
overlay，并把 Nav2 来源写入 manifest；这是为了避免功能分支在不同终端运行到不同版本的 Lifecycle
manager。linked worktree 的 `models/`、`third_party/` 天然为空，语音入口会用 Git common-dir 自动解析
主工作区资产；自定义位置时只设置 `EMBODIED_RUNTIME_ROOT`，不要把 `WORKSPACE` 改回主目录。

Unknown-world 证据在：

```text
logs/acceptance/unknown_world_slam_nav/<session_id>/unknown_world_slam_e2e_report.json
logs/acceptance/unknown_world_slam_nav/<session_id>/runtime.log
logs/acceptance/unknown_world_slam_nav/<session_id>/acceptance_session.json
```

不要用 `logs/acceptance/slam_nav/...` 的旧 known-world 报告代替。正式报告要求 schema v4，并同时记录
strong typed frontier/目标证据、独立 `mission_outcome`、地图覆盖、AMCL/Gazebo 定位误差、动态重规划
和最终零速。Gazebo truth 与静态真值只属于 evaluator，不能复制进 mission YAML 或调参脚本。

C++/仿真：

```bash
colcon test --packages-select embodied_agent_cpp embodied_simulation --event-handlers console_direct+
colcon test-result --verbose
```
