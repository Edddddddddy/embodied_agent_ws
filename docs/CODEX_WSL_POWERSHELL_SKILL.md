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
- 需要临时 Python 探针时，优先写成测试文件，少用 `python - <<EOF`。

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
- 带复杂正则的搜索优先拆成多次固定字符串 `git grep`，不要让 PowerShell、WSL Bash、
  grep 三层同时解释引号。
- 多个互不依赖的简单检查由工具层分别调用；不要为省一行把 `&&` 拼到 PowerShell 命令中。
- 不在 Codex 的 PowerShell 命令字符串里临时写依赖 `$mode`、`$file` 等 Bash 变量的
  `for` 循环。变量可能在 PowerShell、`wsl.exe` 和 `bash -lc` 三层传递中丢失，最终把空参数
  交给脚本。验收模式应使用独立的固定命令；确实需要循环时，把循环写进仓库中的 `.sh`
  文件并直接执行该文件。

例如，下列固定调用比跨 Shell 动态循环更容易审计退出码：

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/ubuntu/embodied_agent_ws bash scripts/acceptance_test.sh continuous-multi-command
wsl.exe -d Ubuntu-24.04 --cd /home/ubuntu/embodied_agent_ws bash scripts/acceptance_test.sh navigation-demo
```

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
source install/setup.bash
```

## 4. GitHub CLI 与 git push

WSL 内优先使用 `gh`：

```bash
gh auth status
gh repo view
gh issue list --limit 5
gh pr checks
```

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
bash tests/integration/test_acceptance_cli.sh
pytest -q src/embodied_agent_core/test src/embodied_voice_frontend/test src/embodied_offline_agent/test
```

连续语音：

```bash
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh voice-readiness
```

C++/仿真：

```bash
colcon test --packages-select embodied_agent_cpp embodied_simulation --event-handlers console_direct+
colcon test-result --verbose
```
