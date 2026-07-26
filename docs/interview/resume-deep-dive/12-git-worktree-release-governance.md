# Git worktree 与发布治理

本章解释一个容易混淆的问题：`~/embodied_agent_ws` 和
`~/embodied_agent_ws_worktrees/*` 通常不是多份项目，而是同一 Git 仓库的多份工作目录。判断版本要看
branch、commit 和 dirty 状态，不能只看目录名。

## 1. 一分钟讲法

口述：

项目使用一个 Git 仓库和多个 worktree 隔离稳定版、集成版及并行功能。`main` 只保存可演示版本，
`dev` 汇总已经通过 PR/CI 的功能，开发从 `dev` 拉 `feature/*` 或 `fix/*`；发布时再用 `release/*`
冻结候选，合入 `main` 后创建 annotated Tag 和 GitHub Release。每个 worktree 使用自己的源码、
`build/install/log`，不能 source 另一份 worktree 的 install。删除 worktree 前先确认 clean、提交已被
目标分支包含且没有需要保留的 ignored 产物。

## 2. 一个仓库为什么能有多个目录

普通 clone 只有一个 working tree。`git worktree add` 可以为同一个 `.git` 对象库增加独立的工作目录和
index，因此不同分支可以同时打开，不必反复切换：

```text
同一个 Git common dir
├── ~/embodied_agent_ws                    → main
├── ~/embodied_agent_ws_worktrees/dev      → dev
└── ~/embodied_agent_ws_worktrees/<topic>  → feature/* 或 fix/*
```

共享的是 commit、branch、tag 和 remote refs；独立的是已检出文件、暂存区以及通常被忽略的
`build/install/log`。以下命令用于确认身份：

```bash
git rev-parse --show-toplevel
git rev-parse --git-common-dir
git branch --show-current
git rev-parse HEAD
git status --short --branch
git worktree list
```

目录名字只是方便人阅读，不能证明当前分支。`git rev-parse --git-common-dir` 指向同一位置，才说明这些
目录属于同一个仓库。

## 3. 分支职责

```text
feature/* 或 fix/*
        │ PR + required CI
        ▼
       dev
        │ release/* 冻结、回归和人工验收
        ▼
       main
        │ annotated tag + GitHub Release
        ▼
   可追溯演示版本
```

| 分支 | 接收什么 | 不接收什么 |
| --- | --- | --- |
| `main` | 已完成发布门槛的稳定版本 | 未验收功能、临时调试提交 |
| `dev` | 已通过 PR/CI、可共同集成的完整功能 | 个人未收口 WIP |
| `feature/*` | 新能力及其测试、文档 | 与主题无关的大范围整理 |
| `fix/*` | 可复现缺陷、回归测试和修复 | 顺手增加的新功能 |
| `release/*` | 版本号、发布说明、最终修复 | 改变范围的大功能 |
| `wip/*` | 必须保留但尚不能交付的本地工作 | 默认推送或合并 |

功能做完整后再 push 和开 PR，PR 中写清变更、自动测试、人工验收和事实边界。required checks 通过只证明
CI 覆盖的范围；麦克风、大模型和长时 Gazebo/SLAM 仍需本地证据。

## 4. ROS 工作空间为什么不能混用 install

Colcon 的 `install/setup.bash` 会把包前缀、Python 路径、插件和 share 目录加入当前 shell。如果在
topic worktree 阅读新源码，却 source 了另一个 worktree 的旧 install，`ros2 launch` 可能加载旧节点；
这会造成“代码已改但行为没变”的假回归。

每个 worktree 都使用自身目录：

```bash
cd ~/embodied_agent_ws_worktrees/<topic>
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j2 -l2" colcon build --symlink-install --executor sequential
source "$PWD/install/setup.bash"
```

`--executor sequential` 只串行调度包，`--parallel-workers` 也主要约束 colcon 的包级调度；它们不会限制
当前 `colcon-cmake` 在单个 Makefile 包内部使用多少编译进程。该版本只有在 `MAKEFLAGS` 已包含
`-j/--jobs` 时，才不会按 CPU 数自动追加 `-j16 -l16`。因此 8 GiB 左右的 WSL 实例使用
`MAKEFLAGS="-j2 -l2"` 明确限制单包 C++ 编译并发。若构建没有编译错误却突然退出，应先看 `dmesg`
是否有 `Out of memory`，不要直接判断为源码失败；中断后只续编失败包，不要删除已有构建产物。

切换 worktree 时最好开新终端。必须复用终端时，先确认：

```bash
printf '%s\n' "$COLCON_PREFIX_PATH"
ros2 pkg prefix embodied_agent_core
```

包前缀必须指向当前 worktree。`build/`、`install/` 和 `log/` 不跨 worktree 复制，也不提交 Git。

## 5. 安全清理 worktree 和分支

清理不是直接删除目录。先记录绝对路径，再逐项证明可以回收：

```bash
git -C <worktree> status --porcelain
git -C <repo> merge-base --is-ancestor <topic-head> origin/dev
git -C <repo> worktree remove <worktree>
git -C <repo> branch -d <topic-branch>
git -C <repo> worktree prune
```

只有 `status --porcelain` 为空且 topic head 已被目标分支包含，才进入自动清理。若有未提交内容，先审查
敏感信息，再放到明确的本地 `wip/*` commit 或生成 bundle；若提交未被包含，先 PR、cherry-pick 或保留
分支。ignored 的模型、bag、日志和证据不会出现在普通 `git status`，删除前还要单独确认是否需要归档。

坚持使用 `git branch -d` 的合并检查，不用 `-D` 绕过保护；不使用 `reset --hard`、`clean -fdx` 或
force push 处理“看起来多余”的版本。

## 6. CI、Tag 与 GitHub Release 各证明什么

- PR：记录为什么改、评审结论以及从哪条分支合入。
- CI：证明该 commit 通过仓库结构、构建、单元和容器门禁。
- annotated Tag：给一个不可歧义的版本名绑定确切 commit。
- GitHub Release：把 Tag、发布说明、已知边界和验收产物组织成可交付入口。

发布前检查：

```bash
git status --short --branch
git fetch --prune --tags origin
gh pr checks <PR_NUMBER>
git tag -a vX.Y.Z -m "vX.Y.Z"
git push origin vX.Y.Z
gh release create vX.Y.Z --verify-tag --generate-notes
```

Tag 必须在 `main` 的待发布 commit 上创建，不能用移动 Tag 修正错误；需要修复时发布下一个 patch 版本。
正式 SLAM 证据还应记录 `source_revision` 且 `source_dirty=false`，这样报告、commit 和 Release 才能
相互核对。

## 7. 面试中的事实边界

可以说：

- 使用一个仓库的多 worktree 隔离稳定、集成和 topic 开发。
- 使用 `feature/fix → dev → release → main`、required CI、Tag 和 Release 管理交付。
- 在清理前验证 dirty 状态、提交可达性和 ignored 资产，ROS install 与源码同 worktree。

不要说：

- worktree 是多份互不相关的仓库或自动产生独立远端历史。
- CI 通过就等于真实麦克风、模型和完整 SLAM 已经验收。
- 目录删掉等于安全删除分支；Tag 存在就等于已经发布 GitHub Release。

当前架构事实见 [系统架构](../../ARCHITECTURE.md)，各级门槛见 [测试手册](../../TESTING.md)。
