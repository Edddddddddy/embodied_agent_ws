# 多模态演示踩坑与防回归清单

## 1. PowerShell 调 WSL 的二次解析

现象：

- Bash 的 `$变量` 被 PowerShell 先展开为空；
- `|`、`&&`、here-doc 到 WSL 后已残缺；
- Windows PATH 中的 `rg` 被 WSL 命中却无执行权限。

处理：

- 复杂逻辑写成仓库脚本；临时诊断使用字面量绝对路径和短命令。
- 必须内联时用 `wsl.exe -d Ubuntu-24.04 -- bash -lc '...'`，避免在 PowerShell
  外层插值 Bash 变量。
- 在 WSL 确认 `command -v rg`；异常时使用 Linux `/usr/bin/grep`。
- 不跨 PowerShell/Bash 生成再删除路径列表，递归操作始终在同一个 shell 完成。

本轮曾因 `$target` 被提前展开，`git worktree add` 收到空路径；修复方式是先核对
`git worktree list --porcelain`，再使用明确 Linux 绝对路径。

## 2. WSL 内存抖动被误判为代码死锁

现象：8 GiB WSL 中重叠 colcon/CMake 与编辑器索引，`vmmemWSL`、Swap 快速上涨，
普通 ROS CLI 数十秒无响应。

处理：

```bash
free -h
swapon --show
```

确认无旧构建后使用 `CMAKE_BUILD_PARALLEL_LEVEL=1`、`--executor sequential` 和
精确 `--packages-select`；禁止并发启动第二个构建。重型验收前先清理旧仿真。

## 3. ROS APT 局部升级造成 ABI 混装

现象：代码未变，Nav2 lifecycle manager 却报 `undefined symbol`。

根因：`rclcpp/diagnostic_updater` 与已安装 Nav2 patch 版本不一致。

处理：演示过程中不改 APT；安装依赖后统一升级同一 ROS 组件族，重新构建并运行
最小 Nav2 lifecycle smoke。

## 4. `/cmd_vel` 有多个发布者

现象：语音、Nav2、键盘速度互相覆盖，小车跳变且无法证明谁拥有控制权。

处理：语音/Nav2 先进入自治 mux，键盘和自治结果进入 typed AuthorityGate，最后
统一经过 Collision Monitor。生产节点禁止直接写最终 `/cmd_vel`。

## 5. 把 mux priority 当作业务控制权

现象：HOLD 下某来源仍可能因相同/更高 priority 通过，旧速度在恢复时复驶。

根因：mux 是数值仲裁器，不理解 Action terminal、manager epoch 或接管。

处理：typed authority 做来源 allowlist；离开 AUTONOMY 时取消任务、发 priority
STOP，并用同代 terminal + 新鲜零速 ACK。deadman/RESET 只到 HOLD，RESUME 显式。

## 6. Lifecycle 请求成功不等于节点 ACTIVE

现象：cold start 中 configure/activate service 超时，TimerAction 到期后 stage 仍
继续启动，最后只表现为 Action server 缺失。

处理：

- persistent 模式关闭 typed bridge、`simulation_control` 的自启动和 stage 内部
  lifecycle manager；
- `SessionOrchestratorNode` 成为唯一状态变更所有者；
- service response 未知时重新读取状态，不把响应丢失等同于成功或失败；
- 每次等待同时检查 base/stage owner，最终状态不是 ACTIVE 就不放行。

不要用固定 sleep 或“进程存在”替代 lifecycle readiness。

## 7. `/initialpose` 发布成功不等于 AMCL 已初始化

现象：脚本无 subscriber 时仍退出 0；导航阶段读到旧 `/amcl_pose` 后假 ready。

处理：

```text
AMCL ACTIVE
 -> /initialpose subscription matched
 -> publish 本次返航终点
 -> 当前 navigation generation 新鲜 /amcl_pose
```

`/initialpose` 使用 RELIABLE + VOLATILE；旧 mapping 缓存不能跨代放行。单独 helper
若 subscriber timeout 必须非零退出。

## 8. 只杀 launch wrapper 会留下孤儿进程

现象：wrapper 已退出，但 detached Gazebo/AMCL/SLAM child 仍占 DDS domain，
下一次启动出现重复节点、SHM 端口或旧 topic。

处理：运行时登记 owned process tree，记录 PID 与 `/proc` starttime；清理时再
校验 starttime/PGID，按 explorer→stage→base 终止。不得只按进程名全局 kill，
也不得对已复用 PID 发信号。

## 9. 用 PID 或 launch parent 证明进程连续

现象：PID 复用、Gazebo Ruby wrapper、Agent launch parent 退出均可造成假 PASS。

处理：先按 `ROS_DOMAIN_ID`/`GZ_PARTITION` 唯一选择实际角色，再比较
PID、start ticks、boot ID、executable 与 argv hash；Agent 角色必须绑定实际
`offline_agent`/`online_agent`。原始 argv 不写报告，避免泄露凭据。

## 10. 缓存 PASS 与旧 evidence 冒充 fresh evidence

现象：

- verifier 只读取 `passed=true`；
- 旧 session 报告路径存在，就被当成本分支运行结果；
- mock ASR PASS 被描述为真实麦克风 PASS。

处理：

- checkpoint 要求 schema、精确 role/label、严格递增 wall time；
- verifier 从原始身份和 checkpoint 重算，不信任缓存结论；
- evidence 必须绑定当前 session、生成时间、revision 与 cleanup manifest；
- strict mock、persistent heavy、live microphone 分层记录。

2026-07-21 的 `20260721T072342Z-2344751-5452a492` 是修改前 strict mock 基线，
不是 `feature/demo-persistent-session` 的 fresh evidence。

## 11. 长会话停车后缺少 STOP 后新鲜零速

现象：机器人已静止，typed STOP 成功，但验证器报没有新鲜 `/cmd_vel=0`。

根因：Collision Monitor 默认 `stop_pub_timeout` 约 2 秒，长时间静止后抑制重复
零速；STOP 之前的零样本没有本次因果关系。

处理：persistent profile 延长最终零速心跳窗口，但不改变唯一最终出口和雷达
裁决；验证仍要求 STOP 边界之后的新鲜零速，不能旁路写底盘。

## 12. frontier 恢复原因遮蔽 hard budget

现象：已达到时间/距离上限后仍进入新恢复 epoch，重型门禁超时。

处理：先裁决 hard budget，再解释 `attempts_exhausted` 等恢复原因。近似完成仍需
连续低收益 epoch、账本排空、地图静默、最终 probe、返航和 typed STOP；不得为
缩短演示直接降低地图质量门槛。

## 13. 阶段参数或 localization provider 泄漏

现象：

- base 与 stage 读取不同 Nav2 参数；
- navigation stage 退出后 AMCL/map server 仍留在常驻 composition container；
- 下次阶段读取旧地图或出现重复节点。

处理：

- base 前原子生成唯一 session 参数快照；
- persistent navigation provider 默认不放入常驻 composition container；
- stage 退出关闭输入闸门、递增 generation 并清空缓存。

## 14. 频繁 push 触发昂贵 CI

现象：每个小改动都触发完整 ROS/容器构建，版本历史也难以评审。

处理：本地按职责拆 commit，但一个完整功能闭环后再 push；PR 目标为 `dev`，CI
全绿后合入。`main` 只通过发布 PR 更新，禁止 feature 分支直推。

## 15. `rosdep update` 不会刷新 Ubuntu 的 apt 索引

现象：frontier patch replay 已正确重放补丁，但安装
`ros-jazzy-image-geometry` 时，Ubuntu 镜像中的旧包地址返回 404。

根因：`rosdep update` 更新的是“ROS 依赖名到系统包”的映射，不等价于
`apt-get update`。长期存在的容器镜像可能携带已经过期的 apt 索引。

处理：在 CI 容器执行 `rosdep install` 前显式运行一次 `apt-get update`。PR #92
首次检查复现 404，加入该步骤后的同一 job 在 3 分 7 秒通过。以后遇到依赖下载
404，应先区分“依赖声明错误”和“软件源索引过期”，不要为了绕过 CI 删除正确依赖。

## 16. Nav2 的布尔 Launch 参数不是 Bash 布尔值

现象：传给 Nav2 的 `use_composition=false` 最终进入
`PythonExpression(['not ', use_composition])`，表达式求值时报
`name 'false' is not defined`，map server 和 AMCL 没有启动。

处理：在边界把字符串统一转换成 Python 字面量 `True`/`False`，并用真实 launch
probe 验证 map server、AMCL 和 lifecycle manager 已启动/激活。不能只验证参数
文件能够解析。

## 17. 可执行 Python 脚本不一定在 argv 中出现 `python`

现象：运行时连续性选择器找不到实际 Agent，但进程确实存在。

根因：带 Python shebang 的可执行入口可以直接 `exec`，`/proc/<pid>/cmdline`
只有脚本路径，不一定包含 `python3`。

处理：角色选择同时识别解释器启动和可信 shebang 入口，随后仍用
PID/start ticks/boot ID/executable/argv hash 校验身份，不能放宽成模糊进程名。

## 18. 接近饱和时仍有增益，不能无限恢复

现象：硬预算已到，但最近一次探索仍增加地图；立即结束过早，继续恢复又可能一直
运行。

处理：只允许一次最多 `240 s` 的最终确认，并在确认后检查连续低收益、账本排空、
最终 probe、地图质量、返航和 typed STOP。它是有界收尾，不是第二个无限探索阶段。

## 19. `colcon --executor sequential` 不等于 C++ 单线程编译

现象：WSL 只有约 8 GiB 内存时，虽然使用了 `colcon build --executor sequential`，
同一个 CMake 包仍同时启动大量 `cc1plus`，最终触发 OOM；第一次 OOM 后立即重跑还
可能留下不稳定实例，表现为 `Wsl/Service/E_UNEXPECTED` 或 VM 重启。

根因：`--executor sequential` 只约束 ROS 包之间的调度，CMake/Make 在包内部仍可
按 CPU 数并行。两层并发需要分别控制。

处理：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --executor sequential --parallel-workers 1
```

如果 OOM 后 WSL 命令持续返回 `E_UNEXPECTED`，先确认没有正在运行的验收，再完整
执行一次 `wsl --shutdown`，不要把新构建叠加在残留编译进程上。本轮在干净实例上
按上述限制完成 12 个包构建，并通过 `core` 门禁。
