# 踩坑与风险清单

## 1. 多个 `/cmd_vel` 发布者

现象：语音原语、Nav2 和键盘同时发布时，小车速度跳变，优先级不可预测。

根因：ROS 2 topic 本身不提供控制权语义。

方案：Nav2/语音由 `twist_mux` 仲裁，键盘与自治结果再经过 typed
`VelocityAuthorityGate`，最后经过 Collision Monitor；禁止其他生产节点直接发布最终 `/cmd_vel`。

## 2. 把 twist_mux lock 当作控制权授权

现象：HOLD 下和 lock 同优先级的键盘 topic 仍可能通过；AUTONOMY 下残留键盘
速度又会压过 Nav2/语音。lock 切换也不会主动产生一帧零速。

根因：`twist_mux` 比较的是全局数值优先级，不是按状态选择来源的 allowlist。

方案：mux 只生成自治速度；C++ Gate 依据 authority、owner、状态心跳与速度
TTL 做正授权，并在任何失效状态周期发布零速。

## 3. 松键后自动任务“复活”

现象：用户键盘接管后松开按键，旧 Nav2 goal 继续输出。

根因：deadman 零速只处理瞬时速度，没有处理旧任务所有权。

方案：接管先 cancel 自动任务并发 priority STOP；deadman 后进入 HOLD；必须显式 RESUME。

## 4. RESET 急停等于恢复运动

现象：解除急停瞬间恢复旧速度。

根因：把“解除锁存”和“授权自动控制”混成一个操作。

方案：RESET 只到 HOLD，RESUME 是第二个显式操作。

## 5. 建图和导航各自重启整个场景

现象：RViz/Gazebo 闪退重开、机器人状态丢失，现场演示割裂。

根因：旧阶段脚本以完整进程组为生命周期边界。

方案：第二轮保持 Gazebo/RViz/Agent 常驻，只切 SLAM/Explore 与 map_server/AMCL/Nav2。

## 6. 快速演示冒充严格验收

现象：现场为节省时间加载旧地图，却声称本次完成未知地图建图。

方案：不同 evidence kind；降级明确记录 `same_session=false`、`degraded=true`。

## 7. WSL PowerShell 引号二次解释

现象：`$variable`、`|` 或 here-doc 被 PowerShell 先展开，Bash 收到残缺命令。

方案：复杂步骤进入版本化脚本；临时诊断使用无变量的短命令。明确使用
`wsl ... -- bash -lc '...'` 把 `$变量`、管道和 `&&` 留给 Bash，或拆成单命令。
禁止用 PowerShell 生成再删除 WSL 路径列表。

本轮实例如下：PowerShell 在 `wsl ... bash -lc` 外层提前展开了 Bash 的
`$target`，导致 `git worktree add` 收到空路径并触发 Git 内部断言。修复时
改用已经核对过的字面量绝对路径，避免跨 shell 变量。

另一个问题是 WSL 继承 Windows PATH 后，`rg` 可能先命中 WindowsApps 内无执行
权限的 Codex 版本。应在 WSL 安装 Linux `ripgrep` 并确认 `command -v rg`，
临时情况下使用 `/usr/bin/grep`，不要误判为仓库权限故障。

## 8. 干净 worktree 缺少 Explore Lite

现象：核心包构建成功，但 unknown-world readiness 报 `explore_lite not found`。

原因：前沿探索使用固定 revision 和仓库补丁，属于单独安装步骤。

方案：

```bash
bash scripts/setup_frontier_exploration.sh
```

workspace doctor 通过后再启动严格 E2E。

## 9. ROS APT 只升级一部分包

现象：运行中安装 `twist_mux` 后，Nav2 lifecycle manager 出现
`undefined symbol`，但代码和 launch 本身没有变化。

根因：`rclcpp/diagnostic_updater` 已升级，而已安装的 `nav2-*` 仍是旧 patch
版本，进程加载了 ABI 不一致的共享库。

方案：演示过程中禁止 APT 变更；安装依赖后统一升级已安装的同一 ROS 组件族，
重新构建工作区，再做一个最小 Nav2 启动烟测。

## 10. 首次 frontier 平台期固定阈值假失败

现象：严格验收在 69.9 m、21/21 目标终态后，因为恢复扫描只增长
28 cells / 0.001088 而判失败；离线正式 evaluator 却得到 99.78% 总覆盖和
98.51% 最差区域覆盖。

根因：首次 `reachable_frontiers_stalled` 被立即按 40 cells / 0.002 双阈值
拒绝，缺少独立确认 epoch。

方案：不降低阈值、不读取真值；使用连续两个低收益 explorer epoch、账本排空、
地图静默、最终 probe 和 typed STOP 形成有界饱和证据。首次 stall 不能直接完成。

## 11. 频繁推送触发昂贵 CI

现象：每个小提交都触发约 20 分钟容器构建。

方案：本地完成一整个功能闭环和相关门禁后再推送；PR 内保留按职责拆分的本地 commit，但减少无意义远端触发。

## 12. 两个键盘进程共享控制权身份

现象：两个终端同时运行 `keyboard_control.sh`，都使用 `keyboard_teleop` requester
和同一个速度 topic，owner 校验无法区分具体进程。

方案：脚本按 Linux 用户和 `ROS_DOMAIN_ID` 获取非阻塞 `flock`；第二个实例立即
报错退出。进程异常结束后内核自动释放锁，不依赖手工删除 PID 文件。

## 13. 验收只等待“出现过一次正确速度”

现象：切权后偶尔出现正确样本就 PASS，但稳定阶段仍可能混入旧来源非零速度。

方案：先隔离 DDS/mux/gate 过渡期，再对稳定窗口内全部 Twist 样本断言；HOLD、
ESTOP、manager lease 失效时任何轴的非零值都直接失败，并实际重启 manager 验证
新 epoch 仍从 HOLD 开始。

## 14. RESUME 后立即持续发送非零速度

现象：控制权已经是 AUTONOMY，但 Gate 一直输出零速，验收似乎“卡住”。

根因：Twist 没有 manager epoch。为防止 DDS 在切权后补送旧非零帧，Gate 会把
恢复后的自治源置入 quarantine；连续非零流会不断重置静默窗，永远不能直接放行。

方案：新一代自治控制器先发送一帧明确零速作为 source-ready 握手，再发送新的
运动命令。验收探针也必须遵守生产协议，不能为了 PASS 绕过隔离。

## 15. Python 与 C++ 租约语义不一致

现象：SLAM 编排层认为 AUTONOMY 可以继续，高层任务已入队，但 ActionGuard 和
最终速度 Gate 永久拒绝，形成 split-brain。

根因：Python 曾在同 epoch 断租后接受迟到 heartbeat；C++ 使用 sticky
fail-closed，只允许新 epoch 的 `seq=0/HOLD` 恢复。

方案：两种语言共享同一 wire invariant；同 epoch 一旦断租，heartbeat 和更高
transition sequence 都不刷新本地时间，必须等待安全的新 manager bootstrap。

## 16. 重叠 colcon 构建耗尽 WSL 内存

现象：8 GiB WSL 中 `vmmemWSL` 超过 6 GiB、Swap 接近耗尽，普通命令数十秒无响应。

根因：两个 `colcon build --packages-up-to` 同时运行，又叠加多个 VS Code C++
索引进程；`--packages-up-to embodied_simulation` 还会拉入本轮无关的在线/离线
Agent 和导航依赖。

方案：先确认没有旧 colcon/cmake 进程，再使用
`CMAKE_BUILD_PARALLEL_LEVEL=1`、`--executor sequential` 和精确
`--packages-select`；本轮只构建 interfaces、middleware、C++ control 和
simulation 四个边界包。禁止并发启动第二次构建。

## 17. manager 重启被误当成安全冷启动

现象：旧 Nav2 goal 仍存活时单独重启 manager，新 epoch 可以直接 RESUME。

根因：历史默认值把 bootstrap 视为已经收到 quiescence ACK。

方案：节点和共享 launch factory 都默认
`bootstrap_quiescence_acknowledged=false`；恢复必须由会话编排器提交同代 typed
ACK。纯 C++ 状态机也使用相同安全默认值。阶段 launch 若试图同时开启
`control_authority_enabled` 和自带 manager 会直接 fail-fast，必须改由
`voice_slam_nav_showcase.sh auto` 的会话根持有 manager/coordinator。STOP 的
`command_id` 同时包含 manager epoch，避免重启后被旧幂等键去重。
