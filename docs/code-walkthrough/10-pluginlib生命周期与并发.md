# 10 pluginlib、生命周期与并发

## 源码导航

| 文件与关键行 | 符号 | 观察点 |
|---|---|---|
| `src/embodied_simulation/include/embodied_simulation/robot_executor.hpp:12` | `RobotExecutor` | 插件契约 |
| `src/embodied_simulation/src/robot_executor_plugins.cpp:15` | `GazeboRobotExecutor` | 仿真实现 |
| `src/embodied_simulation/src/robot_executor_plugins.cpp:85` | `MockRobotExecutor` | 无传感器测试实现 |
| `src/embodied_simulation/robot_executor_plugins.xml:1` | plugin declarations | 稳定插件名到 C++ 类型 |
| `src/embodied_simulation/src/simulation_control_node.cpp:56` | `on_configure` | class loader 与资源创建 |
| `src/embodied_simulation/src/simulation_control_node.cpp:177` | `on_deactivate` | goal/stop/zero/timer 顺序 |
| `src/embodied_simulation/src/simulation_control_node.cpp:681` | `publish_diagnostics` | mutex 快照读取 |

## 1. RobotExecutor 是执行边界

`RobotExecutor` 接口只暴露：

```cpp
configure(config)
execute(command, now)
stop()
update_scan(...)
step(now)
mode_name()
backend_name()
```

`SimulationControlNode` 管 Action、BT、Lifecycle、diagnostics，不知道后端内部细节。后端必须满足两个重要契约：

- `execute/step` 不能长时间阻塞 executor；
- `stop` 必须幂等并立即把输出归零。

这就是深接口：外部调用很少，内部可以封装控制器或测试状态。

## 2. 两个插件

### GazeboRobotExecutor

封装 `SimulationController`：move/turn 转成 manual command，scan 进入控制器，step 返回安全速度。

### MockRobotExecutor

不依赖 Gazebo/scan，按时间保存一个速度。它用于 CI 验证 plugin 加载、Action、BT 和状态，不证明真实物理控制。

两个实现共享同一个 `RobotCommand` 和 `ControllerOutput`，所以切换插件不修改 Action server。

## 3. pluginlib 的发现链

插件能被加载需要四层一致：

1. C++ 类继承 `RobotExecutor`；
2. `PLUGINLIB_EXPORT_CLASS(Derived, Base)` 导出；
3. `robot_executor_plugins.xml` 声明 name/type/base；
4. CMake `pluginlib_export_plugin_description_file()` 安装描述。

运行时：

```cpp
pluginlib::ClassLoader<RobotExecutor> loader(
    "embodied_simulation", "embodied_simulation::RobotExecutor");
executor_ = loader.createSharedInstance(executor_plugin_);
```

参数值是 XML 中稳定的 `name`，如 `embodied_simulation/MockRobotExecutor`，不是任意 C++ 类名。

## 4. 为什么不使用大 if/else

| pluginlib | node 内 if/else |
|---|---|
| 新后端可以独立编译注册 | 每加后端都改核心节点 |
| 核心依赖抽象 | 核心知道所有实现 |
| 可按参数运行时选类 | 分支简单直接 |
| 加载/安装错误更复杂 | 编译期错误更直观 |

只有一个简单实现时 if 更轻；当 Gazebo、mock、不同硬件后端共享任务协议时 plugin seam 才真正有价值。当前硬件控制器走另一套 `HardwareTransport` factory，尚未统一为 `RobotExecutor` plugin，这是架构边界。

## 5. Lifecycle 资源不变量

### configure

- 读取和交叉校验参数；
- 加载 executor plugin 并 configure；
- 加载 BT XML；
- 创建 pub/sub/action server/timer；
- timer 保持 cancel。

任何一步失败返回 FAILURE，不进入 inactive-ready 状态。

### activate

- 激活 lifecycle publishers；
- reset control/diagnostics timers；
- 发布当前 mode。

### deactivate

顺序是安全关键：

1. 若有 active goal，取消 BT 并结束 Action；
2. executor stop；
3. publisher 仍 active 时发布零速度；
4. cancel timers；
5. deactivate publishers。

若先 deactivate publisher，再发零速度，停车消息会丢失。

### cleanup/shutdown

cleanup reset ROS interfaces 和 plugin；shutdown 还先 stop。`reset_interfaces()` 清空 active goal、execution、BT status 和 diagnostics resources，保证下一次 configure 从干净状态开始。

## 6. Executor 与 callback group

独立入口使用 2 线程 `MultiThreadedExecutor`。但多个线程不代表所有 callback 都自动并行：默认 callback group 通常是 MutuallyExclusive，组内 callback 不并发。

代码为 diagnostics timer 创建独立 `MutuallyExclusive` group，因此它可与默认组的 Action/control callback 并行；同一 diagnostics group 内仍不会重入。

目的：1 Hz diagnostics 发布不应该阻止 20 Hz 控制，反之控制繁忙也不应长期饿死诊断。

## 7. 共享状态保护

### diagnostics snapshot

control tick 写 `diagnostic_output_`，diagnostics timer 读。两线程之间用 `diagnostics_mutex_`，读出副本后立即释放锁，再构造/发布消息。

这缩短临界区，避免 publish 时持锁。

### action_active

diagnostics timer 读取，Action callback 写，因此使用 `std::atomic_bool`。

### 其他 Action 状态

`active_goal_`、`action_execution_`、BT 等主要位于默认 MutuallyExclusive callback group 中按序访问。若未来把 Action server或 control timer 分配到 Reentrant/不同 group，现有隐含串行假设会失效，需要增加锁或单线程状态 owner。

## 8. diagnostics 语义

`make_executor_diagnostic()` 是纯函数，优先级：

```text
safety_stopped -> ERROR
else sensor_stale && simulation -> WARN
else -> OK
```

values 包含 lifecycle、plugin、backend、mode、active action、sensor stale、safety stopped、reason。

为什么只对 simulation backend 的 stale scan 报 WARN：mock backend 没有真实传感器，不应因没有 scan 永久告警。

diagnostics 是状态快照，不是健康探针的全部。节点彻底崩溃时它不会发布，监控还要检查消息新鲜度和节点存活。

## 9. Composition

`SimulationControlNode` 同时注册为 component，独立 executable 通过 factory 创建同一类。这样不会因两种部署复制业务逻辑。

Component container 使用 `component_container_mt`。namespace 同时施加给 container、component、lifecycle manager 和 typed bridge，执行链的相对名称保持一致。

风险：多个 component 共享进程，未捕获异常或内存错误影响范围更大；callback group 和线程安全也更复杂。

## 10. 测试证据

- `test_robot_executor_plugins.cpp`：两个类可被发现，同一 move 可通过两种 adapter，stop 后归零。
- `test_node_configuration.cpp`：控制频率、timeout、速度/加速度、距离关系和插件名校验。
- `test_executor_diagnostics.cpp`：OK/WARN/ERROR 优先级和字段。
- integration smoke：mock executor、composition、namespace、lifecycle。

## 11. 面试回答模板

**问题：pluginlib、Lifecycle 和多线程各解决什么？**

它们解决不同问题。pluginlib 把 Action/BT 与具体执行后端解耦，核心节点只依赖 `RobotExecutor`，Gazebo 和 mock 通过 XML 与导出宏动态注册。Lifecycle 管资源状态：configure 才加载插件和创建接口，activate 才启动控制 timer，deactivate 会先终止目标、stop 并发布零速度，再关闭 publisher。多线程 executor 用于让诊断与控制并行，diagnostics timer 在独立 callback group，通过 mutex 读取控制快照，active flag 用 atomic。它们分别处理可替换性、可控启停和并发调度，不能互相替代。

## 12. 自测

1. 新插件需要修改哪四层注册信息？
2. deactivate 为什么要在 publisher 失活前发零速度？
3. MultiThreadedExecutor 为什么不保证默认组内 callback 并行？
4. diagnostics 为什么复制快照后再释放锁？
5. 若把 control timer 放入 Reentrant group，哪些状态需要重新审计？
