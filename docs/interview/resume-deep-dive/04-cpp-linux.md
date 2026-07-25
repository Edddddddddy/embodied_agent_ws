# 现代 C++ 与 Linux 追问

## Q1. RAII 是什么？项目中哪里真正使用了？

口述：

RAII 是让资源生命周期绑定对象生命周期：构造或初始化时取得资源，析构时自动释放。项目中的 UART、SPI 文件描述符由局部 `FileDescriptor` 管理，即使 `open` 后的配置过程抛异常，析构仍会关闭 fd。线程、锁、智能指针和 TTS 模型资源也遵循同样思路，减少每个失败分支手写清理。

源码：[hardware_transport.cpp](../../../src/embodied_agent_cpp/src/hardware_transport.cpp)

```cpp
class FileDescriptor {
public:
  ~FileDescriptor() {
    if (value_ >= 0) {
      ::close(value_);           // 任意返回或异常路径都会执行
    }
  }

  void reset(int value = -1) {
    if (value_ >= 0) {
      ::close(value_);           // 替换资源前先释放旧 fd
    }
    value_ = value;
  }
};
```

取舍：RAII 解决进程内资源释放，但业务关停仍要先执行停止、取消和结果通知；不能只等析构函数“收尾”。

## Q2. `unique_ptr`、`shared_ptr` 和普通值对象怎么选？

口述：

单一所有者、运行时多态或可选的大对象使用 `unique_ptr`，例如调度器、行为树、执行器和文件传输后端。ROS 的 Node、publisher、Action handle 经常由框架和回调共同持有，所以接口类型使用 `shared_ptr`。配置、命令、状态快照等小对象采用值语义，减少共享可变状态。选择依据是所有权，不是对象大小。

项目例子：

| 类型 | 代码例子 | 原因 |
| --- | --- | --- |
| 值对象 | `RobotCommand`、`ControllerOutput`、`SessionSnapshot` | 拷贝后互不影响，便于测试 |
| `unique_ptr` | `ActionScheduler`、`CommandBehaviorTree`、`HardwareTransport` | 生命周期归一个组件独占 |
| `shared_ptr` | ROS Node、Action goal handle、plugin 实例 | 框架回调需要延长生命周期 |
| `optional` | 当前 active command、可选外部执行状态 | 明确表达“当前没有值” |

源码：

- [typed_action_bridge_node.cpp](../../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)
- [robot_executor.hpp](../../../src/embodied_simulation/include/embodied_simulation/robot_executor.hpp)
- [action_scheduler.hpp](../../../src/embodied_agent_cpp/include/embodied_agent_cpp/action_scheduler.hpp)

追问：为什么调度器不直接 `shared_ptr`？它只由桥接节点拥有，额外共享会模糊谁负责清理。

## Q3. 移动语义在项目中起什么作用？

口述：

移动语义把临时对象持有的字符串、容器或所有权转移到目标对象，避免不必要拷贝。调度器派发命令时按值接收，再 `std::move` 到活动槽和事件；行为树实现对象也通过 `unique_ptr` 移动所有权。移动后的对象只保证可析构和可重新赋值，不能继续假设原内容存在。

源码：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)

```cpp
SchedulerEvent ActionScheduler::dispatch(RobotCommand command) {
  active_ = command;                 // 活动槽保留自己的命令
  SchedulerEvent event;
  event.command = std::move(command); // 临时副本的缓冲区转给事件
  return event;                       // 返回值还可由编译器消除拷贝
}
```

取舍：这里活动槽和派发事件都需要完整命令，所以仍有一次拷贝。若只保存 ID 再共享同一对象，会增加生命周期和并发复杂度，收益有限。

## Q4. 项目怎样使用 STL 容器表达业务语义？

口述：

容器选择与操作模式对应。调度等待队列用 `deque`，因为普通命令尾插、急停头插、执行时头删；当前任务用 `optional`，清楚表示 idle 或 active；结果按 ID 查询使用 `dict` 或 `unordered_map`；回调产生的多个副作用先放进 `vector`，释放锁后统一处理。容器不是只看复杂度，还要看状态表达是否清楚。

源码：

- C++ 队列：[action_scheduler.hpp](../../../src/embodied_agent_cpp/include/embodied_agent_cpp/action_scheduler.hpp)
- Python 结果表：[action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)

追问：调度器重复 ID 检查为什么还是线性扫描？等待队列上限很小，线性扫描更简单；若规模显著扩大，可额外维护 ID 集合，但要保证每次入队、出队、清队列同步更新。

## Q5. 音频模块为什么使用三个线程？

口述：

PortAudio 输入回调属于实时敏感线程，只把当前帧复制到有界队列并通知 worker；处理线程完成回声抵消、音频指标、VAD 和 ROS 发布；播放线程输出 TTS PCM，同时把播放样本写入回声参考缓冲。三者隔离后，模型推理、ROS 调度和扬声器写入不会直接阻塞采集回调。

源码：[audio_frontend_node.cpp](../../../src/embodied_agent_cpp/src/audio_frontend_node.cpp)

运行：

```text
PortAudio callback：复制、入队、通知
processing_loop：取帧、AEC、VAD、发布
playback_loop：取播放数据、写扬声器、补充 AEC reference
```

队列上限为 50 帧。处理落后时丢最旧音频，因为实时语音更需要低延迟；无限队列会让系统持续处理已经过时的声音。

## Q6. `mutex`、`condition_variable` 和 `atomic` 分别解决什么问题？

口述：

`mutex` 保护需要一起保持一致的复合状态，例如队列与 active goal；`condition_variable` 让消费者在没有数据或结果时睡眠，避免忙轮询；`atomic` 适合独立的简单状态，例如导航是否 active。项目不会用一个全局锁保护所有内容，而是按调度状态、goal handle、诊断文本和音频缓冲拆锁。

源码：

- 条件变量等待 Action 结果：[action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)
- C++ goal 与状态锁：[nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)
- 音频队列：[audio_frontend_node.cpp](../../../src/embodied_agent_cpp/src/audio_frontend_node.cpp)

关键原则：持锁时只修改内存状态，释放锁后再发送 ROS goal、发布消息或调用外部 provider，避免不可控调用扩大临界区。

## Q7. 怎样避免死锁？

口述：

首先规定每把锁只保护一类状态，减少同时持有多把锁。其次在锁内生成纯 `SchedulerEvent`，出锁后再执行 ROS Action 调用；外部回调不会在持有调度锁时重入同一对象。等待条件变量时使用循环重新检查条件，因为可能虚假唤醒，也可能先被取消。

源码：[typed_action_bridge_node.cpp](../../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)

```cpp
std::vector<SchedulerEvent> events;
{
  std::lock_guard<std::mutex> lock(scheduler_mutex_);
  events = scheduler_->enqueue(command); // 锁内只改纯状态
}
process_events(events);                  // 锁外调用 ROS API
```

追问：固定锁顺序是否有用？有，多锁不可避免时必须定义顺序；本项目更优先减少嵌套锁。

## Q8. 异步回调为什么需要 generation，而不只是 `atomic_bool canceled`？

口述：

布尔值只能说明“现在是否取消”，无法区分连续两代任务。旧任务取消后，新任务可能已开始，此时把布尔值重新设为 false，旧回调就可能误认为自己有效。generation 每开始或取消一代就递增，回调捕获发送时的值；只有与当前值一致才允许写状态。

源码：[nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)

```cpp
const auto my_generation = ++navigate_generation_;

options.result_callback = [this, my_generation](auto result) {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (my_generation != navigate_generation_) {
    return; // 这属于已取消的旧任务
  }
  // 只有当前代可以更新 active 和结果
};
```

相同模式也用于 ASR 延迟提交 timer，防止 Lifecycle 停用后后台线程访问已关闭 provider。

## Q9. PImpl 为什么适合动态目标跟踪器？

口述：

PImpl 把复杂滤波器状态和实现依赖放到 `.cpp` 中，公共头文件只暴露稳定接口。调用方不需要看到 Kalman、IMM、轨迹内部结构，也不会因实现字段变化大面积重新编译。项目的 `DynamicObstacleTracker` 持有 `unique_ptr<Impl>`，构造、移动和析构都集中管理。

源码：

- 接口：[dynamic_obstacle_tracker.hpp](../../../src/embodied_navigation/include/embodied_navigation/dynamic_obstacle_tracker.hpp)
- 实现：[dynamic_obstacle_tracker.cpp](../../../src/embodied_navigation/src/dynamic_obstacle_tracker.cpp)

边界：PImpl 会多一次间接访问和堆分配，不应机械套在每个简单类上。这里的收益是隐藏多个运动模型及跟踪状态。

## Q10. UART 为什么使用非阻塞 fd 和 `poll()`？

口述：

串口以 `O_NONBLOCK` 打开，发送前用 `poll(POLLOUT)` 等待可写，并设置 100 ms 上限，避免设备异常时永久卡住 ROS 回调。`write()` 可能只写一部分，代码循环推进偏移量；遇到 `EINTR` 重试，遇到暂时不可写继续等待，其他错误转成异常。这样既处理 Linux 系统调用语义，也给上层明确超时。

源码：[hardware_transport.cpp](../../../src/embodied_agent_cpp/src/hardware_transport.cpp)

运行：

```text
open(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC)
配置 raw termios 和波特率
poll 等待 POLLOUT
循环 write 直到整帧完成
超时或错误返回上层
```

`O_CLOEXEC` 防止进程以后执行其他程序时意外继承设备 fd。

## Q11. 项目中的进程、线程和 IPC 分别用在哪里？

口述：

音频处理、模型 turn 和 ROS executor 主要使用线程，共享进程内内存并通过锁同步。Gazebo、SLAM、Nav2 和验收场景是独立进程，项目用新 session 和进程组管理整棵子进程树。节点间 IPC 主要由 ROS 2 中间件承担；验收并行资源分配还使用 Linux `flock`，避免两个场景占用同一 ROS domain 或证据目录。

源码：

- 进程组管理：[process_supervisor.py](../../../tools/acceptance/process_supervisor.py)
- 文件锁租约：[leases.py](../../../tools/acceptance/leases.py)
- SLAM 阶段进程：[stage_process_manager.py](../../../src/embodied_slam_tools/embodied_slam_tools/stage_process_manager.py)

取舍：线程通信快，但共享状态更难隔离；进程故障隔离好，但启动、清理和 IPC 成本更高。

## Q12. 为什么停止外部进程不能只调用父进程的 `terminate()`？

口述：

ROS Launch、Gazebo 和 Nav2 会继续创建子进程，个别孙进程还可能建立新 session。只结束最外层父进程会留下后台 server、占用端口和 `/clock`，污染下一次测试。项目启动时建立进程组，清理时记录 `/proc` 子树中的所有组，先发 `SIGTERM` 等待，再用 `SIGKILL` 收口，并回收僵尸进程。

源码：[process_supervisor.py](../../../tools/acceptance/process_supervisor.py)

边界：这是 Linux 进程监督逻辑，不等同于 ROS Lifecycle。Lifecycle 管节点内部资源，进程监督负责 OS 级存活和孤儿清理。

## Q13. 如何用 GDB、Valgrind 和 perf 排查这个项目？

口述：

崩溃或死锁先用 GDB 查看线程栈、断点和 core；内存泄漏、越界和未初始化读取用 Valgrind 或 sanitizer；CPU 占用和热点用 perf 采样，再定位到音频 DSP、滤波或序列化函数。工具前要先复现稳定场景，并区分真实热点和 debug 构建、仿真负载带来的噪声。

典型排查顺序：

```bash
gdb --args ros2 run embodied_agent_cpp audio_frontend
valgrind --leak-check=full <真实可执行文件及参数>
perf record -g <真实可执行文件及参数>
perf report
```

项目落点：音频掉帧时同时观察队列丢帧计数、处理耗时和 perf 火焰图；导航不动先看 Action/TF/costmap，而不是直接用 CPU profiler。

## Q14. 项目怎样降低实时链路中的抖动？

口述：

实时敏感回调只做固定量复制和入队，模型加载放在启动阶段，ASR 尾部静音数组在构造时预分配，队列有界防止内存和延迟无限增长。控制循环使用单调时间计算超时，耗时外部调用放到 worker 或异步 Action 中。真正的硬实时还需要实时调度、内存锁定和可证明的最坏执行时间，本项目只做到软实时工程约束。

源码：

- 音频线程：[audio_frontend_node.cpp](../../../src/embodied_agent_cpp/src/audio_frontend_node.cpp)
- 离线 ASR 预分配：[sherpa_asr.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py)
- 控制周期：[simulation_controller.cpp](../../../src/embodied_simulation/src/simulation_controller.cpp)
