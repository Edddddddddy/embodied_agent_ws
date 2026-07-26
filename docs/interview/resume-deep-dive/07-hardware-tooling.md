# 硬件通信与工程工具追问

## Q1. UART、SPI、CAN 和 EtherCAT 有什么区别？

口述：

UART 是异步串行点对点通信，实现简单但协议和校验通常由应用自定义；SPI 是主从同步总线，吞吐较高、距离短，需要片选；CAN 是多节点差分总线，带仲裁、帧校验和错误处理，适合车载与机器人设备；EtherCAT 面向实时工业以太网，通过从站在线处理帧实现确定性周期通信。当前项目实现了 UART、SPI 和 mock 后端，CAN、EtherCAT 只属于原理了解。

| 协议 | 典型特点 | 常见机器人用途 |
| --- | --- | --- |
| UART | 点对点、低成本、自定义帧 | MCU 调试、简单底盘控制 |
| SPI | 主从、短距离、高速 | 板内传感器、ADC、外设 |
| CAN | 多节点、仲裁和错误检测 | 电机、底盘、车辆总线 |
| EtherCAT | 周期实时、分布式时钟 | 多轴伺服、工业机器人 |

## Q2. 项目的硬件帧怎样设计？

口述：

协议层把结构化机器人命令编码成固定头、版本、操作码、负载长度、序号、负载和 CRC16。固定同步字用于找帧头，版本支持以后升级，序号可用于关联发送与确认，长度解决消息边界，CRC 检测传输错误。MOVE 和 TURN 的浮点参数转换成定点整数，避免上下位机浮点格式差异。

源码：[hardware_protocol.cpp](../../../src/embodied_agent_cpp/src/hardware_protocol.cpp)

```text
0xAA 0x55
version
opcode
payload_length
sequence
payload
crc16
```

运行：

1. `HardwareControllerNode` 收到可信 `RobotCommand`。
2. `HardwareProtocol::encode()` 检查动作并构造 payload。
3. `crc16()` 覆盖帧内容。
4. `HardwareTransport::send()` 交给 mock、UART 或 SPI。
5. 控制节点发布发送状态，并为运动命令启动 watchdog。

边界：当前协议重点验证编码和传输调用，没有实现真实下位机 ACK 解码状态机。

## Q3. 为什么要有序号和 CRC？

口述：

CRC 用于发现比特错误，但不能证明设备已经执行；序号用于区分不同发送请求和重复确认，但也不自动提供可靠重传。完整协议还需要定义 ACK、NACK、超时、重试次数、设备重启后的序号恢复和幂等规则。项目目前有发送序号和 CRC，闭环确认仍是事实边界。

源码：[hardware_protocol.cpp](../../../src/embodied_agent_cpp/src/hardware_protocol.cpp)

追问：为什么不用简单累加和？CRC 对连续突发错误的检测能力更好，且实现成本低。CRC 不是加密签名，不能防恶意篡改。

## Q4. 为什么把协议编码和传输设备分开？

口述：

协议层只负责“一个命令应该变成哪些字节”，传输层只负责“这些字节怎样送到设备”。这样同一帧可以通过 mock、UART 或 SPI 发送，协议单测不需要真实设备，系统调用测试也不需要构造完整 ROS 节点。以后新增 CAN 时可以增加 transport，不必修改动作字段和 CRC 规则。

源码：

- 协议接口：[hardware_protocol.hpp](../../../src/embodied_agent_cpp/include/embodied_agent_cpp/hardware_protocol.hpp)
- 传输实现：[hardware_transport.cpp](../../../src/embodied_agent_cpp/src/hardware_transport.cpp)
- 控制节点：[hardware_controller_node.cpp](../../../src/embodied_agent_cpp/src/hardware_controller_node.cpp)

这是依赖倒置：上层依赖 `HardwareTransport` 能力，而不是依赖 `/dev/ttyUSB0` 的具体系统调用。

## Q5. UART 发送怎样处理部分写和超时？

口述：

串口以非阻塞方式打开并配置 raw termios。发送时先用 `poll()` 等待可写，随后循环 `write()`，每次只推进实际写入的字节数；`EINTR` 表示被信号打断，可以重试，暂时不可写则继续等待。超过 100 ms 或出现其他错误时向上返回失败，不能无限卡住控制线程。

源码：[hardware_transport.cpp](../../../src/embodied_agent_cpp/src/hardware_transport.cpp)

关键点：

```text
write 返回值可能小于剩余长度
0 不代表整帧成功
EINTR 需要重试
EAGAIN 需要重新等待可写
所有路径由 RAII 关闭 fd
```

## Q6. SPI 发送和 UART 有什么不同？

口述：

SPI 打开 `/dev/spidev*` 后通过 `ioctl` 配置 mode、每字位数和时钟频率，再用 `SPI_IOC_MESSAGE` 发起一次传输。UART 是连续字节流，需要自己处理部分写和帧边界；SPI 一次 ioctl 更接近一次事务，但仍要处理设备、片选和驱动错误。当前项目只发送编码帧，没有设计同步读回的设备状态协议。

源码：[hardware_transport.cpp](../../../src/embodied_agent_cpp/src/hardware_transport.cpp)

边界：Linux spidev 验证不等于目标板时序已经满足，需要示波器、逻辑分析仪和真实从设备联调。

## Q7. 运动 watchdog 怎样工作？

口述：

发送 MOVE 或 TURN 后，控制节点根据命令时长设置单调时钟 deadline。20 ms timer 检查是否到期；到期后发送 STOP，若传输失败则保留到下一周期重试。这样上层崩溃或漏发停止时，端侧适配器仍会主动收口运动。

源码：

- deadline 逻辑：[hardware_protocol.cpp](../../../src/embodied_agent_cpp/src/hardware_protocol.cpp)
- timer 与停止：[hardware_controller_node.cpp](../../../src/embodied_agent_cpp/src/hardware_controller_node.cpp)

设计补充：生产底盘还应在 MCU 内实现更底层 watchdog，因为 Linux 进程本身也可能失去调度或整机断联。

## Q8. 怎样证明硬件命令真正执行了？

口述：

“帧编码成功”和“write 成功”只证明字节交给了内核或驱动，不能证明 MCU 收到、更不能证明电机完成动作。完整闭环需要下位机 ACK、设备状态、编码器或 Odometry 反馈，并用 sequence 对应请求；超时后进入失败或安全停车。当前仓库主要验证编码、系统调用和 watchdog，因此简历应写接口与模拟验证，不写真实底盘闭环。

建议的真实闭环：

```text
命令序号
下位机 ACK
执行中状态
编码器或里程反馈
完成或故障终态
超时触发安全停止
```

## Q9. Gazebo 和 RViz 分别做什么？

口述：

Gazebo 模拟物理世界、机器人运动和传感器，项目通过桥接获得 `/scan`、`/odom`、TF、IMU，并把 `/cmd_vel` 送回仿真。RViz 不负责物理模拟，它用于查看地图、激光、Odometry、TF、路径和导航目标。两者经常一起使用，但一个是仿真运行环境，一个是 ROS 数据可视化工具。

源码：

- Gazebo 桥接：[turtlebot3_bridge.yaml](../../../src/embodied_simulation/config/turtlebot3_bridge.yaml)
- RViz 配置：[voice_nav2_demo.rviz](../../../src/embodied_simulation/rviz/voice_nav2_demo.rviz)
- 综合启动：[voice_nav2_turtlebot3.launch.py](../../../src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py)

边界：Gazebo 能验证接口、算法和可重复场景，不能完全复现实机摩擦、传感器噪声、时钟抖动和算力限制。

## Q10. CMake 和 Colcon 在项目中怎样配合？

口述：

CMake 负责单个 C++ 包内的 target、依赖、编译选项、安装规则和 gtest；Ament 提供 ROS 2 包集成；Colcon 读取整个工作空间的包依赖图，按顺序构建和测试。接口包先生成消息代码，控制、仿真和导航包再链接它。Python 包则通过 `setup.py` 安装模块和 ROS 可执行入口。

源码：

- 接口生成：[embodied_agent_interfaces/CMakeLists.txt](../../../src/embodied_agent_interfaces/CMakeLists.txt)
- C++ 构建：[embodied_agent_cpp/CMakeLists.txt](../../../src/embodied_agent_cpp/CMakeLists.txt)
- Python 打包：[embodied_online_agent/setup.py](../../../src/embodied_online_agent/setup.py)

追问：为什么安装 launch、config 和 plugin XML？运行时通过 ament index 查找包 share 目录；只在源码树中存在而未安装，`ros2 launch` 或 pluginlib 在 install 空间会找不到。

## Q11. gtest 和 pytest 分别测什么？

口述：

gtest 主要验证 C++ 纯策略和系统调用封装，例如动作队列、字段校验、导航结果、滤波、图优化和硬件帧；pytest 验证 Python 会话、状态机、provider、任务编排和验收工具。测试语言跟被测模块一致，但共同原则是先把纯逻辑从 ROS Node 中抽出来，再用少量 smoke 和 E2E 验证接线。

项目例子：

| 层级 | 例子 | 能证明什么 |
| --- | --- | --- |
| gtest | `test_action_scheduler.cpp` | 取消、重复 ID、迟到结果 |
| pytest | `test_frontier_monitor.py` | 探索结束、取消和提前失败 |
| ROS smoke | `smoke_test_typed_action_pipeline.sh` | 节点和 Action 实际接线 |
| Gazebo E2E | SLAM/Nav2 acceptance | 运动、地图、定位和终态 |

边界：单元测试通过不能证明 DDS、TF 或 Gazebo 正常；E2E 通过也不能替代边界条件单测。

## Q12. CI 怎样运行？Docker 在项目中是什么边界？

口述：

GitHub Actions 先运行仓库结构契约测试，再在 ROS 2 Jazzy 容器环境中执行 rosdep、Colcon build 和
test。真实模型、麦克风和完整 Gazebo 门禁留在本地，因为云端 runner 缺少设备和稳定图形环境。
仓库已有项目 Dockerfile、Compose、容器测试门禁和运行镜像入口；当前仍不能写成“已完成带模型、
设备映射和全部 sidecar 的生产容器部署”。

源码：

- [ros2-ci.yml](../../../.github/workflows/ros2-ci.yml)
- [Dockerfile](../../../Dockerfile)
- [compose.yaml](../../../compose.yaml)

可以表达：完成多阶段镜像、Compose 测试门禁和可追溯交付入口。不要表达：模型、WSLg 麦克风和
全部在线/离线 provider 已在生产容器中完成设备部署。

## Q13. 如何设计 CAN 或 EtherCAT 的后续接入？

口述：

先保留现有结构化命令和安全调度，在硬件 Adapter 下新增 transport。CAN 需要定义消息 ID、分帧、ACK、bus-off 恢复和设备心跳；EtherCAT 需要定义 PDO、状态机、周期线程、分布式时钟和失步处理。无论哪种协议，机器人任务终态都应来自设备反馈，而不是发送系统调用成功。

回答结构：

1. 先定义业务命令与设备状态。
2. 再定义帧或 PDO 映射及版本。
3. 加入超时、重连、故障状态和幂等。
4. mock 验证状态机。
5. 总线仿真或回环测试。
6. 真实设备测时序、故障注入和安全停车。

边界：这是开发设计，不是当前成果。

## Q14. Git 在这类 ROS 2 多包项目中怎样使用？

口述：

一个仓库可以通过 `git worktree` 同时检出多条分支，它们共享对象库和 refs，但各自拥有工作目录与
index；所以“有几个目录”不等于“有几个项目”。功能从 `dev` 拉 `feature/*` 或 `fix/*`，经 PR/CI
回到 `dev`；发布候选再合入 `main`，并创建 annotated Tag 和 GitHub Release。PR 中同时说明接口变化、
测试命令和事实边界，完整功能收口后再推送，避免每个微小编辑都触发 CI。

源码、Launch、配置和测试进入版本库；`build`、`install`、`log`、模型权重、设备数据和密钥不提交。
每个 worktree 要在自己的源码目录构建，随后 source 自己的 `install/setup.bash`；混用另一个 worktree
的 install 会让“看到的源码”和“实际加载的节点”不一致，产生最难定位的假回归。

项目证据：

- CI：[ros2-ci.yml](../../../.github/workflows/ros2-ci.yml)
- 忽略规则：[.gitignore](../../../.gitignore)
- 完整流程：[12-git-worktree-release-governance.md](12-git-worktree-release-governance.md)

追问：大模型或 bag 如何管理？模型可使用独立制品仓库或下载脚本并记录版本与哈希；大体积测试数据可使用对象存储或 Git LFS，但不能把未脱敏设备数据直接提交到普通仓库。
