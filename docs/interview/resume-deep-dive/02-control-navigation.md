# 机器人控制与自主导航追问

## Q1. 一条“去办公室”指令最终怎样让机器人移动？

口述：

语音层先把“去办公室”解析成带目标名的结构化动作，C++ 校验目标白名单后交给动作调度器。执行节点加载 Nav2 执行插件，把“办公室”从 YAML 转成 `map` 坐标系下的 `PoseStamped`，再发送 `NavigateToPose` goal。Nav2 根据地图、定位和代价地图完成规划与控制，并自己发布 `/cmd_vel`。结果回到执行节点后，再沿 Action 和 `command_id` 返回会话层。

源码：

- 语义地点：[nav2_places.cpp](../../../src/embodied_simulation/src/nav2_places.cpp)
- Nav2 适配：[nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)
- 执行插件接口：[robot_executor.hpp](../../../src/embodied_simulation/include/embodied_simulation/robot_executor.hpp)

运行：

```text
1. RobotCommand{action_type=NAVIGATE_TO, target="办公室"}
2. Nav2Places::to_pose_stamped() 取得 x、y、yaw
3. Nav2RobotExecutor::send_navigate_goal() 发送 goal
4. bt_navigator 调用规划器、控制器和恢复行为
5. Nav2 controller 发布 /cmd_vel
6. result callback 转成项目统一终态
```

取舍：项目没有自己实现完整全局规划器和局部控制器，而是把成熟 Nav2 作为导航引擎，自己负责语义适配、任务状态、安全边界和结果判定。

## Q2. 项目如何控制 `/cmd_vel`？为什么 Nav2 模式不能再发零速度？

口述：

手动运动和简单避障模式由 `SimulationController` 计算线速度、角速度，再由 20 Hz 控制定时器发布 `/cmd_vel`。Nav2 模式下，速度由 Nav2 controller 发布，因此执行插件返回 `publishes_cmd_vel() == false`，控制节点停止周期性发布。否则即使只发布零速度，也会和 Nav2 抢同一 Topic，导致机器人抖动或无法前进。

源码：

- 控制循环：[simulation_control_node.cpp](../../../src/embodied_simulation/src/simulation_control_node.cpp)
- 速度输出：[simulation_ros_io.cpp](../../../src/embodied_simulation/src/simulation_ros_io.cpp)
- Nav2 插件：[nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)

关键代码：

```cpp
const auto output = executor_->step(now);
if (executor_->publishes_cmd_vel()) {
  // 只有当前插件拥有速度控制权时才发布。
  ros_io_->publish_velocity(output.velocity.linear_x, output.velocity.angular_z);
}
```

设计原则：同一时刻只能有一个速度仲裁者。真实机器人若还有遥控、导航和急停多个来源，应增加明确的速度复用或控制权仲裁层。

## Q3. 速度限幅和加速度限制具体怎么实现？

口述：

候选动作进入系统时，C++ 校验层先限制允许的速度和时长；执行端仍再次按控制器配置限幅，防止其他入口旁路上游校验。控制周期中不直接从当前速度跳到目标速度，而是按 `acceleration * dt` 逐步逼近。急停、传感器超时和近障停车不走平滑减速，直接归零。

源码：[simulation_controller.cpp](../../../src/embodied_simulation/src/simulation_controller.cpp)

```cpp
target.linear_x = clamp(target.linear_x, -max_linear, max_linear);
target.angular_z = clamp(target.angular_z, -max_angular, max_angular);

// 普通运动按每周期允许的最大变化量平滑逼近。
current.linear_x = approach(
  current.linear_x, target.linear_x, linear_acceleration * dt);

// 安全停车直接覆盖当前命令，不等待平滑过程。
if (emergency_stop || sensor_stale || safety_stopped) {
  current = target;  // target 此时已经是零速度
}
```

取舍：双层限幅不是重复劳动。边界层定义业务允许范围，执行层保护具体设备和旁路输入。

## Q4. LaserScan 在控制链中怎样使用？

口述：

控制器遍历每个有效量测，根据 `angle_min + index * angle_increment` 得到角度，并统计前、左、右三个扇区的最近距离。前方进入紧急距离时禁止继续前进；避障模式根据左右空间选择转向；超过 `scan_timeout` 没有新数据时，自主前进被停止。这里不用整帧最小值，因为机器人后方的近障不应阻止向前移动。

源码：[simulation_controller.cpp](../../../src/embodied_simulation/src/simulation_controller.cpp)

运行：

1. Gazebo 通过桥接发布 `/scan`。
2. ROS 回调把 ranges 和角度信息交给执行插件。
3. `update_scan()` 更新三个方向距离和最后时间戳。
4. `step()` 在下一控制周期判断近障和传感器是否过期。
5. 输出速度和原因进入 `/cmd_vel` 与状态消息。

边界：这是项目中的轻量控制和安全逻辑；Nav2 模式下主要避障由局部、全局 costmap 和 controller 完成。

## Q5. Odometry 和 TF 有什么区别？

口述：

Odometry 是带时间戳的运动估计消息，通常包含位姿、速度和协方差；TF 描述不同坐标系在某个时刻的空间关系。项目用 `/odom` 累计本次建图行驶里程，用 TF 让 SLAM、AMCL、Nav2 在 `map`、`odom`、`base_link` 和传感器坐标之间转换。只发布 Odometry 不会自动生成 TF，两者要由对应节点分别维护。

源码：

- Gazebo 桥接：[turtlebot3_bridge.yaml](../../../src/embodied_simulation/config/turtlebot3_bridge.yaml)
- 里程累计：[showcase_session_node.py](../../../src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py)
- 回放时显式 TF：[replay_node.py](../../../src/embodied_slam_tools/embodied_slam_tools/replay_node.py)

典型坐标链：

```text
map 到 odom：建图时由 SLAM Toolbox 发布，定位时由 AMCL 发布
odom 到 base_link：由底盘或仿真里程计发布，短时连续但会漂移
base_link 到 laser：由机器人模型发布的固定外参
```

追问：为什么需要 `odom` 中间层？它提供局部连续运动；`map` 允许定位或回环后全局修正。直接修正 `odom` 会让局部控制看到位姿跳变。

## Q6. SLAM Toolbox 在项目中负责什么？

口述：

SLAM Toolbox 消费 LaserScan、里程和 TF，维护位姿图并发布地图以及 `map` 到 `odom` 的变换。项目把它作为主建图引擎，自己实现自动任务编排、探索结束证据和额外回环实验。也就是说，项目不是从零重写完整 SLAM 前端，而是围绕现有建图系统补充可控流程和算法验证。

源码：

- 建图入口：[mapping_baseline.launch.py](../../../src/embodied_slam/launch/mapping_baseline.launch.py)
- 自动任务：[mission_executor.py](../../../src/embodied_slam_tools/embodied_slam_tools/mission_executor.py)
- 位姿图实验：[gtsam_pose_graph.cpp](../../../src/embodied_slam/src/gtsam_pose_graph.cpp)

运行：机器人移动产生 scan 与 odom，SLAM Toolbox 更新图和 OccupancyGrid；任务层观察地图已知栅格、占用栅格和行驶里程；结束后调用 map saver 保存本轮地图。

## Q7. Frontier 探索是怎样实现的？为什么不能只等“没有 frontier”？

口述：

Frontier 是已知自由区域与未知区域的边界，Explore Lite 选择目标并通过 Nav2 到达。项目的监控器不只相信探索器的一条完成状态，还同时检查最短运行时间、已知栅格、占用栅格、建图里程和地图是否进入平台期。这样可以防止传感器尚未稳定、探索器提前退出却被误判为建图完成。

源码：

- 探索监控：[frontier_monitor.py](../../../src/embodied_slam_tools/embodied_slam_tools/frontier_monitor.py)
- 完成策略：[showcase_session.py](../../../src/embodied_slam_tools/embodied_slam_tools/showcase_session.py)
- 证据累计：[mapping_evidence.py](../../../src/embodied_slam_tools/embodied_slam_tools/mapping_evidence.py)

完成原因只有三类：

1. `no_frontiers`：探索器完成，并且覆盖和里程门槛达标。
2. `coverage_plateau`：地图在一段时间内不再增长，并且门槛达标。
3. `time_budget_coverage`：时间预算到达，但当前地图已经满足验收门槛。

取消时先发布停止动作并停止 explorer，再把取消异常交给任务状态机。

## Q8. 为什么建图结束后要先存图，再启动 AMCL 和 Nav2？

口述：

建图和定位对 `map` 到 `odom` 变换的所有权不同，不能让 SLAM Toolbox 和 AMCL 同时竞争。项目先停止探索并保存 YAML 和图像文件，再关闭 mapping 阶段，随后启动 map_server、AMCL 和 Nav2。任务层等待系统健康、两个 Nav2 Action server 和关键 Lifecycle 节点都进入活动状态，之后才发送巡检目标。

源码：

- 阶段进程管理：[stage_process_manager.py](../../../src/embodied_slam_tools/embodied_slam_tools/stage_process_manager.py)
- 高层事务：[mission_executor.py](../../../src/embodied_slam_tools/embodied_slam_tools/mission_executor.py)
- 就绪检查：[showcase_session_node.py](../../../src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py)

运行顺序：

```text
停止 explorer
保存并检查本次地图文件
停止 mapping 进程组
启动 map_server、AMCL、Nav2
等待健康状态、Action server 和 Lifecycle active
执行单点导航与多点巡航
```

## Q9. AMCL 的作用是什么？它和 SLAM 有什么区别？

口述：

SLAM 同时估计轨迹并构建未知地图；AMCL 在已有静态地图上估计机器人位姿。AMCL 用粒子表示多个位姿假设，根据运动模型预测，再用 LaserScan 与地图匹配更新权重，并通过重采样集中到高概率区域。项目存图后切换到 AMCL，是为了复用固定地图进行稳定导航，而不是继续修改地图。

项目落点：[localization_navigation.launch.py](../../../src/embodied_slam/launch/localization_navigation.launch.py)

追问：初始位姿错误怎么办？需要在 RViz 或程序中提供较合理的初始位姿，观察粒子是否收敛，并检查 `map` 到 `odom` TF 和激光与地图是否对齐。仅看到 AMCL 节点启动不代表定位成功。

## Q10. Nav2 内部有哪些关键部分？

口述：

Nav2 的导航任务由行为树组织，规划器在全局代价地图上生成路径，控制器根据局部代价地图和机器人状态输出速度，恢复行为处理清图、旋转或等待等异常。map_server 提供静态地图，AMCL 提供全局定位，TF 把所有数据放到一致坐标系。项目通过 `NavigateToPose` 和 `FollowWaypoints` 接入，不绕过 Nav2 直接发送长距离速度。

项目配置：[voice_nav2_turtlebot3.launch.py](../../../src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py)

边界：路径搜索和局部控制主要由 Nav2 插件实现。面试时可以解释 A*、Dijkstra、DWB 等原理，但不要说成当前仓库自行实现了这些算法。

## Q11. 多点巡航怎样处理轮数和漏点？

口述：

项目把语义地点数组转换成多个 `PoseStamped` 后发送 `FollowWaypoints`。业务接口中的 `number_of_loops` 表示总遍历轮数，而 Nav2 表示首轮之后额外重复次数，所以代码下发 `loops - 1`。结果不能只看 Action 协议的 `SUCCEEDED`，还要检查业务错误码和 `missed_waypoints` 是否为空。

源码：

- goal 转换：[nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)
- 结果策略：[nav2_result_policy.cpp](../../../src/embodied_simulation/src/nav2_result_policy.cpp)

```cpp
goal.number_of_loops = std::max(1U, total_loops) - 1U;

success =
  result_code == SUCCEEDED &&
  result != nullptr &&
  result->error_code == NONE &&
  result->missed_waypoints.empty();
// 协议结束不等于每个巡检点都到达。
```

## Q12. 导航取消和抢占怎样避免“旧任务复活”？

口述：

用户急停时，上层清除等待命令，C++ 调度器请求取消当前 Action，并把停止命令放到队首。Nav2 插件取出当前 goal handle 后递增 generation，随后请求取消。即使旧 goal 的接受回调或结果回调稍后到达，也会发现 generation 已变化；晚到的已接受 goal 还会被立即取消，不能在后台继续导航。

源码：

- 全局调度：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)
- Nav2 代次控制：[nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)

关键竞态：

```text
发送 goal
用户先说“停下”
旧 goal 随后才被 Nav2 接受
```

只忽略旧回调还不够，因为 Nav2 已经开始执行。项目在旧 goal response 到达时主动调用 `async_cancel_goal()`。

## Q13. 动态障碍处理链路怎样运行？

口述：

检测输入先进入动态目标跟踪节点，跟踪器通过门控匹配维持目标 ID，并估计位置、速度和不确定性。预测代价层订阅稳定轨迹，在未来时间窗口内把预测位置写入 Nav2 costmap，控制器随后绕开高代价区域或重新规划。超过观测超时的数据会被清除，防止消失的障碍永久留在地图里。

源码：

- 数据关联：[gated_observation_assignment.cpp](../../../src/embodied_navigation/src/gated_observation_assignment.cpp)
- 运动估计：[dynamic_obstacle_tracker.cpp](../../../src/embodied_navigation/src/dynamic_obstacle_tracker.cpp)
- Nav2 代价层：[predicted_obstacle_layer.cpp](../../../src/embodied_navigation/src/predicted_obstacle_layer.cpp)

默认配置使用约 2 秒预测窗口、0.25 秒步长和 0.8 秒观测超时。项目提供当前点、平滑匀速、Kalman 和 IMM 等模型。

边界：当前端到端演示使用确定性 Gazebo 障碍输入，重点验证“跟踪、预测、costmap、导航”链路，不代表已经完成真实感知。

## Q14. 怎样证明导航真的完成，而不是只证明 goal 发出去了？

口述：

首先检查 Action goal 被接受并取得终态；多点任务还检查漏点。其次检查地图和 TF 是否来自本次会话、Odometry 路径是否发生合理变化、AMCL 和 Nav2 是否就绪。最后检查任务结束或取消后的 `/cmd_vel` 为零。项目把这些证据写入 E2E 报告，不能用一行“success”日志代替。

源码与测试：

- 验收场景：[slam_nav_e2e.py](../../../tools/acceptance/scenarios/slam_nav_e2e.py)
- 证据判定：[slam_nav_evidence.py](../../../tools/acceptance/slam_nav_evidence.py)
- 测试说明：[TESTING.md](../../TESTING.md)

回答重点：发布命令是输入证据，Action result 是协议证据，地图、位姿、路径和零速度才是运行效果证据。
