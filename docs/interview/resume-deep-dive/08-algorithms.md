# 算法与数据结构追问

## Q1. 数据结构和算法在这个项目里不是只用于刷题吗？

口述：

项目中数据结构直接决定任务语义。`deque` 支持普通命令尾插和急停头插，`optional` 表示是否有活动任务，哈希表按 ID 关联异步结果，优先门控和全局匹配用于动态目标关联，图结构用于位姿约束优化。刷题训练的是复杂度和边界意识，工程中还要考虑线程安全、时间戳、异常和状态所有权。

代码例子：

- 队列：[action_scheduler.hpp](../../../src/embodied_agent_cpp/include/embodied_agent_cpp/action_scheduler.hpp)
- 结果表：[action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)
- 数据关联：[gated_observation_assignment.cpp](../../../src/embodied_navigation/src/gated_observation_assignment.cpp)
- 位姿图：[gtsam_pose_graph.cpp](../../../src/embodied_slam/src/gtsam_pose_graph.cpp)

## Q2. DFS 和 BFS 在机器人导航中怎样使用？

口述：

DFS 更适合遍历所有连通区域、回溯或检测结构；BFS 在无权图中按层扩展，可得到最少边数路径。在栅格地图中，四邻域或八邻域栅格可以看作图，BFS 能做可达性、连通域和等代价最短路。实际 Nav2 通常使用更成熟的规划插件，当前仓库没有自行实现 BFS 导航器，所以面试时应讲原理和适用场景。

复杂度：若每个栅格只访问一次，时间和空间通常为 `O(V + E)`；二维规则栅格中可近似看作 `O(width * height)`。

场景题：判断语义目标是否与机器人处于同一自由空间连通域，可以在膨胀后的二值栅格上做 BFS；必须先处理未知区和障碍膨胀，否则算法路径不等于机器人可走路径。

## Q3. Dijkstra 和 A* 有什么区别？

口述：

Dijkstra 按当前累计代价最小的节点扩展，非负边权下保证最短路；A* 在累计代价上加入启发函数，优先接近目标的节点。启发函数不高估真实剩余代价时，A* 仍能保证最优，并通常减少扩展数量。栅格中常用欧氏距离或曼哈顿距离，但还要把障碍膨胀、转弯和代价地图成本加入路径代价。

项目边界：Nav2 的全局规划由所选 planner plugin 完成，本项目负责配置、Action 接入和结果验证，没有把它包装成自研 A*。

追问：Dijkstra 是否属于动态规划？可以把它理解为不断确定当前最优子问题的贪心算法；它利用最短路的最优子结构，但标准实现通常归类为贪心。

## Q4. Frontier 探索和 BFS 有什么关系？

口述：

Frontier 是已知自由栅格与未知栅格的边界。典型实现先在占用栅格中搜索机器人可达自由区域，再提取相邻未知区域的边界簇，对候选按距离、信息增益等评分并发送导航目标。项目使用 Explore Lite 生成 frontier 目标，自己实现的是完成监控和证据门槛，不应说成重写了全部 frontier 搜索。

源码：

- 任务监控：[frontier_monitor.py](../../../src/embodied_slam_tools/embodied_slam_tools/frontier_monitor.py)
- 完成条件：[showcase_session.py](../../../src/embodied_slam_tools/embodied_slam_tools/showcase_session.py)

设计重点：算法返回“当前没有候选”也可能是传感器未就绪或地图过小，所以业务层还检查已知栅格、占用栅格、里程和最短运行时间。

## Q5. 动态目标为什么需要门控和匹配？

口述：

一帧检测只有坐标，没有稳定身份。跟踪器先根据各轨迹预测位置构造代价矩阵；距离或马氏距离超过门槛的边直接禁止，再在可行边中分配观测。门控减少错误关联，匹配保证同一个观测不会同时更新两条轨迹。未匹配轨迹可以短时保留，未匹配观测创建新轨迹。

源码：[gated_observation_assignment.cpp](../../../src/embodied_navigation/src/gated_observation_assignment.cpp)

```text
轨迹预测
构造轨迹与观测的距离矩阵
欧氏距离或 NIS 门控
贪心或全局一对一分配
更新已匹配轨迹
处理未匹配轨迹和新观测
```

马氏距离把预测协方差和测量噪声纳入尺度；不确定性大时允许更宽的位置偏差，但仍受外层最大欧氏距离限制。

## Q6. 贪心最近邻和匈牙利全局匹配有什么区别？

口述：

贪心算法按轨迹顺序选择当前最近的未用观测，实现简单，但先处理的轨迹可能占走另一个轨迹更合适的观测。全局匹配同时最小化总成本，交叉目标或距离接近时更稳定。项目为每条轨迹增加一个私有 dummy 列表示本帧未匹配，再用矩形匈牙利算法求解。

源码：[gated_observation_assignment.cpp](../../../src/embodied_navigation/src/gated_observation_assignment.cpp)

```text
真实观测列：匹配成本
每条轨迹的私有 dummy 列：未匹配成本
被门控拒绝的边：极大禁止成本
```

复杂度：当前实现约为 `O(n²m)`。机器人现场动态目标数量通常很少，开销远低于感知和 costmap。

## Q7. Kalman 滤波器怎样估计动态障碍？

口述：

每个坐标轴使用位置和速度作为状态。预测阶段用匀速模型推进位置，并把过程噪声加入协方差；更新阶段计算观测创新、创新方差和 Kalman 增益，再修正位置、速度及协方差。预测时间越长，不确定性越大；测量噪声越大，更新时越不信任当前观测。

源码：[dynamic_obstacle_tracker.cpp](../../../src/embodied_navigation/src/dynamic_obstacle_tracker.cpp)

```text
状态：x = [position, velocity]
预测：x_k = F x_(k-1)
协方差：P_k = F P F^T + Q
创新：y = z - Hx
增益：K = P H^T / (H P H^T + R)
更新：x = x + Ky
```

项目分别对 x、y 两个轴更新，并把位置方差用于马氏距离关联和预测代价。

## Q8. IMM 为什么比单一匀速模型复杂？

口述：

动态目标可能在静止和机动之间切换，单一模型很难同时平滑静止噪声和快速跟随机动。IMM 维护多个模型及概率：先按 Markov 转移概率混合各模型状态，再分别预测和更新，最后根据观测创新似然更新模型概率并融合结果。项目使用低运动和机动两类模型做实验。

源码：[dynamic_obstacle_tracker.cpp](../../../src/embodied_navigation/src/dynamic_obstacle_tracker.cpp)

三个关键步骤：

1. 交互：按转移概率混合上帧状态与协方差。
2. 模型更新：每个模型独立预测并吸收同一观测。
3. 概率融合：创新越符合某模型，该模型的后验概率越高。

边界：模型更复杂不保证任何场景都更好，需要用轨迹误差、关联稳定性和计算耗时做对照。

## Q9. 位姿图优化解决什么问题？

口述：

位姿图把机器人位姿作为节点，把相邻里程约束和回环约束作为边，求一组位姿使所有约束残差总体最小。项目将 `Pose2` 和约束写入 GTSAM factor graph，给最早位姿加先验消除整体平移旋转自由度，再用 Levenberg-Marquardt 优化。回环边还可使用鲁棒核或 switch variable 降低错误约束破坏整张图的风险。

源码：[gtsam_pose_graph.cpp](../../../src/embodied_slam/src/gtsam_pose_graph.cpp)

```cpp
graph.add(PriorFactor(first_pose));       // 固定坐标基准
graph.add(BetweenFactor(i, j, relative)); // 里程或回环边
result = LevenbergMarquardtOptimizer(graph, initial).optimize();
```

代码还会把接近奇异的协方差对称化并抬高最小特征值，防止求逆和噪声模型失稳。

## Q10. 为什么回环不能只看相似度或 ICP RMSE？

口述：

描述子相似只适合召回候选，长走廊等重复场景可能外观相似；ICP 的低 RMSE 也可能来自小重叠或退化几何。项目依次检查历史间隔、候选相似、局部子图匹配、重叠率、可观测性、歧义和多帧时序一致性，再决定记录或提交约束。后端鲁棒核是最后保护，不能替代前端证据。

源码入口：

- 候选：[lidar_loop_candidate_node.cpp](../../../src/embodied_slam/src/lidar_loop_candidate_node.cpp)
- 几何验证：[lidar_loop_verifier.cpp](../../../src/embodied_slam/src/lidar_loop_verifier.cpp)
- 约束门控：[lidar_loop_constraint_gate.cpp](../../../src/embodied_slam/src/lidar_loop_constraint_gate.cpp)
- GTSAM：[gtsam_pose_graph.cpp](../../../src/embodied_slam/src/gtsam_pose_graph.cpp)

边界：新增 LiDAR 回环约束默认以观察和报告为主，不应说成已在任意真实场景稳定自动入图。

## Q11. 动态规划在机器人任务中可以怎样使用？

口述：

动态规划适合具有重复子问题和最优子结构的决策，例如少量巡检点的最短访问顺序、带能量约束的任务选择或时间窗规划。若巡检点数量很小，可用状态压缩 DP，状态为“已访问集合和当前位置”；数量增大后复杂度 `O(n²2ⁿ)` 很快不可接受，需要启发式或专用求解器。当前项目按用户给定 waypoint 顺序巡检，没有实现路线优化 DP。

回答边界：可以讲算法设计，但不能把 LeetCode 的 DP 经验直接等同于机器人在线规划成果。

## Q12. 如何把 LeetCode 经验转化成工程能力？

口述：

我不会只说完成题数，而会说明它训练了边界条件、复杂度和数据结构选择。项目里具体体现为：有界队列避免无限积压，ID 哈希表关联异步结果，dummy 节点把“未匹配”纳入全局分配，generation 处理旧回调，图约束先做异常门控。工程实现还必须补线程安全、超时、错误恢复、测试和可观测性，这是刷题本身没有覆盖的部分。
