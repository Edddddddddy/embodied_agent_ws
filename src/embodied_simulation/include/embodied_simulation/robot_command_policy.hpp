#ifndef EMBODIED_SIMULATION__ROBOT_COMMAND_POLICY_HPP_
#define EMBODIED_SIMULATION__ROBOT_COMMAND_POLICY_HPP_

#include <embodied_agent_interfaces/msg/robot_command.hpp>

namespace embodied_simulation
{

/// 判断经过 ActionGuard 规范化后的命令能否进入仿真执行层。
///
/// 这里刻意不重复 ActionGuard 的来源、priority 和速度限幅规则：Guard 负责把
/// 候选命令变成安全命令，本策略只定义 BT 与无 BT 执行路径共享的最后一道契约。
bool is_executable_robot_command(
  const embodied_agent_interfaces::msg::RobotCommand & command);

}  // namespace embodied_simulation

#endif  // EMBODIED_SIMULATION__ROBOT_COMMAND_POLICY_HPP_
