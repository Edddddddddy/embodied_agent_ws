#pragma once

#include <string>
#include <vector>

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "embodied_simulation/action_execution.hpp"

namespace embodied_simulation
{

struct Nav2FollowWaypointsOutcome
{
  ActionExecutionState state{ActionExecutionState::kBlocked};
  std::string detail;
};

/// 把 Nav2 的协议终态转换为机器人业务终态，并保留可诊断的漏点信息。
Nav2FollowWaypointsOutcome evaluate_follow_waypoints_result(
  const std::vector<std::string> & waypoints,
  rclcpp_action::ResultCode code,
  const nav2_msgs::action::FollowWaypoints::Result * result);

}  // namespace embodied_simulation
