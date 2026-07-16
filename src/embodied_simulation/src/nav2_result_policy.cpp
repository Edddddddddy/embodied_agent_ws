#include "embodied_simulation/nav2_result_policy.hpp"

#include <sstream>

namespace embodied_simulation
{
namespace
{

std::string result_code_name(rclcpp_action::ResultCode code)
{
  if (code == rclcpp_action::ResultCode::SUCCEEDED) {
    return "succeeded";
  }
  if (code == rclcpp_action::ResultCode::CANCELED) {
    return "canceled";
  }
  if (code == rclcpp_action::ResultCode::ABORTED) {
    return "aborted";
  }
  return "unknown";
}

std::string join(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ',';
    }
    stream << values[index];
  }
  return stream.str();
}

}  // namespace

Nav2FollowWaypointsOutcome evaluate_follow_waypoints_result(
  const std::vector<std::string> & waypoints,
  rclcpp_action::ResultCode code,
  const nav2_msgs::action::FollowWaypoints::Result * result)
{
  Nav2FollowWaypointsOutcome outcome;
  std::ostringstream detail;
  detail << "nav2:follow_waypoints:" << result_code_name(code)
         << " waypoints=" << join(waypoints);

  if (result == nullptr) {
    detail << " result_missing=true";
  } else {
    detail << " error_code=" << result->error_code
           << " missed_waypoints=" << result->missed_waypoints.size();
    if (!result->missed_waypoints.empty()) {
      detail << " missed_detail=";
      for (std::size_t index = 0; index < result->missed_waypoints.size(); ++index) {
        if (index > 0) {
          detail << ',';
        }
        const auto & missed = result->missed_waypoints[index];
        detail << missed.index << ':' << missed.error_code;
      }
    }
    if (!result->error_msg.empty()) {
      detail << " error_msg=" << result->error_msg;
    }
  }

  if (code == rclcpp_action::ResultCode::CANCELED) {
    outcome.state = ActionExecutionState::kCanceled;
  } else if (
    code == rclcpp_action::ResultCode::SUCCEEDED && result != nullptr &&
    result->error_code == nav2_msgs::action::FollowWaypoints::Result::NONE &&
    result->missed_waypoints.empty())
  {
    outcome.state = ActionExecutionState::kSucceeded;
  } else {
    // WaypointFollower 可在 stop_on_failure=false 时返回 SUCCEEDED 并携带漏点。
    // 巡检业务要求每个目标都到达，因此协议成功不能覆盖部分失败。
    outcome.state = ActionExecutionState::kBlocked;
  }
  outcome.detail = detail.str();
  return outcome;
}

}  // namespace embodied_simulation
